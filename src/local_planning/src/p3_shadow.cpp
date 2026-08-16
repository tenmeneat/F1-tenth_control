// Copyright 2026 2026_IFAC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "local_planning/p3_shadow.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "local_planning/p3_analytic_solver.hpp"
#include "local_planning/raceline_spline_planner.hpp"

namespace local_planning
{

// Production-owned runtime port of:
//   corridor_to_analytic_root_mapping_audit.cpp sha256 c3bdfc282d5c0b38342b763e0473d69e
//   p3_branch_analytic_solver.hpp sha256 a5accda81bc18dade6801dcb66973070b65d6f518
// The frozen CURVATURE_CONTINUITY semantic hash is
// b4282a44d50b2f4721d3edf8b7c32c3c8f4a1954a96a01887c2e01ee559eeb31.
//   p3_mapping_static_and_temporal_continuity_design_review.cpp (validated M1 closure).
// Container types are the only adaptation: equations, M0-first invocation, M1 round-robin
// templates, branch filtering, reconstruction, exact validation, ranking, and cap-24 remain
// literal. No external evaluator is linked or invoked at runtime.
class P3ShadowEvaluator
{
public:
  explicit P3ShadowEvaluator(const RacelineSplinePlanner & planner)
  : planner_(planner), parameters_(planner.parameters_)
  {
  }

  P3ShadowResult run(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    std::int64_t snapshot_source_stamp_ns,
    std::uint64_t snapshot_epoch,
    std::uint64_t global_reference_generation,
    const std::string & p0_failure_reason,
    bool relaxed_clearance_gate = false) const
  {
    P3ShadowResult result;
    result.enabled = true;
    result.invoked = true;
    result.snapshot_source_stamp_ns = snapshot_source_stamp_ns;
    result.snapshot_epoch = snapshot_epoch;
    result.global_reference_generation = global_reference_generation;
    result.p0_failure_reason = p0_failure_reason;
    const auto total_start = Clock::now();

    if (!planner_.ready()) {
      result.failure_classification = "REFERENCE_NOT_READY";
      result.runtime_total_us = elapsedUs(total_start);
      return result;
    }
    if (!std::isfinite(ego.s) || !std::isfinite(ego.d) || !std::isfinite(ego.speed)) {
      result.failure_classification = "NONFINITE_EGO";
      result.runtime_total_us = elapsedUs(total_start);
      return result;
    }

    const P3ShadowPlanningContext context = planner_.buildP3ShadowPlanningContext(
      ego, obstacles, relaxed_clearance_gate);
    if (!context.valid) {
      result.failure_classification = context.reason.empty() ?
        "NO_BLOCKING_CLUSTER" : context.reason;
      result.runtime_total_us = elapsedUs(total_start);
      return result;
    }
    result.cluster_obstacle_ids = context.cluster_ids;
    // computeSideTargetRange fills the same longitudinal cluster span for both sides before any
    // side-feasibility rejection. Keep this observation separate from selected-candidate fields.
    result.cluster_start_forward_m = context.right.cluster_start;
    result.cluster_end_forward_m = context.right.cluster_end;

    bool m0_nonpositive_abort = false;
    std::vector<SideResult> sides;
    sides.reserve(2U);
    try {
      for (const bool go_left : {false, true}) {
        const auto & domain = go_left ? context.left : context.right;
        sides.push_back(evaluateSide(
            ego, obstacles, context.visible, domain, context.outside_is_left));
      }
    } catch (const std::runtime_error & error) {
      if (std::string(error.what()) != "non-positive quintic-Hermite segment") {
        throw;
      }
      // The frozen mapping review catches the whole M0 policy at this boundary. Preserve that
      // behavior so h0<=0 becomes a fail-closed no-M0 offer instead of escaping the callback.
      m0_nonpositive_abort = true;
      sides.clear();
      SideResult aborted;
      aborted.domain_valid = true;
      aborted.failure = "NON_POSITIVE_SEGMENT_ABORT";
      sides.push_back(std::move(aborted));
    }

    // Frozen M0 runQuery semantics retain exactly one canonical side record: RIGHT first when
    // neither side has a hard-valid candidate, otherwise the globally ranked feasible side.
    const SideResult * baseline = nullptr;
    for (const auto & side : sides) {
      if (!side.domain_valid) {
        continue;
      }
      if (baseline == nullptr ||
        (side.best_index.has_value() && !baseline->best_index.has_value()) ||
        (side.best_index.has_value() && baseline->best_index.has_value() &&
        betterFeasible(
          side.candidates[*side.best_index], baseline->candidates[*baseline->best_index])))
      {
        baseline = &side;
      }
    }
    if (baseline == nullptr) {
      result.failure_classification = "NO_VALID_SIDE_DOMAIN";
      result.runtime_total_us = elapsedUs(total_start);
      return result;
    }

    const auto append_side_metrics = [&](const SideResult & side) {
        result.raw_root_count += side.raw_root_count;
        result.finite_root_count += side.finite_root_count;
        result.branch_root_count += side.branch_root_count;
        result.bounded_root_count += side.bounded_root_count;
        result.accepted_root_count += side.accepted_root_count;
        result.runtime_corridor_us += side.runtime_corridor_us;
        result.runtime_root_solver_us += side.runtime_root_solver_us;
        result.runtime_reconstruction_us += side.runtime_reconstruction_us;
        result.runtime_hard_validation_us += side.runtime_hard_validation_us;
      };
    append_side_metrics(*baseline);
    result.selected_probe_s = baseline->probe.station;
    result.selected_probe_d = baseline->probe.desired;
    result.m0_candidate_count = baseline->candidates.size();
    result.m0_validator_call_count = baseline->validator_calls;
    result.m0_hard_valid_count = baseline->hard_valid_count;
    result.candidates.insert(
      result.candidates.end(), baseline->candidates.begin(), baseline->candidates.end());

    std::optional<P3ShadowCandidateTrace> selected;
    if (baseline->best_index.has_value()) {
      selected = baseline->candidates[*baseline->best_index];
    } else {
      ExtensionOutcome extension;
      if (!m0_nonpositive_abort) {
        try {
          extension = evaluateM0Extension(ego, obstacles, context);
        } catch (const std::runtime_error & error) {
          if (std::string(error.what()) != "non-positive quintic-Hermite segment") {
            throw;
          }
          // The validated review treats a V2 constructor boundary abort as no M0 mapping offer;
          // M1 then receives the complete cap-24 budget and its strict segment guard is authority.
          m0_nonpositive_abort = true;
        }
      }
      if (m0_nonpositive_abort) {
        result.raw_root_count = 0U;
        result.finite_root_count = 0U;
        result.branch_root_count = 0U;
        result.bounded_root_count = 0U;
        result.accepted_root_count = 0U;
        result.m0_candidate_count = 0U;
        result.m0_validator_call_count = 0U;
        result.m0_hard_valid_count = 0U;
        result.candidates.clear();
      }
      if (!m0_nonpositive_abort) {
        result.raw_root_count += extension.raw_root_count;
        result.finite_root_count += extension.finite_root_count;
        result.branch_root_count += extension.branch_root_count;
        result.bounded_root_count += extension.bounded_root_count;
        result.accepted_root_count += extension.accepted_root_count;
        result.m0_candidate_count += extension.candidates.size();
        result.m0_validator_call_count += extension.validator_calls;
        result.m0_hard_valid_count += extension.hard_valid_count;
        result.runtime_root_solver_us += extension.runtime_root_solver_us;
        result.runtime_reconstruction_us += extension.runtime_reconstruction_us;
        result.runtime_hard_validation_us += extension.runtime_hard_validation_us;
        if (extension.best_index.has_value()) {
          selected = extension.candidates[*extension.best_index];
        }
        result.candidates.insert(
          result.candidates.end(), extension.candidates.begin(), extension.candidates.end());
      }
      if (!selected.has_value()) {
        if (result.m0_candidate_count > kTotalCandidateCap) {
          throw std::runtime_error("frozen M0 candidate count exceeds total predeclared cap");
        }
        result.m1_invoked = true;
        result.m1_budget = kTotalCandidateCap - result.m0_candidate_count;
        ExtensionOutcome m1 = evaluateM1(ego, obstacles, context, result);
        result.runtime_root_solver_us += m1.runtime_root_solver_us;
        result.runtime_reconstruction_us += m1.runtime_reconstruction_us;
        result.runtime_hard_validation_us += m1.runtime_hard_validation_us;
        if (m1.best_index.has_value()) {
          selected = m1.candidates[*m1.best_index];
        }
        result.candidates.insert(
          result.candidates.end(), m1.candidates.begin(), m1.candidates.end());
        if (result.m0_candidate_count + result.m1_candidate_count > kTotalCandidateCap ||
          result.m1_validator_call_count != result.m1_candidate_count)
        {
          throw std::runtime_error("M0+M1 constructed-candidate/validator-call bound violated");
        }
      }
    }

    result.candidate_count = result.m0_candidate_count + result.m1_candidate_count;
    result.hard_validator_call_count =
      result.m0_validator_call_count + result.m1_validator_call_count;
    result.hard_valid_count = result.m0_hard_valid_count + result.m1_hard_valid_count;
    if (selected.has_value()) {
      result.would_recover = true;
      result.selected_go_left = selected->go_left;
      result.selected_obstacle_ids = context.cluster_ids;
      const auto & selected_domain = selected->go_left ? context.left : context.right;
      result.selected_cluster_start_forward_m = selected_domain.cluster_start;
      result.selected_cluster_end_forward_m = selected_domain.cluster_end;
      result.selected_cluster_end_s = planner_.wrapS(ego.s + selected_domain.cluster_end);
      result.selected_source = selected->mapping_source;
      result.selected_candidate_template = selected->candidate_template;
      result.selected_source_cell = selected->source_cell;
      result.selected_component_id = selected->component_id;
      result.selected_source_branch_regime = selected->source_branch_regime;
      result.selected_candidate_identity = selected->candidate_identity;
      result.selected_logical_identity = selected->logical_identity;
      result.selected_path_digest = selected->path_digest;
      result.selected_d_target = selected->d_target;
      result.selected_d_mid = selected->d_mid;
      result.selected_min_track_margin_m = selected->minimum_track_margin_m;
      result.selected_min_obstacle_margin_m = selected->minimum_obstacle_margin_m;
      result.selected_curvature_margin =
        parameters_.maximum_curvature_radpm - selected->peak_curvature_radpm;
      result.selected_curvature_rate_margin =
        parameters_.maximum_curvature_rate_radpm2 - selected->peak_curvature_rate_radpm2;
      result.selected_slope_margin =
        parameters_.maximum_lateral_slope - selected->peak_lateral_slope;
      result.selected_min_speed_mps = selected->minimum_commanded_speed_mps;
      result.selected_max_speed_mps = selected->maximum_commanded_speed_mps;
      result.selected_validation = selected->validation;
      result.selected_validation_available = true;
      result.selected_path = selected->path;
      result.failure_classification = "NONE";
    } else if (result.m1_boundary_handoff_unresolved_count > 0U) {
      result.failure_classification = "BOUNDARY_HANDOFF_UNRESOLVED";
    } else if (result.m1_invoked) {
      result.failure_classification = "NO_HARD_VALID_M1_CANDIDATE";
    } else {
      result.failure_classification = baseline->failure;
    }
    result.runtime_total_us = elapsedUs(total_start);
    return result;
  }

private:
  using Clock = std::chrono::steady_clock;
  static constexpr double kEpsilon = 1.0e-9;
  static constexpr std::size_t kFrozenCandidateCap = 16U;
  static constexpr std::size_t kM0ExtensionCandidateCap = 12U;
  static constexpr std::size_t kTotalCandidateCap = 24U;

  struct Interval
  {
    double lower{0.0};
    double upper{0.0};
  };

  struct CorridorSample
  {
    double station{0.0};
    double curvature{0.0};
    Interval track;
    std::vector<Interval> feasible;
    std::vector<int> active_obstacles;
  };

  struct BranchSample
  {
    double station{0.0};
    double lower{0.0};
    double upper{0.0};
    double center{0.0};
    double width{0.0};
    double curvature{0.0};
  };

  struct Corridor
  {
    std::vector<CorridorSample> samples;
    std::vector<BranchSample> branch;
    std::string branch_id;
    std::size_t branch_count{0U};
    bool connected{false};
  };

  struct Probe
  {
    double station{std::numeric_limits<double>::quiet_NaN()};
    double desired{std::numeric_limits<double>::quiet_NaN()};
    std::string location_rule;
    std::string anchor_rule;
  };

  struct QuinticSegment
  {
    double start{0.0};
    double end{0.0};
    std::array<double, 6> coefficients{};

    double evaluate(double station) const
    {
      const double t = std::clamp((station - start) / (end - start), 0.0, 1.0);
      double value = coefficients[5];
      for (int index = 4; index >= 0; --index) {
        value = value * t + coefficients[static_cast<std::size_t>(index)];
      }
      return value;
    }
  };

  struct KnotStates
  {
    std::array<double, 4> secants{};
    std::array<double, 5> derivatives{};
    std::array<double, 5> accelerations{};
    std::array<std::string, 3> branches{};
  };

  struct RootSolve
  {
    p3_analytic_solver::PolynomialRoots algebraic;
    std::vector<double> finite;
    std::vector<double> branch;
    std::vector<double> bounded;
    std::vector<double> accepted;
  };

  struct SideResult
  {
    bool domain_valid{false};
    bool go_left{false};
    P3ShadowSideDomain domain;
    Corridor corridor;
    Probe probe;
    std::size_t raw_root_count{0U};
    std::size_t finite_root_count{0U};
    std::size_t branch_root_count{0U};
    std::size_t bounded_root_count{0U};
    std::size_t accepted_root_count{0U};
    std::size_t validator_calls{0U};
    std::size_t hard_valid_count{0U};
    std::optional<std::size_t> best_index;
    std::string failure{"NO_ALGEBRAIC_ROOT"};
    double runtime_corridor_us{0.0};
    double runtime_root_solver_us{0.0};
    double runtime_reconstruction_us{0.0};
    double runtime_hard_validation_us{0.0};
    std::vector<P3ShadowCandidateTrace> candidates;
  };

  struct BranchRange
  {
    double lower{0.0};
    double upper{0.0};
    std::string id;
  };

  struct InactiveSolve
  {
    std::size_t raw_roots{0U};
    std::size_t finite_roots{0U};
    std::size_t branch_roots{0U};
    std::size_t bounded_roots{0U};
    std::vector<double> accepted;
  };

  struct M1Context
  {
    P3ShadowSideDomain domain;
    bool outside_is_left{false};
    Corridor corridor;
    BranchRange component;
    std::size_t component_index{0U};
    double boundary_inset{0.0};
    double branch_near{0.0};
    double branch_far{0.0};
    double entry_min{0.0};
    double entry_max{0.0};
    double entry_quarter{0.0};
    double exit_min{0.0};
    double exit_max{0.0};
    double exit_span{0.0};
  };

  struct ExtensionOutcome
  {
    std::vector<P3ShadowCandidateTrace> candidates;
    std::optional<std::size_t> best_index;
    std::size_t validator_calls{0U};
    std::size_t hard_valid_count{0U};
    std::size_t raw_root_count{0U};
    std::size_t finite_root_count{0U};
    std::size_t branch_root_count{0U};
    std::size_t bounded_root_count{0U};
    std::size_t accepted_root_count{0U};
    double runtime_root_solver_us{0.0};
    double runtime_reconstruction_us{0.0};
    double runtime_hard_validation_us{0.0};
  };

  static double elapsedUs(const Clock::time_point & start)
  {
    return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
  }

  static std::uint64_t fnvAppend(std::uint64_t hash, const void * data, std::size_t size)
  {
    const auto * bytes = static_cast<const unsigned char *>(data);
    for (std::size_t index = 0U; index < size; ++index) {
      hash ^= static_cast<std::uint64_t>(bytes[index]);
      hash *= 1099511628211ULL;
    }
    return hash;
  }

  static std::string hexHash(std::uint64_t hash)
  {
    std::ostringstream output;
    output << std::hex << std::setfill('0') << std::setw(16) << hash;
    return output.str();
  }

  static std::string fnvHex(const std::string & value)
  {
    return hexHash(fnvAppend(1469598103934665603ULL, value.data(), value.size()));
  }

  static std::string pathDigest(const f110_msgs::msg::WpntArray & path)
  {
    std::uint64_t hash = 1469598103934665603ULL;
    const std::uint64_t count = path.wpnts.size();
    hash = fnvAppend(hash, &count, sizeof(count));
    for (const auto & waypoint : path.wpnts) {
      for (const double value : std::array<double, 8>{
          waypoint.s_m, waypoint.d_m, waypoint.x_m, waypoint.y_m,
          waypoint.psi_rad, waypoint.kappa_radpm, waypoint.vx_mps, waypoint.ax_mps2})
      {
        std::uint64_t bits = 0U;
        static_assert(sizeof(bits) == sizeof(value));
        std::memcpy(&bits, &value, sizeof(value));
        hash = fnvAppend(hash, &bits, sizeof(bits));
      }
    }
    return hexHash(hash);
  }

  static std::array<double, 5> stationsFor(
    const RacelineSplineParameters & parameters,
    const P3ShadowSideDomain & domain,
    bool outside_is_left,
    double entry,
    double exit)
  {
    const double entry_length = domain.cluster_start *
      parameters.pre_apex_distances_m.front() * entry / parameters.detection_lookahead_m;
    const double outside_multiplier = domain.go_left == outside_is_left ?
      parameters.outside_line_transition_scale : 1.0;
    const double exit_length = parameters.post_apex_distances_m.back() *
      parameters.cappedCombinedExitScale(exit * outside_multiplier);
    return {
      domain.cluster_start - entry_length,
      domain.cluster_start,
      0.5 * (domain.cluster_start + domain.cluster_end),
      domain.cluster_end,
      domain.cluster_end + exit_length};
  }

  static std::vector<double> uniqueSorted(std::vector<double> values)
  {
    std::sort(values.begin(), values.end());
    values.erase(
      std::unique(values.begin(), values.end(), [](double first, double second) {
        return std::abs(first - second) <= 1.0e-12;
      }), values.end());
    return values;
  }

  static std::vector<Interval> subtractInterval(
    const std::vector<Interval> & source, const Interval & forbidden)
  {
    std::vector<Interval> result;
    for (const auto & interval : source) {
      if (forbidden.upper <= interval.lower + kEpsilon ||
        forbidden.lower >= interval.upper - kEpsilon)
      {
        result.push_back(interval);
        continue;
      }
      if (forbidden.lower > interval.lower + kEpsilon) {
        result.push_back({interval.lower, std::min(interval.upper, forbidden.lower)});
      }
      if (forbidden.upper < interval.upper - kEpsilon) {
        result.push_back({std::max(interval.lower, forbidden.upper), interval.upper});
      }
    }
    return result;
  }

  static bool overlaps(const Interval & first, const Interval & second)
  {
    return std::min(first.upper, second.upper) >=
           std::max(first.lower, second.lower) - kEpsilon;
  }

  static QuinticSegment quinticHermite(
    double start, double end,
    double d0, double v0, double a0,
    double d1, double v1, double a1)
  {
    const double h = end - start;
    if (!(h > kEpsilon)) {
      throw std::runtime_error("non-positive quintic-Hermite segment");
    }
    QuinticSegment segment;
    segment.start = start;
    segment.end = end;
    auto & c = segment.coefficients;
    c[0] = d0;
    c[1] = h * v0;
    c[2] = 0.5 * h * h * a0;
    const double r0 = d1 - c[0] - c[1] - c[2];
    const double r1 = h * v1 - c[1] - 2.0 * c[2];
    const double r2 = h * h * a1 - 2.0 * c[2];
    c[3] = 10.0 * r0 - 4.0 * r1 + 0.5 * r2;
    c[4] = -15.0 * r0 + 7.0 * r1 - r2;
    c[5] = 6.0 * r0 - 3.0 * r1 + 0.5 * r2;
    return segment;
  }

  static std::vector<QuinticSegment> makeC2Profile(
    const std::array<double, 5> & stations,
    const std::array<double, 5> & offsets)
  {
    std::array<double, 5> derivative{};
    std::array<double, 5> acceleration{};
    for (std::size_t index = 1U; index + 1U < stations.size(); ++index) {
      const double h_previous = stations[index] - stations[index - 1U];
      const double h_next = stations[index + 1U] - stations[index];
      const double slope_previous = (offsets[index] - offsets[index - 1U]) / h_previous;
      const double slope_next = (offsets[index + 1U] - offsets[index]) / h_next;
      derivative[index] = p3_analytic_solver::sourceHarmonicDerivative(
        h_previous, h_next, slope_previous, slope_next);
      acceleration[index] = 2.0 * (slope_next - slope_previous) /
        (h_previous + h_next);
    }
    std::vector<QuinticSegment> segments;
    segments.reserve(4U);
    for (std::size_t index = 0U; index + 1U < stations.size(); ++index) {
      segments.push_back(quinticHermite(
          stations[index], stations[index + 1U], offsets[index], derivative[index],
          acceleration[index], offsets[index + 1U], derivative[index + 1U],
          acceleration[index + 1U]));
    }
    return segments;
  }

  static double evaluateProfile(
    const std::vector<QuinticSegment> & segments, double station, double ego_d)
  {
    if (station <= segments.front().start) {
      return ego_d;
    }
    for (const auto & segment : segments) {
      if (station <= segment.end) {
        return segment.evaluate(station);
      }
    }
    return 0.0;
  }

  static KnotStates sourceRuleStates(
    const std::array<double, 5> & stations,
    const std::array<double, 5> & offsets)
  {
    KnotStates result;
    for (std::size_t index = 0U; index < 4U; ++index) {
      result.secants[index] = (offsets[index + 1U] - offsets[index]) /
        (stations[index + 1U] - stations[index]);
    }
    for (std::size_t index = 1U; index < 4U; ++index) {
      const double h_left = stations[index] - stations[index - 1U];
      const double h_right = stations[index + 1U] - stations[index];
      const double left = result.secants[index - 1U];
      const double right = result.secants[index];
      result.branches[index - 1U] = p3_analytic_solver::sourceBranch(left, right);
      result.derivatives[index] = p3_analytic_solver::sourceHarmonicDerivative(
        h_left, h_right, left, right);
      result.accelerations[index] = 2.0 * (right - left) / (h_left + h_right);
    }
    return result;
  }

  static std::string sourceBranchRegime(
    const std::array<double, 5> & stations, double ego_d, double target, double middle)
  {
    const auto states = sourceRuleStates(stations, {ego_d, target, middle, target, 0.0});
    return states.branches[0] + "__" + states.branches[1] + "__" + states.branches[2];
  }

  static bool strictPositiveSegments(
    const std::array<double, 5> & stations, double & minimum_length)
  {
    minimum_length = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index + 1U < stations.size(); ++index) {
      const double length = stations[index + 1U] - stations[index];
      minimum_length = std::min(minimum_length, length);
      if (!std::isfinite(length) || !(length > kEpsilon)) {
        return false;
      }
    }
    return true;
  }

  static double segmentD1(const QuinticSegment & segment, double t)
  {
    const auto & c = segment.coefficients;
    const double h = segment.end - segment.start;
    return (c[1] + 2.0 * c[2] * t + 3.0 * c[3] * t * t +
           4.0 * c[4] * t * t * t + 5.0 * c[5] * t * t * t * t) / h;
  }

  static double linearSampleWithoutSideDerivative(
    const std::array<double, 5> & stations, double ego_d, double target,
    double middle, std::size_t segment, double normalized_t)
  {
    const std::array<double, 5> offsets{ego_d, target, middle, target, 0.0};
    KnotStates states = sourceRuleStates(stations, offsets);
    states.derivatives.fill(0.0);
    std::vector<QuinticSegment> segments;
    for (std::size_t index = 0U; index < 4U; ++index) {
      segments.push_back(quinticHermite(
          stations[index], stations[index + 1U], offsets[index], states.derivatives[index],
          states.accelerations[index], offsets[index + 1U], states.derivatives[index + 1U],
          states.accelerations[index + 1U]));
    }
    const double station = stations[segment] + normalized_t *
      (stations[segment + 1U] - stations[segment]);
    return segments[segment].evaluate(station);
  }

  static double sideDerivativeWeight(
    const std::array<double, 5> & stations, std::size_t segment, double t)
  {
    const double h = stations[segment + 1U] - stations[segment];
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    if (segment == 0U || segment == 2U) {
      return h * (-4.0 * t3 + 7.0 * t4 - 3.0 * t5);
    }
    return h * (t - 6.0 * t3 + 8.0 * t4 - 3.0 * t5);
  }

  static bool branchCombinationMatches(
    const std::array<double, 5> & stations, double ego_d, double target,
    double middle, const std::array<std::string, 3> & assumed)
  {
    return sourceRuleStates(
      stations, {ego_d, target, middle, target, 0.0}).branches == assumed;
  }

  static p3_analytic_solver::ActiveBranch activeBranch(const std::string & name)
  {
    return name == "SAME_SIGN_POSITIVE" ?
           p3_analytic_solver::ActiveBranch::SameSignPositive :
           p3_analytic_solver::ActiveBranch::SameSignNegative;
  }

  Corridor makeCorridor(
    const EgoFrenetState & ego,
    const std::vector<P3ShadowObstacleEnvelope> & visible,
    double cluster_start, double cluster_end, bool go_left, bool outside_is_left) const
  {
    Corridor result;
    std::vector<double> stations{
      0.0, cluster_start, cluster_end, 0.5 * (cluster_start + cluster_end)};
    double horizon = cluster_end + parameters_.post_apex_distances_m.back() *
      parameters_.cappedCombinedExitScale(
      *std::max_element(
        parameters_.transition_distance_scales.begin(),
        parameters_.transition_distance_scales.end()) *
      (go_left == outside_is_left ?
      parameters_.outside_line_transition_scale : 1.0));
    for (const auto & obstacle : visible) {
      if (obstacle.end < -kEpsilon || obstacle.start > horizon + kEpsilon) {
        continue;
      }
      stations.push_back(std::max(0.0, obstacle.start));
      stations.push_back(std::clamp(obstacle.center, 0.0, horizon));
      stations.push_back(std::min(horizon, obstacle.end));
    }
    const std::size_t first = planner_.nextReferenceIndex(ego.s);
    for (std::size_t count = 0U; count < planner_.reference_.wpnts.size(); ++count) {
      const auto & waypoint =
        planner_.reference_.wpnts[(first + count) % planner_.reference_.wpnts.size()];
      const double station = planner_.forwardDistance(ego.s, waypoint.s_m);
      if (station > horizon + kEpsilon) {
        break;
      }
      stations.push_back(station);
    }
    stations = uniqueSorted(std::move(stations));

    const double projection = parameters_.vehicle_half_width_m +
      parameters_.wall_safety_margin_m;
    result.samples.reserve(stations.size());
    for (const double station : stations) {
      const auto & reference = planner_.reference_.wpnts[
        planner_.nearestReferenceIndex(planner_.wrapS(ego.s + station))];
      const double left = reference.d_left > 0.05 ?
        reference.d_left : parameters_.fallback_track_half_width_m;
      const double right = reference.d_right > 0.05 ?
        reference.d_right : parameters_.fallback_track_half_width_m;
      CorridorSample sample;
      sample.station = station;
      sample.curvature = reference.kappa_radpm;
      sample.track = {-right + projection, left - projection};
      if (sample.track.upper > sample.track.lower + kEpsilon) {
        sample.feasible.push_back(sample.track);
      }
      for (const auto & obstacle : visible) {
        if (station + kEpsilon < obstacle.start || station - kEpsilon > obstacle.end) {
          continue;
        }
        sample.active_obstacles.push_back(obstacle.id);
        sample.feasible = subtractInterval(
          sample.feasible, {obstacle.d_right, obstacle.d_left});
      }
      result.samples.push_back(std::move(sample));
    }

    result.branch_count = 1U;
    result.connected = true;
    Interval previous{};
    bool have_previous = false;
    for (const auto & sample : result.samples) {
      result.branch_count = std::max(result.branch_count, sample.feasible.size());
      if (sample.feasible.empty()) {
        result.connected = false;
        continue;
      }
      const Interval selected = go_left ? sample.feasible.back() : sample.feasible.front();
      if (have_previous && !overlaps(previous, selected)) {
        result.connected = false;
      }
      previous = selected;
      have_previous = true;
      result.branch.push_back({
          sample.station, selected.lower, selected.upper,
          0.5 * (selected.lower + selected.upper), selected.upper - selected.lower,
          sample.curvature});
    }
    std::ostringstream digest;
    digest << std::setprecision(17) << (go_left ? "L" : "R");
    for (const auto & sample : result.branch) {
      digest << ':' << sample.station << ',' << sample.lower << ',' << sample.upper;
    }
    result.branch_id = fnvHex(digest.str());
    return result;
  }

  static const BranchSample & nearestBranchSample(
    const Corridor & corridor, double station)
  {
    return *std::min_element(
      corridor.branch.begin(), corridor.branch.end(),
      [station](const auto & first, const auto & second) {
        return std::abs(first.station - station) < std::abs(second.station - station);
      });
  }

  Probe chooseProbe(
    const Corridor & corridor, double cluster_start, double cluster_end,
    double target, const std::string & policy = "CURVATURE_CONTINUITY") const
  {
    std::vector<const BranchSample *> span;
    for (const auto & sample : corridor.branch) {
      if (sample.station > cluster_start + kEpsilon &&
        sample.station < cluster_end - kEpsilon)
      {
        span.push_back(&sample);
      }
    }
    if (span.empty()) {
      span.push_back(&nearestBranchSample(corridor, 0.5 * (cluster_start + cluster_end)));
    }
    const BranchSample * selected = span.front();
    Probe probe;
    if (policy == "BOTTLENECK_CENTER") {
      selected = *std::min_element(span.begin(), span.end(),
          [](const auto * first, const auto * second) {
            return std::tie(first->width, first->station) <
                   std::tie(second->width, second->station);
          });
      probe.location_rule = "CORRIDOR_BOTTLENECK";
      probe.anchor_rule = "CORRIDOR_CENTER";
    } else if (policy == "MAX_DISPLACEMENT_CENTER") {
      selected = *std::max_element(
        span.begin(), span.end(), [target](const auto * first, const auto * second) {
          return std::make_tuple(std::abs(first->center - target), first->station) <
                 std::make_tuple(std::abs(second->center - target), second->station);
        });
      probe.location_rule = "MAX_CENTER_DISPLACEMENT";
      probe.anchor_rule = "MAX_CLEARANCE_ANCHOR";
    } else if (policy == "CURVATURE_CONTINUITY") {
      selected = *std::max_element(
        span.begin(), span.end(), [](const auto * first, const auto * second) {
          return std::make_tuple(std::abs(first->curvature), first->station) <
                 std::make_tuple(std::abs(second->curvature), second->station);
        });
      probe.location_rule = "CURVATURE_CORRIDOR_CRITICAL";
      probe.anchor_rule = "CONTINUITY_BIASED_SAFE_ANCHOR";
    } else {
      throw std::runtime_error("unknown P3 mapping policy");
    }
    probe.station = selected->station;
    if (policy == "CURVATURE_CONTINUITY") {
      const double lower = selected->lower + parameters_.safety_margin_m;
      const double upper = selected->upper - parameters_.safety_margin_m;
      probe.desired = lower <= upper ? std::clamp(target, lower, upper) : selected->center;
    } else {
      probe.desired = selected->center;
    }
    return probe;
  }

  double existingTargetAnchor(double minimum_target, double maximum_target) const
  {
    const int count = std::max(1, parameters_.target_d_candidate_count);
    const double low = std::min(minimum_target, maximum_target);
    const double high = std::max(minimum_target, maximum_target);
    std::vector<double> candidates;
    for (int index = 0; index < count; ++index) {
      const double ratio = count == 1 ? 0.0 :
        static_cast<double>(index) / static_cast<double>(count - 1);
      candidates.push_back(low + ratio * (high - low));
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](double first, double second) {
        return std::abs(first) < std::abs(second);
      });
    return candidates.front();
  }

  RootSolve solvePosition(
    const std::array<double, 5> & stations, double ego_d, double target,
    double station, double desired, double lower, double upper) const
  {
    RootSolve result;
    std::size_t segment = 0U;
    while (segment + 1U < 4U && station > stations[segment + 1U]) {
      ++segment;
    }
    const double h = stations[segment + 1U] - stations[segment];
    const double normalized_t = std::clamp(
      (station - stations[segment]) / h, 0.0, 1.0);
    const bool left = target > 0.0;
    const std::array<std::string, 3> assumed{
      left ? "SAME_SIGN_POSITIVE" : "SAME_SIGN_NEGATIVE",
      "SIGN_CHANGE",
      left ? "SAME_SIGN_NEGATIVE" : "SAME_SIGN_POSITIVE"};
    const bool entry = segment <= 1U;
    const auto active = activeBranch(entry ? assumed[0] : assumed[2]);
    const auto map = entry ? p3_analytic_solver::makeEntryMap(
      stations[1] - stations[0], stations[2] - stations[1], ego_d, target, active) :
      p3_analytic_solver::makeExitMap(
      stations[3] - stations[2], stations[4] - stations[3], target, 0.0, active);
    const double intercept = linearSampleWithoutSideDerivative(
      stations, ego_d, target, 0.0, segment, normalized_t);
    const double slope = linearSampleWithoutSideDerivative(
      stations, ego_d, target, 1.0, segment, normalized_t) - intercept;
    const double weight = sideDerivativeWeight(stations, segment, normalized_t);
    const auto polynomial = p3_analytic_solver::samplePolynomial(
      slope, intercept, weight, map, desired);
    result.algebraic = p3_analytic_solver::solvePolynomialStable(polynomial);
    for (const double root : result.algebraic.raw_roots) {
      if (!std::isfinite(root)) {
        continue;
      }
      result.finite.push_back(root);
      if (!branchCombinationMatches(stations, ego_d, target, root, assumed)) {
        continue;
      }
      result.branch.push_back(root);
      if (root < lower - kEpsilon || root > upper + kEpsilon) {
        continue;
      }
      result.bounded.push_back(root);
      const auto segments = makeC2Profile(
        stations, {ego_d, target, root, target, 0.0});
      const double residual = evaluateProfile(segments, station, ego_d) - desired;
      if (std::abs(residual) <= 1.0e-12 * std::max(1.0, std::abs(desired))) {
        result.accepted.push_back(root);
      }
    }
    p3_analytic_solver::sortAndDeduplicate(result.finite);
    p3_analytic_solver::sortAndDeduplicate(result.branch);
    p3_analytic_solver::sortAndDeduplicate(result.bounded);
    p3_analytic_solver::sortAndDeduplicate(result.accepted);
    return result;
  }

  double peakSlope(
    const EgoFrenetState & ego, const f110_msgs::msg::WpntArray & path) const
  {
    double peak = 0.0;
    if (path.wpnts.size() < 2U) {
      return peak;
    }
    double previous_forward = planner_.forwardDistance(ego.s, path.wpnts.front().s_m);
    double previous_d = path.wpnts.front().d_m;
    for (std::size_t index = 1U; index < path.wpnts.size(); ++index) {
      const double forward = planner_.forwardDistance(ego.s, path.wpnts[index].s_m);
      const double ds = forward - previous_forward;
      if (ds > kEpsilon) {
        peak = std::max(peak, std::abs(path.wpnts[index].d_m - previous_d) / ds);
      }
      previous_forward = forward;
      previous_d = path.wpnts[index].d_m;
    }
    return peak;
  }

  P3ShadowCandidateTrace buildCandidate(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    bool go_left, bool outside_is_left, double target, double middle,
    double entry_scale, double exit_scale, const std::array<double, 5> & stations,
    std::size_t generation, double & reconstruction_us, double & hard_validation_us) const
  {
    const auto reconstruction_start = Clock::now();
    const auto segments = makeC2Profile(
      stations, {ego.d, target, middle, target, 0.0});
    (void)outside_is_left;
    const double tail = std::max(
      parameters_.post_merge_lookahead_m,
      std::abs(ego.speed) * parameters_.post_merge_min_time_sec);
    const double path_end = stations.back() + tail;
    const std::size_t first = planner_.nextReferenceIndex(ego.s);
    f110_msgs::msg::WpntArray path;
    path.header = planner_.reference_.header;
    for (std::size_t count = 0U; count < planner_.reference_.wpnts.size(); ++count) {
      const auto & global =
        planner_.reference_.wpnts[(first + count) % planner_.reference_.wpnts.size()];
      const double forward = planner_.forwardDistance(ego.s, global.s_m);
      if (forward > path_end + kEpsilon) {
        break;
      }
      auto waypoint = global;
      waypoint.id = static_cast<std::int32_t>(path.wpnts.size());
      waypoint.d_m = evaluateProfile(segments, forward, ego.d);
      waypoint.x_m = global.x_m - waypoint.d_m * std::sin(global.psi_rad);
      waypoint.y_m = global.y_m + waypoint.d_m * std::cos(global.psi_rad);
      path.wpnts.push_back(waypoint);
    }
    if (path.wpnts.size() >= static_cast<std::size_t>(parameters_.minimum_path_points)) {
      planner_.finalizeP3ShadowPath(path, ego, obstacles);
    }
    reconstruction_us += elapsedUs(reconstruction_start);

    P3ShadowPathEvaluation evaluation;
    if (path.wpnts.size() >= static_cast<std::size_t>(parameters_.minimum_path_points)) {
      const auto validation_start = Clock::now();
      // 후보 검증의 장애물 범위는 이 기동이 책임지는 구간(클러스터 끝 + post_merge_lookahead)
      // 까지다. `generateP3Candidates`가 P0 선택 경로에서 이미 같은 horizon으로 재검증하고
      // 있었는데(2026-08-15 run18: horizon 없이는 12 m 밖 꼬리 충돌로 앞 장애물의 회피
      // 후보가 전멸 → kNoSafePath → 영구 크립), 정작 P3 자신의 후보 인증서는 horizon 없이
      // 만들어져 두 경로의 판정이 갈렸다. 2026-08-16 백에서 P3는 s=31.7 장애물에 대해
      // 3 m 이내 245 콜백 전부 NO_HARD_VALID_M1_CANDIDATE였고, 같은 순간 P0의 plan()은
      // 6개 중 3개를 feasible로 통과시켰다. 트랙 경계·기하 검사는 여전히 경로 전체다.
      const std::optional<double> collision_horizon(
        stations[3] + parameters_.post_merge_lookahead_m);
      evaluation = planner_.validateP3ShadowPath(ego, path, obstacles, 1.0, collision_horizon);
      hard_validation_us += elapsedUs(validation_start);
    } else {
      evaluation.rejection_reason = "spline segment has too few global race-line samples";
    }

    P3ShadowCandidateTrace trace;
    trace.generation_index = generation;
    trace.go_left = go_left;
    trace.entry_scale = entry_scale;
    trace.exit_scale = exit_scale;
    trace.d_target = target;
    trace.d_mid = middle;
    trace.hard_valid = evaluation.hard_valid;
    trace.minimum_normalized_safety_slack = evaluation.minimum_normalized_safety_slack;
    trace.minimum_track_margin_m = evaluation.minimum_track_margin_m;
    trace.minimum_obstacle_margin_m = evaluation.minimum_obstacle_margin_m;
    trace.peak_curvature_radpm = evaluation.peak_curvature_radpm;
    trace.peak_curvature_rate_radpm2 = evaluation.peak_curvature_rate_radpm2;
    trace.peak_lateral_slope = peakSlope(ego, path);
    trace.velocity_loss = evaluation.velocity_loss;
    trace.global_path_deviation_m = evaluation.global_path_deviation_m;
    trace.minimum_commanded_speed_mps = std::numeric_limits<double>::infinity();
    trace.maximum_commanded_speed_mps = 0.0;
    for (const auto & waypoint : path.wpnts) {
      trace.minimum_commanded_speed_mps = std::min(
        trace.minimum_commanded_speed_mps, waypoint.vx_mps);
      trace.maximum_commanded_speed_mps = std::max(
        trace.maximum_commanded_speed_mps, waypoint.vx_mps);
    }
    trace.rejection_reason = evaluation.rejection_reason;
    trace.exit_reaches_next_obstacle = exitReachesNextObstacle(
      ego, path, obstacles, stations[3], stations[4]);
    trace.validation = evaluation;
    trace.path_digest = pathDigest(path);
    trace.source_branch_regime = sourceBranchRegime(stations, ego.d, target, middle);
    trace.path = std::move(path);
    return trace;
  }

  // 클러스터를 지난 뒤(exit 램프 + merge 뒤 꼬리)에도 오프셋이 남아 다음 장애물의 물리
  // 엔벨로프에 닿는가. 닿는다고 후보를 버리지는 않는다 — 장애물 간격이 좁으면 오프셋을
  // 그대로 넘겨주는 것이 설계된 동작이고(AGENTS의 maximum_exit_length 비활성 사유), 여기서
  // 거부하면 2026-08-12/08-15의 "후보 전멸 → 영구 크립" 회귀가 그대로 돌아온다. 대신
  // 순위에서만 뒤로 민다: 다음 장애물을 건드리지 않는 exit이 하나라도 있으면 그쪽을 쓴다.
  // 검사 구간은 **exit 램프뿐**이다: 클러스터 끝 이후 ~ merge 지점까지. merge 뒤 꼬리는
  // 정의상 d=0인 글로벌 라인이라, 다음 장애물이 라인 위에 있으면(이번 백의 s=40.6이 정확히
  // 그렇다) 모든 후보가 무조건 참이 되어 이 우선순위 자체가 무력해진다. 꼬리가 장애물을
  // 지나가는 것은 이 기동의 문제가 아니라 연쇄 기동이 교체할 몫이다.
  bool exitReachesNextObstacle(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    double cluster_end,
    double merge_station) const
  {
    if (!(merge_station > cluster_end + kEpsilon)) {
      return false;
    }
    const double clearance = parameters_.obstacleBaseClearance();
    for (const auto & obstacle : obstacles) {
      const double start = planner_.forwardDistance(ego.s, obstacle.s_start);
      const double end = planner_.forwardDistance(ego.s, obstacle.s_end);
      if (!(start > cluster_end + kEpsilon) || end < start || start > merge_station + kEpsilon) {
        continue;   // 이 기동이 피하는 클러스터이거나, 뒤에 있거나, exit 구간 밖이다.
      }
      for (const auto & waypoint : path.wpnts) {
        const double forward = planner_.forwardDistance(ego.s, waypoint.s_m);
        if (forward + kEpsilon < start || forward > end + kEpsilon ||
          forward > merge_station + kEpsilon)
        {
          continue;
        }
        if (waypoint.d_m > obstacle.d_right - clearance - kEpsilon &&
          waypoint.d_m < obstacle.d_left + clearance + kEpsilon)
        {
          return true;
        }
      }
    }
    return false;
  }

  static bool betterFeasible(
    const P3ShadowCandidateTrace & first, const P3ShadowCandidateTrace & second)
  {
    // 다음 장애물을 건드리지 않는 exit이 항상 우선한다. 이 우선순위가 없으면 safety slack
    // 최대 기준이 가장 긴 exit(스케일 3.699 → 최대 22.9 m)을 고르는데, 그 램프는 8~12 m 뒤
    // 장애물 위를 오프셋을 유지한 채 지나가 커밋 재검증과 계속 충돌한다 (2026-08-16 백:
    // s=31.7 기동의 exit이 s=40.6 장애물을 d=-0.265로 관통 → 랩당 hard collision 41회,
    // 25 ms마다 같은 후보를 재선택하는 무한 재계획).
    if (first.exit_reaches_next_obstacle != second.exit_reaches_next_obstacle) {
      return second.exit_reaches_next_obstacle;
    }
    const double slack_delta = first.minimum_normalized_safety_slack -
      second.minimum_normalized_safety_slack;
    if (std::abs(slack_delta) > kEpsilon) {
      return slack_delta > 0.0;
    }
    const double velocity_delta = first.velocity_loss - second.velocity_loss;
    if (std::abs(velocity_delta) > kEpsilon) {
      return velocity_delta < 0.0;
    }
    const double deviation_delta = first.global_path_deviation_m -
      second.global_path_deviation_m;
    if (std::abs(deviation_delta) > kEpsilon) {
      return deviation_delta < 0.0;
    }
    return first.generation_index < second.generation_index;
  }

  std::vector<double> entryScales(bool extended = true) const
  {
    auto values = parameters_.entry_transition_fractions;
    if (extended) {
      values.push_back(
        parameters_.detection_lookahead_m / parameters_.pre_apex_distances_m.front());
    }
    return uniqueSorted(std::move(values));
  }

  std::vector<double> exitScales(
    const EgoFrenetState & ego, double cluster_end,
    bool go_left, bool outside_is_left, bool extended = true) const
  {
    auto values = parameters_.transition_distance_scales;
    if (!extended || values.empty()) {
      return uniqueSorted(std::move(values));
    }
    const double outside_multiplier = go_left == outside_is_left ?
      parameters_.outside_line_transition_scale : 1.0;
    const double nominal_post_far = parameters_.post_apex_distances_m.back();
    const double current_max_raw = *std::max_element(values.begin(), values.end());
    const double current_effective_length =
      nominal_post_far * current_max_raw * outside_multiplier;
    const double tail = std::max(
      parameters_.post_merge_lookahead_m,
      std::abs(ego.speed) * parameters_.post_merge_min_time_sec);
    const double horizon_limit = std::max(
      0.0, planner_.trackLength() - cluster_end - tail - 1.0e-3);
    for (const double factor : {1.5, 2.0}) {
      const double requested = current_effective_length * factor;
      const double effective = std::min(requested, horizon_limit);
      if (effective > current_effective_length + kEpsilon &&
        nominal_post_far * outside_multiplier > kEpsilon)
      {
        values.push_back(effective / (nominal_post_far * outside_multiplier));
      }
    }
    return uniqueSorted(std::move(values));
  }

  static std::pair<double, double> nearFar(double first, double second)
  {
    const auto key = [](double value) {return std::make_pair(std::abs(value), value);};
    return key(first) <= key(second) ? std::make_pair(first, second) :
           std::make_pair(second, first);
  }

  static std::vector<BranchRange> connectedConstantRanges(
    const Corridor & corridor, const P3ShadowSideDomain & domain)
  {
    std::vector<Interval> active;
    bool initialized = false;
    for (const auto & sample : corridor.samples) {
      if (sample.station < domain.cluster_start - kEpsilon ||
        sample.station > domain.cluster_end + kEpsilon)
      {
        continue;
      }
      if (!initialized) {
        active = sample.feasible;
        initialized = true;
        continue;
      }
      std::vector<Interval> next;
      for (const auto & previous : active) {
        for (const auto & current : sample.feasible) {
          const Interval intersection{
            std::max(previous.lower, current.lower),
            std::min(previous.upper, current.upper)};
          if (intersection.upper >= intersection.lower - kEpsilon) {
            next.push_back(intersection);
          }
        }
      }
      std::sort(next.begin(), next.end(), [](const auto & first, const auto & second) {
          return std::tie(first.lower, first.upper) < std::tie(second.lower, second.upper);
        });
      next.erase(
        std::unique(next.begin(), next.end(), [](const auto & first, const auto & second) {
          return std::abs(first.lower - second.lower) <= 1.0e-12 &&
                 std::abs(first.upper - second.upper) <= 1.0e-12;
        }), next.end());
      active = std::move(next);
      if (active.empty()) {
        break;
      }
    }

    const double domain_low = std::min(domain.minimum_target, domain.maximum_target);
    const double domain_high = std::max(domain.minimum_target, domain.maximum_target);
    std::vector<BranchRange> ranges;
    for (const auto & interval : active) {
      const double lower = std::max(interval.lower, domain_low);
      const double upper = std::min(interval.upper, domain_high);
      if (upper < lower - kEpsilon) {
        continue;
      }
      std::ostringstream identity;
      identity << std::setprecision(17) << (domain.go_left ? "LEFT" : "RIGHT") << ':' <<
        lower << ':' << upper;
      ranges.push_back({lower, upper, fnvHex(identity.str())});
    }
    if (ranges.empty()) {
      std::ostringstream identity;
      identity << std::setprecision(17) <<
        (domain.go_left ? "LEFT_DOMAIN" : "RIGHT_DOMAIN") << ':' <<
        domain_low << ':' << domain_high;
      ranges.push_back({domain_low, domain_high, fnvHex(identity.str())});
    }
    std::stable_sort(ranges.begin(), ranges.end(), [](const auto & first, const auto & second) {
        const double first_width = first.upper - first.lower;
        const double second_width = second.upper - second.lower;
        if (std::abs(first_width - second_width) > 1.0e-12) {
          return first_width > second_width;
        }
        return std::tie(first.lower, first.upper) < std::tie(second.lower, second.upper);
      });
    if (ranges.size() > 2U) {
      ranges.resize(2U);
    }
    return ranges;
  }

  static std::vector<double> rankedTargets(const BranchRange & branch)
  {
    std::vector<double> targets;
    for (const double ratio : {0.0, 0.25, 0.5, 0.75, 1.0}) {
      targets.push_back(branch.lower + ratio * (branch.upper - branch.lower));
    }
    targets = uniqueSorted(std::move(targets));
    std::stable_sort(targets.begin(), targets.end(), [](double first, double second) {
        if (std::abs(std::abs(first) - std::abs(second)) > 1.0e-12) {
          return std::abs(first) < std::abs(second);
        }
        return first < second;
      });
    return targets;
  }

  InactiveSolve solveAllInactive(
    const std::array<double, 5> & stations, double ego_d, double target,
    double station, double desired, double lower, double upper) const
  {
    InactiveSolve result;
    std::size_t segment = 0U;
    while (segment + 1U < 4U && station > stations[segment + 1U]) {
      ++segment;
    }
    const double length = stations[segment + 1U] - stations[segment];
    const double normalized = std::clamp(
      (station - stations[segment]) / length, 0.0, 1.0);
    const double intercept = linearSampleWithoutSideDerivative(
      stations, ego_d, target, 0.0, segment, normalized);
    const double slope = linearSampleWithoutSideDerivative(
      stations, ego_d, target, 1.0, segment, normalized) - intercept;
    const double coefficient_scale = std::max({
        1.0, std::abs(intercept), std::abs(slope), std::abs(desired)});
    const double zero_tolerance =
      p3_analytic_solver::kCoefficientRelativeTolerance * coefficient_scale;
    if (std::abs(slope) <= zero_tolerance) {
      return result;
    }
    const double root = (desired - intercept) / slope;
    ++result.raw_roots;
    if (!std::isfinite(root)) {
      return result;
    }
    ++result.finite_roots;
    const auto states = sourceRuleStates(stations, {ego_d, target, root, target, 0.0});
    const bool all_inactive = std::none_of(
      states.branches.begin(), states.branches.end(), [](const std::string & branch) {
        return branch.rfind("SAME_SIGN", 0U) == 0U;
      });
    if (!all_inactive) {
      return result;
    }
    ++result.branch_roots;
    if (root < lower - kEpsilon || root > upper + kEpsilon) {
      return result;
    }
    ++result.bounded_roots;
    const auto segments = makeC2Profile(stations, {ego_d, target, root, target, 0.0});
    const double residual = evaluateProfile(segments, station, ego_d) - desired;
    if (std::abs(residual) <= 1.0e-12 * std::max(1.0, std::abs(desired))) {
      result.accepted.push_back(root);
    }
    return result;
  }

  void addM0ExtensionCandidate(
    ExtensionOutcome & outcome,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const P3ShadowSideDomain & domain,
    bool outside_is_left,
    const BranchRange & component,
    const std::string & candidate_template,
    const std::string & source_cell,
    double target,
    double middle,
    double entry,
    double exit,
    std::size_t & generation,
    std::set<std::tuple<bool, double, double, double, double>> & seen) const
  {
    if (outcome.candidates.size() >= kM0ExtensionCandidateCap) {
      return;
    }
    const auto key = std::make_tuple(domain.go_left, target, middle, entry, exit);
    if (!seen.insert(key).second) {
      return;
    }
    const auto stations = stationsFor(parameters_, domain, outside_is_left, entry, exit);
    ++outcome.validator_calls;
    auto trace = buildCandidate(
      ego, obstacles, domain.go_left, outside_is_left, target, middle, entry, exit,
      stations, generation++, outcome.runtime_reconstruction_us,
      outcome.runtime_hard_validation_us);
    const std::string side = domain.go_left ? "LEFT" : "RIGHT";
    trace.mapping_source = "FROZEN_V2_EXTENSION";
    trace.candidate_template = candidate_template;
    trace.source_cell = source_cell;
    trace.component_id = component.id;
    trace.candidate_identity = "M0_V2_" + side + "_" + candidate_template + "_" +
      trace.path_digest;
    trace.logical_identity = "M0_V2_" + side + "_" + candidate_template;
    const std::size_t index = outcome.candidates.size();
    if (trace.hard_valid) {
      ++outcome.hard_valid_count;
      if (!outcome.best_index.has_value() ||
        betterFeasible(trace, outcome.candidates[*outcome.best_index]))
      {
        outcome.best_index = index;
      }
    }
    outcome.candidates.push_back(std::move(trace));
  }

  void addM0AnalyticCandidate(
    ExtensionOutcome & outcome,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const P3ShadowSideDomain & domain,
    bool outside_is_left,
    const Corridor & corridor,
    const BranchRange & component,
    double target,
    double entry,
    double exit,
    std::size_t & generation,
    std::set<std::tuple<bool, double, double, double, double>> & seen) const
  {
    if (outcome.candidates.size() >= kM0ExtensionCandidateCap || corridor.branch.empty()) {
      return;
    }
    const Probe probe = chooseProbe(
      corridor, domain.cluster_start, domain.cluster_end, target, "BOTTLENECK_CENTER");
    const auto stations = stationsFor(parameters_, domain, outside_is_left, entry, exit);
    if (probe.station < stations.front() - kEpsilon ||
      probe.station > stations.back() + kEpsilon)
    {
      return;
    }
    const auto root_start = Clock::now();
    const RootSolve roots = solvePosition(
      stations, ego.d, target, probe.station, probe.desired,
      component.lower, component.upper);
    outcome.runtime_root_solver_us += elapsedUs(root_start);
    outcome.raw_root_count += roots.algebraic.raw_roots.size();
    outcome.finite_root_count += roots.finite.size();
    outcome.branch_root_count += roots.branch.size();
    outcome.bounded_root_count += roots.bounded.size();
    outcome.accepted_root_count += roots.accepted.size();
    for (const double root : roots.accepted) {
      addM0ExtensionCandidate(
        outcome, ego, obstacles, domain, outside_is_left, component,
        "BRANCH_AWARE_ANALYTIC_ROOT", "ACTIVE_OUTER", target, root, entry, exit,
        generation, seen);
    }
  }

  ExtensionOutcome evaluateM0Extension(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const P3ShadowPlanningContext & context) const
  {
    ExtensionOutcome outcome;
    std::size_t generation = 1000U;
    std::set<std::tuple<bool, double, double, double, double>> seen;
    for (const bool go_left : {false, true}) {
      const auto & domain = go_left ? context.left : context.right;
      if (!domain.valid) {
        continue;
      }
      const Corridor corridor = makeCorridor(
        ego, context.visible, domain.cluster_start, domain.cluster_end,
        go_left, context.outside_is_left);
      const auto components = connectedConstantRanges(corridor, domain);
      const auto entries = entryScales(true);
      const auto exits = exitScales(
        ego, domain.cluster_end, go_left, context.outside_is_left, true);
      if (entries.empty() || exits.empty()) {
        continue;
      }
      const double entry_full = entries.back();
      const double exit_short = exits.front() + 0.015625 * (exits.back() - exits.front());
      const double entry_quarter = entries.front() + 0.25 * (entries.back() - entries.front());
      const double exit_quarter = exits.front() + 0.25 * (exits.back() - exits.front());
      const double exit_long = exits.back();
      for (const auto & component : components) {
        const auto targets = rankedTargets(component);
        if (targets.empty()) {
          continue;
        }
        const BranchRange side_domain{
          std::min(domain.minimum_target, domain.maximum_target),
          std::max(domain.minimum_target, domain.maximum_target), "SIDE_DOMAIN"};
        const auto domain_targets = rankedTargets(side_domain);
        const double near = targets.front();
        const double center = 0.5 * (component.lower + component.upper);
        const double far = targets.back();
        const double quarter = near + 0.25 * (far - near);
        const double domain_near = domain_targets.front();
        const double domain_far = domain_targets.back();
        const double domain_boundary_inset =
          domain_near + 0.015625 * (domain_far - domain_near);
        addM0ExtensionCandidate(
          outcome, ego, obstacles, domain, context.outside_is_left, component,
          "ZERO_INTERFACE_BOUNDARY_INSET", "ZERO_INTERFACE",
          domain_boundary_inset, domain_boundary_inset, entry_full, exit_short,
          generation, seen);
        addM0ExtensionCandidate(
          outcome, ego, obstacles, domain, context.outside_is_left, component,
          "ZERO_INTERFACE_QUARTER_LONG_EXIT", "ZERO_INTERFACE",
          quarter, quarter, entry_full, exit_long, generation, seen);
        addM0ExtensionCandidate(
          outcome, ego, obstacles, domain, context.outside_is_left, component,
          "ZERO_INTERFACE_CENTER_LONG_EXIT", "ZERO_INTERFACE",
          center, center, entry_full, exit_long, generation, seen);
        addM0ExtensionCandidate(
          outcome, ego, obstacles, domain, context.outside_is_left, component,
          "ZERO_INTERFACE_QUARTER_SCALE", "ZERO_INTERFACE",
          quarter, quarter, entry_quarter, exit_quarter, generation, seen);
        addM0AnalyticCandidate(
          outcome, ego, obstacles, domain, context.outside_is_left, corridor,
          component, near, entry_quarter, exit_quarter, generation, seen);
        if (outcome.candidates.size() >= kM0ExtensionCandidateCap) {
          break;
        }
      }
      if (outcome.candidates.size() >= kM0ExtensionCandidateCap) {
        break;
      }
    }
    return outcome;
  }

  std::vector<M1Context> buildM1Contexts(
    const EgoFrenetState & ego,
    const P3ShadowPlanningContext & planning_context,
    P3ShadowResult & result) const
  {
    std::vector<M1Context> contexts;
    for (const bool go_left : {false, true}) {
      const auto & domain = go_left ? planning_context.left : planning_context.right;
      if (!domain.valid) {
        continue;
      }
      const Corridor corridor = makeCorridor(
        ego, planning_context.visible, domain.cluster_start, domain.cluster_end,
        go_left, planning_context.outside_is_left);
      const auto components = connectedConstantRanges(corridor, domain);
      const auto entries = entryScales(true);
      const auto frozen_exits = exitScales(
        ego, domain.cluster_end, go_left, planning_context.outside_is_left, false);
      const auto exits = exitScales(
        ego, domain.cluster_end, go_left, planning_context.outside_is_left, true);
      if (entries.empty() || exits.empty()) {
        continue;
      }
      if (frozen_exits.empty() || exits.back() + 1.0e-12 < frozen_exits.back()) {
        throw std::runtime_error("extended exit interval does not contain frozen exit interval");
      }
      for (const double frozen_exit : frozen_exits) {
        const bool retained = std::any_of(
          exits.begin(), exits.end(), [frozen_exit](double value) {
            return std::abs(value - frozen_exit) <= 1.0e-12;
          });
        if (!retained) {
          throw std::runtime_error("extended exit interval dropped a frozen exit scale");
        }
      }
      const double entry_min = entries.front();
      const double entry_max = entries.back();
      const double exit_min = exits.front();
      const double exit_max = exits.back();
      const double outside_multiplier = go_left == planning_context.outside_is_left ?
        parameters_.outside_line_transition_scale : 1.0;
      const double cluster_width = domain.cluster_end - domain.cluster_start;
      const double exit_span = std::clamp(
        cluster_width / (parameters_.post_apex_distances_m.back() * outside_multiplier),
        exit_min, exit_max);
      const auto domain_ends = nearFar(
        std::min(domain.minimum_target, domain.maximum_target),
        std::max(domain.minimum_target, domain.maximum_target));
      const double boundary_inset =
        domain_ends.first + (domain_ends.second - domain_ends.first) / 64.0;
      for (std::size_t index = 0U; index < components.size(); ++index) {
        const auto component_ends = nearFar(components[index].lower, components[index].upper);
        contexts.push_back({
            domain, planning_context.outside_is_left, corridor, components[index], index,
            boundary_inset, component_ends.first, component_ends.second,
            entry_min, entry_max, entry_min + (entry_max - entry_min) / 4.0,
            exit_min, exit_max, exit_span});
      }
    }
    result.m1_context_count = contexts.size();
    return contexts;
  }

  void addM1Candidate(
    ExtensionOutcome & outcome,
    P3ShadowResult & result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const M1Context & context,
    const std::string & candidate_template,
    const std::string & source_cell,
    std::size_t source_root_index,
    const Probe * probe,
    double target,
    double middle,
    double entry,
    double exit,
    std::set<std::tuple<bool, double, double, double, double>> & seen) const
  {
    if (result.m1_candidate_count >= result.m1_budget) {
      return;
    }
    const auto key = std::make_tuple(
      context.domain.go_left, target, middle, entry, exit);
    if (!seen.insert(key).second) {
      ++result.m1_exact_tuple_duplicate_count;
      return;
    }
    const auto stations = stationsFor(
      parameters_, context.domain, context.outside_is_left, entry, exit);
    double minimum_length = std::numeric_limits<double>::quiet_NaN();
    if (!strictPositiveSegments(stations, minimum_length)) {
      ++result.m1_positive_segment_rejection_count;
      ++result.m1_boundary_handoff_unresolved_count;
      return;
    }

    const std::size_t generation = 2000U + result.m1_candidate_count;
    ++result.m1_candidate_count;
    ++result.m1_validator_call_count;
    ++outcome.validator_calls;
    auto trace = buildCandidate(
      ego, obstacles, context.domain.go_left, context.outside_is_left,
      target, middle, entry, exit, stations, generation,
      outcome.runtime_reconstruction_us, outcome.runtime_hard_validation_us);
    const std::string side = context.domain.go_left ? "LEFT" : "RIGHT";
    std::ostringstream tuple_identity;
    tuple_identity << std::setprecision(17) << context.domain.go_left << ':' <<
      candidate_template << ':' << source_cell << ':' << context.component_index << ':' <<
      source_root_index << ':' << target << ':' << middle << ':' << entry << ':' << exit;
    trace.mapping_source = "M1_BRANCH_COMPLETE_ACTIVE_SET_CLOSURE";
    trace.candidate_template = candidate_template;
    trace.source_cell = source_cell;
    trace.component_id = context.component.id;
    trace.candidate_identity = "M1_" + side + "_" + candidate_template + "_" +
      source_cell + "_" + fnvHex(tuple_identity.str());
    trace.logical_identity = "M1_" + side + "_" + candidate_template + "_" +
      source_cell + "_c" + std::to_string(context.component_index) + "_r" +
      std::to_string(source_root_index);
    (void)probe;
    const std::size_t index = outcome.candidates.size();
    if (trace.hard_valid) {
      ++result.m1_hard_valid_count;
      ++outcome.hard_valid_count;
      if (!outcome.best_index.has_value() ||
        betterFeasible(trace, outcome.candidates[*outcome.best_index]))
      {
        outcome.best_index = index;
      }
    }
    outcome.candidates.push_back(std::move(trace));
  }

  void offerM1AnalyticTemplate(
    ExtensionOutcome & outcome,
    P3ShadowResult & result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const M1Context & context,
    const std::string & candidate_template,
    double target,
    double entry,
    double exit,
    std::set<std::tuple<bool, double, double, double, double>> & seen) const
  {
    if (result.m1_candidate_count >= result.m1_budget) {
      return;
    }
    const Probe probe = chooseProbe(
      context.corridor, context.domain.cluster_start, context.domain.cluster_end,
      target, "BOTTLENECK_CENTER");
    const auto stations = stationsFor(
      parameters_, context.domain, context.outside_is_left, entry, exit);
    double minimum_length = std::numeric_limits<double>::quiet_NaN();
    if (!strictPositiveSegments(stations, minimum_length)) {
      addM1Candidate(
        outcome, result, ego, obstacles, context, candidate_template,
        "ACTIVE_OUTER", 0U, &probe, target, target, entry, exit, seen);
      return;
    }
    if (probe.station < stations.front() - kEpsilon ||
      probe.station > stations.back() + kEpsilon)
    {
      return;
    }

    const auto active_start = Clock::now();
    const RootSolve active = solvePosition(
      stations, ego.d, target, probe.station, probe.desired,
      context.component.lower, context.component.upper);
    outcome.runtime_root_solver_us += elapsedUs(active_start);
    result.m1_active_outer_raw_root_count += active.algebraic.raw_roots.size();
    result.m1_active_outer_accepted_root_count += active.accepted.size();
    for (std::size_t index = 0U; index < active.accepted.size(); ++index) {
      addM1Candidate(
        outcome, result, ego, obstacles, context, candidate_template,
        "ACTIVE_OUTER", index, &probe, target, active.accepted[index], entry, exit, seen);
    }

    const auto inactive_start = Clock::now();
    const InactiveSolve inactive = solveAllInactive(
      stations, ego.d, target, probe.station, probe.desired,
      context.component.lower, context.component.upper);
    outcome.runtime_root_solver_us += elapsedUs(inactive_start);
    result.m1_all_inactive_raw_root_count += inactive.raw_roots;
    result.m1_all_inactive_accepted_root_count += inactive.accepted.size();
    for (std::size_t index = 0U; index < inactive.accepted.size(); ++index) {
      addM1Candidate(
        outcome, result, ego, obstacles, context, candidate_template,
        "ALL_INACTIVE", index, &probe, target, inactive.accepted[index], entry, exit, seen);
    }
  }

  ExtensionOutcome evaluateM1(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const P3ShadowPlanningContext & planning_context,
    P3ShadowResult & result) const
  {
    ExtensionOutcome outcome;
    const auto contexts = buildM1Contexts(ego, planning_context, result);
    std::set<std::tuple<bool, double, double, double, double>> seen;
    for (const auto & context : contexts) {
      addM1Candidate(
        outcome, result, ego, obstacles, context, "ZERO_BOUNDARY_SHORT",
        "ZERO_INTERFACE", 0U, nullptr, context.boundary_inset, context.boundary_inset,
        context.entry_min, context.exit_min, seen);
    }
    for (const auto & context : contexts) {
      addM1Candidate(
        outcome, result, ego, obstacles, context, "ZERO_BOUNDARY_SPAN",
        "ZERO_INTERFACE", 0U, nullptr, context.boundary_inset, context.boundary_inset,
        context.entry_quarter, context.exit_span, seen);
    }
    for (const auto & context : contexts) {
      offerM1AnalyticTemplate(
        outcome, result, ego, obstacles, context, "NEAR_LONG",
        context.branch_near, context.entry_max, context.exit_max, seen);
    }
    for (const auto & context : contexts) {
      offerM1AnalyticTemplate(
        outcome, result, ego, obstacles, context, "FAR_SPAN",
        context.branch_far, context.entry_quarter, context.exit_span, seen);
    }
    return outcome;
  }

  SideResult evaluateSide(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::vector<P3ShadowObstacleEnvelope> & visible,
    const P3ShadowSideDomain & domain,
    bool outside_is_left) const
  {
    SideResult result;
    result.go_left = domain.go_left;
    result.domain = domain;
    if (!domain.valid) {
      result.failure = domain.reason;
      return result;
    }
    result.domain_valid = true;
    const bool go_left = domain.go_left;
    const double cluster_start = domain.cluster_start;
    const double cluster_end = domain.cluster_end;
    const double minimum_target = domain.minimum_target;
    const double maximum_target = domain.maximum_target;

    const auto corridor_start = Clock::now();
    result.corridor = makeCorridor(
      ego, visible, cluster_start, cluster_end, go_left, outside_is_left);
    result.runtime_corridor_us += elapsedUs(corridor_start);
    if (!result.corridor.connected || result.corridor.branch.empty()) {
      result.failure = "CORRIDOR_BRANCH_WRONG";
      return result;
    }

    const double target = existingTargetAnchor(minimum_target, maximum_target);
    result.probe = chooseProbe(result.corridor, cluster_start, cluster_end, target);
    const auto & mid_interval = nearestBranchSample(
      result.corridor, 0.5 * (cluster_start + cluster_end));
    double lower = std::max(-parameters_.maximum_target_offset_m, mid_interval.lower);
    double upper = std::min(parameters_.maximum_target_offset_m, mid_interval.upper);
    if (go_left) {
      lower = std::max(lower, target);
    } else {
      upper = std::min(upper, target);
    }

    const auto all_entries = entryScales();
    const std::vector<double> entries{all_entries.front(), all_entries.back()};
    const auto all_exits = exitScales(ego, cluster_end, go_left, outside_is_left);
    const std::array<double, 3> exit_ratios{0.015625, 0.5, 1.0};
    std::vector<double> exits;
    for (const double ratio : exit_ratios) {
      exits.push_back(all_exits.front() + ratio * (all_exits.back() - all_exits.front()));
    }
    exits = uniqueSorted(std::move(exits));

    std::size_t generation = 0U;
    for (const double entry : uniqueSorted(entries)) {
      for (const double exit : exits) {
        const double entry_length = cluster_start *
          parameters_.pre_apex_distances_m.front() * entry /
          parameters_.detection_lookahead_m;
        const double effective_exit = parameters_.cappedCombinedExitScale(
          exit *
          (go_left == outside_is_left ? parameters_.outside_line_transition_scale : 1.0));
        const double exit_length = parameters_.post_apex_distances_m.back() * effective_exit;
        const std::array<double, 5> stations{
          cluster_start - entry_length,
          cluster_start,
          0.5 * (cluster_start + cluster_end),
          cluster_end,
          cluster_end + exit_length};
        if (result.probe.station < stations.front() - kEpsilon ||
          result.probe.station > stations.back() + kEpsilon)
        {
          continue;
        }
        const auto root_start = Clock::now();
        const RootSolve roots = solvePosition(
          stations, ego.d, target, result.probe.station, result.probe.desired, lower, upper);
        result.runtime_root_solver_us += elapsedUs(root_start);
        result.raw_root_count += roots.algebraic.raw_roots.size();
        result.finite_root_count += roots.finite.size();
        result.branch_root_count += roots.branch.size();
        result.bounded_root_count += roots.bounded.size();
        result.accepted_root_count += roots.accepted.size();

        for (const double root : roots.accepted) {
          if (result.candidates.size() >= kFrozenCandidateCap) {
            result.failure = "CANDIDATE_CAP_EXCEEDED";
            return result;
          }
          ++result.validator_calls;
          auto trace = buildCandidate(
            ego, obstacles, go_left, outside_is_left, target, root, entry, exit, stations,
            generation++, result.runtime_reconstruction_us,
            result.runtime_hard_validation_us);
          trace.mapping_source = "FROZEN_V1";
          trace.candidate_template = "FROZEN_V1_CURVATURE_CONTINUITY";
          trace.source_cell = "ACTIVE_OUTER";
          trace.component_id = result.corridor.branch_id;
          const std::string side = go_left ? "LEFT" : "RIGHT";
          trace.candidate_identity = "M0_V1_" + side + "_" + trace.path_digest;
          trace.logical_identity = "M0_V1_" + side;
          const std::size_t index = result.candidates.size();
          if (trace.hard_valid) {
            ++result.hard_valid_count;
            if (!result.best_index.has_value() ||
              betterFeasible(trace, result.candidates[*result.best_index]))
            {
              result.best_index = index;
            }
          } else {
            result.failure = "P3_CANDIDATE_HARD_INVALID";
          }
          result.candidates.push_back(std::move(trace));
        }
      }
    }
    if (result.accepted_root_count == 0U) {
      result.failure = result.raw_root_count == 0U ? "NO_ALGEBRAIC_ROOT" :
        (result.branch_root_count == 0U ? "ROOT_BRANCH_MISMATCH" :
        (result.bounded_root_count == 0U ? "ROOT_LATERAL_BOUND" :
        "ANALYTIC_ROOT_EXISTS_BUT_REJECTED"));
    } else if (result.best_index.has_value()) {
      result.failure = "NONE";
    }
    return result;
  }

  const RacelineSplinePlanner & planner_;
  const RacelineSplineParameters & parameters_;
};

P3ShadowResult RacelineSplinePlanner::evaluateP3Shadow(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  std::int64_t snapshot_source_stamp_ns,
  std::uint64_t snapshot_epoch,
  std::uint64_t global_reference_generation,
  const std::string & p0_failure_reason) const
{
  const P3ShadowEvaluator evaluator(*this);
  P3ShadowResult strict = evaluator.run(
    ego, obstacles, snapshot_source_stamp_ns, snapshot_epoch,
    global_reference_generation, p0_failure_reason);
  if (strict.would_recover || !strict.invoked || strict.cluster_obstacle_ids.empty()) {
    return strict;
  }
  // strict(레이스 속도 예약) 게이트가 후보를 하나도 통과시키지 못했다. localization_reserve
  // 인상(0.06→0.12, 2026-08-15) 이후 strict 최소 target이 벽 캡 바로 앞까지 밀려 후보 전체가
  // footprint 검사에서 죽는 구간이 실측됐다(map s=16.5: 우측 여유 0.945 m에서 전멸). 기존
  // 감속-게이트 재시도는 "strict가 트랙에 안 들어갈 때"만 발동해 이 경우를 놓친다. 여기서
  // avoidance_minimum_speed_mps 게이트로 고려 범위만 넓혀 한 번 더 돈다 — 수용 기준(정확
  // 검증, gap 기반 속도 상한)은 동일하므로 "느리지만 가능한" 통로만 추가로 살아난다.
  P3ShadowResult relaxed = evaluator.run(
    ego, obstacles, snapshot_source_stamp_ns, snapshot_epoch,
    global_reference_generation, p0_failure_reason, true);
  if (std::getenv("P3_DEBUG_RELAXED") != nullptr) {
    std::fprintf(stderr, "[RELAXED] recover=%d fail=%s candidates=%zu\n",
      relaxed.would_recover ? 1 : 0, relaxed.failure_classification.c_str(),
      relaxed.candidates.size());
    for (const auto & trace : relaxed.candidates) {
      std::fprintf(stderr, "[RELAXED]  gen=%zu left=%d target=%.3f hard=%d rej=%s\n",
        trace.generation_index, trace.go_left ? 1 : 0, trace.d_target,
        trace.hard_valid ? 1 : 0, trace.rejection_reason.c_str());
    }
  }
  if (relaxed.would_recover) {
    relaxed.selected_source += "+RELAXED_CLEARANCE_GATE";
    return relaxed;
  }
  return strict;
}

}  // namespace local_planning
