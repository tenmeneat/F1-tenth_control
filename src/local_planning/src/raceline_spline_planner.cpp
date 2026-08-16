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

#include "local_planning/raceline_spline_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <utility>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1.0e-6;
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool finiteWaypoint(const f110_msgs::msg::Wpnt & waypoint)
{
  return std::isfinite(waypoint.s_m) && std::isfinite(waypoint.d_m) &&
         std::isfinite(waypoint.x_m) && std::isfinite(waypoint.y_m) &&
         std::isfinite(waypoint.psi_rad) && std::isfinite(waypoint.kappa_radpm) &&
         std::isfinite(waypoint.vx_mps) && std::isfinite(waypoint.ax_mps2) &&
         std::isfinite(waypoint.d_left) && std::isfinite(waypoint.d_right);
}

double pointDistance(
  const f110_msgs::msg::Wpnt & first,
  const f110_msgs::msg::Wpnt & second)
{
  return std::hypot(second.x_m - first.x_m, second.y_m - first.y_m);
}

// Monotone fifth-order smoothstep. Position, first derivative, and second derivative are all
// continuous with a constant-d segment at both ends, so entry/exit steering does not jump.
double quinticSmoothStep(double progress)
{
  const double u = clamp(progress, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

double quinticBlend(double start, double finish, double progress)
{
  return start + (finish - start) * quinticSmoothStep(progress);
}

struct AxisInterpolation
{
  std::size_t lower{0U};
  std::size_t upper{0U};
  double ratio{0.0};
};

AxisInterpolation interpolationFor(const std::vector<double> & bins, double value)
{
  if (bins.size() <= 1U || value <= bins.front()) {
    return {0U, 0U, 0.0};
  }
  if (value >= bins.back()) {
    const std::size_t last = bins.size() - 1U;
    return {last, last, 0.0};
  }
  const auto upper = std::upper_bound(bins.begin(), bins.end(), value);
  const std::size_t upper_index = static_cast<std::size_t>(
    std::distance(bins.begin(), upper));
  const std::size_t lower_index = upper_index - 1U;
  const double span = bins[upper_index] - bins[lower_index];
  return {lower_index, upper_index, (value - bins[lower_index]) / span};
}

}  // namespace

struct RacelineSplinePlanner::ExpandedObstacle
{
  int id{-1};
  double start{0.0};
  double end{0.0};
  double center{0.0};
  double raw_d_right{0.0};
  double raw_d_left{0.0};
  double d_right{0.0};
  double d_left{0.0};
  double target_clearance{0.0};
  // Same gate evaluated at avoidance_minimum_speed_mps. The preferred gate above reserves the
  // race-line speed's tracking error, which keeps a wide gap fast; a gap that only opens once the
  // pass slows down would otherwise be discarded here, before the speed cap ever runs.
  double relaxed_d_right{0.0};
  double relaxed_d_left{0.0};
  double relaxed_target_clearance{0.0};
};

struct RacelineSplinePlanner::FootprintTrackBoundSample
{
  double centerline_clearance_m{std::numeric_limits<double>::infinity()};
  double footprint_clearance_m{std::numeric_limits<double>::infinity()};
  bool invalid{false};
  std::string minimum_side;
  std::size_t waypoint_index{std::numeric_limits<std::size_t>::max()};
  double waypoint_s_m{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_x_m{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_y_m{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double heading_relative_to_reference_rad{std::numeric_limits<double>::quiet_NaN()};
  double wallward_corner_protrusion_m{std::numeric_limits<double>::quiet_NaN()};
};

struct RacelineSplinePlanner::Candidate
{
  bool valid{false};
  bool go_left{false};
  double target_d{0.0};
  double merge_s{0.0};
  double entry_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double exit_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double effective_entry_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double effective_exit_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double requested_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double effective_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double exit_length_m{std::numeric_limits<double>::quiet_NaN()};
  double centerline_wall_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double rectangular_footprint_wall_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
  bool footprint_invalid{false};
  std::string footprint_violation_side;
  std::size_t footprint_violation_waypoint_index{
    std::numeric_limits<std::size_t>::max()};
  double footprint_violation_s_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_x_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_y_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double footprint_heading_relative_to_reference_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double wallward_corner_protrusion_m{std::numeric_limits<double>::quiet_NaN()};
  double wall_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_rate_radpm2{std::numeric_limits<double>::quiet_NaN()};
  double velocity_loss{std::numeric_limits<double>::quiet_NaN()};
  double global_path_deviation_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_normalized_safety_slack{-std::numeric_limits<double>::infinity()};
  // 클러스터를 지난 뒤에도 오프셋이 남아 다음(비클러스터) 장애물의 물리 엔벨로프에 닿는
  // 후보. 유효성은 그대로 두고 순위에서만 뒤로 민다 — P3ShadowCandidateTrace의 같은 이름
  // 필드에서 그대로 옮겨온다.
  bool exit_reaches_next_obstacle{false};
  std::size_t audit_index{std::numeric_limits<std::size_t>::max()};
  f110_msgs::msg::WpntArray path;
  std::vector<SplineControlPoint> control_points;
  std::string reason;
};

RacelineSplinePlanner::RacelineSplinePlanner(RacelineSplineParameters parameters)
: parameters_(std::move(parameters))
{
}

bool RacelineSplineParameters::hasTrackingErrorLut() const
{
  return !tracking_error_lut_speed_bins_mps.empty() &&
         !tracking_error_lut_curvature_bins_radpm.empty() &&
         !tracking_error_lut_values_m.empty();
}

bool RacelineSplineParameters::trackingErrorLutValid() const
{
  const bool all_empty = tracking_error_lut_speed_bins_mps.empty() &&
    tracking_error_lut_curvature_bins_radpm.empty() &&
    tracking_error_lut_values_m.empty();
  if (all_empty) {
    return true;
  }
  if (!hasTrackingErrorLut() ||
    tracking_error_lut_values_m.size() !=
    tracking_error_lut_speed_bins_mps.size() *
    tracking_error_lut_curvature_bins_radpm.size())
  {
    return false;
  }
  const auto valid_axis = [](const std::vector<double> & bins) {
      return std::all_of(
        bins.begin(), bins.end(), [](double value) {
          return std::isfinite(value) && value >= 0.0;
        }) &&
             std::adjacent_find(
        bins.begin(), bins.end(), std::greater_equal<double>()) == bins.end();
    };
  return valid_axis(tracking_error_lut_speed_bins_mps) &&
         valid_axis(tracking_error_lut_curvature_bins_radpm) &&
         std::all_of(
    tracking_error_lut_values_m.begin(), tracking_error_lut_values_m.end(),
    [](double value) {return std::isfinite(value) && value >= 0.0;});
}

bool RacelineSplineParameters::avoidanceVelocityLimitValid() const
{
  const bool all_empty = avoidance_velocity_limit_speed_bins_mps.empty() &&
    avoidance_velocity_limit_lateral_accel_mps2.empty();
  if (all_empty) {
    return true;
  }
  if (avoidance_velocity_limit_speed_bins_mps.size() < 2U ||
    avoidance_velocity_limit_speed_bins_mps.size() !=
    avoidance_velocity_limit_lateral_accel_mps2.size() ||
    std::abs(avoidance_velocity_limit_speed_bins_mps.front()) > kEpsilon)
  {
    return false;
  }
  const bool valid_speed_axis = std::all_of(
    avoidance_velocity_limit_speed_bins_mps.begin(),
    avoidance_velocity_limit_speed_bins_mps.end(),
    [](double value) {return std::isfinite(value) && value >= 0.0;}) &&
    std::adjacent_find(
    avoidance_velocity_limit_speed_bins_mps.begin(),
    avoidance_velocity_limit_speed_bins_mps.end(),
    std::greater_equal<double>()) == avoidance_velocity_limit_speed_bins_mps.end();
  const bool valid_lateral_limits = std::all_of(
    avoidance_velocity_limit_lateral_accel_mps2.begin(),
    avoidance_velocity_limit_lateral_accel_mps2.end(),
    [](double value) {return std::isfinite(value) && value > 0.0;}) &&
    std::adjacent_find(
    avoidance_velocity_limit_lateral_accel_mps2.begin(),
    avoidance_velocity_limit_lateral_accel_mps2.end(),
    std::less<double>()) == avoidance_velocity_limit_lateral_accel_mps2.end();
  return valid_speed_axis && valid_lateral_limits;
}

double RacelineSplineParameters::limitedAvoidanceSpeed(
  double requested_speed_mps, double curvature_radpm) const
{
  const double requested_speed = std::abs(requested_speed_mps);
  const double curvature = std::abs(curvature_radpm);
  if (!(requested_speed > 0.0) || curvature <= kEpsilon ||
    avoidance_velocity_limit_speed_bins_mps.empty() || !avoidanceVelocityLimitValid())
  {
    return requested_speed;
  }

  const auto lateral_limit_at = [&](double speed_mps) {
      const auto interpolation = interpolationFor(
        avoidance_velocity_limit_speed_bins_mps, speed_mps);
      const double lower =
        avoidance_velocity_limit_lateral_accel_mps2[interpolation.lower];
      const double upper =
        avoidance_velocity_limit_lateral_accel_mps2[interpolation.upper];
      return lower + interpolation.ratio * (upper - lower);
    };
  const auto feasible = [&](double speed_mps) {
      return speed_mps * speed_mps * curvature <= lateral_limit_at(speed_mps);
    };
  if (feasible(requested_speed)) {
    return requested_speed;
  }

  double lower = 0.0;
  double upper = requested_speed;
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double middle = 0.5 * (lower + upper);
    if (feasible(middle)) {
      lower = middle;
    } else {
      upper = middle;
    }
  }
  return lower;
}

double RacelineSplineParameters::gapLimitedAvoidanceSpeed(
  double requested_speed_mps, double curvature_radpm, double admissible_reserve_m) const
{
  const double requested_speed = std::max(0.0, requested_speed_mps);
  const double floor_speed = std::max(0.0, avoidance_minimum_speed_mps);
  if (!std::isfinite(admissible_reserve_m) || requested_speed <= floor_speed) {
    return requested_speed;
  }
  const auto fits = [&](double speed_mps) {
      return trackingErrorReserve(speed_mps, curvature_radpm) <= admissible_reserve_m;
    };
  // Deciding whether to slow down at all carries the same tolerance the hard validator applies to
  // this boundary, so a candidate already sitting exactly on its clearance limit is not slowed by
  // rounding alone. The search below stays strict: a reduced speed whose reserve overshot by that
  // tolerance would spend room the path does not have, and the validator -- comparing with the
  // very same tolerance -- would reject the candidate the cap was meant to enable.
  if (trackingErrorReserve(requested_speed, curvature_radpm) <=
    admissible_reserve_m + kEpsilon)
  {
    return requested_speed;
  }
  // Even crawling does not fit this gap. Hold the floor and let the hard validator reject the
  // candidate: silently creeping through a gap the reserve says we cannot hold is not a decision
  // the speed policy is allowed to make.
  if (!fits(floor_speed)) {
    return floor_speed;
  }
  double lower = floor_speed;
  double upper = requested_speed;
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double middle = 0.5 * (lower + upper);
    if (fits(middle)) {
      lower = middle;
    } else {
      upper = middle;
    }
  }
  return lower;
}

double RacelineSplineParameters::trackingErrorReserve(
  double speed_mps, double curvature_radpm) const
{
  // localization_reserve_m is a constant floor added here (the single choke point) so the
  // gap-limited speed inversion, envelope expansion, and hard validation all account for the
  // same localization uncertainty. Constant offset keeps the speed-monotonicity the inversion
  // in gapLimitedAvoidanceSpeed relies on.
  const double localization = std::max(0.0, localization_reserve_m);
  if (!hasTrackingErrorLut() || !trackingErrorLutValid()) {
    return tracking_error_reserve_m + localization;
  }
  const auto speed = interpolationFor(
    tracking_error_lut_speed_bins_mps, std::abs(speed_mps));
  const auto curvature = interpolationFor(
    tracking_error_lut_curvature_bins_radpm, std::abs(curvature_radpm));
  const std::size_t curvature_count = tracking_error_lut_curvature_bins_radpm.size();
  const auto value_at = [&](std::size_t speed_index, std::size_t curvature_index) {
      return tracking_error_lut_values_m[speed_index * curvature_count + curvature_index];
    };
  const double lower =
    value_at(speed.lower, curvature.lower) + curvature.ratio *
    (value_at(speed.lower, curvature.upper) - value_at(speed.lower, curvature.lower));
  const double upper =
    value_at(speed.upper, curvature.lower) + curvature.ratio *
    (value_at(speed.upper, curvature.upper) - value_at(speed.upper, curvature.lower));
  return localization + lower + speed.ratio * (upper - lower);
}

double RacelineSplineParameters::avoidanceTrackingErrorReserve(
  double speed_mps, double curvature_radpm) const
{
  return trackingErrorReserve(
    limitedAvoidanceSpeed(speed_mps, curvature_radpm), curvature_radpm);
}

double RacelineSplineParameters::cappedCombinedExitScale(double combined_exit_scale) const
{
  if (!(maximum_exit_length_m > 0.0) || post_apex_distances_m.empty()) {
    return combined_exit_scale;
  }
  const double post_far = post_apex_distances_m.back();
  if (!(post_far > kEpsilon)) {
    return combined_exit_scale;
  }
  return std::min(combined_exit_scale, maximum_exit_length_m / post_far);
}

double RacelineSplineParameters::obstacleBaseClearance() const
{
  return vehicle_half_width_m + safety_margin_m;
}

double RacelineSplineParameters::obstacleSafetyClearance(
  double speed_mps, double curvature_radpm, double reserve_scale) const
{
  const double scale = std::clamp(reserve_scale, 0.0, 1.0);
  return obstacleBaseClearance() +
         scale * avoidanceTrackingErrorReserve(speed_mps, curvature_radpm);
}

double RacelineSplineParameters::trackBoundaryReserve(
  double speed_mps, double curvature_radpm) const
{
  (void)speed_mps;
  (void)curvature_radpm;
  return wall_safety_margin_m;
}

void RacelineSplinePlanner::setParameters(const RacelineSplineParameters & parameters)
{
  parameters_ = parameters;
}

bool RacelineSplinePlanner::setReference(
  const f110_msgs::msg::WpntArray & reference,
  std::string * error)
{
  auto reject = [&](const std::string & why) {
      reference_.wpnts.clear();
      track_length_ = 0.0;
      if (error != nullptr) {
        *error = why;
      }
      return false;
    };

  if (reference.wpnts.size() < 4U) {
    return reject("global reference needs at least four waypoints");
  }
  std::vector<double> spacing;
  spacing.reserve(reference.wpnts.size() - 1U);
  for (std::size_t i = 0; i < reference.wpnts.size(); ++i) {
    if (!finiteWaypoint(reference.wpnts[i])) {
      return reject("global reference contains a non-finite waypoint");
    }
    if (i > 0U) {
      const double ds = reference.wpnts[i].s_m - reference.wpnts[i - 1U].s_m;
      if (!(ds > kEpsilon)) {
        return reject("global reference s_m must be strictly increasing");
      }
      spacing.push_back(ds);
    }
  }
  std::sort(spacing.begin(), spacing.end());
  const double median_spacing = spacing[spacing.size() / 2U];
  const double inferred_length = reference.wpnts.back().s_m + median_spacing;
  if (!(inferred_length > reference.wpnts.back().s_m) || !std::isfinite(inferred_length)) {
    return reject("failed to infer a positive closed-track length");
  }

  reference_ = reference;
  track_length_ = inferred_length;
  return true;
}

bool RacelineSplinePlanner::ready() const
{
  return reference_.wpnts.size() >= 4U && track_length_ > 0.0;
}

double RacelineSplinePlanner::trackLength() const
{
  return track_length_;
}

double RacelineSplinePlanner::wrapS(double s) const
{
  if (!(track_length_ > 0.0)) {
    return s;
  }
  double wrapped = std::fmod(s, track_length_);
  if (wrapped < 0.0) {
    wrapped += track_length_;
  }
  return wrapped;
}

double RacelineSplinePlanner::forwardDistance(double from_s, double to_s) const
{
  return wrapS(to_s - from_s);
}

std::vector<int> RacelineSplinePlanner::blockingClusterIds(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(ego.d) ||
    !std::isfinite(ego.speed))
  {
    return {};
  }
  const auto cluster = nearestCluster(expandVisibleObstacles(ego, obstacles));
  std::vector<int> ids;
  ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    ids.push_back(obstacle.id);
  }
  return ids;
}

std::size_t RacelineSplinePlanner::nextReferenceIndex(double s) const
{
  const double wrapped_s = wrapS(s);
  const auto iterator = std::lower_bound(
    reference_.wpnts.begin(), reference_.wpnts.end(), wrapped_s,
    [](const f110_msgs::msg::Wpnt & waypoint, double value) {
      return waypoint.s_m < value;
    });
  return iterator == reference_.wpnts.end() ?
         0U : static_cast<std::size_t>(std::distance(reference_.wpnts.begin(), iterator));
}

std::size_t RacelineSplinePlanner::nearestReferenceIndex(double s) const
{
  const std::size_t next = nextReferenceIndex(s);
  const std::size_t previous =
    next == 0U ? reference_.wpnts.size() - 1U : next - 1U;
  const auto circular_distance = [this, s](std::size_t index) {
      const double forward = forwardDistance(s, reference_.wpnts[index].s_m);
      return std::min(forward, track_length_ - forward);
    };
  return circular_distance(next) < circular_distance(previous) ? next : previous;
}

double RacelineSplinePlanner::maximumReferenceTrackingErrorReserve(
  const EgoFrenetState & ego, double start, double end, double speed_cap_mps) const
{
  const double check_start = std::max(0.0, start);
  const double check_end = std::max(check_start, end);
  double maximum = 0.0;
  bool checked_reference = false;
  const std::size_t first_index = nextReferenceIndex(wrapS(ego.s + check_start));
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const auto & reference = reference_.wpnts[(first_index + k) % reference_.wpnts.size()];
    const double forward_s = forwardDistance(ego.s, reference.s_m);
    if (forward_s + kEpsilon < check_start) {
      continue;
    }
    if (forward_s > check_end + kEpsilon) {
      break;
    }
    checked_reference = true;
    maximum = std::max(
      maximum,
      parameters_.avoidanceTrackingErrorReserve(
        std::min(reference.vx_mps, speed_cap_mps), reference.kappa_radpm));
  }
  if (!checked_reference) {
    const auto & reference = reference_.wpnts[
      nearestReferenceIndex(wrapS(ego.s + 0.5 * (check_start + check_end)))];
    maximum = parameters_.avoidanceTrackingErrorReserve(
      std::min(reference.vx_mps, speed_cap_mps), reference.kappa_radpm);
  }
  return maximum;
}

std::vector<RacelineSplinePlanner::ExpandedObstacle>
RacelineSplinePlanner::expandVisibleObstacles(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  // Obstacle input may already be an uncertainty Guard, but vehicle size, physical margin, and
  // the reference-span maximum tracking reserve are applied exactly once to the target gate.
  std::vector<ExpandedObstacle> visible;
  visible.reserve(obstacles.size());
  for (const auto & obstacle : obstacles) {
    if (!std::isfinite(obstacle.s_center) || !std::isfinite(obstacle.s_start) ||
      !std::isfinite(obstacle.s_end) || !std::isfinite(obstacle.d_left) ||
      !std::isfinite(obstacle.d_right))
    {
      continue;
    }
    const double center = forwardDistance(ego.s, obstacle.s_center);
    const double span_forward = forwardDistance(obstacle.s_start, obstacle.s_end);
    const double span_reverse = forwardDistance(obstacle.s_end, obstacle.s_start);
    double span = std::min(span_forward, span_reverse);
    if (!(span > kEpsilon)) {
      span = std::max(0.05, std::abs(obstacle.size));
    }
    const double half_span = 0.5 * span + parameters_.obstacle_longitudinal_padding_m;
    ExpandedObstacle expanded;
    expanded.id = obstacle.id;
    expanded.center = center;
    expanded.start = center - half_span;
    expanded.end = center + half_span;
    expanded.raw_d_right = std::min(obstacle.d_right, obstacle.d_left);
    expanded.raw_d_left = std::max(obstacle.d_right, obstacle.d_left);
    expanded.target_clearance = parameters_.obstacleBaseClearance() +
      maximumReferenceTrackingErrorReserve(ego, expanded.start, expanded.end);
    expanded.d_right = expanded.raw_d_right - expanded.target_clearance;
    expanded.d_left = expanded.raw_d_left + expanded.target_clearance;
    expanded.relaxed_target_clearance = parameters_.obstacleBaseClearance() +
      maximumReferenceTrackingErrorReserve(
      ego, expanded.start, expanded.end, parameters_.avoidance_minimum_speed_mps);
    expanded.relaxed_d_right = expanded.raw_d_right - expanded.relaxed_target_clearance;
    expanded.relaxed_d_left = expanded.raw_d_left + expanded.relaxed_target_clearance;
    if (expanded.end >= 0.0 &&
      expanded.start <= parameters_.detection_lookahead_m)
    {
      visible.push_back(expanded);
    }
  }
  std::sort(
    visible.begin(), visible.end(),
    [](const ExpandedObstacle & first, const ExpandedObstacle & second) {
      return first.start < second.start;
    });
  return visible;
}

bool RacelineSplinePlanner::isBlockingRaceline(const ExpandedObstacle & obstacle) const
{
  return obstacle.d_right <= 0.0 && obstacle.d_left >= 0.0;
}

bool RacelineSplinePlanner::physicallyBlocksRaceline(const ExpandedObstacle & obstacle) const
{
  // Physical predicate only: raw measured envelope plus the vehicle's physical footprint
  // clearance (half width + safety margin). The tracking-error reserve deliberately does not
  // participate — it decides whether avoidance/slow-down is wanted, not whether the line is
  // physically drivable.
  const double physical_clearance = parameters_.obstacleBaseClearance();
  return obstacle.raw_d_right - physical_clearance <= 0.0 &&
         obstacle.raw_d_left + physical_clearance >= 0.0;
}

bool RacelineSplinePlanner::clusterPhysicallyBlocksRaceline(
  const std::vector<ExpandedObstacle> & cluster) const
{
  return std::any_of(
    cluster.begin(), cluster.end(),
    [this](const ExpandedObstacle & obstacle) {return physicallyBlocksRaceline(obstacle);});
}

bool RacelineSplinePlanner::obstaclesPhysicallyBlockRaceline(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(ego.d) ||
    !std::isfinite(ego.speed))
  {
    return false;
  }
  return clusterPhysicallyBlocksRaceline(expandVisibleObstacles(ego, obstacles));
}

RacelineSplineResult RacelineSplinePlanner::buildMarginSlowPass(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & cluster) const
{
  RacelineSplineResult result;
  result.kind = SplinePlanKind::kNoSafePath;
  if (cluster.empty()) {
    result.reason = "margin slow pass requested without a blocking cluster";
    return result;
  }
  double cluster_end = cluster.front().end;
  double raw_d_sum = 0.0;
  result.obstacle_ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    cluster_end = std::max(cluster_end, obstacle.end);
    raw_d_sum += 0.5 * (obstacle.raw_d_left + obstacle.raw_d_right);
    result.obstacle_ids.push_back(obstacle.id);
  }
  const double cap = parameters_.margin_pass_speed_cap_mps;
  if (!(cap > 0.0)) {
    result.reason = "margin slow pass disabled (margin_pass_speed_cap_mps <= 0)";
    return result;
  }
  // 접근 실현성 램프(2026-08-14 실차): 자차가 cap보다 빠를 때 flat cap을 자차 위치부터
  // 그대로 명령하면 계단 감속이 된다 — 서비스 브레이크가 포화하고 마찰 한계를 넘겨
  // 슬립(조향 상실)으로 이어졌다(4.4 m/s 접근에 flat 2.0 → 벽 충돌). 군집 시작 전
  // 구간은 실측 자차 속도에서 approach_feasibility_decel_mps2로 내려가는 프로파일까지
  // 허용하되, 그 감속으로 군집 시작까지 cap에 못 닿으면 닿는 만큼만 더 가파르게 잡는다
  // (여유가 전혀 없으면 기존 flat cap으로 수렴). 군집 스팬부터는 항상 cap 그대로다.
  double cluster_start = cluster.front().start;
  for (const auto & obstacle : cluster) {
    cluster_start = std::min(cluster_start, obstacle.start);
  }
  cluster_start = std::max(0.0, cluster_start);
  const double approach_decel = parameters_.approach_feasibility_decel_mps2;
  const double ego_speed =
    std::isfinite(ego.speed) ? std::max(0.0, std::abs(ego.speed)) : 0.0;
  const bool ramp_active =
    approach_decel > 0.0 && ego_speed > cap && cluster_start > kEpsilon;
  double ramp_decel = approach_decel;
  if (ramp_active) {
    ramp_decel = std::max(
      approach_decel,
      (ego_speed * ego_speed - cap * cap) / (2.0 * cluster_start));
  }
  // End far enough past the cluster that the merge-completion check (tail reach) fires with the
  // full vehicle clear of the obstacle span.
  const double end_at = std::min(
    cluster_end + parameters_.safe_stop_buffer_m + parameters_.vehicle_length_m,
    track_length_ - kEpsilon);
  result.path.header = reference_.header;
  const std::size_t first_index = nextReferenceIndex(ego.s);
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const std::size_t index = (first_index + k) % reference_.wpnts.size();
    const double forward_s = forwardDistance(ego.s, reference_.wpnts[index].s_m);
    if (forward_s > end_at + kEpsilon) {
      break;
    }
    auto waypoint = reference_.wpnts[index];
    waypoint.id = static_cast<int32_t>(result.path.wpnts.size());
    waypoint.d_m = 0.0;
    double allowed = cap;
    if (ramp_active && forward_s < cluster_start) {
      allowed = std::sqrt(
        std::max(cap * cap, ego_speed * ego_speed - 2.0 * ramp_decel * forward_s));
    }
    waypoint.vx_mps = std::min(std::max(0.0, waypoint.vx_mps), allowed);
    result.path.wpnts.push_back(waypoint);
  }
  if (result.path.wpnts.size() < 2U) {
    result.path.wpnts.clear();
    result.reason = "margin slow pass span is already behind ego";
    return result;
  }
  updateGeometryAndAcceleration(result.path);
  result.kind = SplinePlanKind::kAvoidance;
  result.margin_pass = true;
  // The pass stays on the line; the side flag only records which side of the obstacle that is.
  result.go_left = raw_d_sum < 0.0;
  result.target_d = 0.0;
  result.merge_s = result.path.wpnts.back().s_m;
  result.obstacle_id = result.obstacle_ids.front();
  result.reason =
    "margin-only blocking cluster (raw envelope + physical clearance stays clear of the race "
    "line); passing on the line at the margin_pass speed cap";
  return result;
}

std::vector<RacelineSplinePlanner::ExpandedObstacle>
RacelineSplinePlanner::nearestCluster(
  const std::vector<ExpandedObstacle> & obstacles) const
{
  auto first = std::find_if(
    obstacles.begin(), obstacles.end(),
    [this](const ExpandedObstacle & obstacle) {return isBlockingRaceline(obstacle);});
  if (first == obstacles.end()) {
    return {};
  }

  std::vector<ExpandedObstacle> cluster;
  cluster.push_back(*first);
  double cluster_end = first->end;
  for (auto current = std::next(first); current != obstacles.end(); ++current) {
    if (current->start > cluster_end + parameters_.obstacle_cluster_gap_m) {
      break;
    }
    cluster.push_back(*current);
    cluster_end = std::max(cluster_end, current->end);
  }
  return cluster;
}

bool RacelineSplinePlanner::outsideIsLeft(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & cluster) const
{
  if (cluster.empty()) {
    return false;
  }
  double cluster_start = cluster.front().start;
  double cluster_end = cluster.front().end;
  for (const auto & obstacle : cluster) {
    cluster_start = std::min(cluster_start, obstacle.start);
    cluster_end = std::max(cluster_end, obstacle.end);
  }
  double curvature_sum = 0.0;
  const std::size_t obstacle_index = nearestReferenceIndex(
    wrapS(ego.s + 0.5 * (cluster_start + cluster_end)));
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const std::size_t index = (obstacle_index + k) % reference_.wpnts.size();
    const double forward = forwardDistance(
      reference_.wpnts[obstacle_index].s_m, reference_.wpnts[index].s_m);
    if (forward > 2.0) {
      break;
    }
    curvature_sum += reference_.wpnts[index].kappa_radpm;
  }
  return curvature_sum < 0.0;
}

bool RacelineSplinePlanner::computeSideTargetRange(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & cluster,
  bool go_left,
  double & cluster_start,
  double & cluster_end,
  double & minimum_clearance_target_d,
  double & maximum_track_target_d,
  std::string & reason,
  bool relaxed_clearance_gate) const
{
  if (cluster.empty()) {
    reason = "empty obstacle cluster";
    return false;
  }

  cluster_start = cluster.front().start;
  cluster_end = cluster.front().end;
  minimum_clearance_target_d = go_left ? cluster.front().d_left : cluster.front().d_right;
  double relaxed_clearance_target_d =
    go_left ? cluster.front().relaxed_d_left : cluster.front().relaxed_d_right;
  for (const auto & obstacle : cluster) {
    cluster_start = std::min(cluster_start, obstacle.start);
    cluster_end = std::max(cluster_end, obstacle.end);
    minimum_clearance_target_d = go_left ?
      std::max(minimum_clearance_target_d, obstacle.d_left) :
      std::min(minimum_clearance_target_d, obstacle.d_right);
    relaxed_clearance_target_d = go_left ?
      std::max(relaxed_clearance_target_d, obstacle.relaxed_d_left) :
      std::min(relaxed_clearance_target_d, obstacle.relaxed_d_right);
  }
  if (relaxed_clearance_gate) {
    // 2차 시도 전용(evaluateP3Shadow 참고): strict 게이트의 전 후보가 exact validator에서
    // 기각된 뒤에만 들어온다. 감속 통과가 필요로 하는 게이트로 최소 target을 낮춰 "느리지만
    // 가능한" 통로를 고려 대상에 넣는다. 수용 기준(정확 검증·속도 상한 역산)은 그대로이므로
    // 고려 범위만 넓어질 뿐 수용이 넓어지지는 않는다.
    minimum_clearance_target_d = relaxed_clearance_target_d;
  }
  if (go_left) {
    minimum_clearance_target_d = std::max(
      minimum_clearance_target_d,
      parameters_.minimum_target_offset_m);
  } else {
    minimum_clearance_target_d = std::min(
      minimum_clearance_target_d,
      -parameters_.minimum_target_offset_m);
  }
  if (std::abs(minimum_clearance_target_d) > parameters_.maximum_target_offset_m) {
    reason = "required d-offset exceeds maximum_target_offset_m";
    return false;
  }

  // Find the farthest target that remains inside the centre-of-vehicle track boundary for the
  // complete obstacle span. The closest endpoint is determined only by obstacle clearance; the
  // other endpoint is determined only by track geometry and maximum_target_offset_m.
  maximum_track_target_d = go_left ? parameters_.maximum_target_offset_m :
    -parameters_.maximum_target_offset_m;
  const auto update_track_limit = [&](const f110_msgs::msg::Wpnt & reference) {
      // The hard validator checks the rotated vehicle rectangle, not the path centre, so the
      // centre may only approach the wall to within its own half width. Leaving that out here
      // offers target offsets the footprint check can never accept, and a gap that is only
      // reachable closer to the obstacle looks unreachable instead of merely slower.
      const double reserve = parameters_.trackBoundaryReserve(
        reference.vx_mps, reference.kappa_radpm) + parameters_.vehicle_half_width_m;
      const double left_width = reference.d_left > 0.05 ?
        reference.d_left : parameters_.fallback_track_half_width_m;
      const double right_width = reference.d_right > 0.05 ?
        reference.d_right : parameters_.fallback_track_half_width_m;
      if (go_left) {
        maximum_track_target_d = std::min(maximum_track_target_d, left_width - reserve);
      } else {
        maximum_track_target_d = std::max(maximum_track_target_d, -right_width + reserve);
      }
    };

  const double check_start = std::max(0.0, cluster_start);
  bool checked_reference = false;
  const std::size_t first_index = nextReferenceIndex(wrapS(ego.s + check_start));
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const auto & reference = reference_.wpnts[(first_index + k) % reference_.wpnts.size()];
    const double forward_s = forwardDistance(ego.s, reference.s_m);
    if (forward_s + kEpsilon < check_start) {
      continue;
    }
    if (forward_s > cluster_end + kEpsilon) {
      break;
    }
    checked_reference = true;
    update_track_limit(reference);
  }
  if (!checked_reference) {
    const double midpoint = 0.5 * (check_start + std::max(check_start, cluster_end));
    update_track_limit(
      reference_.wpnts[nearestReferenceIndex(wrapS(ego.s + midpoint))]);
  }

  const auto fits_track = [&](double target_d) {
      return go_left ?
             target_d <= maximum_track_target_d + kEpsilon :
             target_d >= maximum_track_target_d - kEpsilon;
    };
  if (!fits_track(minimum_clearance_target_d)) {
    // The race-line-speed gate does not fit between the obstacle and the wall. Retry against the
    // gate the pass would need at avoidance_minimum_speed_mps: the speed cap lowers this waypoint
    // to match, and the hard validator still checks the candidate at whatever speed it ends up
    // with, so this widens what is considered without widening what is accepted.
    const double relaxed_target_d = go_left ?
      std::max(relaxed_clearance_target_d, parameters_.minimum_target_offset_m) :
      std::min(relaxed_clearance_target_d, -parameters_.minimum_target_offset_m);
    if (std::abs(relaxed_target_d) <= parameters_.maximum_target_offset_m &&
      fits_track(relaxed_target_d))
    {
      minimum_clearance_target_d = relaxed_target_d;
      return true;
    }
    reason = go_left ?
      "left target d exceeds track bound in obstacle span before spline construction" :
      "right target d exceeds track bound in obstacle span before spline construction";
    return false;
  }
  return true;
}

P3ShadowPlanningContext RacelineSplinePlanner::buildP3ShadowPlanningContext(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  bool relaxed_clearance_gate) const
{
  P3ShadowPlanningContext context;
  const auto visible = expandVisibleObstacles(ego, obstacles);
  const auto cluster = nearestCluster(visible);
  if (cluster.empty()) {
    context.reason = "no static obstacle blocks the global race line";
    return context;
  }
  context.valid = true;
  context.outside_is_left = outsideIsLeft(ego, cluster);
  context.cluster_ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    context.cluster_ids.push_back(obstacle.id);
  }
  context.visible.reserve(visible.size());
  for (const auto & obstacle : visible) {
    // relaxed 모드에서는 코리도의 스테이션별 통과 가능 구간도 감속 게이트 외피로 계산해야
    // 한다 — 도메인 끝점만 낮추면 코리도 교집합이 레이스 속도 외피로 다시 닫아 버린다.
    context.visible.push_back({
        obstacle.id, obstacle.start, obstacle.end, obstacle.center,
        relaxed_clearance_gate ? obstacle.relaxed_d_right : obstacle.d_right,
        relaxed_clearance_gate ? obstacle.relaxed_d_left : obstacle.d_left});
  }
  const auto fill_side = [&](bool go_left, P3ShadowSideDomain & side) {
      side.go_left = go_left;
      side.valid = computeSideTargetRange(
        ego, cluster, go_left, side.cluster_start, side.cluster_end,
        side.minimum_target, side.maximum_target, side.reason, relaxed_clearance_gate);
    };
  fill_side(false, context.right);
  fill_side(true, context.left);
  return context;
}

void RacelineSplinePlanner::finalizeP3ShadowPath(
  f110_msgs::msg::WpntArray & path,
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  const auto visible = expandVisibleObstacles(ego, obstacles);
  updateGeometryAndAcceleration(path);
  applyAvoidanceVelocityLimit(path, ego, visible);
  updateGeometryAndAcceleration(path);
}

P3ShadowPathEvaluation RacelineSplinePlanner::validateP3ShadowPath(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  double obstacle_reserve_scale,
  const std::optional<double> & collision_horizon) const
{
  P3ShadowPathEvaluation result;
  Candidate candidate;
  candidate.path = path;
  const auto visible = expandVisibleObstacles(ego, obstacles);
  measureCandidate(ego, visible, candidate);
  result.hard_valid = validateCandidate(
    ego, candidate.path, visible, candidate.reason, 0U, 0U, nullptr, collision_horizon,
    obstacle_reserve_scale);
  result.minimum_normalized_safety_slack = candidate.minimum_normalized_safety_slack;
  result.minimum_track_margin_m = candidate.rectangular_footprint_wall_clearance_m;
  result.minimum_obstacle_margin_m = candidate.obstacle_clearance_m;
  result.peak_curvature_radpm = candidate.peak_curvature_radpm;
  result.peak_curvature_rate_radpm2 = candidate.peak_curvature_rate_radpm2;
  result.velocity_loss = candidate.velocity_loss;
  result.global_path_deviation_m = candidate.global_path_deviation_m;
  result.rejection_reason = result.hard_valid ? std::string() : candidate.reason;
  return result;
}

P3ShadowPathEvaluation RacelineSplinePlanner::evaluateP3PathCurrent(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  double obstacle_reserve_scale,
  const std::optional<double> & collision_horizon) const
{
  if (path.wpnts.size() < static_cast<std::size_t>(parameters_.minimum_path_points)) {
    P3ShadowPathEvaluation result;
    result.rejection_reason = "spline segment has too few global race-line samples";
    return result;
  }
  return validateP3ShadowPath(
    ego, path, obstacles, obstacle_reserve_scale, collision_horizon);
}

bool RacelineSplinePlanner::targetFitsTrackBounds(
  const EgoFrenetState & ego,
  double cluster_start,
  double cluster_end,
  bool go_left,
  double target_d,
  std::string & reason,
  double * min_headroom) const
{
  // Global waypoint d_left/d_right are centre-of-vehicle limits. Vehicle width, obstacle
  // clearance, and tracking-error reserve do not belong here; subtract only the wall reserve.
  if (min_headroom != nullptr) {
    *min_headroom = std::numeric_limits<double>::infinity();
  }
  const auto fits_at = [&](const f110_msgs::msg::Wpnt & reference) {
      const double center_boundary_clearance = parameters_.trackBoundaryReserve(
        reference.vx_mps, reference.kappa_radpm);
      const double left_width = reference.d_left > 0.05 ?
        reference.d_left : parameters_.fallback_track_half_width_m;
      const double right_width = reference.d_right > 0.05 ?
        reference.d_right : parameters_.fallback_track_half_width_m;
      const double headroom = go_left ?
        left_width - center_boundary_clearance - target_d :
        target_d + right_width - center_boundary_clearance;
      if (min_headroom != nullptr) {
        *min_headroom = std::min(*min_headroom, headroom);
      }
      return headroom >= -kEpsilon;
    };

  // The target offset is held across the expanded obstacle-cluster span. Reject an obviously
  // impossible side before fitting/sampling up to three splines, but retain the full candidate
  // validation because the transition can still meet a narrower wall before or after this span.
  const double check_start = std::max(0.0, cluster_start);
  bool checked_reference = false;
  const std::size_t first_index = nextReferenceIndex(wrapS(ego.s + check_start));
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const auto & reference =
      reference_.wpnts[(first_index + k) % reference_.wpnts.size()];
    const double forward_s = forwardDistance(ego.s, reference.s_m);
    if (forward_s + kEpsilon < check_start) {
      continue;
    }
    if (forward_s > cluster_end + kEpsilon) {
      break;
    }
    checked_reference = true;
    if (!fits_at(reference)) {
      reason = go_left ?
        "left target d exceeds track bound in obstacle span before spline construction" :
        "right target d exceeds track bound in obstacle span before spline construction";
      return false;
    }
  }

  // A very short obstacle span can fall between two global samples. Check its midpoint against the
  // nearest reference width so the fast gate remains useful without inventing a Cartesian wall.
  if (!checked_reference) {
    const double midpoint = 0.5 * (check_start + std::max(check_start, cluster_end));
    const auto & reference = reference_.wpnts[
      nearestReferenceIndex(wrapS(ego.s + midpoint))];
    if (!fits_at(reference)) {
      reason = go_left ?
        "left target d exceeds track bound in obstacle span before spline construction" :
        "right target d exceeds track bound in obstacle span before spline construction";
      return false;
    }
  }
  return true;
}

f110_msgs::msg::WpntArray RacelineSplinePlanner::buildGlobalHandoffPath(
  const EgoFrenetState & ego, double state_tail_distance_m, double speed_cap_mps) const
{
  f110_msgs::msg::WpntArray path;
  path.header = reference_.header;
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(state_tail_distance_m) ||
    !(state_tail_distance_m > 0.0) || !(speed_cap_mps > 0.0))
  {
    return path;
  }

  const std::size_t total = reference_.wpnts.size();
  // tail 창은 경로 끝에서 거꾸로 잰 호 길이[m]다. 비율이던 시절에는 창이 경로 길이에
  // 비례해 요동했다(전체 루프 43 m의 10% = 4.3 m). state_machine의
  // enter_global_tail_distance_m와 같은 정의·같은 값이어야 한다.
  std::size_t tail_count = 1U;
  double walked_m = 0.0;
  while (tail_count < total) {
    const std::size_t index = total - tail_count;
    const double segment = forwardDistance(
      reference_.wpnts[(index - 1U) % total].s_m, reference_.wpnts[index % total].s_m);
    if (walked_m + segment > state_tail_distance_m) {
      break;
    }
    walked_m += segment;
    ++tail_count;
  }
  const std::size_t tail_begin = total - tail_count;
  const std::size_t ego_index = nearestReferenceIndex(ego.s);

  // 계획된 복귀 램프: ego의 현재 d에서 0까지 smoothstep으로 내려간다. 램프 없이 d=0
  // 라인만 주면 복귀가 컨트롤러 자연 수렴(실측 0.055 m/m)에 맡겨져, 연속 장애물에서
  // 다음 기동이 남은 오프셋 위에서 시작되고 FSM은 |d| 게이트에 오래 붙잡힌다.
  const double ramp_d0 = (std::isfinite(ego.d) ? ego.d : 0.0);
  const double ramp_length = std::max(
    std::max(0.0, parameters_.merge_ramp_min_length_m),
    std::abs(ego.speed) * std::max(0.0, parameters_.merge_ramp_time_sec));
  const bool apply_ramp = std::abs(ramp_d0) > 0.03 && ramp_length > 1.0e-6;

  // 전체 global loop를 회전시켜 현재 ego를 마지막 tail 구간의 **첫 점**에 놓는다.
  // state_machine은 이 경로의 ot_line=raceline_global_handoff 표식을 GLOBAL 복귀의
  // 전제로 요구하고(2026-08-16 계약), 그 위에서 tail 도달·횡오차·지속시간 검사를
  // 평가한다. ego를 tail 첫 점에 놓으므로 tail 도달 게이트는 즉시 성립하고, 실질
  // 결정은 물리적 |ego_d| 게이트와 지속시간이 한다.
  const std::size_t first_index = (ego_index + total - tail_begin) % total;
  path.wpnts.reserve(total);
  for (std::size_t k = 0; k < total; ++k) {
    auto waypoint = reference_.wpnts[(first_index + k) % total];
    waypoint.id = static_cast<int32_t>(k);
    waypoint.d_m = 0.0;
    waypoint.vx_mps = std::min(std::max(0.0, waypoint.vx_mps), speed_cap_mps);
    path.wpnts.push_back(waypoint);
  }

  // 회전 배열에서 ego는 위치 tail_begin에 놓인다(전방 순서: tail_begin → total-1).
  // 그 구간에 전방 호 길이 기준으로 램프를 적용한다. 기본 tail 길이(≈수 m)가
  // ramp_length보다 짧으면 램프가 잘리지만, 잘린 끝에서도 d는 단조 감소라 안전 방향이다.
  if (apply_ramp) {
    // 벽 협착부 클램프: 고정 길이 램프가 좁아지는 구간에서 오프셋을 유지한 채 지나가면
    // 벽 여유가 깎인다(.regression_check10: FINALS 최소 벽 여유 0.117→0.082 m — s≈23
    // 왼쪽 협착부). 각 지점의 트랙 여유로 |d|를 제한하고, 클램프로 내려간 뒤 다시
    // 벌어지는 구간에서도 단조 감소를 유지해 차가 도로 바깥쪽으로 되돌지 않게 한다.
    const double wall_keepout =
      std::max(0.0, parameters_.vehicle_half_width_m) +
      std::max(0.0, parameters_.wall_safety_margin_m);
    double forward_m = 0.0;
    double previous_magnitude = std::abs(ramp_d0);
    const double sign = (ramp_d0 >= 0.0 ? 1.0 : -1.0);
    for (std::size_t k = tail_begin; k < total; ++k) {
      const auto & global = reference_.wpnts[(first_index + k) % total];
      if (k > tail_begin) {
        const auto & previous = reference_.wpnts[(first_index + k - 1U) % total];
        forward_m += std::hypot(
          global.x_m - previous.x_m, global.y_m - previous.y_m);
      }
      if (forward_m >= ramp_length) {
        break;
      }
      const double t = std::clamp(forward_m / ramp_length, 0.0, 1.0);
      // smoothstep의 여집합 (1-t)^2(1+2t): d(0)=d0, d(L)=0, 양 끝 기울기 0.
      const double profile = std::abs(ramp_d0) * (1.0 - t) * (1.0 - t) * (1.0 + 2.0 * t);
      const double side_room = (sign > 0.0 ? global.d_left : global.d_right);
      const double allowed =
        std::isfinite(side_room) ? std::max(0.0, side_room - wall_keepout) :
        previous_magnitude;
      const double magnitude = std::min({profile, allowed, previous_magnitude});
      previous_magnitude = magnitude;
      const double d = sign * magnitude;
      auto & waypoint = path.wpnts[k];
      waypoint.d_m = d;
      waypoint.x_m = global.x_m - d * std::sin(global.psi_rad);
      waypoint.y_m = global.y_m + d * std::cos(global.psi_rad);
    }
  }
  return path;
}

f110_msgs::msg::WpntArray RacelineSplinePlanner::buildEmergencyStopPath(
  const EgoFrenetState & ego) const
{
  f110_msgs::msg::WpntArray path;
  path.header = reference_.header;
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(ego.d)) {
    return path;
  }
  const std::size_t count = std::min(
    reference_.wpnts.size(),
    std::max<std::size_t>(
      2U, static_cast<std::size_t>(parameters_.minimum_path_points)));
  const std::size_t first_index = nextReferenceIndex(ego.s);
  path.wpnts.reserve(count);
  for (std::size_t k = 0; k < count; ++k) {
    const auto & global = reference_.wpnts[(first_index + k) % reference_.wpnts.size()];
    auto waypoint = global;
    waypoint.id = static_cast<int32_t>(k);
    waypoint.d_m = ego.d;
    waypoint.x_m = global.x_m - ego.d * std::sin(global.psi_rad);
    waypoint.y_m = global.y_m + ego.d * std::cos(global.psi_rad);
    waypoint.vx_mps = 0.0;
    waypoint.ax_mps2 = 0.0;
    path.wpnts.push_back(waypoint);
  }
  return path;
}

f110_msgs::msg::WpntArray RacelineSplinePlanner::buildLastPathBrake(
  const EgoFrenetState & ego, const f110_msgs::msg::WpntArray & path) const
{
  f110_msgs::msg::WpntArray braked;
  braked.header = path.header;
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(ego.speed) ||
    path.wpnts.empty())
  {
    return braked;
  }
  std::size_t start_index = 0U;
  double nearest_forward = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.wpnts.size(); ++i) {
    const double forward = forwardDistance(ego.s, path.wpnts[i].s_m);
    if (forward < nearest_forward) {
      nearest_forward = forward;
      start_index = i;
    }
  }
  if (nearest_forward > 0.5 * track_length_) {
    return braked;
  }
  const double deceleration = std::max(parameters_.safe_stop_deceleration_mps2, 0.1);
  const double speed = std::max(0.0, ego.speed);
  const double stop_at = nearest_forward + speed * speed / (2.0 * deceleration);
  for (std::size_t i = start_index; i < path.wpnts.size(); ++i) {
    const double forward_s = forwardDistance(ego.s, path.wpnts[i].s_m);
    if (forward_s > stop_at + kEpsilon) {
      break;
    }
    auto waypoint = path.wpnts[i];
    waypoint.id = static_cast<int32_t>(braked.wpnts.size());
    waypoint.vx_mps = std::min(
      std::max(0.0, waypoint.vx_mps),
      std::sqrt(2.0 * deceleration * std::max(0.0, stop_at - forward_s)));
    braked.wpnts.push_back(waypoint);
  }
  if (braked.wpnts.size() < 2U) {
    braked.wpnts.clear();
    return braked;
  }
  braked.wpnts.back().vx_mps = 0.0;
  updateGeometryAndAcceleration(braked);
  return braked;
}

RacelineSplineResult RacelineSplinePlanner::buildCommittedPathStop(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & committed_path,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  RacelineSplineResult result;
  result.kind = SplinePlanKind::kNoSafePath;
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(ego.d) ||
    !std::isfinite(ego.speed) || committed_path.wpnts.empty())
  {
    result.reason = "cannot build a committed-path stop from invalid inputs";
    return result;
  }

  std::size_t start_index = 0U;
  double nearest_forward = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < committed_path.wpnts.size(); ++i) {
    const double forward = forwardDistance(ego.s, committed_path.wpnts[i].s_m);
    if (forward < nearest_forward) {
      nearest_forward = forward;
      start_index = i;
    }
  }
  if (nearest_forward > 0.5 * track_length_) {
    result.reason = "committed path has no waypoint ahead for braking";
    return result;
  }

  const auto visible = expandVisibleObstacles(ego, obstacles);
  double first_collision_forward = std::numeric_limits<double>::infinity();
  int first_collision_id = -1;
  for (std::size_t i = start_index; i < committed_path.wpnts.size(); ++i) {
    const auto & waypoint = committed_path.wpnts[i];
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    const double clearance = parameters_.obstacleSafetyClearance(
      waypoint.vx_mps, waypoint.kappa_radpm);
    for (const auto & obstacle : visible) {
      if (forward_s >= obstacle.start && forward_s <= obstacle.end &&
        waypoint.d_m > obstacle.raw_d_right - clearance + kEpsilon &&
        waypoint.d_m < obstacle.raw_d_left + clearance - kEpsilon)
      {
        first_collision_forward = forward_s;
        first_collision_id = obstacle.id;
        break;
      }
    }
    if (std::isfinite(first_collision_forward)) {
      break;
    }
  }
  if (!std::isfinite(first_collision_forward)) {
    result.reason = "committed path has no obstacle collision before which to stop";
    return result;
  }

  const double stop_at = std::max(
    0.0, first_collision_forward - parameters_.safe_stop_buffer_m);
  result.path.header = committed_path.header;
  for (std::size_t i = start_index; i < committed_path.wpnts.size(); ++i) {
    const double forward_s = forwardDistance(ego.s, committed_path.wpnts[i].s_m);
    if (forward_s > stop_at + kEpsilon) {
      break;
    }
    auto waypoint = committed_path.wpnts[i];
    waypoint.id = static_cast<int32_t>(result.path.wpnts.size());
    result.path.wpnts.push_back(waypoint);
  }
  if (result.path.wpnts.size() < 2U) {
    result.path.wpnts.clear();
    result.reason = "committed path has no collision-free braking prefix";
    return result;
  }

  const double first_forward = forwardDistance(ego.s, result.path.wpnts.front().s_m);
  const double first_lateral_delta =
    std::abs(result.path.wpnts.front().d_m - ego.d);
  const bool same_s_lateral_jump =
    first_forward <= kEpsilon && first_lateral_delta > kEpsilon;
  const bool excessive_entry_slope =
    first_forward > kEpsilon &&
    first_lateral_delta / first_forward > parameters_.maximum_lateral_slope;
  if (same_s_lateral_jump || excessive_entry_slope) {
    result.path.wpnts.clear();
    result.reason = "committed braking prefix is discontinuous from the current ego d";
    return result;
  }

  result.path.wpnts.back().vx_mps = 0.0;
  for (std::size_t reverse = result.path.wpnts.size() - 1U; reverse > 0U; --reverse) {
    const std::size_t previous = reverse - 1U;
    const double distance = pointDistance(
      result.path.wpnts[previous], result.path.wpnts[reverse]);
    const double next_speed = std::max(0.0, result.path.wpnts[reverse].vx_mps);
    const double braking_speed = std::sqrt(
      next_speed * next_speed +
      2.0 * parameters_.safe_stop_deceleration_mps2 * std::max(0.0, distance));
    result.path.wpnts[previous].vx_mps = std::min(
      std::max(0.0, result.path.wpnts[previous].vx_mps), braking_speed);
  }
  updateGeometryAndAcceleration(result.path);

  std::string validation_reason;
  if (!validateCandidate(ego, result.path, visible, validation_reason, 0U, 2U)) {
    result.path.wpnts.clear();
    result.reason = "committed braking prefix rejected: " + validation_reason;
    return result;
  }
  result.kind = SplinePlanKind::kSafeStop;
  result.obstacle_id = first_collision_id;
  result.merge_s = result.path.wpnts.back().s_m;
  result.reason = "braking on the remaining committed geometry before a collision";
  return result;
}

void RacelineSplinePlanner::measureCandidate(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible,
  Candidate & candidate) const
{
  if (candidate.path.wpnts.empty()) {
    return;
  }

  double wall_clearance = std::numeric_limits<double>::infinity();
  // Ranking-side wall room. Unlike the legacy centerline headroom above, this is measured from the
  // same reference the obstacle term uses: the vehicle body plus the tracking-error tube. Without
  // it the two ranking terms are not commensurate -- the obstacle side already spends half width +
  // safety margin + tube while the wall side spent only wall_safety_margin_m -- and maximizing
  // their minimum biases every selection toward the wall by (obstacle clearance - wall margin)/2.
  double body_wall_clearance = std::numeric_limits<double>::infinity();
  double obstacle_clearance = std::numeric_limits<double>::infinity();
  double peak_curvature = 0.0;
  double peak_curvature_rate = 0.0;
  double velocity_loss_sum = 0.0;
  double deviation_sum = 0.0;
  bool measured_obstacle = false;
  double previous_forward = 0.0;
  double previous_curvature = candidate.path.wpnts.front().kappa_radpm;
  FootprintTrackBoundSample minimum_footprint;

  for (std::size_t i = 0; i < candidate.path.wpnts.size(); ++i) {
    const auto & waypoint = candidate.path.wpnts[i];
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    const auto & reference = reference_.wpnts[nearestReferenceIndex(waypoint.s_m)];
    const double reserve = parameters_.trackBoundaryReserve(
      waypoint.vx_mps, waypoint.kappa_radpm);
    const double left_width = reference.d_left > 0.05 ?
      reference.d_left : parameters_.fallback_track_half_width_m;
    const double right_width = reference.d_right > 0.05 ?
      reference.d_right : parameters_.fallback_track_half_width_m;
    wall_clearance = std::min(
      wall_clearance,
      std::min(left_width - reserve - waypoint.d_m,
      waypoint.d_m + right_width - reserve));
    // Track-bound VALIDATION still spends wall_safety_margin_m exactly once; this extra body and
    // tube allowance is a ranking quantity only and never rejects a candidate.
    const double body_reserve = reserve + parameters_.vehicle_half_width_m +
      parameters_.avoidanceTrackingErrorReserve(waypoint.vx_mps, waypoint.kappa_radpm);
    body_wall_clearance = std::min(
      body_wall_clearance,
      std::min(left_width - body_reserve - waypoint.d_m,
      waypoint.d_m + right_width - body_reserve));
    const auto footprint = measureFootprintTrackBound(waypoint, i);
    if (footprint.footprint_clearance_m < minimum_footprint.footprint_clearance_m) {
      minimum_footprint = footprint;
    }

    for (const auto & obstacle : visible) {
      if (forward_s + kEpsilon < obstacle.start || forward_s > obstacle.end + kEpsilon) {
        continue;
      }
      measured_obstacle = true;
      const double clearance = parameters_.obstacleSafetyClearance(
        waypoint.vx_mps, waypoint.kappa_radpm);
      const double test_right = obstacle.raw_d_right - clearance;
      const double test_left = obstacle.raw_d_left + clearance;
      double signed_clearance = 0.0;
      if (waypoint.d_m >= test_left) {
        signed_clearance = waypoint.d_m - test_left;
      } else if (waypoint.d_m <= test_right) {
        signed_clearance = test_right - waypoint.d_m;
      } else {
        signed_clearance = -std::min(
          waypoint.d_m - test_right, test_left - waypoint.d_m);
      }
      obstacle_clearance = std::min(obstacle_clearance, signed_clearance);
    }

    peak_curvature = std::max(peak_curvature, std::abs(waypoint.kappa_radpm));
    if (i > 0U) {
      const double ds = forward_s - previous_forward;
      if (ds > kEpsilon) {
        peak_curvature_rate = std::max(
          peak_curvature_rate,
          std::abs(waypoint.kappa_radpm - previous_curvature) / ds);
      }
    }
    const double reference_speed = std::max(0.0, reference.vx_mps);
    if (reference_speed > kEpsilon) {
      velocity_loss_sum += std::max(
        0.0, reference_speed - std::max(0.0, waypoint.vx_mps)) / reference_speed;
    }
    deviation_sum += std::abs(waypoint.d_m);
    previous_forward = forward_s;
    previous_curvature = waypoint.kappa_radpm;
  }

  const double normalization_distance = std::max(
    kEpsilon, parameters_.maximum_target_offset_m);
  if (!measured_obstacle) {
    obstacle_clearance = normalization_distance;
  }
  const double wall_slack = body_wall_clearance / normalization_distance;
  const double obstacle_slack = obstacle_clearance / normalization_distance;
  const double curvature_slack =
    (parameters_.maximum_curvature_radpm - peak_curvature) /
    std::max(kEpsilon, parameters_.maximum_curvature_radpm);
  const double curvature_rate_slack =
    (parameters_.maximum_curvature_rate_radpm2 - peak_curvature_rate) /
    std::max(kEpsilon, parameters_.maximum_curvature_rate_radpm2);

  candidate.centerline_wall_clearance_m = wall_clearance;
  candidate.rectangular_footprint_wall_clearance_m =
    minimum_footprint.footprint_clearance_m;
  candidate.footprint_invalid = minimum_footprint.invalid;
  candidate.footprint_violation_side =
    minimum_footprint.invalid ? minimum_footprint.minimum_side : std::string();
  candidate.footprint_violation_waypoint_index = minimum_footprint.waypoint_index;
  candidate.footprint_violation_s_m = minimum_footprint.waypoint_s_m;
  candidate.footprint_violation_x_m = minimum_footprint.waypoint_x_m;
  candidate.footprint_violation_y_m = minimum_footprint.waypoint_y_m;
  candidate.footprint_violation_yaw_rad = minimum_footprint.waypoint_yaw_rad;
  candidate.footprint_heading_relative_to_reference_rad =
    minimum_footprint.heading_relative_to_reference_rad;
  candidate.wallward_corner_protrusion_m =
    minimum_footprint.wallward_corner_protrusion_m;
  // Ranking compares body-referenced room on both sides (see body_wall_clearance above). The
  // rectangular footprint stays an additional hard gate, not a ranking weight or objective, and
  // centerline_wall_clearance_m above keeps the legacy headroom for audit continuity.
  candidate.wall_clearance_m = body_wall_clearance;
  candidate.obstacle_clearance_m = obstacle_clearance;
  candidate.peak_curvature_radpm = peak_curvature;
  candidate.peak_curvature_rate_radpm2 = peak_curvature_rate;
  candidate.velocity_loss = velocity_loss_sum /
    static_cast<double>(candidate.path.wpnts.size());
  candidate.global_path_deviation_m = deviation_sum /
    static_cast<double>(candidate.path.wpnts.size());
  candidate.minimum_normalized_safety_slack = std::min(
    {wall_slack, obstacle_slack, curvature_slack, curvature_rate_slack});
}

RacelineSplinePlanner::FootprintTrackBoundSample
RacelineSplinePlanner::measureFootprintTrackBound(
  const f110_msgs::msg::Wpnt & waypoint,
  std::size_t waypoint_index) const
{
  FootprintTrackBoundSample result;
  result.waypoint_index = waypoint_index;
  result.waypoint_s_m = waypoint.s_m;
  result.waypoint_x_m = waypoint.x_m;
  result.waypoint_y_m = waypoint.y_m;
  result.waypoint_yaw_rad = waypoint.psi_rad;

  const auto & reference = reference_.wpnts[nearestReferenceIndex(waypoint.s_m)];
  const double reserve = parameters_.trackBoundaryReserve(
    waypoint.vx_mps, waypoint.kappa_radpm);
  const double left_track_width = reference.d_left > 0.05 ?
    reference.d_left : parameters_.fallback_track_half_width_m;
  const double right_track_width = reference.d_right > 0.05 ?
    reference.d_right : parameters_.fallback_track_half_width_m;
  result.centerline_clearance_m = std::min(
    left_track_width - reserve - waypoint.d_m,
    waypoint.d_m + right_track_width - reserve);

  // Global d_left/d_right are distances from the reference to the physical track boundaries.
  // Project every rotated rectangle corner onto the matching local reference segment instead of
  // subtracting a heading-independent half-width from the candidate centre. The s-window keeps a
  // nearby parallel branch of a snake-shaped track from becoming the corner's reference. The
  // existing wall reserve is applied once; simulator TTC/noise guards intentionally do not enter.
  const double half_length = 0.5 * parameters_.vehicle_length_m;
  const double half_width = parameters_.vehicle_half_width_m;
  const double footprint_radius = std::hypot(half_length, half_width);
  const double candidate_cos = std::cos(waypoint.psi_rad);
  const double candidate_sin = std::sin(waypoint.psi_rad);
  const std::size_t anchor_index = nearestReferenceIndex(waypoint.s_m);
  std::vector<std::size_t> local_reference_segments;
  local_reference_segments.reserve(10U);
  const auto append_segment = [&](std::size_t index) {
      if (std::find(
          local_reference_segments.begin(), local_reference_segments.end(), index) ==
        local_reference_segments.end())
      {
        local_reference_segments.push_back(index);
      }
    };
  double covered_forward_s = 0.0;
  std::size_t forward_index = anchor_index;
  for (std::size_t count = 0; count < reference_.wpnts.size(); ++count) {
    if (covered_forward_s > footprint_radius + kEpsilon) {
      break;
    }
    append_segment(forward_index);
    const std::size_t next = (forward_index + 1U) % reference_.wpnts.size();
    covered_forward_s += forwardDistance(
      reference_.wpnts[forward_index].s_m, reference_.wpnts[next].s_m);
    forward_index = next;
  }
  double covered_backward_s = 0.0;
  std::size_t backward_end_index = anchor_index;
  for (std::size_t count = 0; count < reference_.wpnts.size(); ++count) {
    if (covered_backward_s > footprint_radius + kEpsilon) {
      break;
    }
    const std::size_t previous = backward_end_index == 0U ?
      reference_.wpnts.size() - 1U : backward_end_index - 1U;
    append_segment(previous);
    covered_backward_s += forwardDistance(
      reference_.wpnts[previous].s_m, reference_.wpnts[backward_end_index].s_m);
    backward_end_index = previous;
  }
  struct CornerResult
  {
    double clearance_m{std::numeric_limits<double>::infinity()};
    double heading_relative_rad{0.0};
    double wallward_protrusion_m{0.0};
    bool left_is_minimum{true};
  };
  const auto project_corner = [&](double longitudinal, double lateral) {
      const double corner_x = waypoint.x_m + longitudinal * candidate_cos -
        lateral * candidate_sin;
      const double corner_y = waypoint.y_m + longitudinal * candidate_sin +
        lateral * candidate_cos;
      double nearest_distance_squared = std::numeric_limits<double>::infinity();
      CornerResult corner;
      for (const std::size_t index : local_reference_segments) {
        const std::size_t next = (index + 1U) % reference_.wpnts.size();
        const auto & first = reference_.wpnts[index];
        const auto & second = reference_.wpnts[next];
        const double segment_x = second.x_m - first.x_m;
        const double segment_y = second.y_m - first.y_m;
        const double segment_length_squared =
          segment_x * segment_x + segment_y * segment_y;
        if (!(segment_length_squared > kEpsilon)) {
          continue;
        }
        const double ratio = std::clamp(
          ((corner_x - first.x_m) * segment_x +
          (corner_y - first.y_m) * segment_y) / segment_length_squared,
          0.0, 1.0);
        const double projected_x = first.x_m + ratio * segment_x;
        const double projected_y = first.y_m + ratio * segment_y;
        const double delta_x = corner_x - projected_x;
        const double delta_y = corner_y - projected_y;
        const double distance_squared = delta_x * delta_x + delta_y * delta_y;
        if (!(distance_squared < nearest_distance_squared)) {
          continue;
        }
        nearest_distance_squared = distance_squared;
        const double segment_yaw = std::atan2(segment_y, segment_x);
        const double segment_sin = std::sin(segment_yaw);
        const double segment_cos = std::cos(segment_yaw);
        const double corner_d = -delta_x * segment_sin + delta_y * segment_cos;
        const double center_d =
          -(waypoint.x_m - projected_x) * segment_sin +
          (waypoint.y_m - projected_y) * segment_cos;
        const double interpolated_left =
          first.d_left + ratio * (second.d_left - first.d_left);
        const double interpolated_right =
          first.d_right + ratio * (second.d_right - first.d_right);
        const double left_width = interpolated_left > 0.05 ?
          interpolated_left : parameters_.fallback_track_half_width_m;
        const double right_width = interpolated_right > 0.05 ?
          interpolated_right : parameters_.fallback_track_half_width_m;
        const double left_clearance = left_width - reserve - corner_d;
        const double right_clearance = corner_d + right_width - reserve;
        corner.left_is_minimum = left_clearance <= right_clearance;
        corner.clearance_m = std::min(left_clearance, right_clearance);
        corner.heading_relative_rad = normalizeAngle(waypoint.psi_rad - segment_yaw);
        const double wallward_extent = corner.left_is_minimum ?
          corner_d - center_d : center_d - corner_d;
        corner.wallward_protrusion_m = std::max(
          0.0, wallward_extent - half_width);
      }
      return corner;
    };
  const std::array<CornerResult, 4> corners{
    project_corner(half_length, half_width),
    project_corner(half_length, -half_width),
    project_corner(-half_length, half_width),
    project_corner(-half_length, -half_width)};
  const auto minimum = std::min_element(
    corners.begin(), corners.end(),
    [](const CornerResult & first, const CornerResult & second) {
      return first.clearance_m < second.clearance_m;
    });
  result.minimum_side = minimum->left_is_minimum ? "left" : "right";
  result.footprint_clearance_m = minimum->clearance_m;
  result.invalid = result.footprint_clearance_m < -kEpsilon;
  result.heading_relative_to_reference_rad = minimum->heading_relative_rad;
  result.wallward_corner_protrusion_m = minimum->wallward_protrusion_m;
  return result;
}

void RacelineSplinePlanner::applyAvoidanceVelocityLimit(
  f110_msgs::msg::WpntArray & path,
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible) const
{
  for (auto & waypoint : path.wpnts) {
    double speed = parameters_.limitedAvoidanceSpeed(
      std::max(0.0, waypoint.vx_mps), waypoint.kappa_radpm);

    // Lateral room left over between this waypoint and the face of every obstacle it is passing.
    // The hard validator spends exactly obstacleBaseClearance() + reserve here, so the reserve
    // this waypoint may afford is whatever remains once the base clearance is paid.
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    double admissible_reserve = std::numeric_limits<double>::infinity();
    for (const auto & obstacle : visible) {
      if (forward_s < obstacle.start || forward_s > obstacle.end) {
        continue;
      }
      const double side_room = (waypoint.d_m <= obstacle.raw_d_right) ?
        obstacle.raw_d_right - waypoint.d_m :
        (waypoint.d_m >= obstacle.raw_d_left ? waypoint.d_m - obstacle.raw_d_left : 0.0);
      admissible_reserve = std::min(
        admissible_reserve, side_room - parameters_.obstacleBaseClearance());
    }
    speed = parameters_.gapLimitedAvoidanceSpeed(
      speed, waypoint.kappa_radpm, admissible_reserve);

    // The reserve shrinks with speed, so the curvature cap has to be re-imposed on the reduced
    // value; it can only lower the speed further, never raise it.
    waypoint.vx_mps = parameters_.limitedAvoidanceSpeed(speed, waypoint.kappa_radpm);
  }

  // 접근 실현성 후방 제동 램프(2026-08-14 실차): 위 캡들은 장애물 스팬 안에서만 속도를
  // 낮추므로, 접근 구간은 프로파일 속도 그대로다가 스팬 경계에서 속도가 계단으로
  // 떨어진다. 실차에서는 그 계단이 서비스 브레이크 포화 → 마찰 한계 초과 슬립 →
  // 조향 상실로 이어졌다. 스팬 시작 시점의 계획 속도에서 approach_feasibility_decel_mps2로
  // 거꾸로 올라가는 제동 프로파일을 접근 구간에 씌워(낮추기만 한다) 제동이 스팬 훨씬
  // 전에 완만하게 시작되게 한다. 스팬 내부 속도는 건드리지 않는다.
  const double approach_decel = parameters_.approach_feasibility_decel_mps2;
  if (!(approach_decel > 0.0) || path.wpnts.empty()) {
    return;
  }
  // 🔴 램프는 **스팬마다** 건다 (2026-08-16). 예전에는 가장 가까운 스팬 하나만
  // (min(obstacle.start)) 대상으로 삼아서, 경로가 장애물 스팬을 두 개 이상 지나면 두 번째
  // 스팬 앞에는 램프가 전혀 없었다. 캡은 스팬 안에서만 속도를 낮추므로 그 경계에 계단이
  // 그대로 남는다 — 2026-08-16 백에서 s=39.67 vx=4.62 → s=39.92 vx=1.00, 즉 0.25 m 만에
  // 3.6 m/s를 요구했다(decel 2.0으로는 5.0 m가 필요). 이 계단이 실차에서 브레이크 포화 →
  // 마찰 한계 초과 → 조향 상실로 이어진 형태이고, 이 램프는 애초에 그것 때문에 들어갔다.
  //
  // 램프는 낮추기만 하므로 여러 스팬의 프로파일을 waypoint별 min으로 합성해도 정의가
  // 깨지지 않는다. 목표 속도는 캡이 모두 반영된 프로파일에서 먼저 모아두고(램프끼리 서로의
  // 목표를 갉아먹어 순서 의존이 생기지 않도록) 그 다음에 일괄 적용한다.
  struct ApproachTarget
  {
    double span_start{0.0};
    double target_speed{0.0};
  };
  std::vector<ApproachTarget> targets;
  targets.reserve(visible.size());
  for (const auto & obstacle : visible) {
    const double span_start = obstacle.start;
    if (!(span_start > kEpsilon)) {
      continue;   // 스팬이 자차에 붙어 있음(정지 탈출 등) — 접근 구간이 없다.
    }
    for (const auto & waypoint : path.wpnts) {
      if (forwardDistance(ego.s, waypoint.s_m) >= span_start) {
        targets.push_back({span_start, std::max(0.0, waypoint.vx_mps)});
        break;   // 경로가 스팬까지 안 이어지면 대상에서 빠진다.
      }
    }
  }
  if (targets.empty()) {
    return;
  }
  for (auto & waypoint : path.wpnts) {
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    for (const auto & target : targets) {
      if (forward_s >= target.span_start) {
        continue;
      }
      const double braking_speed = std::sqrt(
        target.target_speed * target.target_speed +
        2.0 * approach_decel * (target.span_start - forward_s));
      waypoint.vx_mps = std::min(std::max(0.0, waypoint.vx_mps), braking_speed);
    }
  }
}

void RacelineSplinePlanner::updateGeometryAndAcceleration(
  f110_msgs::msg::WpntArray & path) const
{
  auto & waypoints = path.wpnts;
  if (waypoints.size() < 2U) {
    return;
  }

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const std::size_t previous = (i == 0U) ? 0U : i - 1U;
    const std::size_t next = std::min(i + 1U, waypoints.size() - 1U);
    const double dx = waypoints[next].x_m - waypoints[previous].x_m;
    const double dy = waypoints[next].y_m - waypoints[previous].y_m;
    if (std::hypot(dx, dy) > kEpsilon) {
      waypoints[i].psi_rad = std::atan2(dy, dx);
    }
  }

  for (std::size_t i = 1; i + 1U < waypoints.size(); ++i) {
    const auto & first = waypoints[i - 1U];
    const auto & middle = waypoints[i];
    const auto & last = waypoints[i + 1U];
    const double a = pointDistance(first, middle);
    const double b = pointDistance(middle, last);
    const double c = pointDistance(first, last);
    const double cross =
      (middle.x_m - first.x_m) * (last.y_m - first.y_m) -
      (middle.y_m - first.y_m) * (last.x_m - first.x_m);
    const double denominator = a * b * c;
    waypoints[i].kappa_radpm = (denominator > kEpsilon) ? 2.0 * cross / denominator : 0.0;
  }
  waypoints.front().kappa_radpm = waypoints[1].kappa_radpm;
  waypoints.back().kappa_radpm = waypoints[waypoints.size() - 2U].kappa_radpm;

  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    const double distance = pointDistance(waypoints[i - 1U], waypoints[i]);
    if (distance > kEpsilon) {
      waypoints[i - 1U].ax_mps2 =
        (waypoints[i].vx_mps * waypoints[i].vx_mps -
        waypoints[i - 1U].vx_mps * waypoints[i - 1U].vx_mps) / (2.0 * distance);
    }
  }
  waypoints.back().ax_mps2 = 0.0;
}

bool RacelineSplinePlanner::validateCandidate(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<ExpandedObstacle> & visible,
  std::string & reason,
  std::size_t start_index,
  std::size_t minimum_points,
  PathValidationFailure * failure,
  const std::optional<double> & maximum_collision_forward_m,
  double obstacle_reserve_scale) const
{
  if (failure != nullptr) {
    *failure = PathValidationFailure();
  }
  const auto reject = [&reason, failure](
    PathValidationFailureKind kind,
    const std::string & message,
    std::size_t waypoint_index = std::numeric_limits<std::size_t>::max(),
    const f110_msgs::msg::Wpnt * waypoint = nullptr,
    const ExpandedObstacle * obstacle = nullptr,
    double obstacle_s_start = std::numeric_limits<double>::quiet_NaN(),
    double obstacle_s_end = std::numeric_limits<double>::quiet_NaN(),
    double obstacle_test_d_right = std::numeric_limits<double>::quiet_NaN(),
    double obstacle_test_d_left = std::numeric_limits<double>::quiet_NaN(),
    double obstacle_clearance = std::numeric_limits<double>::quiet_NaN())
    {
      reason = message;
      if (failure != nullptr) {
        failure->kind = kind;
        failure->reason = message;
        failure->waypoint_index = waypoint_index;
        if (waypoint != nullptr) {
          failure->waypoint_s = waypoint->s_m;
          failure->waypoint_d = waypoint->d_m;
        }
        if (obstacle != nullptr) {
          failure->obstacle_id = obstacle->id;
          failure->obstacle_s_start = obstacle_s_start;
          failure->obstacle_s_end = obstacle_s_end;
          failure->obstacle_source_d_right = obstacle->raw_d_right;
          failure->obstacle_source_d_left = obstacle->raw_d_left;
          failure->obstacle_test_d_right = obstacle_test_d_right;
          failure->obstacle_test_d_left = obstacle_test_d_left;
          failure->obstacle_clearance = obstacle_clearance;
        }
      }
      return false;
    };
  if (start_index >= path.wpnts.size()) {
    return reject(
      PathValidationFailureKind::kNoForwardPath,
      "path has no waypoint ahead of ego");
  }
  if (minimum_points == 0U) {
    minimum_points = static_cast<std::size_t>(parameters_.minimum_path_points);
  }
  if (path.wpnts.size() - start_index < minimum_points) {
    return reject(
      PathValidationFailureKind::kNoForwardPath,
      "path does not meet minimum_path_points");
  }
  double previous_d = path.wpnts[start_index].d_m;
  double previous_s = 0.0;
  double previous_curvature = path.wpnts[start_index].kappa_radpm;
  for (std::size_t i = start_index; i < path.wpnts.size(); ++i) {
    const auto & waypoint = path.wpnts[i];
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    const std::size_t reference_index = nearestReferenceIndex(waypoint.s_m);
    const auto & reference = reference_.wpnts[reference_index];
    const double center_boundary_clearance = parameters_.trackBoundaryReserve(
      waypoint.vx_mps, waypoint.kappa_radpm);
    const double left_width = reference.d_left > 0.05 ?
      reference.d_left : parameters_.fallback_track_half_width_m;
    const double right_width = reference.d_right > 0.05 ?
      reference.d_right : parameters_.fallback_track_half_width_m;
    if (waypoint.d_m > left_width - center_boundary_clearance + 1.0e-6 ||
      waypoint.d_m < -right_width + center_boundary_clearance - 1.0e-6)
    {
      return reject(
        PathValidationFailureKind::kTrackBoundary,
        "d-offset leaves the global waypoint track bounds", i, &waypoint);
    }
    const auto footprint = measureFootprintTrackBound(waypoint, i);
    if (footprint.invalid) {
      const bool rejected = reject(
        PathValidationFailureKind::kTrackBoundary,
        "footprint_track_bound", i, &waypoint);
      if (failure != nullptr) {
        failure->centerline_wall_clearance = footprint.centerline_clearance_m;
        failure->rectangular_footprint_wall_clearance = footprint.footprint_clearance_m;
        failure->footprint_violation_side = footprint.minimum_side;
        failure->waypoint_x = footprint.waypoint_x_m;
        failure->waypoint_y = footprint.waypoint_y_m;
        failure->waypoint_yaw = footprint.waypoint_yaw_rad;
        failure->heading_relative_to_reference =
          footprint.heading_relative_to_reference_rad;
        failure->wallward_corner_protrusion =
          footprint.wallward_corner_protrusion_m;
      }
      return rejected;
    }
    for (const auto & obstacle : visible) {
      const double obstacle_clearance = parameters_.obstacleSafetyClearance(
        waypoint.vx_mps, waypoint.kappa_radpm, obstacle_reserve_scale);
      const double obstacle_test_d_right = obstacle.raw_d_right - obstacle_clearance;
      const double obstacle_test_d_left = obstacle.raw_d_left + obstacle_clearance;
      if ((!maximum_collision_forward_m.has_value() ||
        forward_s <= maximum_collision_forward_m.value() + kEpsilon) &&
        forward_s >= obstacle.start && forward_s <= obstacle.end &&
        waypoint.d_m > obstacle_test_d_right + 1.0e-6 &&
        waypoint.d_m < obstacle_test_d_left - 1.0e-6)
      {
        return reject(
          PathValidationFailureKind::kObstacleCollision,
          "d-offset intersects an inflated static-obstacle box", i, &waypoint, &obstacle,
          wrapS(ego.s + obstacle.start), wrapS(ego.s + obstacle.end),
          obstacle_test_d_right, obstacle_test_d_left, obstacle_clearance);
      }
    }
    if (i > start_index) {
      const double ds = forward_s - previous_s;
      if (!(ds > kEpsilon)) {
        return reject(
          PathValidationFailureKind::kGeometry,
          "candidate no longer follows increasing global race-line order", i, &waypoint);
      }
      const double slope = std::abs(waypoint.d_m - previous_d) / ds;
      if (slope > parameters_.maximum_lateral_slope) {
        return reject(
          PathValidationFailureKind::kGeometry,
          "quintic d-offset exceeds maximum_lateral_slope", i, &waypoint);
      }
      const double curvature_rate =
        std::abs(waypoint.kappa_radpm - previous_curvature) / ds;
      if (curvature_rate > parameters_.maximum_curvature_rate_radpm2) {
        return reject(
          PathValidationFailureKind::kGeometry,
          "shifted race line exceeds maximum_curvature_rate_radpm2", i, &waypoint);
      }
    }
    if (std::abs(waypoint.kappa_radpm) > parameters_.maximum_curvature_radpm) {
      return reject(
        PathValidationFailureKind::kGeometry,
        "shifted race line exceeds maximum_curvature_radpm", i, &waypoint);
    }
    previous_d = waypoint.d_m;
    previous_s = forward_s;
    previous_curvature = waypoint.kappa_radpm;
  }
  return true;
}

bool RacelineSplinePlanner::validatePath(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  std::string * error,
  PathValidationFailure * failure,
  const std::optional<double> & maximum_collision_forward_m,
  double obstacle_reserve_scale) const
{
  if (failure != nullptr) {
    *failure = PathValidationFailure();
  }
  auto reject = [error, failure](
    PathValidationFailureKind kind,
    const std::string & reason)
    {
      if (error != nullptr) {
        *error = reason;
      }
      if (failure != nullptr) {
        failure->kind = kind;
        failure->reason = reason;
      }
      return false;
    };
  if (!ready()) {
    return reject(
      PathValidationFailureKind::kInput,
      "global race-line reference is not ready");
  }
  if (!std::isfinite(ego.s) || !std::isfinite(ego.d) || !std::isfinite(ego.speed)) {
    return reject(PathValidationFailureKind::kInput, "ego Frenet state is non-finite");
  }
  if (path.wpnts.empty()) {
    return reject(PathValidationFailureKind::kInput, "path is empty");
  }
  if (maximum_collision_forward_m.has_value() &&
    (!std::isfinite(maximum_collision_forward_m.value()) ||
    maximum_collision_forward_m.value() < 0.0))
  {
    return reject(
      PathValidationFailureKind::kInput,
      "maximum collision-forward distance is invalid");
  }

  std::size_t start_index = 0U;
  double nearest_forward = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.wpnts.size(); ++i) {
    const double forward = forwardDistance(ego.s, path.wpnts[i].s_m);
    if (forward < nearest_forward) {
      nearest_forward = forward;
      start_index = i;
    }
  }
  if (nearest_forward > 0.5 * track_length_) {
    return reject(
      PathValidationFailureKind::kNoForwardPath,
      "committed path has no remaining waypoint ahead of ego");
  }

  const auto visible = expandVisibleObstacles(ego, obstacles);
  std::string reason;
  if (!validateCandidate(
      ego, path, visible, reason, start_index, 1U, failure,
      maximum_collision_forward_m, obstacle_reserve_scale))
  {
    if (error != nullptr) {
      *error = reason;
    }
    return false;
  }
  return true;
}

std::size_t RacelineSplinePlanner::generateP3Candidates(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const std::vector<ExpandedObstacle> & visible,
  const std::optional<bool> & preferred_left,
  bool allow_side_switch,
  bool stop_on_first_feasible,
  std::vector<Candidate> & candidates,
  std::string & reason) const
{
  // 이 패키지의 유일한 회피 후보 생성기. 후보 생성은 여기 한 곳에만 두어야 한다 —
  // 안전정지 탈출 검증(anyFeasibleCandidateFrom)이 같은 함수를 쓰므로 "정지점에서 회피
  // 가능"이라는 판정과 실제 재계획이 어긋날 수 없다. 두 번째 사본이 생기면 그 덫이 돌아온다.
  const P3ShadowResult p3 = evaluateP3Shadow(ego, obstacles, 0, 0U, 0U, "PLAN");
  if (!p3.invoked) {
    reason = p3.failure_classification.empty() ? "P3 not invoked" : p3.failure_classification;
    return 0U;
  }
  std::size_t feasible = 0U;
  std::size_t considered = 0U;
  for (const auto & trace : p3.candidates) {
    // 측 잠금: 커밋된 측이 있고 전환이 금지된 상태면 그 측 후보만 본다.
    if (preferred_left.has_value() && !allow_side_switch &&
      trace.go_left != preferred_left.value())
    {
      continue;
    }
    if (trace.path.wpnts.size() < static_cast<std::size_t>(parameters_.minimum_path_points)) {
      continue;
    }
    ++considered;
    Candidate candidate;
    candidate.go_left = trace.go_left;
    candidate.target_d = trace.d_target;
    candidate.path = trace.path;
    candidate.audit_index = trace.generation_index;
    candidate.exit_reaches_next_obstacle = trace.exit_reaches_next_obstacle;
    candidate.entry_transition_scale = trace.entry_scale;
    candidate.exit_transition_scale = trace.exit_scale;
    candidate.effective_exit_transition_scale = trace.exit_scale;
    // merge_s는 발행 세그먼트 끝이 아니라 d-offset이 실제로 d=0으로 복귀하는 지점이다.
    // 뒤에서부터 |d|가 tolerance를 넘는 마지막 점을 찾고 그 다음 점을 합류점으로 쓴다.
    std::size_t last_offset = 0U;
    for (std::size_t i = 0U; i < candidate.path.wpnts.size(); ++i) {
      if (std::abs(candidate.path.wpnts[i].d_m) > 1.0e-3) {
        last_offset = i;
      }
    }
    const std::size_t merge_index =
      std::min(last_offset + 1U, candidate.path.wpnts.size() - 1U);
    candidate.merge_s = candidate.path.wpnts[merge_index].s_m;
    // P3 trace의 지표를 그대로 믿지 않고 P0와 동일한 안전 계층으로 재측정한다. 순위·감사·
    // 하류 필드가 전부 같은 출처에서 나오도록 하기 위함이다.
    //
    // 장애물 충돌 검사는 이번 기동이 책임지는 범위(클러스터 끝 + post_merge_lookahead)
    // 까지만 본다 (AGENTS.md: "A post-merge controller-tail obstacle must not make the
    // current maneuver fail" — 커밋 경로 재검증이 merge horizon으로 지키는 원칙과 동일).
    // 그 너머의 다음 장애물은 연쇄 기동과 안전정지 사다리의 몫이다: long exit은 설계상
    // 뒤따르는 장애물 위로 오프셋을 끌고 가고, 연쇄 재계획이 도착 전에 경로를 교체한다
    // (AGENTS의 maximum_exit_length 비활성 사유 참고). 이 horizon이 없으면 라인 위
    // 장애물(예: map s=40.6)이 lookahead 안에 있는 동안 앞선 장애물(s=31.6)의 모든 회피
    // 후보가 12 m 밖 꼬리 충돌로 전멸해 kNoSafePath→영구 크립이 된다 (2026-08-15 run18
    // 실측). 트랙 경계·기하 검사는 horizon과 무관하게 경로 전체에 적용된다.
    measureCandidate(ego, visible, candidate);
    std::string validation_reason;
    const std::optional<double> collision_horizon =
      std::isfinite(p3.cluster_end_forward_m) ?
      std::optional<double>(
      p3.cluster_end_forward_m + parameters_.post_merge_lookahead_m) :
      std::nullopt;
    candidate.valid = validateCandidate(
      ego, candidate.path, visible, validation_reason, 0U, 0U, nullptr, collision_horizon);
    candidate.reason = candidate.valid ? std::string() :
      (validation_reason.empty() ? trace.rejection_reason : validation_reason);
    if (candidate.valid) {
      ++feasible;
    }
    const bool candidate_valid = candidate.valid;
    candidates.push_back(std::move(candidate));
    if (candidate_valid && stop_on_first_feasible) {
      return feasible;
    }
  }
  if (feasible == 0U) {
    reason = considered == 0U ?
      (p3.failure_classification.empty() ? "no P3 candidate on the permitted side" :
      p3.failure_classification) :
      std::to_string(considered) + " P3 candidates rejected by the exact validator";
  } else {
    reason = std::to_string(feasible) + "/" + std::to_string(considered) +
      " P3 candidates feasible";
  }
  return feasible;
}

bool RacelineSplinePlanner::anyFeasibleCandidateFrom(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const std::vector<ExpandedObstacle> & visible,
  const std::vector<ExpandedObstacle> & cluster) const
{
  if (cluster.empty()) {
    return true;   // 막는 것이 없으면 굳이 정지할 이유도 없다
  }
  std::vector<Candidate> candidates;
  std::string reason;
  // plan()과 반드시 같은 생성기를 쓴다(위 generateP3Candidates 주석 참고).
  return generateP3Candidates(
    ego, obstacles, visible, std::nullopt, true, true, candidates, reason) > 0U;
}

void RacelineSplinePlanner::densifyPath(
  f110_msgs::msg::WpntArray & path, std::size_t minimum_points) const
{
  if (path.wpnts.size() < 2U || path.wpnts.size() >= minimum_points) {
    return;
  }
  // 가장 긴 구간을 반복해서 이등분한다. d가 일정한 정지 prefix라 선형 보간으로 충분하고,
  // 곡률·가속도는 뒤에서 updateGeometryAndAcceleration이 다시 계산한다.
  while (path.wpnts.size() < minimum_points) {
    std::size_t longest = 0U;
    double longest_length = -1.0;
    for (std::size_t i = 0U; i + 1U < path.wpnts.size(); ++i) {
      const double dx = path.wpnts[i + 1U].x_m - path.wpnts[i].x_m;
      const double dy = path.wpnts[i + 1U].y_m - path.wpnts[i].y_m;
      const double length = std::hypot(dx, dy);
      if (length > longest_length) {
        longest_length = length;
        longest = i;
      }
    }
    if (!(longest_length > kEpsilon)) {
      break;      // 모든 구간이 이미 퇴화 — 더 쪼개도 의미가 없다
    }
    const auto & a = path.wpnts[longest];
    const auto & b = path.wpnts[longest + 1U];
    f110_msgs::msg::Wpnt mid = a;
    mid.x_m = 0.5 * (a.x_m + b.x_m);
    mid.y_m = 0.5 * (a.y_m + b.y_m);
    mid.d_m = 0.5 * (a.d_m + b.d_m);
    mid.d_left = 0.5 * (a.d_left + b.d_left);
    mid.d_right = 0.5 * (a.d_right + b.d_right);
    mid.vx_mps = 0.5 * (a.vx_mps + b.vx_mps);
    mid.s_m = wrapS(a.s_m + 0.5 * forwardDistance(a.s_m, b.s_m));
    mid.psi_rad = a.psi_rad + 0.5 * normalizeAngle(b.psi_rad - a.psi_rad);
    mid.kappa_radpm = 0.5 * (a.kappa_radpm + b.kappa_radpm);
    path.wpnts.insert(path.wpnts.begin() + static_cast<std::ptrdiff_t>(longest) + 1, mid);
  }
  for (std::size_t i = 0U; i < path.wpnts.size(); ++i) {
    path.wpnts[i].id = static_cast<int32_t>(i);
  }
}

RacelineSplineResult RacelineSplinePlanner::buildSafeStop(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible,
  const std::vector<ExpandedObstacle> & cluster,
  const std::vector<f110_msgs::msg::Obstacle> & raw_obstacles,
  const ExpandedObstacle & blocking) const
{
  (void)cluster;   // 탈출 검증은 정지점 기준으로 다시 확장한 cluster를 쓴다(아래 참고)
  RacelineSplineResult result;
  result.kind = SplinePlanKind::kNoSafePath;
  result.obstacle_id = blocking.id;
  double stop_at = std::max(0.0, blocking.start - parameters_.safe_stop_buffer_m);

  // 🔴 정지점 탈출 검증 (2026-08-14). 정지 자체보다 **어디에 서느냐**가 교착을 만든다.
  // 실차에서 safe_stop_buffer_m 1.20 m는 탈출 임계(2.0~2.5 m)보다 작아서, 정지하는
  // 순간 이미 회피 후보가 0개 생성되는 구역이었다(run_0101_090639: 장애물 1.6 m 앞에
  // 28.8초 정지, 좌 1.17 m/우 1.36 m로 횡공간은 충분했음). 정지점에서 v=0으로
  // 재계획이 되는지 먼저 묻고, 안 되면 뒤로 물린다.
  bool escape_verified = !parameters_.safe_stop_escape_check_enable;
  if (parameters_.safe_stop_escape_check_enable) {
    const double requested_stop_at = stop_at;
    const auto escapable_at = [&](double forward) {
        EgoFrenetState at_stop;
        at_stop.s = wrapS(ego.s + forward);
        at_stop.d = ego.d;
        at_stop.speed = 0.0;
        // ⚠️ ExpandedObstacle의 center/start/end는 **자차 상대거리**다
        //    (expandVisibleObstacles: center = forwardDistance(ego.s, obstacle.s_center)).
        //    그래서 자차 s만 정지점으로 옮기고 기존 visible/cluster를 재사용하면 장애물이
        //    정지점에서도 여전히 같은 거리에 있는 것으로 보여, 검증이 "현재 위치에서
        //    v=0으로 회피 가능한가"를 물을 뿐 정지점과 무관해진다(이분탐색도 무의미해진다).
        //    반드시 절대 s를 담은 원본으로 정지점 기준 재확장해야 한다.
        const auto at_stop_visible = expandVisibleObstacles(at_stop, raw_obstacles);
        const auto at_stop_cluster = nearestCluster(at_stop_visible);
        return anyFeasibleCandidateFrom(
          at_stop, raw_obstacles, at_stop_visible, at_stop_cluster);
      };

    // 탈출 가능성은 정지점을 **뒤로 물릴수록**(=forward가 작을수록) 단조 증가한다:
    // 장애물까지 남는 진입 거리가 그만큼 길어지기 때문이다. 그래서 선형 후퇴 대신
    // 이분탐색으로 "탈출 가능한 가장 늦은 정지점"을 찾는다. 선형 후퇴는 탈출이 아예
    // 불가능한 경우에 매 사이클 (후퇴횟수 × 2면 × 36후보)를 다 태워 실측 23.8 ms가
    // 나왔다(40 Hz 예산 25 ms를 젯슨에서 확실히 초과). 이분탐색은 흔한 경우 1회,
    // 가망 없는 경우 2회 탐침으로 끝난다.
    // ⚠️ 트랙 폭이 구간마다 달라 단조성이 국소적으로 깨질 수 있다. 그때 이분탐색은
    // 중간의 통과 가능 지점을 놓칠 수 있는데, 결과는 "원래 정지점 유지"라 보수적이다.
    if (escapable_at(stop_at)) {
      escape_verified = true;                      // 탐침 1회 — 정상 경로
    } else if (stop_at > kEpsilon && escapable_at(0.0)) {
      // 자차 자리에서는 탈출 가능 → 그 사이 어딘가가 경계다. 가장 늦은 탈출 가능점 탐색.
      escape_verified = true;
      double feasible = 0.0;                       // 탈출 가능이 확인된 값
      double infeasible = stop_at;                 // 탈출 불가가 확인된 값
      const int probes = std::clamp(parameters_.safe_stop_escape_max_retreats, 0, 12);
      const double resolution = std::max(kEpsilon, parameters_.safe_stop_escape_retreat_step_m);
      for (int i = 0; i < probes && (infeasible - feasible) > resolution; ++i) {
        const double middle = 0.5 * (feasible + infeasible);
        if (escapable_at(middle)) {
          feasible = middle;
        } else {
          infeasible = middle;
        }
      }
      stop_at = feasible;
    }
    // 어느 지점에서도 탈출이 안 되면 후퇴는 아무것도 사지 못한다. 축소된 stop_at을 그대로
    // 쓰면 (a) 필요보다 훨씬 일찍 서고 (b) 정지 prefix가 짧아져 경로가 통째로 무효화된다
    // (이 복원이 없을 때 kSafeStop이 kNoSafePath로 퇴화하는 것을 확인했다).
    if (!escape_verified) {
      stop_at = requested_stop_at;
    }
  }

  const std::size_t first_index = nextReferenceIndex(ego.s);
  result.path.header = reference_.header;
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const std::size_t index = (first_index + k) % reference_.wpnts.size();
    const double forward_s = forwardDistance(ego.s, reference_.wpnts[index].s_m);
    if (forward_s > stop_at + kEpsilon) {
      break;
    }
    const auto & global = reference_.wpnts[index];
    auto waypoint = global;
    waypoint.id = static_cast<int32_t>(result.path.wpnts.size());
    waypoint.d_m = ego.d;
    waypoint.x_m = global.x_m - ego.d * std::sin(global.psi_rad);
    waypoint.y_m = global.y_m + ego.d * std::cos(global.psi_rad);
    waypoint.vx_mps = std::min(
      std::max(0.0, waypoint.vx_mps),
      std::sqrt(
        2.0 * parameters_.safe_stop_deceleration_mps2 *
        std::max(0.0, stop_at - forward_s)));
    result.path.wpnts.push_back(waypoint);
  }
  if (result.path.wpnts.size() < 2U) {
    result.reason = "blocking obstacle is inside the safe-stop buffer";
    result.path.wpnts.clear();
    return result;
  }
  // 🔴 minimum_path_points를 안전정지 경로에도 적용한다 (2026-08-14). 이전에는 가드가
  // size()<2뿐이라 2점 경로 [v, 0]이 그대로 나갔는데, 제어기는 룩어헤드 지점의 속도를
  // 읽으므로 2점에서는 룩어헤드가 곧바로 끝점 0에 걸려 감속 프로파일을 통째로 건너뛰고
  // 즉시 0을 명령한다(실차 관측: /local_waypoints [1.08, 0.00] → /drive_autonomous 0.00).
  densifyPath(result.path, static_cast<std::size_t>(std::max(2, parameters_.minimum_path_points)));
  // 보간으로 생긴 점에도 같은 제동 프로파일을 다시 씌운다 — 선형 보간된 속도는
  // sqrt(2·a·거리) 곡선보다 항상 크거나 같아 낙관적이다.
  for (auto & waypoint : result.path.wpnts) {
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    waypoint.vx_mps = std::min(
      std::max(0.0, waypoint.vx_mps),
      std::sqrt(
        2.0 * parameters_.safe_stop_deceleration_mps2 *
        std::max(0.0, stop_at - forward_s)));
  }
  result.path.wpnts.back().vx_mps = 0.0;
  updateGeometryAndAcceleration(result.path);

  std::string validation_reason;
  if (!validateCandidate(ego, result.path, visible, validation_reason, 0U, 2U)) {
    result.path.wpnts.clear();
    result.reason = "safe-stop path rejected: " + validation_reason;
    return result;
  }
  result.kind = SplinePlanKind::kSafeStop;
  result.merge_s = result.path.wpnts.back().s_m;
  result.safe_stop_escape_verified = escape_verified;
  result.safe_stop_forward_m = stop_at;
  result.reason = escape_verified ?
    "both spline sides rejected; braking before the static obstacle" :
    // 여기까지 왔다면 자차 위치까지 물러나도 회피 후보가 하나도 생성되지 않는다. 전진
    // 계획으로는 풀 수 없는 상태(후진이 필요)이므로, 조용히 매달려 있지 말고 알린다.
    "both spline sides rejected; braking before the static obstacle; "
    "WARNING no escapable stop point exists — the car will not be able to resume from this "
    "stop by forward planning";
  return result;
}

RacelineSplineResult RacelineSplinePlanner::buildPreparationStop(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  RacelineSplineResult result;
  if (!ready()) {
    result.kind = SplinePlanKind::kNoSafePath;
    result.reason = "global race-line reference is not ready";
    return result;
  }
  if (!std::isfinite(ego.s) || !std::isfinite(ego.d) || !std::isfinite(ego.speed)) {
    result.kind = SplinePlanKind::kNoSafePath;
    result.reason = "ego Frenet state is non-finite";
    return result;
  }

  const auto visible = expandVisibleObstacles(ego, obstacles);
  const auto cluster = nearestCluster(visible);
  if (cluster.empty()) {
    result.kind = SplinePlanKind::kNoObstacle;
    result.reason = "no static obstacle blocks the global race line";
    return result;
  }

  // A margin-only cluster never requires stopping: the line itself is physically drivable, so
  // stabilization is spent holding the line at the capped speed instead of braking to a halt
  // (and instead of a zero-speed hold when the cluster is first seen inside the stop buffer).
  if (!clusterPhysicallyBlocksRaceline(cluster)) {
    auto slow_pass = buildMarginSlowPass(ego, cluster);
    if (slow_pass.kind == SplinePlanKind::kAvoidance) {
      slow_pass.reason =
        "margin-only cluster during initial stabilization; holding the race line at the "
        "margin_pass speed cap";
      return slow_pass;
    }
  }

  result = buildSafeStop(ego, visible, cluster, obstacles, cluster.front());
  result.obstacle_ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    result.obstacle_ids.push_back(obstacle.id);
  }
  if (result.kind == SplinePlanKind::kSafeStop) {
    result.kind = SplinePlanKind::kPreparation;
    result.reason =
      "waiting for initial obstacle-cluster stabilization; braking before commitment";
  }
  return result;
}

RacelineSplineResult RacelineSplinePlanner::plan(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const std::optional<bool> & preferred_left,
  bool allow_side_switch) const
{
  RacelineSplineResult result;
  if (!ready()) {
    result.kind = SplinePlanKind::kNoSafePath;
    result.reason = "global race-line reference is not ready";
    return result;
  }
  if (!std::isfinite(ego.s) || !std::isfinite(ego.d) || !std::isfinite(ego.speed)) {
    result.kind = SplinePlanKind::kNoSafePath;
    result.reason = "ego Frenet state is non-finite";
    return result;
  }

  const auto visible = expandVisibleObstacles(ego, obstacles);
  const auto cluster = nearestCluster(visible);
  if (cluster.empty()) {
    result.kind = SplinePlanKind::kNoObstacle;
    result.reason = "no static obstacle blocks the global race line";
    return result;
  }

  const bool outside_is_left = outsideIsLeft(ego, cluster);
  std::vector<Candidate> candidates;
  std::string left_reason = "not evaluated: side locked by active commitment";
  std::string right_reason = "not evaluated: side locked by active commitment";
  bool left_evaluated = false;
  bool right_evaluated = false;

  // 회피 후보 생성은 P3(analytic corridor) 한 곳뿐이다. P0 quintic 격자는 2026-08-15에
  // 제거됐다: 실차 시험에서 P0가 통과 가능한 모든 지점을 P3도 통과했고, 두 생성기를 함께
  // 두면 "어느 쪽이 답했는가"에 따라 연쇄 기동·안전정지 해제가 갈라진다.
  std::string p3_reason;
  (void)generateP3Candidates(
    ego, obstacles, visible, preferred_left, allow_side_switch, false,
    candidates, p3_reason);
  left_reason = p3_reason;
  right_reason = p3_reason;

  std::vector<std::size_t> feasible_order;
  feasible_order.reserve(candidates.size());
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (candidates[index].valid) {
      feasible_order.push_back(index);
    }
  }
  const auto better_candidate = [&](std::size_t first_index, std::size_t second_index) {
      const auto & first = candidates[first_index];
      const auto & second = candidates[second_index];
      // 다음 장애물을 건드리지 않는 exit이 먼저다 (P3ShadowEvaluator::betterFeasible와 동일
      // 계약 — 두 순위가 갈리면 P3가 고른 것과 다른 경로를 P0가 커밋한다). 전부 닿는
      // 경우에는 이 항이 무력해져 기존 slack 기준이 그대로 결정한다.
      if (first.exit_reaches_next_obstacle != second.exit_reaches_next_obstacle) {
        return second.exit_reaches_next_obstacle;
      }
      const double slack_delta =
        first.minimum_normalized_safety_slack - second.minimum_normalized_safety_slack;
      if (std::abs(slack_delta) > kEpsilon) {
        return slack_delta > 0.0;
      }
      const double speed_loss_delta = first.velocity_loss - second.velocity_loss;
      if (std::abs(speed_loss_delta) > kEpsilon) {
        return speed_loss_delta < 0.0;
      }
      const double deviation_delta =
        first.global_path_deviation_m - second.global_path_deviation_m;
      if (std::abs(deviation_delta) > kEpsilon) {
        return deviation_delta < 0.0;
      }
      // Exact remaining ties use generation order only. This adds no optimization weight and
      // guarantees deterministic selection for symmetric geometry.
      return first.audit_index < second.audit_index;
    };
  std::stable_sort(feasible_order.begin(), feasible_order.end(), better_candidate);

  std::vector<int> ranks(candidates.size(), -1);
  for (std::size_t rank = 0; rank < feasible_order.size(); ++rank) {
    ranks[feasible_order[rank]] = static_cast<int>(rank + 1U);
  }
  // 강등 전 가상 순위(exit_reaches_next_obstacle 항을 뺀 순수 slack 순위). 강등이 실제로
  // 선택을 바꿨는지 감사 스트림에서 직접 확인하기 위한 것으로, 선택에는 쓰지 않는다:
  // rank_without_exit_demotion==1인 후보와 final_rank==1인 후보가 다르면 강등이 결정했다.
  const auto slack_only_candidate = [&](std::size_t first_index, std::size_t second_index) {
      const auto & first = candidates[first_index];
      const auto & second = candidates[second_index];
      const double slack_delta =
        first.minimum_normalized_safety_slack - second.minimum_normalized_safety_slack;
      if (std::abs(slack_delta) > kEpsilon) {
        return slack_delta > 0.0;
      }
      const double speed_loss_delta = first.velocity_loss - second.velocity_loss;
      if (std::abs(speed_loss_delta) > kEpsilon) {
        return speed_loss_delta < 0.0;
      }
      const double deviation_delta =
        first.global_path_deviation_m - second.global_path_deviation_m;
      if (std::abs(deviation_delta) > kEpsilon) {
        return deviation_delta < 0.0;
      }
      return first.audit_index < second.audit_index;
    };
  std::vector<std::size_t> slack_order = feasible_order;
  std::stable_sort(slack_order.begin(), slack_order.end(), slack_only_candidate);
  std::vector<int> slack_ranks(candidates.size(), -1);
  for (std::size_t rank = 0; rank < slack_order.size(); ++rank) {
    slack_ranks[slack_order[rank]] = static_cast<int>(rank + 1U);
  }
  const std::size_t selected_index = feasible_order.empty() ?
    std::numeric_limits<std::size_t>::max() : feasible_order.front();
  const auto build_audits = [&]() {
      std::vector<SplineCandidateAudit> audits;
      audits.reserve(candidates.size());
      for (std::size_t index = 0; index < candidates.size(); ++index) {
        const auto & candidate = candidates[index];
        SplineCandidateAudit audit;
        audit.generation_index = candidate.audit_index;
        audit.feasible = candidate.valid;
        audit.selected = index == selected_index;
        audit.go_left = candidate.go_left;
        audit.final_rank = ranks[index];
        audit.exit_reaches_next_obstacle = candidate.exit_reaches_next_obstacle;
        audit.rank_without_exit_demotion = slack_ranks[index];
        audit.target_d = candidate.target_d;
        audit.entry_fraction = candidate.entry_transition_scale;
        audit.exit_transition_scale = candidate.effective_exit_transition_scale;
        audit.requested_entry_length_m = candidate.requested_entry_length_m;
        audit.effective_entry_length_m = candidate.effective_entry_length_m;
        audit.exit_length_m = candidate.exit_length_m;
        audit.centerline_wall_clearance_m = candidate.centerline_wall_clearance_m;
        audit.rectangular_footprint_wall_clearance_m =
          candidate.rectangular_footprint_wall_clearance_m;
        audit.footprint_invalid = candidate.footprint_invalid;
        audit.footprint_violation_side = candidate.footprint_violation_side;
        audit.footprint_violation_waypoint_index =
          candidate.footprint_violation_waypoint_index ==
          std::numeric_limits<std::size_t>::max() ?
          -1 : static_cast<std::int64_t>(candidate.footprint_violation_waypoint_index);
        audit.footprint_violation_s_m = candidate.footprint_violation_s_m;
        audit.footprint_violation_x_m = candidate.footprint_violation_x_m;
        audit.footprint_violation_y_m = candidate.footprint_violation_y_m;
        audit.footprint_violation_yaw_rad = candidate.footprint_violation_yaw_rad;
        audit.footprint_heading_relative_to_reference_rad =
          candidate.footprint_heading_relative_to_reference_rad;
        audit.wallward_corner_protrusion_m = candidate.wallward_corner_protrusion_m;
        audit.wall_clearance_m = candidate.wall_clearance_m;
        audit.obstacle_clearance_m = candidate.obstacle_clearance_m;
        audit.peak_curvature_radpm = candidate.peak_curvature_radpm;
        audit.peak_curvature_rate_radpm2 = candidate.peak_curvature_rate_radpm2;
        audit.velocity_loss = candidate.velocity_loss;
        audit.global_path_deviation_m = candidate.global_path_deviation_m;
        audit.minimum_normalized_safety_slack =
          candidate.minimum_normalized_safety_slack;
        audit.rejection_reason = candidate.valid ? std::string() : candidate.reason;
        audits.push_back(std::move(audit));
      }
      return audits;
    };

  if (selected_index == std::numeric_limits<std::size_t>::max()) {
    // Both spline sides failed. If every cluster member blocks the line only through the
    // inflated tracking/uncertainty margin, the line itself is physically drivable: degrade to
    // a capped-speed lane hold instead of braking (2026-08-12 21:11 run: an off-line hairpin
    // obstacle escalated to a zero-speed hold on every lap through this branch).
    if (!clusterPhysicallyBlocksRaceline(cluster)) {
      auto slow_pass = buildMarginSlowPass(ego, cluster);
      if (slow_pass.kind == SplinePlanKind::kAvoidance) {
        slow_pass.reason += "; left: " + left_reason + "; right: " + right_reason;
        slow_pass.candidate_audits = build_audits();
        return slow_pass;
      }
    }
    auto safe_stop = buildSafeStop(ego, visible, cluster, obstacles, cluster.front());
    safe_stop.obstacle_ids.reserve(cluster.size());
    for (const auto & obstacle : cluster) {
      safe_stop.obstacle_ids.push_back(obstacle.id);
    }
    safe_stop.reason =
      left_evaluated && right_evaluated ?
      "both spline sides rejected; braking before the static obstacle" :
      "committed side rejected; alternate side locked after lateral engagement; braking before "
      "the static obstacle";
    safe_stop.reason += "; left: " + left_reason + "; right: " + right_reason;
    // buildSafeStop이 붙인 탈출-불가 경고는 위 대입으로 지워지므로 여기서 다시 붙인다.
    if (!safe_stop.safe_stop_escape_verified) {
      safe_stop.reason +=
        "; WARNING no escapable stop point exists — the car will not be able to resume from "
        "this stop by forward planning";
    }
    safe_stop.candidate_audits = build_audits();
    return safe_stop;
  }
  Candidate selected = std::move(candidates[selected_index]);
  result.kind = SplinePlanKind::kAvoidance;
  result.path = std::move(selected.path);
  result.go_left = selected.go_left;
  result.target_d = selected.target_d;
  result.merge_s = selected.merge_s;
  result.entry_transition_scale = selected.entry_transition_scale;
  result.exit_transition_scale = selected.exit_transition_scale;
  result.effective_entry_transition_scale = selected.effective_entry_transition_scale;
  result.effective_exit_transition_scale = selected.effective_exit_transition_scale;
  result.requested_entry_length_m = selected.requested_entry_length_m;
  result.effective_entry_length_m = selected.effective_entry_length_m;
  result.exit_length_m = selected.exit_length_m;
  result.obstacle_id = cluster.front().id;
  result.obstacle_ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    result.obstacle_ids.push_back(obstacle.id);
  }
  result.control_points = std::move(selected.control_points);
  result.candidate_audits = build_audits();
  result.reason =
    "selected maximum normalized safety-slack candidate from " +
    std::to_string(feasible_order.size()) + "/" + std::to_string(candidates.size()) +
    " feasible/generated local quintic paths";
  return result;
}

void RacelineSplinePlanner::toCartesian(
  double s, double d, double & x, double & y, double & yaw) const
{
  if (!ready()) {
    x = 0.0;
    y = 0.0;
    yaw = 0.0;
    return;
  }
  const double wrapped = wrapS(s);
  std::size_t first = 0U;
  for (std::size_t i = 1; i < reference_.wpnts.size(); ++i) {
    if (reference_.wpnts[i].s_m > wrapped) {
      break;
    }
    first = i;
  }
  const std::size_t second = (first + 1U) % reference_.wpnts.size();
  const double s0 = reference_.wpnts[first].s_m;
  const double s1 = second == 0U ? track_length_ : reference_.wpnts[second].s_m;
  const double fraction = clamp((wrapped - s0) / std::max(kEpsilon, s1 - s0), 0.0, 1.0);
  const auto & a = reference_.wpnts[first];
  const auto & b = reference_.wpnts[second];
  const double base_x = a.x_m + fraction * (b.x_m - a.x_m);
  const double base_y = a.y_m + fraction * (b.y_m - a.y_m);
  yaw = a.psi_rad + fraction * normalizeAngle(b.psi_rad - a.psi_rad);
  x = base_x - d * std::sin(yaw);
  y = base_y + d * std::cos(yaw);
}

}  // namespace local_planning
