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

#include "local_planning/p3_maneuver_lifecycle.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

#include "local_planning/obstacle_guard.hpp"

namespace local_planning
{
namespace
{

constexpr double kLifecycleEpsilon = 1.0e-9;

void hashBytes(std::uint64_t & hash, const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0U; index < size; ++index) {
    hash ^= static_cast<std::uint64_t>(bytes[index]);
    hash *= 1099511628211ULL;
  }
}

void hashDouble(std::uint64_t & hash, double value)
{
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
  std::memcpy(&bits, &value, sizeof(bits));
  hashBytes(hash, &bits, sizeof(bits));
}

}  // namespace

const char * p3ManeuverLifecycleStateName(P3ManeuverLifecycleState state)
{
  switch (state) {
    case P3ManeuverLifecycleState::kIdle: return "IDLE";
    case P3ManeuverLifecycleState::kFreshSelected: return "FRESH_SELECTED";
    case P3ManeuverLifecycleState::kCommitted: return "COMMITTED";
    case P3ManeuverLifecycleState::kContinuing: return "CONTINUING";
    case P3ManeuverLifecycleState::kInvalidated: return "INVALIDATED";
    case P3ManeuverLifecycleState::kComplete: return "COMPLETE";
  }
  return "UNKNOWN";
}

P3ManeuverLifecycle::Record::Record(
  f110_msgs::msg::WpntArray path,
  std::vector<int> ids,
  std::vector<f110_msgs::msg::Obstacle> guards,
  bool selected_go_left,
  double ego_s,
  double cluster_end_forward_m,
  double cluster_end_s,
  std::uint64_t epoch,
  std::uint64_t reference_generation,
  std::int64_t source_stamp_ns,
  std::uint64_t obstacle_sequence,
  std::string selected_source,
  std::string selected_source_cell,
  std::string selected_candidate_identity,
  std::string selected_logical_identity,
  std::string path_digest)
: original_path(std::move(path)),
  obstacle_ids(std::move(ids)),
  obstacle_guards(std::move(guards)),
  go_left(selected_go_left),
  creation_ego_s(ego_s),
  expanded_cluster_end_forward_m(cluster_end_forward_m),
  expanded_cluster_end_s(cluster_end_s),
  source_epoch(epoch),
  global_reference_generation(reference_generation),
  creation_source_stamp_ns(source_stamp_ns),
  creation_obstacle_sequence(obstacle_sequence),
  source(std::move(selected_source)),
  source_cell(std::move(selected_source_cell)),
  candidate_identity(std::move(selected_candidate_identity)),
  logical_identity(std::move(selected_logical_identity)),
  original_path_digest(std::move(path_digest)),
  last_source_stamp_ns(source_stamp_ns)
{
}

bool P3ManeuverLifecycle::active() const
{
  return record_.has_value();
}

P3ManeuverLifecycleState P3ManeuverLifecycle::state() const
{
  return state_;
}

void P3ManeuverLifecycle::reset()
{
  record_.reset();
  state_ = P3ManeuverLifecycleState::kIdle;
}

P3ManeuverLifecycleDecision P3ManeuverLifecycle::invalidateExternal(
  const std::string & reason)
{
  P3ManeuverLifecycleDecision decision;
  decision.state = state_;
  if (!record_.has_value()) {
    decision.reason = reason;
    return decision;
  }
  decision.go_left = record_->go_left;
  decision.expanded_cluster_end_forward_m = record_->expanded_cluster_end_forward_m;
  decision.original_candidate_identity = record_->candidate_identity;
  decision.original_logical_identity = record_->logical_identity;
  decision.original_path_digest = record_->original_path_digest;
  decision.obstacle_ids = record_->obstacle_ids;
  return invalidate(reason, decision);
}

std::string P3ManeuverLifecycle::suffixDigest(const f110_msgs::msg::WpntArray & path)
{
  std::uint64_t hash = 1469598103934665603ULL;
  const std::uint64_t count = static_cast<std::uint64_t>(path.wpnts.size());
  hashBytes(hash, &count, sizeof(count));
  for (const auto & waypoint : path.wpnts) {
    hashDouble(hash, waypoint.s_m);
    hashDouble(hash, waypoint.d_m);
    hashDouble(hash, waypoint.x_m);
    hashDouble(hash, waypoint.y_m);
    hashDouble(hash, waypoint.psi_rad);
    hashDouble(hash, waypoint.kappa_radpm);
    hashDouble(hash, waypoint.vx_mps);
    hashDouble(hash, waypoint.ax_mps2);
  }
  std::ostringstream output;
  output << std::hex << std::setw(16) << std::setfill('0') << hash;
  return output.str();
}

P3ManeuverLifecycleDecision P3ManeuverLifecycle::selectFresh(
  const P3ManeuverSnapshot & snapshot,
  const P3ShadowResult & selected,
  const RacelineSplinePlanner & planner)
{
  P3ManeuverLifecycleDecision decision;
  if (!selected.invoked || !selected.would_recover || selected.selected_path.wpnts.empty()) {
    decision.reason = "FRESH_P3_NOT_HARD_VALID";
    return decision;
  }
  if (snapshot.source_stale || snapshot.safe_stop_authority) {
    decision.reason = snapshot.source_stale ? "STALE_SOURCE" : "SAFE_STOP_AUTHORITY";
    return decision;
  }
  if (selected.snapshot_source_stamp_ns != snapshot.source_stamp_ns ||
    selected.snapshot_epoch != snapshot.source_epoch ||
    selected.global_reference_generation != snapshot.global_reference_generation)
  {
    decision.reason = "FRESH_RESULT_SNAPSHOT_LINEAGE_MISMATCH";
    return decision;
  }
  if (selected.selected_obstacle_ids.empty() ||
    !std::isfinite(selected.selected_cluster_end_forward_m) ||
    !(selected.selected_cluster_end_forward_m > 0.0) ||
    !std::isfinite(selected.selected_cluster_end_s))
  {
    decision.reason = "FRESH_RESULT_INTERACTION_IDENTITY_INCOMPLETE";
    return decision;
  }

  // Guard formation is not a standalone veto. The selected path was generated from this
  // callback's accumulated/conservative geometry, but ownership is granted only after the exact
  // validator independently confirms both that authority and the detector-owned raw geometry.
  // This adds no waiting frame or tolerance and cannot publish a hard-invalid path.
  decision.guarded_validation_attempted = true;
  // The evaluator already validated this exact path against this exact ego/guarded-obstacle pair
  // while constructing the candidate, and the FRESH_RESULT_SNAPSHOT_LINEAGE_MISMATCH guard above
  // has already proven the inputs are that same snapshot. Reuse the certificate instead of running
  // a bit-identical validation again; only a result produced before this field existed (or by a
  // non-production evaluator) still needs the fallback. The RAW check below is NOT redundant and
  // always runs: it tests different geometry.
  // 인증서를 재사용하지 못할 때의 대체 검증도 후보 생성이 쓴 것과 같은 범위를 써야 한다.
  // 범위가 다르면 "인증서 있음/없음"이라는 부수적 사정만으로 판정이 갈린다.
  const std::optional<double> fresh_collision_horizon =
    std::isfinite(selected.selected_cluster_end_forward_m) ?
    std::optional<double>(
    selected.selected_cluster_end_forward_m + planner.postMergeLookaheadM()) :
    std::nullopt;
  decision.guarded_validation_reused_certificate = selected.selected_validation_available;
  decision.validation = selected.selected_validation_available ?
    selected.selected_validation :
    planner.evaluateP3PathCurrent(
    snapshot.ego, selected.selected_path, snapshot.obstacles, 1.0, fresh_collision_horizon);
  decision.guarded_validation_hard_valid = decision.validation.hard_valid;
  decision.guarded_validation_rejection = decision.validation.rejection_reason.empty() ?
    "NONE" : decision.validation.rejection_reason;
  if (!decision.validation.hard_valid) {
    decision.reason = "FRESH_CONSERVATIVE_EXACT_HARD_INVALID:" +
      decision.guarded_validation_rejection;
    return decision;
  }

  if (!containsOriginalObstacleIds(selected.selected_obstacle_ids, snapshot.raw_obstacles)) {
    decision.reason = "FRESH_RAW_OBSTACLE_IDENTITY_INCOMPLETE";
    return decision;
  }
  decision.raw_validation_attempted = true;
  const auto raw_validation = planner.evaluateP3PathCurrent(
    snapshot.ego, selected.selected_path, snapshot.raw_obstacles, 1.0,
    fresh_collision_horizon);
  decision.raw_validation_hard_valid = raw_validation.hard_valid;
  decision.raw_validation_rejection = raw_validation.rejection_reason.empty() ?
    "NONE" : raw_validation.rejection_reason;
  if (!raw_validation.hard_valid) {
    decision.validation = raw_validation;
    decision.reason = "FRESH_RAW_EXACT_HARD_INVALID:" + decision.raw_validation_rejection;
    return decision;
  }

  std::vector<int> canonical_ids = selected.selected_obstacle_ids;
  std::sort(canonical_ids.begin(), canonical_ids.end());
  canonical_ids.erase(std::unique(canonical_ids.begin(), canonical_ids.end()), canonical_ids.end());
  std::vector<f110_msgs::msg::Obstacle> selected_guards;
  selected_guards.reserve(canonical_ids.size());
  for (const int id : canonical_ids) {
    const auto guard = std::find_if(
      snapshot.obstacles.begin(), snapshot.obstacles.end(),
      [id](const auto & obstacle) {return obstacle.id == id;});
    if (guard == snapshot.obstacles.end()) {
      decision.reason = "FRESH_RESULT_OBSTACLE_GUARD_INCOMPLETE";
      return decision;
    }
    selected_guards.push_back(*guard);
  }
  record_.emplace(
    selected.selected_path, std::move(canonical_ids), std::move(selected_guards),
    selected.selected_go_left,
    snapshot.ego.s, selected.selected_cluster_end_forward_m, selected.selected_cluster_end_s,
    snapshot.source_epoch, snapshot.global_reference_generation, snapshot.source_stamp_ns,
    snapshot.obstacle_sequence, selected.selected_source, selected.selected_source_cell,
    selected.selected_candidate_identity, selected.selected_logical_identity,
    selected.selected_path_digest);
  state_ = P3ManeuverLifecycleState::kFreshSelected;

  decision.state = state_;
  decision.has_output = true;
  decision.fresh_selected = true;
  decision.suffix_hard_valid = true;
  decision.guarded_validation_attempted = true;
  decision.guarded_validation_hard_valid = true;
  decision.raw_validation_attempted = true;
  decision.raw_validation_hard_valid = true;
  decision.go_left = selected.selected_go_left;
  decision.suffix_point_count = selected.selected_path.wpnts.size();
  decision.expanded_cluster_end_forward_m = selected.selected_cluster_end_forward_m;
  decision.reason = "FRESH_HARD_VALID_P3_M1";
  decision.original_candidate_identity = selected.selected_candidate_identity;
  decision.original_logical_identity = selected.selected_logical_identity;
  decision.original_path_digest = selected.selected_path_digest;
  decision.output_path_digest = selected.selected_path_digest;
  decision.obstacle_ids = record_->obstacle_ids;
  decision.output_path = selected.selected_path;
  return decision;
}

bool P3ManeuverLifecycle::containsOriginalObstacleIds(
  const std::vector<int> & original_ids,
  const std::vector<f110_msgs::msg::Obstacle> & current_obstacles)
{
  std::set<int> current_ids;
  for (const auto & obstacle : current_obstacles) {
    current_ids.insert(obstacle.id);
  }
  return std::all_of(
    original_ids.begin(), original_ids.end(),
    [&current_ids](int id) {return current_ids.count(id) != 0U;});
}

P3ManeuverLifecycleDecision P3ManeuverLifecycle::invalidate(
  const std::string & reason,
  const P3ManeuverLifecycleDecision & partial)
{
  P3ManeuverLifecycleDecision decision = partial;
  decision.state = P3ManeuverLifecycleState::kInvalidated;
  decision.has_output = false;
  decision.invalidated = true;
  decision.reason = reason;
  decision.output_path = f110_msgs::msg::WpntArray();
  state_ = decision.state;
  record_.reset();
  return decision;
}

P3ManeuverLifecycleDecision P3ManeuverLifecycle::continueCurrent(
  const P3ManeuverSnapshot & snapshot,
  const RacelineSplinePlanner & planner,
  int soft_violation_confirm_cycles)
{
  P3ManeuverLifecycleDecision decision;
  decision.state = state_;
  if (!record_.has_value()) {
    decision.reason = "NO_ACTIVE_MANEUVER";
    return decision;
  }

  Record & record = *record_;
  if (soft_violation_confirm_cycles <= 0) {
    return invalidate("INVALID_SOFT_VIOLATION_CONFIRM_CYCLES", decision);
  }
  decision.go_left = record.go_left;
  decision.expanded_cluster_end_forward_m = record.expanded_cluster_end_forward_m;
  decision.original_candidate_identity = record.candidate_identity;
  decision.original_logical_identity = record.logical_identity;
  decision.original_path_digest = record.original_path_digest;
  decision.obstacle_ids = record.obstacle_ids;
  decision.record_source_epoch = record.source_epoch;
  decision.current_source_epoch = snapshot.source_epoch;
  decision.record_reference_generation = record.global_reference_generation;
  decision.current_reference_generation = snapshot.global_reference_generation;
  decision.record_creation_source_stamp_ns = record.creation_source_stamp_ns;
  decision.record_last_source_stamp_ns = record.last_source_stamp_ns;
  decision.current_source_stamp_ns = snapshot.source_stamp_ns;
  decision.record_obstacle_sequence = record.creation_obstacle_sequence;
  decision.current_obstacle_sequence = snapshot.obstacle_sequence;

  if (snapshot.source_epoch != record.source_epoch) {
    return invalidate("SOURCE_EPOCH_MISMATCH", decision);
  }
  if (snapshot.global_reference_generation != record.global_reference_generation) {
    return invalidate("GLOBAL_REFERENCE_GENERATION_MISMATCH", decision);
  }
  if (snapshot.safe_stop_authority) {
    return invalidate("SAFE_STOP_AUTHORITY", decision);
  }
  if (snapshot.source_stale) {
    return invalidate("STALE_SOURCE", decision);
  }
  if (snapshot.source_stamp_ns < record.last_source_stamp_ns) {
    return invalidate("SOURCE_STAMP_REGRESSION", decision);
  }

  decision.progress_m = planner.forwardDistance(record.creation_ego_s, snapshot.ego.s);
  if (!std::isfinite(decision.progress_m) ||
    decision.progress_m > 0.5 * planner.trackLength())
  {
    return invalidate("WRAP_OR_BACKWARD_PROGRESS", decision);
  }

  // This ordering is the production resolution of the already-identified minimum-point conflict.
  // Once the ego has passed the core-computed expanded cluster end, no short terminal suffix is
  // submitted to the validator.
  if (decision.progress_m + kLifecycleEpsilon >= record.expanded_cluster_end_forward_m) {
    decision.completion_handoff_path.header = record.original_path.header;
    for (const auto & waypoint : record.original_path.wpnts) {
      const double waypoint_progress = planner.forwardDistance(
        record.creation_ego_s, waypoint.s_m);
      if (!std::isfinite(waypoint_progress) ||
        waypoint_progress > 0.5 * planner.trackLength())
      {
        continue;
      }
      if (waypoint_progress + kLifecycleEpsilon >= record.expanded_cluster_end_forward_m) {
        decision.completion_handoff_path.wpnts.push_back(waypoint);
      }
    }
    decision.completion_handoff_available =
      !decision.completion_handoff_path.wpnts.empty();
    if (decision.completion_handoff_available) {
      decision.completion_handoff_path_digest = suffixDigest(
        decision.completion_handoff_path);
    }
    decision.state = P3ManeuverLifecycleState::kComplete;
    decision.complete = true;
    decision.reason = "EXPANDED_OBSTACLE_REGION_FULLY_PASSED";
    state_ = decision.state;
    record_.reset();
    return decision;
  }

  // A fresh empty detector snapshot is a measurement dropout, not evidence of a different
  // maneuver lineage. Preserve the immutable path and let the current exact validator plus the
  // latched expanded-region progress gate decide whether it may continue. A non-empty snapshot
  // that no longer contains the original IDs is an actual lineage conflict and still invalidates
  // immediately.
  if (!snapshot.obstacles.empty() &&
    !containsOriginalObstacleIds(record.obstacle_ids, snapshot.obstacles))
  {
    return invalidate("OBSTACLE_MANEUVER_IDENTITY_MISMATCH", decision);
  }
  if (record.original_path.wpnts.empty()) {
    return invalidate("ORIGINAL_PATH_EMPTY", decision);
  }

  std::size_t start_index = record.original_path.wpnts.size();
  double smallest_forward = std::numeric_limits<double>::infinity();
  std::size_t minimum_ties = 0U;
  for (std::size_t index = 0U; index < record.original_path.wpnts.size(); ++index) {
    const double forward = planner.forwardDistance(
      snapshot.ego.s, record.original_path.wpnts[index].s_m);
    if (forward < smallest_forward - kLifecycleEpsilon) {
      smallest_forward = forward;
      start_index = index;
      minimum_ties = 1U;
    } else if (std::abs(forward - smallest_forward) <= kLifecycleEpsilon) {
      ++minimum_ties;
    }
  }
  if (start_index >= record.original_path.wpnts.size() || minimum_ties != 1U) {
    return invalidate("AMBIGUOUS_PROGRESS_PROJECTION", decision);
  }
  if (start_index < record.progress_index) {
    return invalidate("BACKWARD_PROGRESS_INDEX", decision);
  }

  f110_msgs::msg::WpntArray suffix;
  suffix.header = record.original_path.header;
  double previous_forward = -1.0;
  for (std::size_t index = start_index; index < record.original_path.wpnts.size(); ++index) {
    const double forward = planner.forwardDistance(
      snapshot.ego.s, record.original_path.wpnts[index].s_m);
    if (!std::isfinite(forward) || forward + kLifecycleEpsilon < previous_forward ||
      forward > 0.5 * planner.trackLength())
    {
      break;
    }
    auto waypoint = record.original_path.wpnts[index];
    waypoint.id = static_cast<std::int32_t>(suffix.wpnts.size());
    suffix.wpnts.push_back(waypoint);
    previous_forward = forward;
  }
  decision.suffix_point_count = suffix.wpnts.size();
  decision.output_path_digest = suffixDigest(suffix);
  decision.suffix_revalidated = true;

  // 🔴 이 기동이 책임지는 범위. `generateP3Candidates`가 후보를 고를 때 쓰는 것과 **같은**
  // 정의(클러스터 끝 + post_merge_lookahead)를 그대로 쓴다. 그 너머의 장애물은 다음 기동과
  // 연쇄 재계획의 몫이고, merge 뒤 글로벌 꼬리는 애초에 이 기동의 기하가 아니다
  // (AGENTS: "A post-merge controller-tail obstacle must not make the current maneuver fail").
  //
  // 이게 없으면 선택과 재검증이 서로 다른 범위를 본다: 선택기는 12 m lookahead 안에서
  // horizon까지만 보고 통과시킨 경로를, 재검증은 트랙의 모든 장애물에 대해 꼬리 끝까지
  // 검사해 다음 콜백에 폐기한다. 그러면 매 사이클 "선택 → 폐기 → 재선택"이 반복되고,
  // 폐기 시점이 안전정지 버퍼 안이면 그대로 정지로 굳는다 (2026-08-16 시뮬 백
  // rosbag2_2026_08_16-08_50_21: s=31.7 장애물 기동의 d=0 글로벌 꼬리가 12 m 앞 s=40.6
  // 장애물을 지나가는 것 때문에 랩마다 s=28~30에서 완전 정지, 5랩 8회).
  const std::optional<double> collision_horizon =
    std::isfinite(record.expanded_cluster_end_s) ?
    std::optional<double>(
    planner.forwardDistance(snapshot.ego.s, record.expanded_cluster_end_s) +
    planner.postMergeLookaheadM()) :
    std::nullopt;

  // Reuse the pre-P3 committed-path Guard contract exactly: the guard frozen at selection owns
  // each exact obstacle ID while the live guarded envelope remains contained. A fresh empty
  // snapshot is treated as detector dropout and also retains the frozen guards. A non-empty
  // conflicting identity was rejected above before reaching this point.
  std::vector<f110_msgs::msg::Obstacle> validation_obstacles = snapshot.obstacles;
  if (validation_obstacles.empty()) {
    validation_obstacles = record.obstacle_guards;
  } else {
    for (const auto & frozen_guard : record.obstacle_guards) {
      const auto current = std::find_if(
        validation_obstacles.begin(), validation_obstacles.end(),
        [&frozen_guard](const auto & obstacle) {return obstacle.id == frozen_guard.id;});
      if (current != validation_obstacles.end() &&
        obstacleEnvelopeContained(*current, frozen_guard, planner.trackLength()))
      {
        *current = frozen_guard;
        ++decision.guard_contained_same_id_count;
      }
    }
  }
  decision.guard_observations.reserve(record.obstacle_guards.size());
  for (const auto & frozen_guard : record.obstacle_guards) {
    P3GuardObservation observation;
    observation.obstacle_id = frozen_guard.id;
    observation.frozen_guard = frozen_guard;
    const auto live = std::find_if(
      snapshot.raw_obstacles.begin(), snapshot.raw_obstacles.end(),
      [&frozen_guard](const auto & obstacle) {return obstacle.id == frozen_guard.id;});
    if (live != snapshot.raw_obstacles.end()) {
      observation.live_present = true;
      observation.live_envelope = *live;
    }
    const auto accumulated = std::find_if(
      snapshot.selection_envelope_obstacles.begin(), snapshot.selection_envelope_obstacles.end(),
      [&frozen_guard](const auto & obstacle) {return obstacle.id == frozen_guard.id;});
    if (accumulated != snapshot.selection_envelope_obstacles.end()) {
      observation.accumulated_present = true;
      observation.accumulated_envelope = *accumulated;
    }
    const auto guarded = std::find_if(
      snapshot.obstacles.begin(), snapshot.obstacles.end(),
      [&frozen_guard](const auto & obstacle) {return obstacle.id == frozen_guard.id;});
    if (guarded != snapshot.obstacles.end()) {
      observation.guarded_present = true;
      observation.guarded_envelope = *guarded;
      observation.contained_in_frozen_guard = obstacleEnvelopeContained(
        *guarded, frozen_guard, planner.trackLength());
    }
    decision.guard_observations.push_back(std::move(observation));
  }
  decision.guarded_validation_attempted = true;
  decision.validation = planner.evaluateP3PathCurrent(
    snapshot.ego, suffix, validation_obstacles, 1.0, collision_horizon);
  decision.suffix_hard_valid = decision.validation.hard_valid;
  decision.guarded_validation_hard_valid = decision.validation.hard_valid;
  decision.guarded_validation_rejection = decision.validation.rejection_reason.empty() ?
    "NONE" : decision.validation.rejection_reason;
  if (!decision.validation.hard_valid) {
    const std::string detail = decision.validation.rejection_reason.empty() ?
      "UNSPECIFIED" : decision.validation.rejection_reason;

    // Only an uncertainty-guard obstacle rejection enters the established raw-obstacle fallback.
    // All other exact-validator failures remain immediate hard invalidations.
    PathValidationFailure guarded_failure;
    (void)planner.validatePath(
      snapshot.ego, suffix, validation_obstacles, nullptr, &guarded_failure,
      collision_horizon);
    if (guarded_failure.kind != PathValidationFailureKind::kObstacleCollision) {
      return invalidate("CURRENT_EXACT_HARD_INVALID:" + detail, decision);
    }

    decision.guard_raw_revalidated = true;
    decision.raw_validation_attempted = true;
    const auto & raw_obstacles = snapshot.raw_obstacles;
    PathValidationFailure raw_failure;
    P3ShadowPathEvaluation raw_validation = planner.evaluateP3PathCurrent(
      snapshot.ego, suffix, raw_obstacles, 1.0, collision_horizon);
    decision.raw_validation_hard_valid = raw_validation.hard_valid;
    decision.raw_validation_rejection = raw_validation.rejection_reason.empty() ?
      "NONE" : raw_validation.rejection_reason;
    if (!raw_validation.hard_valid) {
      (void)planner.validatePath(
        snapshot.ego, suffix, raw_obstacles, nullptr, &raw_failure, collision_horizon);
      // Committed-path retention band (2026-08-12, user-requested freeze): when the full-reserve
      // raw validation fails only through obstacle clearance, re-validate with the retention
      // fraction of the tracking reserve (physical base clearance always intact). Progressive
      // reveal and envelope jitter inside the released band then freeze the committed geometry
      // instead of re-shaping it on every callback; invalidation requires an actual
      // retention-margin violation.
      const double retention_fraction = planner.commitmentRetentionReserveFraction();
      bool retention_holds = false;
      P3ShadowPathEvaluation retention_validation;
      if (raw_failure.kind == PathValidationFailureKind::kObstacleCollision &&
        retention_fraction >= 0.0 && retention_fraction < 1.0)
      {
        retention_validation = planner.evaluateP3PathCurrent(
          snapshot.ego, suffix, raw_obstacles, retention_fraction, collision_horizon);
        retention_holds = retention_validation.hard_valid;
      }
      if (!retention_holds) {
        decision.validation = std::move(raw_validation);
        decision.suffix_hard_valid = false;
        const std::string raw_detail = decision.validation.rejection_reason.empty() ?
          "UNSPECIFIED" : decision.validation.rejection_reason;
        return invalidate(
          raw_failure.kind == PathValidationFailureKind::kObstacleCollision ?
          "CURRENT_RAW_OBSTACLE_COLLISION:" + raw_detail :
          "CURRENT_EXACT_HARD_INVALID_RAW:" + raw_detail,
          decision);
      }
      ++record.guard_soft_violation_count;
      decision.guard_soft_violation_count = record.guard_soft_violation_count;
      decision.raw_validation_rejection =
        "RETENTION_HOLD:" + decision.raw_validation_rejection;
      decision.validation = std::move(retention_validation);
      decision.suffix_hard_valid = true;
      decision.guard_soft_violation_pending = true;
    } else {
      // The immutable suffix remains hard-valid against the raw detector geometry with the FULL
      // reserve. Hold the frozen path with no cycle expiry: a broken uncertainty guard alone is
      // not a margin violation, and the previous N-cycle expiry re-planned an almost identical
      // geometry on every progressive reveal — the dominant visible path churn. The counter is
      // kept for diagnostics only.
      (void)soft_violation_confirm_cycles;
      ++record.guard_soft_violation_count;
      decision.guard_soft_violation_count = record.guard_soft_violation_count;
      decision.validation = std::move(raw_validation);
      decision.suffix_hard_valid = true;
      decision.guard_soft_violation_pending = true;
    }
  } else {
    record.guard_soft_violation_count = 0;
  }

  record.progress_index = start_index;
  record.last_source_stamp_ns = snapshot.source_stamp_ns;
  decision.has_output = true;
  decision.output_path = std::move(suffix);
  if (decision.guard_soft_violation_pending) {
    state_ = record.committed_once ?
      P3ManeuverLifecycleState::kContinuing : P3ManeuverLifecycleState::kCommitted;
    record.committed_once = true;
    decision.reason = "GUARD_SOFT_OBSTACLE_VIOLATION_PENDING_RAW_HARD_VALID";
  } else if (!record.committed_once) {
    record.committed_once = true;
    state_ = P3ManeuverLifecycleState::kCommitted;
    decision.reason = "ORIGINAL_MANEUVER_COMMITTED_CURRENT_HARD_VALID";
  } else {
    state_ = P3ManeuverLifecycleState::kContinuing;
    decision.reason = "ORIGINAL_MANEUVER_SUFFIX_CURRENT_HARD_VALID";
  }
  decision.state = state_;
  return decision;
}

}  // namespace local_planning
