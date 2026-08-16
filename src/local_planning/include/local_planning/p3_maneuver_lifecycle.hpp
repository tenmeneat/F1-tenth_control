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

#ifndef LOCAL_PLANNING__P3_MANEUVER_LIFECYCLE_HPP_
#define LOCAL_PLANNING__P3_MANEUVER_LIFECYCLE_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>

#include "local_planning/p3_shadow.hpp"
#include "local_planning/raceline_spline_planner.hpp"

namespace local_planning
{

// These states intentionally describe one concrete P3 maneuver. They are not a generic
// previous-path preference or a temporal hysteresis policy.
enum class P3ManeuverLifecycleState
{
  kIdle,
  kFreshSelected,
  kCommitted,
  kContinuing,
  kInvalidated,
  kComplete,
};

const char * p3ManeuverLifecycleStateName(P3ManeuverLifecycleState state);

struct P3ManeuverSnapshot
{
  EgoFrenetState ego;
  // `obstacles` is the production uncertainty-guard authority used by P3/M1 and the exact
  // validator. `raw_obstacles` is the same callback's detector-owned geometry and is used only
  // for the existing guard-only soft-violation fallback contract.
  std::vector<f110_msgs::msg::Obstacle> obstacles;
  std::vector<f110_msgs::msg::Obstacle> raw_obstacles;
  // Raw conservative same-ID union before buildUncertaintyGuard(). This is observation-only
  // provenance for lifecycle invalidation logs; validator authority remains `obstacles`.
  std::vector<f110_msgs::msg::Obstacle> selection_envelope_obstacles;
  std::int64_t source_stamp_ns{0};
  std::uint64_t source_epoch{0U};
  std::uint64_t global_reference_generation{0U};
  std::uint64_t obstacle_sequence{0U};
  bool source_stale{false};
  bool safe_stop_authority{false};
  // Reports whether the existing production Guard observation-count/time contract has completed.
  // This is diagnostic provenance, not a standalone ownership veto: a fresh P3 path may own the
  // output earlier only after the same callback proves it exact-hard-valid against both the
  // accumulated conservative geometry and the current raw detector geometry.
  bool selection_guard_ready{true};
};

struct P3GuardObservation
{
  int obstacle_id{-1};
  bool live_present{false};
  bool accumulated_present{false};
  bool guarded_present{false};
  bool contained_in_frozen_guard{false};
  f110_msgs::msg::Obstacle live_envelope;
  f110_msgs::msg::Obstacle accumulated_envelope;
  f110_msgs::msg::Obstacle guarded_envelope;
  f110_msgs::msg::Obstacle frozen_guard;
};

struct P3ManeuverLifecycleDecision
{
  P3ManeuverLifecycleState state{P3ManeuverLifecycleState::kIdle};
  bool has_output{false};
  bool fresh_selected{false};
  bool suffix_revalidated{false};
  bool suffix_hard_valid{false};
  bool invalidated{false};
  bool complete{false};
  bool guard_raw_revalidated{false};
  bool guarded_validation_attempted{false};
  bool guarded_validation_hard_valid{false};
  // True when the guarded verdict came from the evaluator's own certificate instead of a repeated
  // validation. Diagnostic only: the verdict itself is identical either way.
  bool guarded_validation_reused_certificate{false};
  bool raw_validation_attempted{false};
  bool raw_validation_hard_valid{false};
  bool guard_soft_violation_pending{false};
  bool completion_handoff_available{false};
  bool go_left{false};
  std::size_t suffix_point_count{0U};
  std::size_t guard_contained_same_id_count{0U};
  int guard_soft_violation_count{0};
  double progress_m{0.0};
  double expanded_cluster_end_forward_m{0.0};
  std::string reason{"IDLE"};
  std::string original_candidate_identity{"NONE"};
  std::string original_logical_identity{"NONE"};
  std::string original_path_digest{"NONE"};
  std::string output_path_digest{"NONE"};
  std::string completion_handoff_path_digest{"NONE"};
  std::string guarded_validation_rejection{"NOT_ATTEMPTED"};
  std::string raw_validation_rejection{"NOT_ATTEMPTED"};
  std::uint64_t record_source_epoch{0U};
  std::uint64_t current_source_epoch{0U};
  std::uint64_t record_reference_generation{0U};
  std::uint64_t current_reference_generation{0U};
  std::int64_t record_creation_source_stamp_ns{0};
  std::int64_t record_last_source_stamp_ns{0};
  std::int64_t current_source_stamp_ns{0};
  std::uint64_t record_obstacle_sequence{0U};
  std::uint64_t current_obstacle_sequence{0U};
  std::vector<int> obstacle_ids;
  std::vector<P3GuardObservation> guard_observations;
  P3ShadowPathEvaluation validation;
  f110_msgs::msg::WpntArray output_path;
  // This is the immutable, already validated post-obstacle exit tail from the original P3 path.
  // It is produced only on COMPLETE and is not an active obstacle suffix or a new connector.
  f110_msgs::msg::WpntArray completion_handoff_path;
};

class P3ManeuverLifecycle
{
public:
  P3ManeuverLifecycle() = default;

  bool active() const;
  P3ManeuverLifecycleState state() const;
  void reset();
  P3ManeuverLifecycleDecision invalidateExternal(const std::string & reason);

  // Persists the core-selected path and identity exactly once. The original path is immutable
  // until completion/invalidation/reset; subsequent output is copied from this stored geometry.
  P3ManeuverLifecycleDecision selectFresh(
    const P3ManeuverSnapshot & snapshot,
    const P3ShadowResult & selected,
    const RacelineSplinePlanner & planner);

  // Completion is checked before suffix validation so a safely passed expanded obstacle region
  // cannot be misclassified as an invalid maneuver merely because too few tail samples remain.
  // A fresh empty obstacle snapshot is treated as detector dropout; the immutable path remains
  // subject to exact current validation. A conflicting non-empty identity still invalidates.
  P3ManeuverLifecycleDecision continueCurrent(
    const P3ManeuverSnapshot & snapshot,
    const RacelineSplinePlanner & planner,
    int soft_violation_confirm_cycles);

private:
  struct Record
  {
    const f110_msgs::msg::WpntArray original_path;
    const std::vector<int> obstacle_ids;
    const std::vector<f110_msgs::msg::Obstacle> obstacle_guards;
    const bool go_left;
    const double creation_ego_s;
    const double expanded_cluster_end_forward_m;
    const double expanded_cluster_end_s;
    const std::uint64_t source_epoch;
    const std::uint64_t global_reference_generation;
    const std::int64_t creation_source_stamp_ns;
    const std::uint64_t creation_obstacle_sequence;
    const std::string source;
    const std::string source_cell;
    const std::string candidate_identity;
    const std::string logical_identity;
    const std::string original_path_digest;
    std::size_t progress_index{0U};
    std::int64_t last_source_stamp_ns{0};
    bool committed_once{false};
    int guard_soft_violation_count{0};

    Record(
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
      std::string path_digest);
  };

  P3ManeuverLifecycleDecision invalidate(
    const std::string & reason,
    const P3ManeuverLifecycleDecision & partial);
  static bool containsOriginalObstacleIds(
    const std::vector<int> & original_ids,
    const std::vector<f110_msgs::msg::Obstacle> & current_obstacles);
  static std::string suffixDigest(const f110_msgs::msg::WpntArray & path);

  P3ManeuverLifecycleState state_{P3ManeuverLifecycleState::kIdle};
  std::optional<Record> record_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__P3_MANEUVER_LIFECYCLE_HPP_
