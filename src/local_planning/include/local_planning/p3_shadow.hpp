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

#ifndef LOCAL_PLANNING__P3_SHADOW_HPP_
#define LOCAL_PLANNING__P3_SHADOW_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <f110_msgs/msg/wpnt_array.hpp>

namespace local_planning
{

struct P3ShadowObstacleEnvelope
{
  int id{-1};
  double start{0.0};
  double end{0.0};
  double center{0.0};
  double d_right{0.0};
  double d_left{0.0};
};

struct P3ShadowSideDomain
{
  bool valid{false};
  bool go_left{false};
  double cluster_start{0.0};
  double cluster_end{0.0};
  double minimum_target{0.0};
  double maximum_target{0.0};
  std::string reason;
};

struct P3ShadowPlanningContext
{
  bool valid{false};
  bool outside_is_left{false};
  std::string reason;
  std::vector<P3ShadowObstacleEnvelope> visible;
  std::vector<int> cluster_ids;
  P3ShadowSideDomain right;
  P3ShadowSideDomain left;
};

struct P3ShadowPathEvaluation
{
  bool hard_valid{false};
  double minimum_normalized_safety_slack{-std::numeric_limits<double>::infinity()};
  double minimum_track_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_obstacle_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_rate_radpm2{std::numeric_limits<double>::quiet_NaN()};
  double velocity_loss{std::numeric_limits<double>::quiet_NaN()};
  double global_path_deviation_m{std::numeric_limits<double>::quiet_NaN()};
  std::string rejection_reason;
};

// Complete production P3 candidate trace. The evaluator ranks these internally; the node may
// publish only selected_path and only in explicit TEST_ACTIVE mode. SHADOW remains log-only.
struct P3ShadowCandidateTrace
{
  std::size_t generation_index{0U};
  bool go_left{false};
  double entry_scale{std::numeric_limits<double>::quiet_NaN()};
  double exit_scale{std::numeric_limits<double>::quiet_NaN()};
  double d_target{std::numeric_limits<double>::quiet_NaN()};
  double d_mid{std::numeric_limits<double>::quiet_NaN()};
  bool hard_valid{false};
  std::string mapping_source;
  std::string candidate_template;
  std::string source_cell;
  std::string component_id;
  std::string source_branch_regime;
  std::string candidate_identity;
  std::string logical_identity;
  std::string path_digest;
  double minimum_normalized_safety_slack{-std::numeric_limits<double>::infinity()};
  double minimum_track_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_obstacle_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_rate_radpm2{std::numeric_limits<double>::quiet_NaN()};
  double peak_lateral_slope{std::numeric_limits<double>::quiet_NaN()};
  double velocity_loss{std::numeric_limits<double>::quiet_NaN()};
  double global_path_deviation_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_commanded_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double maximum_commanded_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  std::string rejection_reason;
  // True when this candidate's post-cluster exit ramp still carries enough lateral offset to reach
  // a NOT-in-cluster obstacle's physical envelope. Such a candidate is legal — the maneuver-scope
  // collision horizon deliberately stops before it, and on tightly spaced obstacles carrying the
  // offset over is the intended behaviour — but it is the last resort, never the preference.
  bool exit_reaches_next_obstacle{false};
  // The exact-validator verdict this candidate was already measured with, kept verbatim so the
  // maneuver lifecycle can reuse it instead of re-running an identical validation. Valid only
  // together with the owning result's snapshot lineage (stamp/epoch/reference generation).
  P3ShadowPathEvaluation validation;
  f110_msgs::msg::WpntArray path;
};

struct P3ShadowResult
{
  bool enabled{false};
  bool invoked{false};
  std::int64_t snapshot_source_stamp_ns{0};
  std::uint64_t snapshot_epoch{0U};
  std::uint64_t global_reference_generation{0U};
  std::string p0_failure_reason;
  std::string mapping_semantics{"M1_BRANCH_COMPLETE_ACTIVE_SET_CLOSURE"};

  std::size_t raw_root_count{0U};
  std::size_t finite_root_count{0U};
  std::size_t branch_root_count{0U};
  std::size_t bounded_root_count{0U};
  std::size_t accepted_root_count{0U};
  std::size_t candidate_count{0U};
  std::size_t hard_validator_call_count{0U};
  std::size_t hard_valid_count{0U};

  std::size_t m0_candidate_count{0U};
  std::size_t m0_validator_call_count{0U};
  std::size_t m0_hard_valid_count{0U};
  bool m1_invoked{false};
  std::size_t m1_budget{0U};
  std::size_t m1_context_count{0U};
  std::size_t m1_candidate_count{0U};
  std::size_t m1_validator_call_count{0U};
  std::size_t m1_hard_valid_count{0U};
  std::size_t m1_active_outer_raw_root_count{0U};
  std::size_t m1_active_outer_accepted_root_count{0U};
  std::size_t m1_all_inactive_raw_root_count{0U};
  std::size_t m1_all_inactive_accepted_root_count{0U};
  std::size_t m1_exact_tuple_duplicate_count{0U};
  std::size_t m1_positive_segment_rejection_count{0U};
  std::size_t m1_boundary_handoff_unresolved_count{0U};

  // Blocking-cluster observation is independent of whether any candidate is selected. This keeps
  // SHADOW diagnostics complete on fail-closed and committed-suffix callbacks.
  std::vector<int> cluster_obstacle_ids;
  double cluster_start_forward_m{std::numeric_limits<double>::quiet_NaN()};
  double cluster_end_forward_m{std::numeric_limits<double>::quiet_NaN()};

  bool would_recover{false};
  bool selected_go_left{false};
  std::vector<int> selected_obstacle_ids;
  double selected_cluster_start_forward_m{std::numeric_limits<double>::quiet_NaN()};
  double selected_cluster_end_forward_m{std::numeric_limits<double>::quiet_NaN()};
  double selected_cluster_end_s{std::numeric_limits<double>::quiet_NaN()};
  std::string selected_source{"NONE"};
  std::string selected_candidate_template{"NONE"};
  std::string selected_source_cell{"NONE"};
  std::string selected_component_id{"NONE"};
  std::string selected_source_branch_regime{"NONE"};
  std::string selected_candidate_identity{"NONE"};
  std::string selected_logical_identity{"NONE"};
  std::string selected_path_digest{"NONE"};
  double selected_d_target{std::numeric_limits<double>::quiet_NaN()};
  double selected_d_mid{std::numeric_limits<double>::quiet_NaN()};
  double selected_probe_s{std::numeric_limits<double>::quiet_NaN()};
  double selected_probe_d{std::numeric_limits<double>::quiet_NaN()};
  double selected_min_track_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double selected_min_obstacle_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double selected_curvature_margin{std::numeric_limits<double>::quiet_NaN()};
  double selected_curvature_rate_margin{std::numeric_limits<double>::quiet_NaN()};
  double selected_slope_margin{std::numeric_limits<double>::quiet_NaN()};
  double selected_min_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double selected_max_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  // Guarded-geometry validation certificate for selected_path, produced during candidate
  // construction against exactly the ego/obstacles this result was evaluated with.
  bool selected_validation_available{false};
  P3ShadowPathEvaluation selected_validation;
  f110_msgs::msg::WpntArray selected_path;

  double runtime_total_us{0.0};
  double runtime_corridor_us{0.0};
  double runtime_root_solver_us{0.0};
  double runtime_reconstruction_us{0.0};
  double runtime_hard_validation_us{0.0};
  std::string failure_classification{"NOT_INVOKED"};
  std::vector<P3ShadowCandidateTrace> candidates;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__P3_SHADOW_HPP_
