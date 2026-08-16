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

#ifndef LOCAL_PLANNING__SAFE_STOP_LIFECYCLE_HPP_
#define LOCAL_PLANNING__SAFE_STOP_LIFECYCLE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace local_planning
{

enum class SafeStopReleaseReason
{
  kNone,
  kObstaclePassed,
  kValidAvoidance,
  kStoppedCorridorClear,
};

struct SafeStopActivation
{
  std::vector<int> obstacle_ids;
  std::uint64_t obstacle_sequence{0};
  std::int64_t obstacle_source_stamp_ns{0};
  double obstacle_s_start{0.0};
  double obstacle_s_end{0.0};
  double safe_stop_target_s{0.0};
  double activation_ego_s{0.0};
  std::int64_t activation_timestamp_ns{0};
  double danger_start_distance_m{0.0};
  double danger_end_distance_m{0.0};
  double safe_stop_target_distance_m{0.0};
};

struct SafeStopCycleInput
{
  double ego_s{0.0};
  double ego_speed_mps{0.0};
  bool static_obstacles_empty{true};
  bool hard_valid_avoidance_for_latched_obstacle{false};
  bool state_can_select_avoidance{false};
  bool explicit_forward_corridor_clear{false};
  bool replanned_no_obstacle{false};
  std::uint64_t obstacle_sequence{0};
};

struct SafeStopCycleDecision
{
  bool obstacle_passed{false};
  bool vehicle_stopped{false};
  bool release_condition_a{false};
  bool release_condition_b{false};
  bool release_condition_c{false};
  int feasible_avoidance_count{0};
  int stopped_clear_count{0};
  bool release_authorized{false};
  SafeStopReleaseReason release_reason{SafeStopReleaseReason::kNone};
  bool raceline_global_handoff_allowed{false};
};

class SafeStopLifecycle
{
public:
  void activate(SafeStopActivation activation);
  void reset();
  bool active() const;
  const SafeStopActivation & activation() const;

  SafeStopCycleDecision evaluate(
    const SafeStopCycleInput & input,
    double track_length_m,
    double obstacle_pass_margin_m,
    double stopped_speed_threshold_mps,
    int release_confirmation_cycles);

private:
  bool active_{false};
  SafeStopActivation activation_;
  int feasible_avoidance_count_{0};
  int stopped_clear_count_{0};
  std::uint64_t last_counted_clear_sequence_{0};
};

const char * safeStopReleaseReasonName(SafeStopReleaseReason reason);

}  // namespace local_planning

#endif  // LOCAL_PLANNING__SAFE_STOP_LIFECYCLE_HPP_
