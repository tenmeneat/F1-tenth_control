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

#include <gtest/gtest.h>

#include <vector>

#include "local_planning/safe_stop_lifecycle.hpp"

namespace local_planning
{
namespace
{

constexpr double kTrackLength = 100.0;
constexpr double kPassMargin = 0.4;
constexpr double kStoppedSpeed = 0.0625;
constexpr int kReleaseCycles = 8;

SafeStopActivation activation()
{
  SafeStopActivation value;
  value.obstacle_ids = {29};
  value.obstacle_sequence = 41;
  value.obstacle_source_stamp_ns = 10170000000LL;
  value.obstacle_s_start = 15.0;
  value.obstacle_s_end = 15.6;
  value.safe_stop_target_s = 14.6;
  value.activation_ego_s = 10.0;
  value.activation_timestamp_ns = 10170000000LL;
  value.danger_start_distance_m = 5.0;
  value.danger_end_distance_m = 5.6;
  value.safe_stop_target_distance_m = 4.6;
  return value;
}

TEST(SafeStopLifecycle, EmptyDropoutKeepsObstacleAheadLatchedBeyondEightCycles)
{
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 11.0;
  input.ego_speed_mps = 3.0;
  input.static_obstacles_empty = true;

  for (int cycle = 0; cycle < 20; ++cycle) {
    input.obstacle_sequence = 42 + cycle;
    const auto decision = lifecycle.evaluate(
      input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
    EXPECT_FALSE(decision.release_authorized);
    EXPECT_FALSE(decision.raceline_global_handoff_allowed);
    EXPECT_EQ(decision.stopped_clear_count, 0);
  }
  EXPECT_TRUE(lifecycle.active());
}

TEST(SafeStopLifecycle, ReleasesAfterEgoPassesDangerRegionWithMargin)
{
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 16.01;
  input.ego_speed_mps = 1.0;
  input.static_obstacles_empty = true;
  input.obstacle_sequence = 50;

  const auto decision = lifecycle.evaluate(
    input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
  EXPECT_TRUE(decision.obstacle_passed);
  EXPECT_TRUE(decision.release_authorized);
  EXPECT_EQ(decision.release_reason, SafeStopReleaseReason::kObstaclePassed);
  EXPECT_TRUE(decision.raceline_global_handoff_allowed);
}

TEST(SafeStopLifecycle, ReleasesToSelectableHardValidAvoidanceOnlyAfterDebounce)
{
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 11.0;
  input.ego_speed_mps = 2.0;
  input.static_obstacles_empty = false;
  input.hard_valid_avoidance_for_latched_obstacle = true;
  input.state_can_select_avoidance = true;

  for (int cycle = 1; cycle <= kReleaseCycles; ++cycle) {
    input.obstacle_sequence = 42 + cycle;
    const auto decision = lifecycle.evaluate(
      input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
    EXPECT_EQ(decision.release_authorized, cycle == kReleaseCycles);
    if (cycle == kReleaseCycles) {
      EXPECT_EQ(decision.release_reason, SafeStopReleaseReason::kValidAvoidance);
      EXPECT_FALSE(decision.raceline_global_handoff_allowed);
    }
  }
}

TEST(SafeStopLifecycle, EmptyFramesCannotProveStoppedCorridorClear)
{
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 14.5;
  input.ego_speed_mps = 0.0;
  input.static_obstacles_empty = true;
  input.explicit_forward_corridor_clear = true;

  for (int cycle = 0; cycle < 20; ++cycle) {
    input.obstacle_sequence = 42 + cycle;
    const auto decision = lifecycle.evaluate(
      input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
    EXPECT_FALSE(decision.release_condition_c);
    EXPECT_EQ(decision.stopped_clear_count, 0);
  }
}

TEST(SafeStopLifecycle, ReleasesStoppedVehicleAfterExplicitPersistentClearFrames)
{
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 14.5;
  input.ego_speed_mps = 0.0;
  input.static_obstacles_empty = false;
  input.explicit_forward_corridor_clear = true;

  for (int cycle = 1; cycle <= kReleaseCycles; ++cycle) {
    input.obstacle_sequence = 42 + cycle;
    const auto decision = lifecycle.evaluate(
      input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
    EXPECT_EQ(decision.release_condition_c, cycle == kReleaseCycles);
    if (cycle == kReleaseCycles) {
      EXPECT_EQ(
        decision.release_reason, SafeStopReleaseReason::kStoppedCorridorClear);
      EXPECT_TRUE(decision.raceline_global_handoff_allowed);
    }
  }
}

TEST(SafeStopLifecycle, DeterministicForRepeatedInputSequence)
{
  std::vector<SafeStopCycleDecision> first;
  std::vector<SafeStopCycleDecision> second;
  for (auto * output : {&first, &second}) {
    SafeStopLifecycle lifecycle;
    lifecycle.activate(activation());
    SafeStopCycleInput input;
    input.ego_s = 11.0;
    input.ego_speed_mps = 2.0;
    input.static_obstacles_empty = false;
    input.hard_valid_avoidance_for_latched_obstacle = true;
    input.state_can_select_avoidance = true;
    for (int cycle = 0; cycle < kReleaseCycles; ++cycle) {
      input.obstacle_sequence = 42 + cycle;
      output->push_back(lifecycle.evaluate(
          input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles));
    }
  }

  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0; index < first.size(); ++index) {
    EXPECT_EQ(first[index].obstacle_passed, second[index].obstacle_passed);
    EXPECT_EQ(first[index].feasible_avoidance_count, second[index].feasible_avoidance_count);
    EXPECT_EQ(first[index].release_authorized, second[index].release_authorized);
    EXPECT_EQ(first[index].release_reason, second[index].release_reason);
  }
}

TEST(SafeStopLifecycle, FsmFlapDuringHoldMustNotStarveAvoidanceRelease)
{
  // 2026-08-13 실차 재현 (run_0813_220641/221339): 홀드 중 2점 정지경로 때문에 state
  // machine이 AVOID↔GLOBAL로 틱마다 요동한다. hard-valid 회피가 매 틱 존재해도
  // GLOBAL 틱이 streak을 0으로 리셋하면 조건 B(kValidAvoidance)가 영원히 미달해
  // 수동 개입 없이는 홀드에서 빠져나오지 못한다. streak은 계획 유효성만 세고,
  // 해제 순간에만 FSM이 AVOID일 것을 요구해야 한다.
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 10.0;
  input.ego_speed_mps = 0.0;
  input.static_obstacles_empty = false;
  input.hard_valid_avoidance_for_latched_obstacle = true;

  bool released = false;
  for (int cycle = 0; cycle < 8 * kReleaseCycles; ++cycle) {
    input.obstacle_sequence = 42 + cycle;
    input.state_can_select_avoidance = (cycle % 2) == 0;
    const auto decision = lifecycle.evaluate(
      input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
    if (decision.release_authorized) {
      EXPECT_EQ(decision.release_reason, SafeStopReleaseReason::kValidAvoidance);
      // 해제는 반드시 FSM이 회피를 선택할 수 있는 틱에서만 일어나야 한다. GLOBAL 틱에
      // 해제하면 state machine이 글로벌 라인을 전달해 장애물을 관통한다.
      EXPECT_TRUE(input.state_can_select_avoidance);
      released = true;
      break;
    }
  }
  EXPECT_TRUE(released);
}

TEST(SafeStopLifecycle, InvalidPlanStillResetsStreakUnderFlap)
{
  // FSM 요동 내성이 계획 유효성 요건 자체를 약화하면 안 된다: hard-valid가 끊기면
  // streak은 여전히 0으로 돌아가야 한다.
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 10.0;
  input.ego_speed_mps = 0.0;
  input.static_obstacles_empty = false;

  for (int cycle = 0; cycle < 8 * kReleaseCycles; ++cycle) {
    input.obstacle_sequence = 42 + cycle;
    input.state_can_select_avoidance = (cycle % 2) == 0;
    // 유효 계획이 릴리즈 임계 직전까지 쌓이다 한 번 끊기는 패턴을 반복한다.
    input.hard_valid_avoidance_for_latched_obstacle =
      (cycle % kReleaseCycles) != kReleaseCycles - 1;
    const auto decision = lifecycle.evaluate(
      input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
    EXPECT_FALSE(decision.release_authorized) << "cycle " << cycle;
  }
  EXPECT_TRUE(lifecycle.active());
}

}  // namespace
}  // namespace local_planning
