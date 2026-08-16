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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "local_planning/raceline_spline_planner.hpp"

namespace local_planning
{
namespace
{

f110_msgs::msg::WpntArray makeStraightReference(
  int count = 300, double spacing = 0.1,
  double left_width = 1.5, double right_width = 1.5)
{
  f110_msgs::msg::WpntArray reference;
  reference.header.frame_id = "map";
  reference.wpnts.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = i;
    waypoint.s_m = static_cast<double>(i) * spacing;
    waypoint.x_m = waypoint.s_m;
    waypoint.y_m = 0.0;
    waypoint.psi_rad = 0.0;
    waypoint.kappa_radpm = 0.0;
    waypoint.vx_mps = 3.0;
    waypoint.d_left = left_width;
    waypoint.d_right = right_width;
    reference.wpnts.push_back(waypoint);
  }
  return reference;
}

f110_msgs::msg::WpntArray makeCircularReference(
  int count = 200, double radius = 5.0)
{
  f110_msgs::msg::WpntArray reference;
  reference.header.frame_id = "map";
  reference.wpnts.reserve(static_cast<std::size_t>(count));
  const double spacing = 2.0 * M_PI * radius / static_cast<double>(count);
  for (int i = 0; i < count; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count);
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = i;
    waypoint.s_m = static_cast<double>(i) * spacing;
    waypoint.x_m = radius * std::cos(angle);
    waypoint.y_m = radius * std::sin(angle);
    waypoint.psi_rad = angle + 0.5 * M_PI;
    waypoint.kappa_radpm = 1.0 / radius;
    waypoint.vx_mps = 3.0;
    waypoint.d_left = 1.5;
    waypoint.d_right = 1.5;
    reference.wpnts.push_back(waypoint);
  }
  return reference;
}

f110_msgs::msg::Obstacle makeObstacle(
  int id, double s, double d_right = -0.20, double d_left = 0.20)
{
  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = id;
  obstacle.s_center = s;
  obstacle.s_start = s - 0.20;
  obstacle.s_end = s + 0.20;
  obstacle.d_center = 0.5 * (d_right + d_left);
  obstacle.d_right = d_right;
  obstacle.d_left = d_left;
  obstacle.size = 0.40;
  obstacle.is_static = true;
  return obstacle;
}

RacelineSplineParameters testParameters()
{
  RacelineSplineParameters parameters;
  parameters.detection_lookahead_m = 12.0;
  parameters.vehicle_half_width_m = 0.12;
  parameters.safety_margin_m = 0.03;
  parameters.tracking_error_reserve_m = 0.0;
  parameters.maximum_curvature_radpm = 5.0;
  parameters.maximum_curvature_rate_radpm2 = 50.0;
  // Legacy safety-constraint tests isolate one target. Multi-target behavior is exercised by
  // dedicated candidate-generation tests below.
  parameters.target_d_candidate_count = 1;
  return parameters;
}

f110_msgs::msg::WpntArray makeStraightCandidate(
  const f110_msgs::msg::WpntArray & reference,
  double d,
  double yaw,
  std::size_t count = 20U)
{
  f110_msgs::msg::WpntArray path;
  path.header = reference.header;
  count = std::min(count, reference.wpnts.size());
  path.wpnts.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    auto waypoint = reference.wpnts[index];
    waypoint.id = static_cast<int32_t>(index);
    waypoint.d_m = d;
    waypoint.x_m = reference.wpnts[index].x_m;
    waypoint.y_m = reference.wpnts[index].y_m + d;
    waypoint.psi_rad = yaw;
    waypoint.kappa_radpm = 0.0;
    path.wpnts.push_back(waypoint);
  }
  return path;
}

TEST(RacelineSplinePlanner, RejectsNonMonotonicGlobalReference)
{
  auto reference = makeStraightReference(20);
  reference.wpnts[10].s_m = reference.wpnts[9].s_m;
  RacelineSplinePlanner planner(testParameters());
  std::string error;
  EXPECT_FALSE(planner.setReference(reference, &error));
  EXPECT_FALSE(error.empty());
}

TEST(RacelineSplinePlanner, P3NonpositiveEntryBoundaryFailsClosedWithoutThrowing)
{
  const auto reference = makeStraightReference(300, 0.1, 1.5, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));

  P3ShadowResult result;
  EXPECT_NO_THROW(
    result = planner.evaluateP3Shadow(
      EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(41, 0.2)},
      100, 1U, 1U, "NONPOSITIVE_BOUNDARY_TEST"));
  EXPECT_TRUE(result.invoked);
  EXPECT_FALSE(result.would_recover);
  EXPECT_TRUE(result.m1_invoked);
  EXPECT_EQ(result.m0_candidate_count, 0U);
  EXPECT_LE(result.candidate_count, 24U);
  ASSERT_EQ(result.cluster_obstacle_ids.size(), 1U);
  EXPECT_EQ(result.cluster_obstacle_ids.front(), 41);
  EXPECT_TRUE(std::isfinite(result.cluster_start_forward_m));
  EXPECT_TRUE(std::isfinite(result.cluster_end_forward_m));
  EXPECT_LE(result.cluster_start_forward_m, result.cluster_end_forward_m);
}

TEST(RacelineSplinePlanner, RejectsRotatedFootprintWhenCenterlineRemainsInsideLeftBound)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto rotated = makeStraightCandidate(reference, 0.15, 0.40);
  std::string reason;
  PathValidationFailure failure;

  EXPECT_FALSE(planner.validatePath(ego, rotated, {}, &reason, &failure));
  EXPECT_EQ(reason, "footprint_track_bound");
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kTrackBoundary);
  EXPECT_EQ(failure.footprint_violation_side, "left");
  EXPECT_GT(failure.centerline_wall_clearance, 0.0);
  EXPECT_LT(failure.rectangular_footprint_wall_clearance, 0.0);
  EXPECT_GT(failure.wallward_corner_protrusion, 0.0);
  EXPECT_NEAR(failure.heading_relative_to_reference, 0.40, 1.0e-12);
}

TEST(RacelineSplinePlanner, AcceptsHeadingAlignedFootprintAtSameCenterline)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const auto aligned = makeStraightCandidate(reference, 0.15, 0.0);
  std::string reason;
  PathValidationFailure failure;

  EXPECT_TRUE(planner.validatePath(
      EgoFrenetState{0.0, 0.0, 2.0}, aligned, {}, &reason, &failure)) << reason;
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kNone);
}

TEST(RacelineSplinePlanner, DetectsBothLeftAndRightFootprintViolations)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  for (const auto & test : std::vector<std::pair<double, std::string>>{
        {0.15, "left"}, {-0.15, "right"}})
  {
    PathValidationFailure failure;
    EXPECT_FALSE(planner.validatePath(
        ego, makeStraightCandidate(reference, test.first, 0.40), {}, nullptr, &failure));
    EXPECT_EQ(failure.kind, PathValidationFailureKind::kTrackBoundary);
    EXPECT_EQ(failure.footprint_violation_side, test.second);
    EXPECT_GT(failure.centerline_wall_clearance, 0.0);
    EXPECT_LT(failure.rectangular_footprint_wall_clearance, 0.0);
  }
}

TEST(RacelineSplinePlanner, PreservesOtherHardConstraintsAfterFootprintValidation)
{
  const auto reference = makeStraightReference(100, 0.1, 2.0, 2.0);
  auto parameters = testParameters();
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  auto path = makeStraightCandidate(reference, 0.0, 0.0);
  PathValidationFailure failure;

  EXPECT_FALSE(planner.validatePath(
      ego, path, {makeObstacle(501, 1.0)}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);

  path.wpnts.front().kappa_radpm = parameters.maximum_curvature_radpm + 0.1;
  EXPECT_FALSE(planner.validatePath(ego, path, {}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kGeometry);

  parameters.maximum_curvature_radpm = 100.0;
  parameters.maximum_curvature_rate_radpm2 = 1.0;
  RacelineSplinePlanner rate_planner(parameters);
  ASSERT_TRUE(rate_planner.setReference(reference));
  path = makeStraightCandidate(reference, 0.0, 0.0);
  path.wpnts[2].kappa_radpm = 1.0;
  EXPECT_FALSE(rate_planner.validatePath(ego, path, {}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kGeometry);
}

TEST(RacelineSplinePlanner, FootprintValidationIsBitDeterministic)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto path = makeStraightCandidate(reference, -0.15, -0.40);
  PathValidationFailure expected;
  ASSERT_FALSE(planner.validatePath(ego, path, {}, nullptr, &expected));

  for (int repetition = 0; repetition < 10; ++repetition) {
    PathValidationFailure actual;
    EXPECT_FALSE(planner.validatePath(ego, path, {}, nullptr, &actual));
    EXPECT_EQ(actual.kind, expected.kind);
    EXPECT_EQ(actual.reason, expected.reason);
    EXPECT_EQ(actual.waypoint_index, expected.waypoint_index);
    EXPECT_EQ(actual.footprint_violation_side, expected.footprint_violation_side);
    EXPECT_EQ(actual.centerline_wall_clearance, expected.centerline_wall_clearance);
    EXPECT_EQ(
      actual.rectangular_footprint_wall_clearance,
      expected.rectangular_footprint_wall_clearance);
    EXPECT_EQ(actual.waypoint_x, expected.waypoint_x);
    EXPECT_EQ(actual.waypoint_y, expected.waypoint_y);
    EXPECT_EQ(actual.waypoint_yaw, expected.waypoint_yaw);
    EXPECT_EQ(
      actual.heading_relative_to_reference,
      expected.heading_relative_to_reference);
    EXPECT_EQ(actual.wallward_corner_protrusion, expected.wallward_corner_protrusion);
  }
}

TEST(RacelineSplinePlanner, ShiftsOnlyOrderedGlobalRaceLineSamples)
{
  const auto reference = makeStraightReference();
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(7, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.path.wpnts.empty());

  double previous_forward = -1.0;
  bool saw_offset = false;
  for (const auto & waypoint : result.path.wpnts) {
    const double forward = planner.forwardDistance(ego.s, waypoint.s_m);
    EXPECT_GT(forward, previous_forward);
    previous_forward = forward;
    const std::size_t reference_index = static_cast<std::size_t>(
      std::llround(waypoint.s_m / 0.1));
    ASSERT_LT(reference_index, reference.wpnts.size());
    const auto & global = reference.wpnts[reference_index];
    EXPECT_NEAR(waypoint.x_m, global.x_m, 1.0e-9);
    EXPECT_NEAR(waypoint.y_m, waypoint.d_m, 1.0e-9);
    EXPECT_EQ(waypoint.s_m, global.s_m);
    EXPECT_EQ(waypoint.vx_mps, global.vx_mps);
    saw_offset = saw_offset || std::abs(waypoint.d_m) > 0.20;
  }
  EXPECT_TRUE(saw_offset);
  EXPECT_NEAR(result.path.wpnts.back().d_m, 0.0, 1.0e-6);
}

// 2026-08-16 시뮬 백(rosbag2_2026_08_16-08_50_21)의 실측 기하. 앞 장애물은 라인 왼쪽에
// 치우쳐 있어 우측으로 피하는데, 8 m 뒤 장애물은 라인 위에 걸쳐 있다. exit 스케일이 길면
// 복귀 램프가 오프셋을 유지한 채 뒤 장애물의 물리 엔벨로프를 지나가고, 그 경로는 커밋
// 재검증과 매 사이클 충돌해 25 ms마다 같은 후보를 다시 고르는 무한 재계획이 된다
// (실측: 랩당 hard collision 41회, s=28~30에서 완전 정지 랩당 2~4회).
// 뒤 장애물을 건드리지 않는 exit이 존재하면 그쪽이 선택되어야 한다.
TEST(RacelineSplinePlanner, PrefersExitThatClearsTheFollowingObstacle)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  parameters.entry_transition_fractions = {0.5, 0.75, 1.0};
  // 짧은/중간/아주 긴 exit. 마지막 값이 운영 YAML의 3.699 자리이며, 이것이 뒤 장애물을
  // 관통하는 후보를 만든다.
  parameters.transition_distance_scales = {0.5, 0.7, 3.7};
  RacelineSplinePlanner planner(parameters);
  // 백의 s=28~42 구간 회랑(d_left 1.18~1.28, d_right 0.73~0.90) 중 좁은 쪽으로 고정한다.
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.25, 1.20, 0.75)));

  auto blocking = makeObstacle(10, 3.57, 0.14, 0.47);   // 백 id10: s=31.44~31.77
  blocking.s_start = 3.40;
  blocking.s_end = 3.73;
  auto following = makeObstacle(0, 11.75, -0.18, 0.10);  // 백 id0: s=40.31~40.83, 라인 위
  following.s_start = 11.50;
  following.s_end = 12.00;

  const EgoFrenetState ego{0.0, -0.136, 2.17};
  const auto shadow = planner.evaluateP3Shadow(
    ego, {blocking, following}, 100, 1U, 1U, "FOLLOWING_OBSTACLE_EXIT_TEST");
  ASSERT_TRUE(shadow.invoked);
  ASSERT_FALSE(shadow.candidates.empty());
  ASSERT_NE(shadow.selected_path_digest, "NONE") << shadow.failure_classification;

  // 상황이 실제로 재현됐는지부터 확인한다: 뒤 장애물을 관통하는 exit 후보가 존재해야
  // 우선순위가 시험된다. 이게 0이면 테스트가 무의미하게 통과한다.
  std::size_t reaching = 0U;
  std::size_t clear_and_valid = 0U;
  for (const auto & candidate : shadow.candidates) {
    if (candidate.exit_reaches_next_obstacle) {
      ++reaching;
    } else if (candidate.hard_valid) {
      ++clear_and_valid;
    }
  }
  ASSERT_GT(reaching, 0U) << "no candidate carried its offset into the following obstacle; "
    "the ranking preference is not being exercised";
  ASSERT_GT(clear_and_valid, 0U) << "no clear alternative existed";

  // 선택된 후보는 그 관통 후보가 아니어야 한다.
  bool selected_found = false;
  for (const auto & candidate : shadow.candidates) {
    if (candidate.path_digest != shadow.selected_path_digest) {
      continue;
    }
    selected_found = true;
    EXPECT_FALSE(candidate.exit_reaches_next_obstacle)
      << "selected an exit ramp that carries offset into the following obstacle while a clear "
      "alternative existed";
  }
  EXPECT_TRUE(selected_found);

  // plan()의 순위도 같은 계약을 따라야 한다 (P3와 P0가 갈리면 서로 다른 경로를 커밋한다).
  const auto result = planner.plan(ego, {blocking, following});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  const double clearance = parameters.vehicle_half_width_m + parameters.safety_margin_m;
  for (const auto & waypoint : result.path.wpnts) {
    const double forward = planner.forwardDistance(ego.s, waypoint.s_m);
    if (forward + 1.0e-9 < following.s_start || forward > following.s_end + 1.0e-9) {
      continue;
    }
    if (std::abs(waypoint.d_m) <= 1.0e-3) {
      break;   // 합류 뒤 글로벌 꼬리 — 이 기동의 기하가 아니다.
    }
    EXPECT_FALSE(
      waypoint.d_m > following.d_right - clearance &&
      waypoint.d_m < following.d_left + clearance)
      << "plan() committed an exit ramp inside the following obstacle's envelope at forward="
      << forward << " d=" << waypoint.d_m;
  }
}

// 접근 제동 램프는 스팬마다 걸려야 한다. 예전에는 가장 가까운 스팬 하나만 대상이라, 두 번째
// 장애물 앞에서 gap 캡이 그대로 계단으로 나타났다 (2026-08-16 백: 0.25 m 만에 4.62 → 1.00,
// decel 2.0으로는 5.0 m가 필요한 감속을 요구).
TEST(RacelineSplinePlanner, BrakingRampCoversEveryObstacleSpanNotOnlyTheNearest)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  parameters.approach_feasibility_decel_mps2 = 2.0;
  // gap 기반 캡은 추종오차 tube가 속도에 따라 커질 때만 속도를 끌어내린다. 평평한
  // fallback reserve로는 감속해도 tube가 그대로라 캡 자체가 동작하지 않는다.
  parameters.tracking_error_lut_speed_bins_mps = {1.0, 5.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  parameters.tracking_error_lut_values_m = {0.05, 0.50};
  RacelineSplinePlanner planner(parameters);
  auto reference = makeStraightReference(300, 0.25, 1.20, 0.75);
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 5.0;   // 캡이 실제로 속도를 끌어내리도록 여유를 준다.
  }
  ASSERT_TRUE(planner.setReference(reference));

  // 두 장애물 모두 라인을 넘어 오른쪽까지 걸쳐 있어, 우측 통과 폭이 tube보다 좁다 —
  // 그래야 gap 캡이 실제로 속도를 끌어내린다.
  auto first = makeObstacle(10, 4.00, -0.10, 0.47);
  first.s_start = 3.85;
  first.s_end = 4.15;
  auto second = makeObstacle(0, 11.00, -0.14, 0.45);
  second.s_start = 10.85;
  second.s_end = 11.15;

  const EgoFrenetState ego{0.0, 0.0, 3.0};
  const auto result = planner.plan(ego, {first, second});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;

  // 두 스팬 모두에서 캡이 실제로 걸렸는지 먼저 확인한다. 안 걸렸으면 테스트가 무의미하다.
  const auto span_minimum = [&](double start, double end) {
      double minimum = std::numeric_limits<double>::infinity();
      for (const auto & waypoint : result.path.wpnts) {
        const double forward = planner.forwardDistance(ego.s, waypoint.s_m);
        if (forward >= start && forward <= end) {
          minimum = std::min(minimum, waypoint.vx_mps);
        }
      }
      return minimum;
    };
  ASSERT_LT(span_minimum(first.s_start, first.s_end), 5.0);
  ASSERT_LT(span_minimum(second.s_start, second.s_end), 5.0);

  // 어떤 연속 구간도 approach_feasibility_decel_mps2로 실현 불가능한 감속을 요구하면 안 된다.
  for (std::size_t i = 1; i < result.path.wpnts.size(); ++i) {
    const auto & previous = result.path.wpnts[i - 1U];
    const auto & current = result.path.wpnts[i];
    const double ds = planner.forwardDistance(previous.s_m, current.s_m);
    if (!(ds > 1.0e-6) || current.vx_mps >= previous.vx_mps) {
      continue;   // 가속 구간은 이 램프의 대상이 아니다.
    }
    const double required_decel =
      (previous.vx_mps * previous.vx_mps - current.vx_mps * current.vx_mps) / (2.0 * ds);
    EXPECT_LE(required_decel, parameters.approach_feasibility_decel_mps2 * 1.10)
      << "unreachable deceleration step at forward="
      << planner.forwardDistance(ego.s, current.s_m)
      << " (" << previous.vx_mps << " -> " << current.vx_mps << " over " << ds << " m)";
  }
}

TEST(RacelineSplinePlanner, RankingCentresPassBetweenObstacleAndWall)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 9;
  parameters.entry_transition_fractions = {1.0};
  parameters.transition_distance_scales = {1.0};
  parameters.maximum_lateral_slope = 100.0;
  parameters.maximum_curvature_radpm = 100.0;
  parameters.maximum_curvature_rate_radpm2 = 1000.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 1.20, 1.20)));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(311, 8.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;

  // Obstacle left face 0.20 against a 1.20 wall leaves a 1.00 m gap the car does not need all of.
  // Both ranking terms are measured from the vehicle body, so the selected offset must leave
  // comparable room on each side instead of hugging the wall. The pre-fix centerline metric
  // biased this by (obstacle clearance - wall margin) / 2 and left 0.21 m more room at the
  // obstacle than at the wall.
  const double body_to_wall = 1.20 - result.target_d - parameters.vehicle_half_width_m;
  const double body_to_obstacle = result.target_d - 0.20 - parameters.vehicle_half_width_m;
  EXPECT_GT(body_to_wall, 0.0);
  EXPECT_GT(body_to_obstacle, 0.0);
  EXPECT_LT(std::abs(body_to_wall - body_to_obstacle), 0.10);
}

TEST(RacelineSplinePlanner, RepeatedCandidateSelectionIsBitDeterministic)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const std::vector<f110_msgs::msg::Obstacle> obstacles{makeObstacle(121, 8.0)};
  const auto reference = planner.plan(ego, obstacles);
  ASSERT_EQ(reference.kind, SplinePlanKind::kAvoidance) << reference.reason;

  for (int repetition = 0; repetition < 10; ++repetition) {
    const auto repeated = planner.plan(ego, obstacles);
    ASSERT_EQ(repeated.kind, reference.kind);
    ASSERT_EQ(repeated.target_d, reference.target_d);
    ASSERT_EQ(repeated.go_left, reference.go_left);
    ASSERT_EQ(repeated.candidate_audits.size(), reference.candidate_audits.size());
    ASSERT_EQ(repeated.path.wpnts.size(), reference.path.wpnts.size());
    for (std::size_t index = 0; index < reference.path.wpnts.size(); ++index) {
      EXPECT_EQ(repeated.path.wpnts[index].s_m, reference.path.wpnts[index].s_m);
      EXPECT_EQ(repeated.path.wpnts[index].d_m, reference.path.wpnts[index].d_m);
      EXPECT_EQ(repeated.path.wpnts[index].x_m, reference.path.wpnts[index].x_m);
      EXPECT_EQ(repeated.path.wpnts[index].y_m, reference.path.wpnts[index].y_m);
      EXPECT_EQ(
        repeated.path.wpnts[index].kappa_radpm,
        reference.path.wpnts[index].kappa_radpm);
    }
    for (std::size_t index = 0; index < reference.candidate_audits.size(); ++index) {
      EXPECT_EQ(
        repeated.candidate_audits[index].final_rank,
        reference.candidate_audits[index].final_rank);
      EXPECT_EQ(
        repeated.candidate_audits[index].selected,
        reference.candidate_audits[index].selected);
    }
  }
}

TEST(RacelineSplinePlanner, KeepsQuinticAvoidanceValidOnCurvedReference)
{
  auto parameters = testParameters();
  parameters.transition_distance_scales = {1.0};
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeCircularReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto obstacle = makeObstacle(19, 8.0);

  const auto result = planner.plan(ego, {obstacle}, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  std::string reason;
  EXPECT_TRUE(planner.validatePath(ego, result.path, {obstacle}, &reason)) << reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_TRUE(std::isfinite(waypoint.kappa_radpm));
    EXPECT_LE(std::abs(waypoint.kappa_radpm), parameters.maximum_curvature_radpm);
  }
}

TEST(RacelineSplinePlanner, AppendsSpeedAwareGlobalTailAfterActualMerge)
{
  auto parameters = testParameters();
  parameters.post_merge_lookahead_m = 2.0;
  parameters.post_merge_min_time_sec = 1.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));

  const EgoFrenetState ego{0.0, 0.0, 6.0};
  const auto result = planner.plan(ego, {makeObstacle(12, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.path.wpnts.empty());

  const double tail_distance =
    planner.forwardDistance(result.merge_s, result.path.wpnts.back().s_m);
  EXPECT_GE(tail_distance, 5.8);
  EXPECT_NEAR(result.path.wpnts.back().d_m, 0.0, 1.0e-6);
}

TEST(RacelineSplinePlanner, BuildsClosedGlobalHandoffWithEgoInStateTail)
{
  const auto reference = makeCircularReference();
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));

  constexpr double kTailRatio = 0.10;
  const double ego_s = reference.wpnts[37].s_m;
  local_planning::EgoFrenetState handoff_ego;
  handoff_ego.s = ego_s;
  handoff_ego.d = 0.0;  // 라인 위에서의 핸드오프 — 램프 없이 순수 레이스라인이어야 한다
  handoff_ego.speed = 2.5;
  const auto path = planner.buildGlobalHandoffPath(handoff_ego, kTailRatio, 2.5);
  ASSERT_EQ(path.wpnts.size(), reference.wpnts.size());

  std::size_t closest_index = 0U;
  double closest_gap = std::numeric_limits<double>::infinity();
  double path_length = 0.0;
  for (std::size_t i = 0; i < path.wpnts.size(); ++i) {
    const double gap = planner.forwardDistance(ego_s, path.wpnts[i].s_m);
    const double circular_gap = std::min(gap, planner.trackLength() - gap);
    if (circular_gap < closest_gap) {
      closest_gap = circular_gap;
      closest_index = i;
    }
    EXPECT_DOUBLE_EQ(path.wpnts[i].d_m, 0.0);
    EXPECT_LE(path.wpnts[i].vx_mps, 2.5);
    if (i > 0U) {
      path_length += std::hypot(
        path.wpnts[i].x_m - path.wpnts[i - 1U].x_m,
        path.wpnts[i].y_m - path.wpnts[i - 1U].y_m);
    }
  }
  const std::size_t tail_count = static_cast<std::size_t>(
    std::ceil(kTailRatio * static_cast<double>(path.wpnts.size())));
  EXPECT_GE(closest_index, path.wpnts.size() - tail_count);

  const double average_spacing =
    path_length / static_cast<double>(path.wpnts.size() - 1U);
  const double closing_gap = std::hypot(
    path.wpnts.front().x_m - path.wpnts.back().x_m,
    path.wpnts.front().y_m - path.wpnts.back().y_m);
  EXPECT_LE(closing_gap, 2.0 * average_spacing);
}

TEST(RacelineSplinePlanner, RampedGlobalHandoffDecaysEgoOffsetToZero)
{
  const auto reference = makeCircularReference();
  auto ramp_params = testParameters();
  ramp_params.merge_ramp_min_length_m = 3.0;  // 기본 0 = 비활성이므로 명시 활성화
  ramp_params.merge_ramp_time_sec = 1.5;
  RacelineSplinePlanner planner(ramp_params);
  ASSERT_TRUE(planner.setReference(reference));

  constexpr double kTailDistanceM = 6.28;   // 구 0.20 비율과 같은 호 길이 (0.2 * 31.4 m)
  local_planning::EgoFrenetState ego;
  ego.s = reference.wpnts[37].s_m;
  ego.d = 0.40;   // 완료 시점에 남아 있는 회피 오프셋
  ego.speed = 2.0;  // ramp_length = max(3.0, 2.0*1.5) = 3.0 m
  const auto path = planner.buildGlobalHandoffPath(ego, kTailDistanceM, 2.5);
  ASSERT_EQ(path.wpnts.size(), reference.wpnts.size());

  const std::size_t total = path.wpnts.size();
  // 구현과 같은 정의: 경로 끝에서 거꾸로 호 길이를 걸어 tail 창을 정한다(균일 간격).
  const double spacing = 2.0 * M_PI * 5.0 / static_cast<double>(total);
  std::size_t tail_count = 1U;
  double walked = 0.0;
  while (tail_count < total && walked + spacing <= kTailDistanceM) {
    walked += spacing;
    ++tail_count;
  }
  const std::size_t tail_begin = total - tail_count;

  // ego 위치(회전 배열의 tail 첫 점)에서 d는 ego.d로 시작한다.
  EXPECT_NEAR(path.wpnts[tail_begin].d_m, ego.d, 1.0e-9);

  // 램프는 단조 감소하고, 3 m 전방 이후에는 정확히 0(레이스라인)이다.
  double forward_m = 0.0;
  double previous_d = path.wpnts[tail_begin].d_m;
  bool reached_zero = false;
  for (std::size_t k = tail_begin + 1U; k < total; ++k) {
    forward_m += std::hypot(
      path.wpnts[k].x_m - path.wpnts[k - 1U].x_m,
      path.wpnts[k].y_m - path.wpnts[k - 1U].y_m);
    EXPECT_LE(path.wpnts[k].d_m, previous_d + 1.0e-9);
    EXPECT_GE(path.wpnts[k].d_m, -1.0e-9);
    previous_d = path.wpnts[k].d_m;
    if (forward_m >= 3.1) {
      EXPECT_NEAR(path.wpnts[k].d_m, 0.0, 1.0e-9);
      reached_zero = true;
    }
  }
  EXPECT_TRUE(reached_zero);

  // 램프 구간의 좌표는 레이스라인 법선으로 d만큼 밀려 있어야 한다(경로-참조점 거리 = d).
  const auto & ramp_start = path.wpnts[tail_begin];
  const auto & reference_at_ego = reference.wpnts[37];  // ego_s = wpnts[37].s_m
  const double offset_distance = std::hypot(
    ramp_start.x_m - reference_at_ego.x_m, ramp_start.y_m - reference_at_ego.y_m);
  EXPECT_NEAR(offset_distance, std::abs(ego.d), 1.0e-6);

  // 램프 앞(한 바퀴 돌아오는 원거리 구간)은 순수 레이스라인이다.
  for (std::size_t k = 0; k < tail_begin; ++k) {
    EXPECT_DOUBLE_EQ(path.wpnts[k].d_m, 0.0);
  }
}

TEST(RacelineSplinePlanner, RampedGlobalHandoffClampsInsideWallPinch)
{
  auto reference = makeCircularReference();
  // ego(인덱스 37) 전방 5~12점 구간을 왼쪽 벽 협착부로 만든다.
  for (std::size_t i = 42; i <= 49; ++i) {
    reference.wpnts[i].d_left = 0.20;
  }
  auto params = testParameters();
  params.merge_ramp_min_length_m = 3.0;  // 기본 0 = 비활성이므로 명시 활성화
  params.merge_ramp_time_sec = 1.5;
  RacelineSplinePlanner planner(params);
  ASSERT_TRUE(planner.setReference(reference));

  constexpr double kTailDistanceM = 6.28;   // 구 0.20 비율과 같은 호 길이
  local_planning::EgoFrenetState ego;
  ego.s = reference.wpnts[37].s_m;
  ego.d = 0.40;  // 왼쪽 오프셋 → 왼쪽 협착부가 클램프를 강제한다
  ego.speed = 2.0;
  const auto path = planner.buildGlobalHandoffPath(ego, kTailDistanceM, 2.5);
  ASSERT_FALSE(path.wpnts.empty());

  const std::size_t total = path.wpnts.size();
  const double spacing = 2.0 * M_PI * 5.0 / static_cast<double>(total);
  std::size_t tail_count = 1U;
  double walked = 0.0;
  while (tail_count < total && walked + spacing <= kTailDistanceM) {
    walked += spacing;
    ++tail_count;
  }
  const std::size_t tail_begin = total - tail_count;
  const double keepout = params.vehicle_half_width_m + params.wall_safety_margin_m;
  const double allowed_in_pinch = std::max(0.0, 0.20 - keepout);

  double previous_d = path.wpnts[tail_begin].d_m;
  for (std::size_t k = tail_begin; k < total; ++k) {
    const std::size_t reference_index =
      static_cast<std::size_t>((37 + (k - tail_begin)) % total);
    if (reference_index >= 42 && reference_index <= 49) {
      // 협착부에서는 프로파일이 아직 크더라도 벽 여유 한도 안으로 눌린다.
      EXPECT_LE(path.wpnts[k].d_m, allowed_in_pinch + 1.0e-9)
        << "k=" << k << " ref=" << reference_index;
    }
    // 클램프 후 다시 넓어져도 되돌아 나가지 않는다(단조 비증가).
    EXPECT_LE(path.wpnts[k].d_m, previous_d + 1.0e-9);
    previous_d = path.wpnts[k].d_m;
  }
}

TEST(RacelineSplinePlanner, UsesRightSideWhenLeftTrackSpaceIsInsufficient)
{
  auto reference = makeStraightReference(300, 0.1, 0.45, 1.5);
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.plan(EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.go_left);
  EXPECT_LT(result.target_d, 0.0);
}

TEST(RacelineSplinePlanner, RejectsWallBlockedTargetsBeforeSplineConstruction)
{
  auto reference = makeStraightReference(300, 0.1, 0.34, 0.34);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});

  // 양쪽 모두 트랙 경계에 막히면 스플라인을 만들기 전에 안전정지로 간다. 사유 문자열은
  // 후보 생성기(P3)의 것이므로 문구가 아니라 판정만 계약으로 본다.
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  EXPECT_FALSE(result.reason.empty());
}

TEST(RacelineSplinePlanner, AppliesPhysicalVehicleClearanceOnceToTrackBounds)
{
  // d_left/d_right describe physical boundaries. At heading=0 with half-width 0.12, d=0.23 is
  // exactly tangent for a 0.35 m left boundary and remains valid at numerical tolerance.
  const auto feasible_reference = makeStraightReference(300, 0.1, 0.35, 0.35);
  RacelineSplinePlanner feasible_planner(testParameters());
  ASSERT_TRUE(feasible_planner.setReference(feasible_reference));
  std::string reason;
  EXPECT_TRUE(feasible_planner.validatePath(
      EgoFrenetState{0.0, 0.0, 2.0},
      makeStraightCandidate(feasible_reference, 0.23, 0.0), {}, &reason)) << reason;

  const auto blocked_reference = makeStraightReference(300, 0.1, 0.34, 0.34);
  RacelineSplinePlanner blocked_planner(testParameters());
  ASSERT_TRUE(blocked_planner.setReference(blocked_reference));
  EXPECT_FALSE(blocked_planner.validatePath(
      EgoFrenetState{0.0, 0.0, 2.0},
      makeStraightCandidate(blocked_reference, 0.23, 0.0), {}, &reason));
}

TEST(RacelineSplinePlanner, BreaksCentredObstacleScoreTieWithTrackHeadroom)
{
  auto reference = makeCircularReference();
  for (auto & waypoint : reference.wpnts) {
    waypoint.d_left = 0.9;
  }
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  // The right side offers more balanced obstacle/wall safety slack than the narrow left side.
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 8.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.go_left);
  EXPECT_LT(result.target_d, -0.35);
}

TEST(RacelineSplinePlanner, HonorsCommittedSideWhenItRemainsFeasible)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(2, 7.0)}, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_TRUE(result.go_left);
}

TEST(RacelineSplinePlanner, AppliesSingleSafetyMarginToAvoidanceTarget)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(2, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  // raw d_left 0.20 + vehicle half-width 0.12 + the only safety margin 0.03 = 0.35 최소치.
  // P3는 최소 clearance 지점이 아니라 slack이 가장 큰 후보를 고르므로 그보다 멀 수 있다.
  // 여기서 지켜야 할 계약은 "안전마진이 정확히 한 번만 적용된다"이므로 하한으로 검사한다.
  EXPECT_GE(result.target_d, 0.35 - 1.0e-9);
}

TEST(RacelineSplinePlanner, AppliesTrackingErrorReserveAsSeparateClearanceTerm)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.14;
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0), 0.29, 1.0e-9);
  EXPECT_DOUBLE_EQ(parameters.trackBoundaryReserve(2.0, 0.0), 0.0);

  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(2, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  // raw d_left 0.20 + half-width 0.12 + safety 0.03 + tracking reserve 0.14 = 0.49 최소치.
  // 추종오차 예약이 별도 항으로 한 번 더 들어간다는 계약을 하한으로 검사한다(위 참고).
  EXPECT_GE(result.target_d, 0.49 - 1.0e-9);
}

TEST(RacelineSplinePlanner, BilinearlyInterpolatesTrackingErrorLut)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.99;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 2.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 0.5};
  parameters.tracking_error_lut_values_m = {0.02, 0.04, 0.06, 0.10};

  ASSERT_TRUE(parameters.trackingErrorLutValid());
  EXPECT_NEAR(parameters.trackingErrorReserve(1.0, 0.25), 0.055, 1.0e-9);
  EXPECT_NEAR(parameters.trackingErrorReserve(-1.0, -0.25), 0.055, 1.0e-9);
  EXPECT_NEAR(parameters.trackingErrorReserve(10.0, 2.0), 0.10, 1.0e-9);

  parameters.tracking_error_lut_values_m.pop_back();
  EXPECT_FALSE(parameters.trackingErrorLutValid());
  EXPECT_NEAR(parameters.trackingErrorReserve(1.0, 0.25), 0.99, 1.0e-9);
}

TEST(RacelineSplinePlanner, LimitsAvoidanceSpeedFromVelocityTable)
{
  auto parameters = testParameters();
  parameters.avoidance_velocity_limit_speed_bins_mps =
  {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 =
  {7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 6.5, 6.5, 6.5, 6.5};

  ASSERT_TRUE(parameters.avoidanceVelocityLimitValid());
  EXPECT_DOUBLE_EQ(parameters.limitedAvoidanceSpeed(6.5, 0.0), 6.5);
  EXPECT_DOUBLE_EQ(parameters.limitedAvoidanceSpeed(3.0, 0.5), 3.0);
  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(6.5, 0.2), 5.754463, 1.0e-6);
  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(6.5, 0.5), std::sqrt(14.0), 1.0e-9);
  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(-6.5, -0.9), std::sqrt(7.0 / 0.9), 1.0e-9);

  parameters.avoidance_velocity_limit_lateral_accel_mps2[2] = 7.5;
  EXPECT_FALSE(parameters.avoidanceVelocityLimitValid());
}

TEST(RacelineSplinePlanner, UsesLimitedAvoidanceSpeedForObstacleTrackingLut)
{
  auto parameters = testParameters();
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 4.0, 6.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  parameters.tracking_error_lut_values_m = {0.0, 0.04, 0.12};
  parameters.avoidance_velocity_limit_speed_bins_mps = {0.0, 9.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 = {2.0, 2.0};

  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(6.0, 0.5), 2.0, 1.0e-9);
  EXPECT_NEAR(parameters.avoidanceTrackingErrorReserve(6.0, 0.5), 0.02, 1.0e-9);
  EXPECT_NEAR(parameters.obstacleSafetyClearance(6.0, 0.5), 0.17, 1.0e-9);
}

TEST(RacelineSplinePlanner, CapsPublishedAvoidanceWaypointSpeeds)
{
  auto parameters = testParameters();
  parameters.avoidance_velocity_limit_speed_bins_mps = {0.0, 9.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 = {2.0, 2.0};
  auto reference = makeStraightReference();
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 6.0;
  }

  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(33, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_LE(
      waypoint.vx_mps * waypoint.vx_mps * std::abs(waypoint.kappa_radpm),
      2.0 + 1.0e-9);
  }
}

TEST(RacelineSplinePlanner, UsesObstacleSpanMaximumTrackingLutReserveForTarget)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.0;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 3.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  parameters.tracking_error_lut_values_m = {0.0, 0.10};
  // 트랙을 좁혀 유효 target 창을 [0.45, 0.46]으로 만든다: 벽 상한 = 0.58 − 0.12(반폭)
  // − 0.0(기본 벽마진) = 0.46. 스팬 최대 LUT 예약(0.10)을 빠뜨린 플래너라면 0.45 미만
  // (예: [0.35, 0.46]의 slack 최대 지점)을 골라 하한 검사에 걸린다.
  auto reference = makeStraightReference(300, 0.1, 0.58, 0.58);
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = waypoint.s_m >= 6.0 && waypoint.s_m <= 8.0 ? 3.0 : 0.0;
  }

  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(31, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  // raw d_left 0.20 + base clearance 0.15 + maximum span LUT reserve 0.10 = 0.45 최소치.
  EXPECT_GE(result.target_d, 0.45 - 1.0e-9);
  EXPECT_LE(result.target_d, 0.46 + 1.0e-9);
}

TEST(RacelineSplinePlanner, AppliesOnlyWallMarginToTrackBounds)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.0;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 3.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 1.0};
  parameters.tracking_error_lut_values_m = {0.0, 0.0, 0.10, 0.10};
  parameters.wall_safety_margin_m = 0.04;
  EXPECT_DOUBLE_EQ(parameters.trackBoundaryReserve(3.0, 1.0), 0.04);

  RacelineSplinePlanner feasible_planner(parameters);
  ASSERT_TRUE(feasible_planner.setReference(makeStraightReference(300, 0.1, 0.62, 0.62)));
  const auto feasible = feasible_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(32, 7.0)}, true, false);
  ASSERT_EQ(feasible.kind, SplinePlanKind::kAvoidance) << feasible.reason;
  // 하한 = 장애물 clearance(0.45), 상한 = 벽 한계 0.62 − 0.12(반폭) − 0.04(벽마진) = 0.46.
  // P3는 이 창 안에서 slack 최대 지점을 고르므로 정확값이 아니라 창 준수를 계약으로 본다.
  EXPECT_GE(feasible.target_d, 0.45 - 1.0e-9);
  EXPECT_LE(feasible.target_d, 0.46 + 1.0e-9);

  // 0.60 m of room no longer safe-stops: the strict gate does not fit, but slowing the pass to
  // avoidance_minimum_speed_mps shrinks the reserve enough that a target does. Only a corridor too
  // narrow even at that floor is refused, and the target gate now says so before any spline is
  // built instead of generating candidates the footprint check must throw away.
  RacelineSplinePlanner slowed_planner(parameters);
  ASSERT_TRUE(slowed_planner.setReference(makeStraightReference(300, 0.1, 0.60, 0.60)));
  const auto slowed = slowed_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(32, 7.0)}, true, false);
  ASSERT_EQ(slowed.kind, SplinePlanKind::kAvoidance) << slowed.reason;
  EXPECT_LT(slowed.target_d, 0.45);
  EXPECT_LE(slowed.target_d + parameters.vehicle_half_width_m, 0.60 - 0.04 + 1.0e-9);

  RacelineSplinePlanner blocked_planner(parameters);
  ASSERT_TRUE(blocked_planner.setReference(makeStraightReference(300, 0.1, 0.55, 0.55)));
  const auto blocked = blocked_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(32, 7.0)}, true, false);
  EXPECT_EQ(blocked.kind, SplinePlanKind::kSafeStop) << blocked.reason;
  // 사유 문자열은 후보 생성기(P3)의 것이므로 판정만 계약으로 본다.
  EXPECT_FALSE(blocked.reason.empty());
}

TEST(RacelineSplinePlanner, AppliesIndependentWallSafetyMarginOnlyToTrackBounds)
{
  auto parameters = testParameters();
  parameters.wall_safety_margin_m = 0.05;
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0), 0.15, 1.0e-9);
  EXPECT_DOUBLE_EQ(parameters.trackBoundaryReserve(2.0, 0.0), 0.05);

  RacelineSplinePlanner feasible_planner(parameters);
  ASSERT_TRUE(feasible_planner.setReference(makeStraightReference(300, 0.1, 0.53, 0.53)));
  const auto feasible = feasible_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});
  ASSERT_EQ(feasible.kind, SplinePlanKind::kAvoidance) << feasible.reason;
  // 하한 = clearance 0.35, 상한 = 0.53 − 0.12(반폭) − 0.05(벽마진) = 0.36 (위 테스트 참고).
  EXPECT_GE(std::abs(feasible.target_d), 0.35 - 1.0e-9);
  EXPECT_LE(std::abs(feasible.target_d), 0.36 + 1.0e-9);

  const auto tangent_reference = makeStraightReference(300, 0.1, 0.40, 0.40);
  RacelineSplinePlanner tangent_planner(parameters);
  ASSERT_TRUE(tangent_planner.setReference(tangent_reference));
  std::string reason;
  EXPECT_TRUE(tangent_planner.validatePath(
      EgoFrenetState{0.0, 0.0, 2.0},
      makeStraightCandidate(tangent_reference, 0.23, 0.0), {}, &reason)) << reason;

  RacelineSplinePlanner blocked_planner(parameters);
  ASSERT_TRUE(blocked_planner.setReference(makeStraightReference(300, 0.1, 0.39, 0.39)));
  const auto blocked = blocked_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});
  EXPECT_EQ(blocked.kind, SplinePlanKind::kSafeStop) << blocked.reason;
}

TEST(RacelineSplinePlanner, RejectsCommittedPathWhenObstacleEnvelopeGrows)
{
  // 왼쪽 폭 0.475로 커밋 경로의 plateau를 [0.35, 0.355]로 강제한다(벽 상한 = 0.475 −
  // 0.12(반폭) − 0.0(기본 벽마진) = 0.355). 그래야 장애물 1 cm 성장(외피 0.35 → 0.36)이
  // 실제 위반이 된다. 넓은 트랙에서는 P3가 slack 최대 지점을 골라 위반이 안 난다.
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.475, 1.5)));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(2, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  std::string reason;
  EXPECT_TRUE(
    planner.validatePath(
      EgoFrenetState{0.2, 0.0, 2.0}, committed.path,
      {makeObstacle(2, 7.0)}, &reason)) << reason;
  EXPECT_FALSE(
    planner.validatePath(
      EgoFrenetState{0.2, 0.0, 2.0}, committed.path,
      {makeObstacle(2, 7.0, -0.21, 0.21)}, &reason));
}

TEST(RacelineSplinePlanner, UsesSameClearanceForGuardAndRawObstacleInputs)
{
  // 위 테스트와 같은 이유로 plateau를 [0.35, 0.355]로 강제한다.
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.475, 1.5)));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(23, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  // Soft/hard behavior differs only by the input envelope. Both calls add the same 0.15 m:
  // the uncertainty Guard reaches d=0.21 and collides, while raw geometry reaches d=0.19
  // and remains clear.
  const auto uncertainty_guard = makeObstacle(23, 7.0, -0.20, 0.21);
  std::string reason;
  PathValidationFailure failure;
  EXPECT_FALSE(
    planner.validatePath(
      ego, committed.path, {uncertainty_guard}, &reason, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);
  EXPECT_EQ(failure.obstacle_id, 23);
  EXPECT_TRUE(std::isfinite(failure.waypoint_s));
  EXPECT_TRUE(std::isfinite(failure.waypoint_d));
  EXPECT_NEAR(failure.obstacle_source_d_left, 0.21, 1.0e-9);
  EXPECT_NEAR(failure.obstacle_test_d_left, 0.36, 1.0e-9);
  EXPECT_NEAR(failure.obstacle_clearance, 0.15, 1.0e-9);

  EXPECT_TRUE(
    planner.validatePath(
      ego, committed.path, {makeObstacle(23, 7.0, -0.20, 0.19)},
      &reason, &failure)) << reason;
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kNone);
}

TEST(RacelineSplinePlanner, IgnoresPostMergeTailCollisionForCurrentCommitment)
{
  auto parameters = testParameters();
  parameters.post_apex_distances_m = {1.5, 3.0, 4.0};
  parameters.transition_distance_scales = {1.0};
  // P3 경로는 merge가 s≈15에 온다. merge 뒤에 놓는 다음 장애물이 기본 lookahead(12 m)
  // 밖으로 나가 검증이 공허하게 통과하지 않도록 늘린다.
  parameters.detection_lookahead_m = 20.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(24, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  const double next_obstacle_s = committed.merge_s + 0.7;
  const auto next_obstacle = makeObstacle(25, next_obstacle_s, -0.20, 0.20);
  std::string reason;
  PathValidationFailure failure;
  EXPECT_FALSE(
    planner.validatePath(
      ego, committed.path, {next_obstacle}, &reason, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);
  EXPECT_GT(failure.waypoint_s, committed.merge_s);

  const double merge_horizon = planner.forwardDistance(ego.s, committed.merge_s);
  EXPECT_TRUE(
    planner.validatePath(
      ego, committed.path, {next_obstacle}, &reason, &failure,
      merge_horizon)) << reason;
}

TEST(RacelineSplinePlanner, StartsNextManeuverContinuouslyFromNonzeroEgoD)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{9.05, 0.50, 2.0};
  const auto result = planner.plan(
    ego, {makeObstacle(26, 13.5)}, true, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.path.wpnts.empty());

  double previous_s = ego.s;
  double previous_d = ego.d;
  for (const auto & waypoint : result.path.wpnts) {
    const double ds = planner.forwardDistance(previous_s, waypoint.s_m);
    ASSERT_GT(ds, 0.0);
    EXPECT_LE(
      std::abs(waypoint.d_m - previous_d) / ds,
      testParameters().maximum_lateral_slope + 1.0e-6);
    previous_s = waypoint.s_m;
    previous_d = waypoint.d_m;
  }
  EXPECT_NEAR(result.path.wpnts.front().d_m, ego.d, 0.02);
}

TEST(RacelineSplinePlanner, DoesNotReverseCommittedSideWhenItBecomesBlocked)
{
  auto reference = makeStraightReference(300, 0.1, 0.34, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto unlocked = planner.plan(ego, {makeObstacle(3, 7.0)}, true, true);
  ASSERT_EQ(unlocked.kind, SplinePlanKind::kAvoidance) << unlocked.reason;
  EXPECT_FALSE(unlocked.go_left);

  const auto locked = planner.plan(ego, {makeObstacle(3, 7.0)}, true, false);
  EXPECT_EQ(locked.kind, SplinePlanKind::kSafeStop) << locked.reason;
}

TEST(RacelineSplinePlanner, IgnoresObstacleWithEnoughRawRacelineClearance)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(4, 7.0, 0.35, 0.55)});
  EXPECT_EQ(result.kind, SplinePlanKind::kNoObstacle) << result.reason;
  EXPECT_TRUE(result.path.wpnts.empty());
}

TEST(RacelineSplinePlanner, BuildsCollisionFreeStopWhenBothSidesAreClosed)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 7.0, -0.40, 0.40)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 2U);
  EXPECT_NEAR(result.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.0);
  }
  EXPECT_TRUE(std::any_of(
      result.path.wpnts.begin(), result.path.wpnts.end(),
      [&parameters](const auto & waypoint) {
        return waypoint.ax_mps2 <
               -0.99 * parameters.safe_stop_deceleration_mps2;
      }));

  const auto repeated = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 7.0, -0.40, 0.40)});
  ASSERT_EQ(repeated.kind, SplinePlanKind::kSafeStop) << repeated.reason;
  ASSERT_EQ(repeated.path.wpnts.size(), result.path.wpnts.size());
  for (std::size_t index = 0; index < result.path.wpnts.size(); ++index) {
    EXPECT_DOUBLE_EQ(repeated.path.wpnts[index].vx_mps, result.path.wpnts[index].vx_mps);
    EXPECT_DOUBLE_EQ(repeated.path.wpnts[index].ax_mps2, result.path.wpnts[index].ax_mps2);
  }
}

TEST(RacelineSplinePlanner, BuildsSafeStopAtCurrentLateralOffset)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.30, 2.0},
    {makeObstacle(27, 7.0, -0.40, 0.40)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 2U);
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_NEAR(waypoint.d_m, 0.30, 1.0e-9);
  }
  EXPECT_NEAR(result.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
}

TEST(RacelineSplinePlanner, BrakesOnCommittedGeometryBeforeEmergencyHold)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto committed = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(28, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  const auto ego_waypoint = std::find_if(
    committed.path.wpnts.begin(), committed.path.wpnts.end(),
    [](const auto & waypoint) {return waypoint.s_m >= 3.0;});
  ASSERT_NE(ego_waypoint, committed.path.wpnts.end());
  const EgoFrenetState ego{ego_waypoint->s_m, ego_waypoint->d_m, 2.0};
  const auto stop = planner.buildCommittedPathStop(
    ego, committed.path, {makeObstacle(29, 9.0, -1.0, 1.0)});

  ASSERT_EQ(stop.kind, SplinePlanKind::kSafeStop) << stop.reason;
  ASSERT_GE(stop.path.wpnts.size(), 2U);
  EXPECT_NEAR(stop.path.wpnts.front().d_m, ego.d, 0.05);
  EXPECT_NEAR(stop.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
  EXPECT_TRUE(
    std::any_of(
      stop.path.wpnts.begin(), stop.path.wpnts.end(),
      [](const auto & waypoint) {return std::abs(waypoint.d_m) > 0.10;}));

  auto off_path_ego = ego;
  off_path_ego.d += 0.10;
  const auto discontinuous_stop = planner.buildCommittedPathStop(
    off_path_ego, committed.path, {makeObstacle(29, 9.0, -1.0, 1.0)});
  EXPECT_EQ(discontinuous_stop.kind, SplinePlanKind::kNoSafePath);
  EXPECT_TRUE(discontinuous_stop.path.wpnts.empty());
  EXPECT_NE(
    discontinuous_stop.reason.find("discontinuous from the current ego d"),
    std::string::npos);
}

TEST(RacelineSplinePlanner, BuildsPreparationStopForInitialBlockingCluster)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.buildPreparationStop(
    EgoFrenetState{0.0, 0.0, 2.0},
    {makeObstacle(5, 7.0), makeObstacle(6, 7.6)});

  ASSERT_EQ(result.kind, SplinePlanKind::kPreparation) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 2U);
  EXPECT_EQ(result.obstacle_id, 5);
  EXPECT_EQ(result.obstacle_ids, (std::vector<int>{5, 6}));
  EXPECT_NEAR(result.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.0);
  }
}

TEST(RacelineSplinePlanner, ReportsWholeBlockingClusterInAvoidanceResult)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0},
    {makeObstacle(5, 7.0), makeObstacle(6, 7.6)});

  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_EQ(result.obstacle_ids, (std::vector<int>{5, 6}));
}

TEST(RacelineSplinePlanner, ReportsNearestBlockingClusterIdsForManeuverChaining)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto cluster_ids = planner.blockingClusterIds(
    EgoFrenetState{0.0, 0.0, 2.0},
      {
        makeObstacle(5, 7.0),
        makeObstacle(6, 7.6),
        makeObstacle(9, 10.0),
      });

  EXPECT_EQ(cluster_ids, (std::vector<int>{5, 6}));
}

TEST(RacelineSplinePlanner, RefusesPreparationDelayInsideSafeStopBuffer)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.buildPreparationStop(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 1.0)});

  EXPECT_EQ(result.kind, SplinePlanKind::kNoSafePath);
  EXPECT_TRUE(result.path.wpnts.empty());
  EXPECT_NE(result.reason.find("inside the safe-stop buffer"), std::string::npos);
}

// 짧은 안전정지 prefix는 여전히 허용된다(minimum_path_points 미달을 이유로 거부하지
// 않는다). 다만 2026-08-14부터는 그 상태로 내보내지 않고 minimum_path_points까지
// 세분 보간한다. 제어기가 룩어헤드 지점의 속도를 읽기 때문에, 2~3점짜리 경로에서는
// 룩어헤드가 곧바로 끝점 0에 걸려 감속 프로파일을 통째로 건너뛰고 즉시 정지를 명령한다
// (실차 관측: /local_waypoints [1.08, 0.00] -> /drive_autonomous 0.00, 28.8초 교착).
// 보간은 점 수만 늘릴 뿐 정지 지점(기하 구간)을 늘려서는 안 된다.
TEST(RacelineSplinePlanner, DensifiesShortSafeStopPrefixToMinimumPoints)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  parameters.minimum_path_points = 8;
  parameters.safe_stop_buffer_m = 0.80;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 1.65, -0.40, 0.40)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 8U);

  // 기하 구간은 짧은 그대로여야 한다 — 보간이 정지점을 밀어내지 않았음을 확인한다.
  const double span = result.path.wpnts.back().s_m - result.path.wpnts.front().s_m;
  EXPECT_LT(span, 1.30);

  // 감속 프로파일이 살아 있어야 한다: 단조 비증가 + 종점 0.
  EXPECT_DOUBLE_EQ(result.path.wpnts.back().vx_mps, 0.0);
  EXPECT_GT(result.path.wpnts.front().vx_mps, 0.0);
  for (std::size_t i = 1U; i < result.path.wpnts.size(); ++i) {
    EXPECT_LE(result.path.wpnts[i].vx_mps, result.path.wpnts[i - 1U].vx_mps + 1e-9)
      << "index " << i;
  }
}

// 정지점 탈출 검증: 정지한 자리에서 회피 후보가 하나도 생성되지 않으면 그 사실이
// 결과에 남아야 한다. 이전에는 아무 표시 없이 정지해 현장에서 30초씩 매달렸다.
TEST(RacelineSplinePlanner, ReportsWhenSafeStopPointIsNotEscapable)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  parameters.minimum_path_points = 8;
  parameters.safe_stop_buffer_m = 0.80;
  parameters.safe_stop_escape_check_enable = true;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  // 트랙 폭을 가득 막는 장애물 — 어느 지점에서도 회피가 불가능하다.
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 3.0, -1.20, 1.20)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  EXPECT_FALSE(result.safe_stop_escape_verified);
  EXPECT_NE(result.reason.find("no escapable stop point"), std::string::npos);
  // 탈출이 어차피 불가능하면 후퇴는 아무것도 사지 못하므로 원래 정지점을 지켜야 한다.
  EXPECT_GT(result.safe_stop_forward_m, 1.0);
}

// 🔴 좌표계 회귀 가드 (2026-08-14 리뷰). ExpandedObstacle의 center/start/end는 자차
// 상대거리라, 가상의 정지점으로 ego.s만 옮기고 기존 visible/cluster를 재사용하면 장애물이
// 정지점에서도 같은 거리에 있는 것으로 보여 검증이 통째로 무의미해진다(이분탐색도 항상
// 같은 답을 낸다). buildSafeStop은 반드시 절대 s 원본으로 정지점 기준 재확장해야 한다.
//
// 검사 방법: 버퍼를 탈출 임계보다 크게 잡아 요청 정지점을 "장애물에서 너무 먼" 쪽이 아니라
// 자차에 가까운 쪽으로 두고, 요청 정지점과 실제 채택된 정지점이 다른지 본다. 좌표계가
// 틀렸다면 재확장이 없으므로 후퇴 탐색이 아무 효과를 못 내고 요청값 그대로 남는다.
TEST(RacelineSplinePlanner, EscapeCheckReexpandsObstaclesAtTheCandidateStopPoint)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  parameters.minimum_path_points = 8;
  // 임계보다 작은 버퍼 → 요청 정지점은 탈출 불가 구역 안. 검증이 살아 있으면 뒤로 물린다.
  parameters.safe_stop_buffer_m = 0.30;
  parameters.safe_stop_escape_check_enable = true;
  parameters.safe_stop_escape_retreat_step_m = 0.10;
  parameters.safe_stop_escape_max_retreats = 10;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto obstacle = makeObstacle(5, 6.0, -0.40, 0.40);
  const auto result = planner.plan(ego, {obstacle});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;

  if (result.safe_stop_escape_verified) {
    // 후퇴가 성공했다면 채택된 정지점에서 실제로 회피가 나와야 한다 — 판정과 재계획이
    // 같은 후보 생성기를 쓰므로 이 두 값은 반드시 일치한다.
    const EgoFrenetState at_stop{
      ego.s + result.safe_stop_forward_m, ego.d, 0.0};
    const auto replan = planner.plan(at_stop, {obstacle});
    EXPECT_EQ(replan.kind, SplinePlanKind::kAvoidance)
      << "정지점에서 회피 가능하다고 판정했는데 실제 재계획은 실패했다: " << replan.reason;
  }
  // 좌표계가 틀렸을 때 나타나는 형태: 정지점이 자차 뒤로 가거나 장애물을 넘어선다.
  EXPECT_GE(result.safe_stop_forward_m, 0.0);
  EXPECT_LT(result.safe_stop_forward_m, 6.0);
}

// 탈출 검증을 끄면 이전 동작(정지점 무검증)으로 돌아간다 — 회귀 시 즉시 되돌릴 수 있어야 한다.
TEST(RacelineSplinePlanner, SafeStopEscapeCheckCanBeDisabled)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  parameters.minimum_path_points = 8;
  parameters.safe_stop_buffer_m = 0.80;
  parameters.safe_stop_escape_check_enable = false;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 3.0, -1.20, 1.20)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  EXPECT_TRUE(result.safe_stop_escape_verified);   // 검증 자체를 안 했으므로 참으로 둔다
  EXPECT_EQ(result.reason.find("no escapable stop point"), std::string::npos);
}

TEST(RacelineSplinePlanner, BuildsZeroSpeedEmergencyHold)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto path = planner.buildEmergencyStopPath(EgoFrenetState{2.05, 0.18, 2.0});
  ASSERT_EQ(path.wpnts.size(), 8U);
  for (const auto & waypoint : path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.18);
    EXPECT_DOUBLE_EQ(waypoint.vx_mps, 0.0);
    EXPECT_DOUBLE_EQ(waypoint.ax_mps2, 0.0);
  }
}

TEST(RacelineSplinePlanner, HandlesObstacleAcrossTrackWrap)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  auto obstacle = makeObstacle(9, 0.30);
  obstacle.s_start = 0.10;
  obstacle.s_end = 0.50;
  constexpr double kEgoS = 24.0;
  const auto result = planner.plan(EgoFrenetState{kEgoS, 0.0, 2.0}, {obstacle});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  bool wrapped = false;
  double previous = -1.0;
  for (const auto & waypoint : result.path.wpnts) {
    const double forward = planner.forwardDistance(kEgoS, waypoint.s_m);
    EXPECT_GT(forward, previous);
    previous = forward;
    wrapped = wrapped || waypoint.s_m < 1.0;
  }
  EXPECT_TRUE(wrapped);
}

TEST(RacelineSplinePlanner, NeverJumpsToNearbyWrongSnakeBranch)
{
  auto reference = makeStraightReference();
  for (std::size_t i = 220U; i < reference.wpnts.size(); ++i) {
    reference.wpnts[i].x_m = 30.0 - reference.wpnts[i].s_m;
    reference.wpnts[i].y_m = 0.55;
    reference.wpnts[i].psi_rad = 3.14159265358979323846;
  }
  // 복귀 가지가 y=0.55에 있으므로 자유폭도 그에 맞게 제한한다. 폭을 1.5로 두면 P3가
  // slack 최대 지점(0.86)까지 합법적으로 벌려 기하 모순(가지 관통)이 생긴다. 0.70이면
  // plateau ≤ 0.70 − 0.12 − 0.10 = 0.48 < 0.55로 어느 측을 골라도 가지 앞에서 멈춘다.
  for (auto & waypoint : reference.wpnts) {
    waypoint.d_left = 0.70;
    waypoint.d_right = 0.70;
  }
  auto parameters = testParameters();
  parameters.maximum_curvature_radpm = 100.0;
  parameters.maximum_curvature_rate_radpm2 = 1000.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(11, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_LT(waypoint.s_m, 22.0)
      << "planner selected a geometrically nearby but topologically wrong snake branch";
    EXPECT_LT(waypoint.y_m, 0.55)
      << "path must remain an offset of the ordered first race-line branch";
  }
}

// A gap wide enough for the full-speed reserve must not cost any speed: obstacle-free laps and
// roomy avoidances keep the race-line profile.
TEST(RacelineSplinePlanner, WideGapKeepsRaceLineSpeed)
{
  const auto reference = makeStraightReference();
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.10;
  parameters.avoidance_minimum_speed_mps = 1.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(7, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.vx_mps, 3.0);
  }
}

// The lateral-acceleration cap is blind to a straight, so before the gap-driven limit an obstacle
// on a straight was planned at full race-line speed and reserved the widest tracking error the LUT
// has. Squeeze the corridor until only the slow end of the LUT fits and the pass must slow down.
TEST(RacelineSplinePlanner, TightGapOnStraightSlowsDownInsteadOfReservingFullSpeedError)
{
  // 0.40 m of room each side: the race-line-speed reserve (0.30) cannot fit beside the obstacle,
  // the slow-end reserve (0.02) can.
  const auto reference = makeStraightReference(300, 0.1, 0.45, 0.45);
  auto parameters = testParameters();
  parameters.avoidance_minimum_speed_mps = 1.0;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 1.5, 3.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 1.0};
  parameters.tracking_error_lut_values_m = {0.02, 0.02, 0.02, 0.02, 0.30, 0.30};
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(7, 7.0, -0.05, 0.05)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;

  double slowest = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : result.path.wpnts) {
    slowest = std::min(slowest, waypoint.vx_mps);
    EXPECT_LE(waypoint.vx_mps, 3.0 + 1.0e-9);
    EXPECT_GE(waypoint.vx_mps, parameters.avoidance_minimum_speed_mps - 1.0e-9);
  }
  EXPECT_LT(slowest, 3.0) << "the pass should have been slowed to afford its reserve";
}

// Speed may only be traded for reserve down to the configured floor. Below it the maneuver is
// reported infeasible rather than crawled through, so safe-stop stays the authority.
TEST(RacelineSplinePlanner, GapLimitedSpeedNeverFallsBelowTheFloor)
{
  auto parameters = testParameters();
  parameters.avoidance_minimum_speed_mps = 2.5;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 6.5};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 1.0};
  parameters.tracking_error_lut_values_m = {0.05, 0.05, 0.30, 0.30};

  EXPECT_DOUBLE_EQ(parameters.gapLimitedAvoidanceSpeed(6.0, 0.0, 1.0), 6.0);
  EXPECT_DOUBLE_EQ(parameters.gapLimitedAvoidanceSpeed(6.0, 0.0, -1.0), 2.5);
  EXPECT_DOUBLE_EQ(parameters.gapLimitedAvoidanceSpeed(2.0, 0.0, -1.0), 2.0);

  const double capped = parameters.gapLimitedAvoidanceSpeed(6.0, 0.0, 0.15);
  EXPECT_GE(capped, 2.5);
  EXPECT_LT(capped, 6.0);
  EXPECT_LE(parameters.trackingErrorReserve(capped, 0.0), 0.15 + 1.0e-6);
}

TEST(RacelineSplinePlanner, MaximumExitLengthCapsCombinedExitScale)
{
  // The slack ranking always prefers the gentlest (longest) exit, and without an absolute cap
  // post_apex_far x transition_long extends the published avoidance path up to ~18 m past the
  // obstacle, deferring the merge back to the global line by that whole tail.
  auto parameters = testParameters();
  parameters.post_apex_distances_m = {1.0, 2.0, 5.0};
  parameters.maximum_exit_length_m = 6.0;
  EXPECT_DOUBLE_EQ(parameters.cappedCombinedExitScale(0.5), 0.5);
  EXPECT_DOUBLE_EQ(parameters.cappedCombinedExitScale(3.58), 6.0 / 5.0);

  parameters.maximum_exit_length_m = 0.0;  // non-positive disables the cap
  EXPECT_DOUBLE_EQ(parameters.cappedCombinedExitScale(3.58), 3.58);
}

// P0 격자를 제거하고 plan()의 후보 생성기를 P3로 옮긴 뒤에도, plan()은 여전히
// "회피가 가능하면 kAvoidance"라는 계약을 지켜야 한다. 이 계약이 깨지면 연쇄 기동
// (tryEarlyChainedManeuver)·안전정지 해제 조건 B·안정화 중 조기 회피가 전부 죽는다 —
// 2026-08-15 시뮬에서 실제로 그렇게 되어 차가 다음 장애물 앞에서 정지했다.
TEST(RacelineSplinePlanner, PlanReturnsAvoidanceForAPassableObstacle)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 1.20, 1.20)));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(401, 8.0)}, std::nullopt, true);
  EXPECT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.path.wpnts.empty());
  EXPECT_GT(std::abs(result.target_d), 0.0);
}

// 연쇄 기동이 쓰는 형태: 자차가 이미 라인에서 벗어나 있고(직전 회피의 여파) 다음 장애물이
// 앞에 있는 상태. 여기서 kAvoidance가 나오지 않으면 연쇄가 성립하지 않는다.
TEST(RacelineSplinePlanner, PlanChainsFromANonZeroEgoOffset)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 1.20, 1.20)));

  const auto result = planner.plan(
    EgoFrenetState{0.0, -0.35, 2.0}, {makeObstacle(402, 9.0)}, std::nullopt, true);
  EXPECT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.path.wpnts.empty());
}

TEST(RacelineSplinePlanner, MarginOnlyClusterDegradesToSlowPassInsteadOfSafeStop)
{
  // Track so narrow that both spline sides fail. The obstacle sits entirely left of the line:
  // margin-blocking through the 0.3 m fallback reserve, but its raw envelope plus the physical
  // clearance (0.12 + 0.03) never reaches d = 0, so the line itself stays drivable.
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  parameters.margin_pass_speed_cap_mps = 2.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.3, 0.3)));
  const EgoFrenetState ego{0.0, 0.0, 3.0};
  const auto margin_only = makeObstacle(90, 5.0, 0.20, 0.60);

  const auto result = planner.plan(ego, {margin_only});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance);
  EXPECT_TRUE(result.margin_pass);
  ASSERT_GE(result.path.wpnts.size(), 2U);
  // 접근 실현성 램프: 자차(3.0 m/s)가 cap(2.0)보다 빠르므로 앞머리는 flat 2.0이 아니라
  // 자차 속도에서 내려오는 프로파일이다. 전 구간 단조 비증가 + 자차 속도 이하이고,
  // 장애물 스팬(및 그 이후)은 cap을 넘지 않아야 한다.
  double previous = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.0);
    EXPECT_LE(waypoint.vx_mps, ego.speed + 1e-9);
    EXPECT_LE(waypoint.vx_mps, previous + 1e-9);
    previous = waypoint.vx_mps;
    if (waypoint.s_m >= margin_only.s_start) {
      EXPECT_LE(waypoint.vx_mps, 2.0 + 1e-9);
    }
  }
  EXPECT_GT(result.path.wpnts.front().vx_mps, 2.5);   // 계단(즉시 2.0) 금지 = 램프 실존
  EXPECT_GT(result.merge_s, margin_only.s_end);

  // Contrast: the same track with a genuinely line-straddling obstacle must still stop.
  const auto physically_blocking = planner.plan(ego, {makeObstacle(91, 5.0)});
  EXPECT_EQ(physically_blocking.kind, SplinePlanKind::kSafeStop);
  EXPECT_FALSE(physically_blocking.margin_pass);
}

TEST(RacelineSplinePlanner, MarginPassApproachRampReachesCapBeforeClusterStart)
{
  // 0814 실차 회귀(run_0814_111210): 4.4 m/s 접근에 flat 2.0 margin pass가 발행돼 계단
  // 감속 → 서비스 브레이크 포화 → 마찰 한계 초과 슬립 → 벽. 램프는 실측 자차 속도에서
  // approach_feasibility_decel로 내려가되 군집 시작 전에 cap에 도달해야 한다.
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  parameters.margin_pass_speed_cap_mps = 2.0;
  parameters.approach_feasibility_decel_mps2 = 2.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.3, 0.3)));
  const auto margin_only = makeObstacle(92, 9.0, 0.20, 0.60);
  const EgoFrenetState ego{0.0, 0.0, 4.4};

  const auto result = planner.plan(ego, {margin_only});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance);
  EXPECT_TRUE(result.margin_pass);
  ASSERT_GE(result.path.wpnts.size(), 2U);
  // 필요 감속 (4.4²-2.0²)/(2·~8.5) ≈ 0.9 < 2.0 → 완만한 파라미터 감속이 그대로 쓰이고,
  // cap 도달 지점은 (4.4²-2.0²)/(2·2.0) = 3.84 m — 군집 시작(≈8.5 m)보다 훨씬 앞이다.
  const double reach_cap_at = (ego.speed * ego.speed - 4.0) / (2.0 * 2.0);
  for (const auto & waypoint : result.path.wpnts) {
    // 프로파일 상한(3.0)은 절대 넘지 않는다: 램프값이 그보다 커도 참조 프로파일이 이긴다.
    EXPECT_LE(waypoint.vx_mps, 3.0 + 1e-9);
    if (waypoint.s_m >= reach_cap_at + 0.2) {
      EXPECT_LE(waypoint.vx_mps, 2.0 + 1e-9);
    }
  }
  // 앞머리는 램프를 따른다(참조 프로파일 3.0에 클램프): flat 2.0이 아니어야 한다.
  EXPECT_NEAR(result.path.wpnts.front().vx_mps, 3.0, 1e-9);

  // 비활성(<=0)이면 구 거동(flat cap) 그대로다.
  parameters.approach_feasibility_decel_mps2 = 0.0;
  RacelineSplinePlanner flat_planner(parameters);
  ASSERT_TRUE(flat_planner.setReference(makeStraightReference(300, 0.1, 0.3, 0.3)));
  const auto flat = flat_planner.plan(ego, {margin_only});
  ASSERT_EQ(flat.kind, SplinePlanKind::kAvoidance);
  for (const auto & waypoint : flat.path.wpnts) {
    EXPECT_LE(waypoint.vx_mps, 2.0 + 1e-9);
  }
}

TEST(RacelineSplinePlanner, AvoidanceApproachBrakesBeforeSpanInsteadOfStepping)
{
  // 회피 스플라인의 접근 구간: 간극/곡률 캡은 스팬 안에서만 작동하므로 접근은 프로파일
  // 속도 그대로다가 스팬 경계에서 계단으로 떨어진다. 후방 제동 램프는 그 계단을 스팬
  // 시작 속도로 미리 내려가는 프로파일로 바꾼다(낮추기만 함). 스팬 내부는 비트 동일.
  auto ramp_parameters = testParameters();
  // 속도에 가파르게 커지는 추적오차 예약 LUT + 좁은 왼쪽 통로: 스팬 안에서 간극 캡이
  // 프로파일(3.0)보다 확실히 낮은 속도를 강제해 "접근 빠름 / 스팬 느림" 계단을 만든다.
  ramp_parameters.tracking_error_lut_speed_bins_mps = {0.0, 2.0, 4.0};
  ramp_parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  ramp_parameters.tracking_error_lut_values_m = {0.05, 0.08, 0.60};
  ramp_parameters.approach_feasibility_decel_mps2 = 2.0;
  auto flat_parameters = ramp_parameters;
  flat_parameters.approach_feasibility_decel_mps2 = 0.0;
  RacelineSplinePlanner ramp_planner(ramp_parameters);
  RacelineSplinePlanner flat_planner(flat_parameters);
  const auto reference = makeStraightReference(300, 0.1, 0.6, 0.6);
  ASSERT_TRUE(ramp_planner.setReference(reference));
  ASSERT_TRUE(flat_planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  // 비대칭 장애물(오른쪽으로 치우침): 후보 랭킹 1순위(safety slack)에서 왼쪽이 명확히
  // 이기게 해, 램프로 달라지는 2순위(velocity_loss)가 후보 선택을 못 바꾸게 고정한다.
  const std::vector<f110_msgs::msg::Obstacle> obstacles{makeObstacle(93, 8.0, -0.35, 0.05)};

  const auto ramped = ramp_planner.plan(ego, obstacles);
  const auto flat = flat_planner.plan(ego, obstacles);
  ASSERT_EQ(ramped.kind, SplinePlanKind::kAvoidance);
  ASSERT_EQ(flat.kind, SplinePlanKind::kAvoidance);
  ASSERT_EQ(ramped.path.wpnts.size(), flat.path.wpnts.size());
  const double span_start = obstacles.front().s_start;
  bool lowered_somewhere = false;
  for (std::size_t index = 0; index < ramped.path.wpnts.size(); ++index) {
    const auto & with_ramp = ramped.path.wpnts[index];
    const auto & without = flat.path.wpnts[index];
    ASSERT_DOUBLE_EQ(with_ramp.s_m, without.s_m);
    // 램프는 어디서도 속도를 올리지 않는다.
    EXPECT_LE(with_ramp.vx_mps, without.vx_mps + 1e-9);
    if (with_ramp.s_m >= span_start) {
      // 스팬 및 그 이후는 손대지 않는다(간극/예약 캡 보존).
      EXPECT_DOUBLE_EQ(with_ramp.vx_mps, without.vx_mps);
    } else if (with_ramp.vx_mps < without.vx_mps - 1e-6) {
      lowered_somewhere = true;
    }
  }
  EXPECT_TRUE(lowered_somewhere);
}

TEST(RacelineSplinePlanner, RetentionReserveScaleHoldsCommittedPathThroughEnvelopeGrowth)
{
  // Full clearance = 0.15 (base) + 0.30 (reserve) = 0.45; retention 0.5 keeps 0.15 + 0.15.
  // The obstacle's grown left edge (0.42) violates the full margin against a path at d = 0.8
  // (0.42 + 0.45 > 0.8) but stays inside the retention band (0.42 + 0.30 < 0.8).
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  RacelineSplinePlanner planner(parameters);
  const auto reference = makeStraightReference();
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.8, 2.0};
  const auto path = makeStraightCandidate(reference, 0.8, 0.0, 60U);

  const auto grown = makeObstacle(7, 2.0, -0.2, 0.42);
  PathValidationFailure failure;
  EXPECT_FALSE(planner.validatePath(ego, path, {grown}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);
  EXPECT_TRUE(planner.validatePath(ego, path, {grown}, nullptr, nullptr, std::nullopt, 0.5));

  // Growth past the retention band must still invalidate the committed path.
  const auto beyond_retention = makeObstacle(8, 2.0, -0.2, 0.55);
  EXPECT_FALSE(
    planner.validatePath(ego, path, {beyond_retention}, nullptr, nullptr, std::nullopt, 0.5));

  // The same contract through the P3 suffix validator.
  EXPECT_FALSE(planner.evaluateP3PathCurrent(ego, path, {grown}).hard_valid);
  EXPECT_TRUE(planner.evaluateP3PathCurrent(ego, path, {grown}, 0.5).hard_valid);
}

TEST(RacelineSplinePlanner, LocalizationReserveAddsConstantFloorAndScalesWithRetention)
{
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  const double base = parameters.obstacleBaseClearance();
  const double without = parameters.obstacleSafetyClearance(2.0, 0.0);
  EXPECT_NEAR(without, base + 0.30, 1e-9);

  parameters.localization_reserve_m = 0.06;
  // Full reserve: the localization floor adds verbatim.
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0), base + 0.36, 1e-9);
  // Retention scale halves the WHOLE reserve including the localization floor.
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0, 0.5), base + 0.18, 1e-9);
  // The base clearance itself is never touched.
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0, 0.0), base, 1e-9);
  // The gap-limited speed inversion sees the same floor: a gap that admits exactly the
  // tracking reserve no longer fits once the localization floor is added, so the requested
  // speed must drop to the avoidance floor (the constant reserve cannot be shed by slowing).
  const double at_floor = parameters.gapLimitedAvoidanceSpeed(4.0, 0.0, 0.30);
  EXPECT_NEAR(at_floor, parameters.avoidance_minimum_speed_mps, 1e-9);
}

TEST(RacelineSplinePlanner, BuildLastPathBrakeStopsAlongGivenGeometry)
{
  RacelineSplinePlanner planner(testParameters());
  const auto reference = makeStraightReference();
  ASSERT_TRUE(planner.setReference(reference));
  const auto path = makeStraightCandidate(reference, 0.0, 0.0, 60U);
  const EgoFrenetState ego{1.0, 0.0, 2.0};

  const auto braked = planner.buildLastPathBrake(ego, path);
  ASSERT_GE(braked.wpnts.size(), 2U);
  // Stop distance from 2.0 m/s at the default 2.5 m/s^2 deceleration is 0.8 m.
  EXPECT_LE(planner.forwardDistance(ego.s, braked.wpnts.back().s_m), 0.8 + 0.2);
  EXPECT_DOUBLE_EQ(braked.wpnts.back().vx_mps, 0.0);
  for (std::size_t i = 1; i < braked.wpnts.size(); ++i) {
    EXPECT_LE(braked.wpnts[i].vx_mps, braked.wpnts[i - 1].vx_mps + 1e-9);
  }

  f110_msgs::msg::WpntArray empty_path;
  EXPECT_TRUE(planner.buildLastPathBrake(ego, empty_path).wpnts.empty());
}

TEST(RacelineSplinePlanner, StandstillCloseBehindObstacleStillPlansEscape)
{
  // 2026-08-13 실차 재현 (run_0813_221339 s≈29.8): 코너 뒤 늦은 발견으로 장애물
  // ~1.5 m 앞에 정지. 갭은 기하학적으로 충분한데(비대칭 코리도 1.97 m, 반대쪽 여유
  // ~1.4 m) 정지 상태 재계획이 회피를 내지 못하면 safe-stop 홀드에서 영원히 못
  // 나온다. 진입 길이는 자차→클러스터 실거리에 비례하므로 짧은 거리에서도 후보가
  // 성립해야 한다.
  auto reference = makeStraightReference(300, 0.1, 1.40, 0.57);
  RacelineSplineParameters parameters;
  parameters.maximum_curvature_radpm = 3.2;
  parameters.maximum_curvature_rate_radpm2 = 60.0;
  // 실차 yaml과 같은 예약 구조: 기어가기 속도에서 LUT 바닥 0.20 + 위치추정 0.06.
  parameters.tracking_error_reserve_m = 0.20;
  parameters.localization_reserve_m = 0.06;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  // 자차: 정지, 라인 살짝 오른쪽(-0.10). 장애물: 우벽 쪽 박스(폭 0.4), 전방
  // 클러스터 시작 ≈ 1.5 m (s_start 2.05 − 종방향 패딩 0.35 − 자차 s 0.2).
  const EgoFrenetState ego{0.2, -0.10, 0.0};
  const auto obstacle = makeObstacle(29, 2.45, -0.45, -0.05);

  const auto result = planner.plan(ego, {obstacle});
  EXPECT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.path.wpnts.empty());
  // 탈출은 여유가 있는 왼쪽으로 나가야 한다.
  double max_d = -10.0;
  for (const auto & waypoint : result.path.wpnts) {
    max_d = std::max(max_d, static_cast<double>(waypoint.d_m));
  }
  EXPECT_GT(max_d, 0.10);
}


}  // namespace
}  // namespace local_planning
