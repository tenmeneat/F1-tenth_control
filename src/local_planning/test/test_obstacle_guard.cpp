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

#include <limits>

#include "local_planning/obstacle_guard.hpp"

namespace local_planning
{
namespace
{

f110_msgs::msg::Obstacle makeObstacle(
  double s_center,
  double s_start,
  double s_end,
  double d_right = -0.2,
  double d_left = 0.2)
{
  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = 7;
  obstacle.s_center = s_center;
  obstacle.s_start = s_start;
  obstacle.s_end = s_end;
  obstacle.d_center = 0.5 * (d_right + d_left);
  obstacle.d_right = d_right;
  obstacle.d_left = d_left;
  obstacle.size = 0.4;
  return obstacle;
}

TEST(ObstacleGuard, AddsFixedInflationAndKalmanStandardDeviation)
{
  auto obstacle = makeObstacle(10.0, 9.8, 10.2);
  obstacle.s_var = 0.01;
  obstacle.d_var = 0.0025;

  ObstacleGuardParameters parameters;
  parameters.uncertainty_sigma_scale = 3.0;
  parameters.minimum_longitudinal_inflation_m = 0.05;
  parameters.minimum_lateral_inflation_m = 0.03;
  parameters.maximum_lateral_inflation_m = 1.0;  // test the uncapped inflation formula
  const auto guard = buildUncertaintyGuard(obstacle, 100.0, parameters);

  EXPECT_NEAR(guard.s_start, 9.45, 1.0e-9);
  EXPECT_NEAR(guard.s_end, 10.55, 1.0e-9);
  EXPECT_NEAR(guard.d_right, -0.38, 1.0e-9);
  EXPECT_NEAR(guard.d_left, 0.38, 1.0e-9);
}

TEST(ObstacleGuard, KeepsSmallSameIdMotionInsideFrozenGuard)
{
  auto initial = makeObstacle(10.0, 9.8, 10.2);
  initial.s_var = 0.0004;
  initial.d_var = 0.0001;
  ObstacleGuardParameters parameters;
  parameters.minimum_lateral_inflation_m = 0.03;
  parameters.maximum_lateral_inflation_m = 0.15;
  const auto frozen_guard = buildUncertaintyGuard(initial, 100.0, parameters);

  auto shifted = makeObstacle(10.04, 9.84, 10.24, -0.18, 0.22);
  const auto shifted_envelope = buildUncertaintyGuard(shifted, 100.0, parameters);
  EXPECT_TRUE(obstacleEnvelopeContained(shifted_envelope, frozen_guard, 100.0));

  auto accumulated_shift = makeObstacle(10.10, 9.90, 10.30, -0.18, 0.22);
  const auto breached_envelope =
    buildUncertaintyGuard(accumulated_shift, 100.0, parameters);
  EXPECT_FALSE(obstacleEnvelopeContained(breached_envelope, frozen_guard, 100.0));
}

TEST(ObstacleGuard, HandlesClosedTrackWrap)
{
  ObstacleGuardParameters parameters;
  parameters.uncertainty_sigma_scale = 0.0;
  parameters.minimum_longitudinal_inflation_m = 0.10;
  parameters.minimum_lateral_inflation_m = 0.05;
  parameters.maximum_lateral_inflation_m = 0.15;

  const auto initial = makeObstacle(0.05, 99.85, 0.25);
  const auto frozen_guard = buildUncertaintyGuard(initial, 100.0, parameters);
  EXPECT_NEAR(frozen_guard.s_start, 99.75, 1.0e-9);
  EXPECT_NEAR(frozen_guard.s_end, 0.35, 1.0e-9);

  const auto shifted = makeObstacle(0.08, 99.90, 0.24);
  const auto shifted_envelope = buildUncertaintyGuard(shifted, 100.0, parameters);
  EXPECT_TRUE(obstacleEnvelopeContained(shifted_envelope, frozen_guard, 100.0));
}

TEST(ObstacleGuard, FallsBackToFixedInflationForInvalidVariance)
{
  auto obstacle = makeObstacle(10.0, 9.8, 10.2);
  obstacle.s_var = std::numeric_limits<double>::quiet_NaN();
  obstacle.d_var = -1.0;
  ObstacleGuardParameters parameters;
  parameters.minimum_lateral_inflation_m = 0.03;
  parameters.maximum_lateral_inflation_m = 0.15;
  const auto guard = buildUncertaintyGuard(obstacle, 100.0, parameters);

  EXPECT_NEAR(guard.s_start, 9.75, 1.0e-9);
  EXPECT_NEAR(guard.s_end, 10.25, 1.0e-9);
  EXPECT_NEAR(guard.d_right, -0.23, 1.0e-9);
  EXPECT_NEAR(guard.d_left, 0.23, 1.0e-9);
}

TEST(ObstacleGuard, CapsLateralInflationAtConfiguredMaximum)
{
  auto obstacle = makeObstacle(10.0, 9.8, 10.2);
  obstacle.s_var = 0.01;
  obstacle.d_var = 4.0;

  ObstacleGuardParameters parameters;
  parameters.maximum_lateral_inflation_m = 0.20;
  const auto guard = buildUncertaintyGuard(obstacle, 100.0, parameters);

  // 3 * sqrt(4.0) = 6 m of lateral inflation is capped at
  // maximum_lateral_inflation_m (0.20);
  // the longitudinal inflation stays uncapped.
  EXPECT_NEAR(guard.d_right, -0.40, 1.0e-9);
  EXPECT_NEAR(guard.d_left, 0.40, 1.0e-9);
  EXPECT_NEAR(guard.s_start, 9.45, 1.0e-9);
  EXPECT_NEAR(guard.s_end, 10.55, 1.0e-9);
}

TEST(ObstacleGuard, KeepsRawLateralBoundsWhenInflationIsDisabled)
{
  auto obstacle = makeObstacle(10.0, 9.8, 10.2, -0.17, 0.23);
  obstacle.s_var = 0.01;
  obstacle.d_var = 100.0;

  const ObstacleGuardParameters parameters;
  const auto guard = buildUncertaintyGuard(obstacle, 100.0, parameters);

  EXPECT_NEAR(guard.d_right, obstacle.d_right, 1.0e-9);
  EXPECT_NEAR(guard.d_left, obstacle.d_left, 1.0e-9);
  EXPECT_LT(guard.s_start, obstacle.s_start);
  EXPECT_GT(guard.s_end, obstacle.s_end);
}

}  // namespace
}  // namespace local_planning
