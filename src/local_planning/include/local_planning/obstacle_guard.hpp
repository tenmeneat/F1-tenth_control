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

#ifndef LOCAL_PLANNING__OBSTACLE_GUARD_HPP_
#define LOCAL_PLANNING__OBSTACLE_GUARD_HPP_

#include <f110_msgs/msg/obstacle.hpp>

namespace local_planning
{

struct ObstacleGuardParameters
{
  double uncertainty_sigma_scale{3.0};
  double minimum_longitudinal_inflation_m{0.05};
  // Keep detector-owned lateral AABB bounds unchanged by default. Non-zero values are retained
  // only for explicit experiments; normal planning adds no covariance-based lateral extent.
  double minimum_lateral_inflation_m{0.0};
  double maximum_lateral_inflation_m{0.0};
};

// Expand a projected Frenet AABB by configured fixed floors plus k standard deviations of the
// Kalman centre-position estimate. The operational lateral limits are zero, so d_right/d_left stay
// detector-owned while longitudinal uncertainty can still protect approach and stop timing.
f110_msgs::msg::Obstacle buildUncertaintyGuard(
  const f110_msgs::msg::Obstacle & obstacle,
  double track_length,
  const ObstacleGuardParameters & parameters);

// Return true only when the complete candidate Frenet envelope is contained in the frozen guard.
// Longitudinal containment is closed-track/wrap aware.
bool obstacleEnvelopeContained(
  const f110_msgs::msg::Obstacle & candidate,
  const f110_msgs::msg::Obstacle & guard,
  double track_length,
  double tolerance_m = 1.0e-6);

}  // namespace local_planning

#endif  // LOCAL_PLANNING__OBSTACLE_GUARD_HPP_
