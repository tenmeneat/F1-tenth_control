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

#include "local_planning/obstacle_guard.hpp"

#include <algorithm>
#include <cmath>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

double wrapS(double s, double track_length)
{
  if (!(track_length > kEpsilon) || !std::isfinite(s)) {
    return s;
  }
  s = std::fmod(s, track_length);
  return s < 0.0 ? s + track_length : s;
}

double forwardDistance(double from_s, double to_s, double track_length)
{
  return wrapS(to_s - from_s, track_length);
}

double shortestSpan(
  const f110_msgs::msg::Obstacle & obstacle,
  double track_length)
{
  const double forward = forwardDistance(obstacle.s_start, obstacle.s_end, track_length);
  const double reverse = forwardDistance(obstacle.s_end, obstacle.s_start, track_length);
  const double span = std::min(forward, reverse);
  if (std::isfinite(span) && span > kEpsilon) {
    return span;
  }
  return std::max(0.0, std::abs(obstacle.size));
}

double positionSigma(double variance)
{
  if (!std::isfinite(variance) || variance <= 0.0) {
    return 0.0;
  }
  return std::sqrt(variance);
}

}  // namespace

f110_msgs::msg::Obstacle buildUncertaintyGuard(
  const f110_msgs::msg::Obstacle & obstacle,
  double track_length,
  const ObstacleGuardParameters & parameters)
{
  auto guard = obstacle;
  const double longitudinal_inflation =
    parameters.minimum_longitudinal_inflation_m +
    parameters.uncertainty_sigma_scale * positionSigma(obstacle.s_var);
  const double lateral_inflation = std::min(
    parameters.minimum_lateral_inflation_m +
    parameters.uncertainty_sigma_scale * positionSigma(obstacle.d_var),
    parameters.maximum_lateral_inflation_m);

  const double half_span = 0.5 * shortestSpan(obstacle, track_length) + longitudinal_inflation;
  guard.s_start = wrapS(obstacle.s_center - half_span, track_length);
  guard.s_end = wrapS(obstacle.s_center + half_span, track_length);

  const double raw_right = std::min(obstacle.d_right, obstacle.d_left);
  const double raw_left = std::max(obstacle.d_right, obstacle.d_left);
  guard.d_right = raw_right - lateral_inflation;
  guard.d_left = raw_left + lateral_inflation;
  guard.size = std::hypot(2.0 * half_span, guard.d_left - guard.d_right);
  return guard;
}

bool obstacleEnvelopeContained(
  const f110_msgs::msg::Obstacle & candidate,
  const f110_msgs::msg::Obstacle & guard,
  double track_length,
  double tolerance_m)
{
  if (!(track_length > kEpsilon) ||
    !std::isfinite(candidate.s_center) || !std::isfinite(guard.s_center) ||
    !std::isfinite(candidate.d_right) || !std::isfinite(candidate.d_left) ||
    !std::isfinite(guard.d_right) || !std::isfinite(guard.d_left))
  {
    return false;
  }

  double center_delta = forwardDistance(guard.s_center, candidate.s_center, track_length);
  if (center_delta > 0.5 * track_length) {
    center_delta -= track_length;
  }
  const double candidate_half_span = 0.5 * shortestSpan(candidate, track_length);
  const double guard_half_span = 0.5 * shortestSpan(guard, track_length);
  const bool longitudinally_contained =
    center_delta - candidate_half_span >= -guard_half_span - tolerance_m &&
    center_delta + candidate_half_span <= guard_half_span + tolerance_m;

  const double candidate_right = std::min(candidate.d_right, candidate.d_left);
  const double candidate_left = std::max(candidate.d_right, candidate.d_left);
  const double guard_right = std::min(guard.d_right, guard.d_left);
  const double guard_left = std::max(guard.d_right, guard.d_left);
  const bool laterally_contained =
    candidate_right >= guard_right - tolerance_m &&
    candidate_left <= guard_left + tolerance_m;
  return longitudinally_contained && laterally_contained;
}

}  // namespace local_planning
