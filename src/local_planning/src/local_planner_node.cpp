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

#include "local_planning/local_planner_node.hpp"

#include <algorithm>
#include <cinttypes>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace local_planning
{
namespace
{

constexpr double kGeometryEpsilon = 1.0e-9;

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

std::string pathDigest(const f110_msgs::msg::WpntArray & path)
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

std::int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

bool finiteOdometry(const nav_msgs::msg::Odometry & odometry)
{
  return std::isfinite(odometry.pose.pose.position.x) &&
         std::isfinite(odometry.pose.pose.position.y) &&
         std::isfinite(odometry.twist.twist.linear.x);
}

double conservativeVariance(double first, double second)
{
  const double finite_first = std::isfinite(first) && first >= 0.0 ? first : 0.0;
  const double finite_second = std::isfinite(second) && second >= 0.0 ? second : 0.0;
  return std::max(finite_first, finite_second);
}

std::string jsonEscape(const std::string & input)
{
  std::ostringstream escaped;
  for (const char character : input) {
    switch (character) {
      case '\\': escaped << "\\\\"; break;
      case '"': escaped << "\\\""; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default: escaped << character; break;
    }
  }
  return escaped.str();
}

std::string jsonNumber(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream output;
  output << std::setprecision(17) << value;
  return output.str();
}

void mergeObstacleVariances(
  f110_msgs::msg::Obstacle & target,
  const f110_msgs::msg::Obstacle & other)
{
  target.x_var = conservativeVariance(target.x_var, other.x_var);
  target.y_var = conservativeVariance(target.y_var, other.y_var);
  target.s_var = conservativeVariance(target.s_var, other.s_var);
  target.d_var = conservativeVariance(target.d_var, other.d_var);
  target.vs_var = conservativeVariance(target.vs_var, other.vs_var);
  target.vd_var = conservativeVariance(target.vd_var, other.vd_var);
}

double wrapS(double s, double track_length)
{
  if (!(track_length > kGeometryEpsilon) || !std::isfinite(s)) {
    return s;
  }
  s = std::fmod(s, track_length);
  return s < 0.0 ? s + track_length : s;
}

double forwardDistance(double from_s, double to_s, double track_length)
{
  return wrapS(to_s - from_s, track_length);
}

double obstacleLongitudinalSpan(
  const f110_msgs::msg::Obstacle & obstacle,
  double track_length)
{
  const double forward = forwardDistance(obstacle.s_start, obstacle.s_end, track_length);
  const double reverse = forwardDistance(obstacle.s_end, obstacle.s_start, track_length);
  const double span = std::min(forward, reverse);
  if (std::isfinite(span) && span > kGeometryEpsilon) {
    return span;
  }
  return std::max(0.0, std::abs(obstacle.size));
}

double signedTrackDelta(double from_s, double to_s, double track_length)
{
  double delta = forwardDistance(from_s, to_s, track_length);
  if (delta > 0.5 * track_length) {
    delta -= track_length;
  }
  return delta;
}

bool validFrenetObstacle(
  const f110_msgs::msg::Obstacle & obstacle,
  double track_length)
{
  if (!(track_length > kGeometryEpsilon) ||
    !std::isfinite(obstacle.s_center) ||
    !std::isfinite(obstacle.s_start) ||
    !std::isfinite(obstacle.s_end) ||
    !std::isfinite(obstacle.d_center) ||
    !std::isfinite(obstacle.d_right) ||
    !std::isfinite(obstacle.d_left) ||
    !std::isfinite(obstacle.size) ||
    obstacle.size < 0.0 ||
    obstacle.d_right > obstacle.d_left + kGeometryEpsilon)
  {
    return false;
  }
  const double longitudinal_span = obstacleLongitudinalSpan(obstacle, track_length);
  const double lateral_span = obstacle.d_left - obstacle.d_right;
  return longitudinal_span > kGeometryEpsilon || lateral_span > kGeometryEpsilon;
}

bool validCartesianAabb(const f110_msgs::msg::Obstacle & obstacle)
{
  return obstacle.has_cartesian &&
         std::isfinite(obstacle.x_min) &&
         std::isfinite(obstacle.x_max) &&
         std::isfinite(obstacle.y_min) &&
         std::isfinite(obstacle.y_max) &&
         obstacle.x_min <= obstacle.x_max &&
         obstacle.y_min <= obstacle.y_max;
}

f110_msgs::msg::Obstacle mergeObstacleEnvelopes(
  const f110_msgs::msg::Obstacle & first,
  const f110_msgs::msg::Obstacle & second,
  double track_length)
{
  auto merged = first;
  const double first_half_span =
    0.5 * obstacleLongitudinalSpan(first, track_length);
  const double second_half_span =
    0.5 * obstacleLongitudinalSpan(second, track_length);
  const double second_center =
    signedTrackDelta(first.s_center, second.s_center, track_length);
  const double lower = std::min(
    -first_half_span, second_center - second_half_span);
  const double upper = std::max(
    first_half_span, second_center + second_half_span);

  merged.s_start = wrapS(first.s_center + lower, track_length);
  merged.s_end = wrapS(first.s_center + upper, track_length);
  merged.s_center = wrapS(
    first.s_center + 0.5 * (lower + upper), track_length);
  merged.d_right = std::min(first.d_right, second.d_right);
  merged.d_left = std::max(first.d_left, second.d_left);
  merged.d_center = 0.5 * (merged.d_right + merged.d_left);
  merged.size = std::hypot(upper - lower, merged.d_left - merged.d_right);
  mergeObstacleVariances(merged, second);

  const bool first_has_aabb = validCartesianAabb(first);
  const bool second_has_aabb = validCartesianAabb(second);
  if (first_has_aabb || second_has_aabb) {
    const auto & seed = first_has_aabb ? first : second;
    merged.has_cartesian = true;
    merged.x_min = seed.x_min;
    merged.x_max = seed.x_max;
    merged.y_min = seed.y_min;
    merged.y_max = seed.y_max;
    if (first_has_aabb && second_has_aabb) {
      merged.x_min = std::min(first.x_min, second.x_min);
      merged.x_max = std::max(first.x_max, second.x_max);
      merged.y_min = std::min(first.y_min, second.y_min);
      merged.y_max = std::max(first.y_max, second.y_max);
    }
    merged.x_center = 0.5 * (merged.x_min + merged.x_max);
    merged.y_center = 0.5 * (merged.y_min + merged.y_max);
    merged.radius = 0.5 * std::hypot(
      merged.x_max - merged.x_min, merged.y_max - merged.y_min);
  } else {
    merged.has_cartesian = false;
  }
  return merged;
}

}  // namespace

const char * p3RuntimeModeName(P3RuntimeMode mode)
{
  switch (mode) {
    case P3RuntimeMode::kOff: return "OFF";
    case P3RuntimeMode::kShadow: return "SHADOW";
    case P3RuntimeMode::kTestActive: return "TEST_ACTIVE";
  }
  return "UNKNOWN";
}

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: Node("local_planner_node", options), planner_(planner_parameters_)
{
  initializeParameters();
  planner_.setParameters(planner_parameters_);
  initializeInterfaces();
  last_side_switch_time_ = eventNow();
  RCLCPP_INFO(
    get_logger(),
    "Race-line-locked static planner started: lookahead=%.1f m, obstacle topic=%s, "
    "tracking_error_lut=%zux%zu fallback=%.3f m, avoidance_velocity_limits=%zu rows, "
    "wall_margin=%.3f m, lateral_guard=[%.3f, %.3f] m, P3_MODE=%s",
    planner_parameters_.detection_lookahead_m, obstacles_topic_.c_str(),
    planner_parameters_.tracking_error_lut_speed_bins_mps.size(),
    planner_parameters_.tracking_error_lut_curvature_bins_radpm.size(),
    planner_parameters_.tracking_error_reserve_m,
    planner_parameters_.avoidance_velocity_limit_speed_bins_mps.size(),
    planner_parameters_.wall_safety_margin_m,
    guard_parameters_.minimum_lateral_inflation_m,
    guard_parameters_.maximum_lateral_inflation_m,
    p3RuntimeModeName(p3_mode_));
}

void LocalPlannerNode::initializeParameters()
{
  planner_parameters_.detection_lookahead_m =
    declare_parameter<double>("detection_lookahead_m", 12.0);
  planner_parameters_.obstacle_cluster_gap_m =
    declare_parameter<double>("obstacle_cluster_gap_m", 0.8);
  planner_parameters_.obstacle_longitudinal_padding_m =
    declare_parameter<double>("obstacle_longitudinal_padding_m", 0.35);
  planner_parameters_.vehicle_length_m =
    declare_parameter<double>("vehicle_length_m", 0.56);
  planner_parameters_.vehicle_half_width_m =
    declare_parameter<double>("vehicle_half_width_m", 0.1435);
  planner_parameters_.safety_margin_m =
    declare_parameter<double>("safety_margin_m", 0.03);
  planner_parameters_.tracking_error_reserve_m =
    declare_parameter<double>("tracking_error_reserve_m", 0.14);
  planner_parameters_.tracking_error_lut_speed_bins_mps =
    declare_parameter<std::vector<double>>(
    "tracking_error_lut_speed_bins_mps",
    std::vector<double>{0.0, 1.5, 3.0, 4.5, 6.5});
  planner_parameters_.tracking_error_lut_curvature_bins_radpm =
    declare_parameter<std::vector<double>>(
    "tracking_error_lut_curvature_bins_radpm",
    std::vector<double>{0.0, 0.2, 0.5, 0.9, 1.316266519079011});
  planner_parameters_.tracking_error_lut_values_m =
    declare_parameter<std::vector<double>>(
    "tracking_error_lut_values_m",
    std::vector<double>{
      0.14, 0.14, 0.14, 0.14, 0.14,
      0.14, 0.14, 0.14, 0.14, 0.14,
      0.14, 0.14, 0.14, 0.14, 0.14,
      0.14, 0.14, 0.14, 0.14, 0.14,
      0.14, 0.14, 0.14, 0.14, 0.14});
  planner_parameters_.avoidance_velocity_limit_speed_bins_mps =
    declare_parameter<std::vector<double>>(
    "avoidance_velocity_limit_speed_bins_mps",
    std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0});
  planner_parameters_.avoidance_velocity_limit_lateral_accel_mps2 =
    declare_parameter<std::vector<double>>(
    "avoidance_velocity_limit_lateral_accel_mps2",
    std::vector<double>{7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 6.5, 6.5, 6.5, 6.5});
  planner_parameters_.avoidance_minimum_speed_mps =
    declare_parameter<double>("avoidance_minimum_speed_mps", 1.0);
  planner_parameters_.margin_pass_speed_cap_mps =
    declare_parameter<double>("margin_pass_speed_cap_mps", 2.0);
  planner_parameters_.approach_feasibility_decel_mps2 =
    declare_parameter<double>("approach_feasibility_decel_mps2", 2.0);
  planner_parameters_.commitment_retention_reserve_fraction =
    declare_parameter<double>("commitment_retention_reserve_fraction", 0.5);
  planner_parameters_.localization_reserve_m =
    declare_parameter<double>("localization_reserve_m", 0.06);
  planner_parameters_.wall_safety_margin_m =
    declare_parameter<double>("wall_safety_margin_m", 0.04);
  planner_parameters_.fallback_track_half_width_m =
    declare_parameter<double>("fallback_track_half_width_m", 1.50);
  planner_parameters_.pre_apex_distances_m =
    declare_parameter<std::vector<double>>(
    "pre_apex_distances_m", std::vector<double>{6.0, 4.0, 2.0});
  planner_parameters_.post_apex_distances_m =
    declare_parameter<std::vector<double>>(
    "post_apex_distances_m", std::vector<double>{1.0, 2.0, 3.0});
  planner_parameters_.entry_transition_fractions =
    declare_parameter<std::vector<double>>(
    "entry_transition_fractions", std::vector<double>{0.50, 0.75, 1.00});
  planner_parameters_.transition_distance_scales =
    declare_parameter<std::vector<double>>(
    "transition_distance_scales", std::vector<double>{1.0, 1.25, 1.50});
  planner_parameters_.outside_line_transition_scale =
    declare_parameter<double>("outside_line_transition_scale", 1.35);
  planner_parameters_.maximum_exit_length_m =
    declare_parameter<double>("maximum_exit_length_m", 5.0);
  planner_parameters_.post_merge_lookahead_m =
    declare_parameter<double>("post_merge_lookahead_m", 2.0);
  planner_parameters_.post_merge_min_time_sec =
    declare_parameter<double>("post_merge_min_time_sec", 1.0);
  planner_parameters_.merge_ramp_min_length_m =
    declare_parameter<double>("merge_ramp_min_length_m", 0.0);
  planner_parameters_.merge_ramp_time_sec =
    declare_parameter<double>("merge_ramp_time_sec", 0.0);
  planner_parameters_.minimum_target_offset_m =
    declare_parameter<double>("minimum_target_offset_m", 0.20);
  planner_parameters_.maximum_target_offset_m =
    declare_parameter<double>("maximum_target_offset_m", 1.50);
  planner_parameters_.target_d_candidate_count =
    declare_parameter<int>("target_d_candidate_count", 5);
  planner_parameters_.maximum_lateral_slope =
    declare_parameter<double>("maximum_lateral_slope", 0.65);
  planner_parameters_.maximum_curvature_radpm =
    declare_parameter<double>("maximum_curvature_radpm", 3.20);
  planner_parameters_.maximum_curvature_rate_radpm2 =
    declare_parameter<double>("maximum_curvature_rate_radpm2", 20.0);
  planner_parameters_.safe_stop_buffer_m =
    declare_parameter<double>("safe_stop_buffer_m", 0.40);
  planner_parameters_.safe_stop_deceleration_mps2 =
    declare_parameter<double>("safe_stop_deceleration_mps2", 2.5);
  planner_parameters_.minimum_path_points =
    declare_parameter<int>("minimum_path_points", 8);
  planner_parameters_.safe_stop_escape_check_enable =
    declare_parameter<bool>("safe_stop_escape_check_enable", true);
  planner_parameters_.safe_stop_escape_retreat_step_m =
    std::max(0.05, declare_parameter<double>("safe_stop_escape_retreat_step_m", 0.30));
  planner_parameters_.safe_stop_escape_max_retreats =
    std::clamp(static_cast<int>(declare_parameter<int>("safe_stop_escape_max_retreats", 8)), 0, 40);

  require_obstacles_message_ = declare_parameter<bool>("require_obstacles_message", true);
  obstacle_stale_timeout_sec_ = declare_parameter<double>("obstacle_stale_timeout_sec", 0.75);
  odometry_stale_timeout_sec_ = declare_parameter<double>("odometry_stale_timeout_sec", 0.50);
  merge_lateral_tolerance_m_ = declare_parameter<double>("merge_lateral_tolerance_m", 0.15);
  merge_confirm_cycles_ = declare_parameter<int>("merge_confirm_cycles", 15);
  safe_stop_release_cycles_ = declare_parameter<int>("safe_stop_release_cycles", 8);
  planning_period_ms_ = declare_parameter<int>("planning_period_ms", 50);
  state_handoff_tail_distance_m_ =
    declare_parameter<double>("state_handoff_tail_distance_m", 6.0);
  state_handoff_speed_cap_mps_ =
    declare_parameter<double>("state_handoff_speed_cap_mps", 6.0);
  initial_observation_count_ =
    declare_parameter<int>("initial_observation_count", 3);
  initial_observation_min_duration_sec_ =
    declare_parameter<double>("initial_observation_min_duration_sec", 0.15);
  initial_observation_max_wait_sec_ =
    declare_parameter<double>("initial_observation_max_wait_sec", 0.35);
  commitment_soft_violation_confirm_cycles_ =
    declare_parameter<int>("commitment_soft_violation_confirm_cycles", 3);
  chain_release_distance_m_ =
    declare_parameter<double>("chain_release_distance_m", 0.20);
  guard_parameters_.uncertainty_sigma_scale =
    declare_parameter<double>("uncertainty_sigma_scale", 3.0);
  guard_parameters_.minimum_longitudinal_inflation_m =
    declare_parameter<double>("uncertainty_min_longitudinal_inflation_m", 0.05);
  guard_parameters_.minimum_lateral_inflation_m =
    declare_parameter<double>("uncertainty_min_lateral_inflation_m", 0.0);
  guard_parameters_.maximum_lateral_inflation_m =
    declare_parameter<double>("uncertainty_max_lateral_inflation_m", 0.0);
  commitment_lock_lateral_threshold_m_ =
    declare_parameter<double>("commitment_lock_lateral_threshold_m", 0.10);
  commitment_lock_longitudinal_m_ =
    declare_parameter<double>("commitment_lock_longitudinal_m", 0.50);

  global_waypoints_topic_ =
    declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  // 기본 입력은 detector Layer 2의 confirmed-only 뷰다 (2026-08-16). /static_obs는
  // CONFIRMED+UNKNOWN(정지 증거를 아직 못 모은 provisional 객체)까지 실어, 벽 조각·
  // 산란 클러스터가 수십 ms만 살아도 플래너가 회피/정지 경로를 발행하고 state_machine이
  // STATE_AVOID로 넘어간다. /confirmed_static_obs는 map-frame 위치 지속성(static vote
  // 10/15, RMS<0.10 m)까지 통과한 객체만 싣는다. 같은 track/ID를 쓰므로 ID 계약은 동일하다.
  obstacles_topic_ =
    declare_parameter<std::string>(
    "obstacles_topic", "/confirmed_static_obs");
  frenet_odom_topic_ =
    declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
  state_topic_ =
    declare_parameter<std::string>("state_topic", "/state");
  ot_waypoints_topic_ =
    declare_parameter<std::string>("ot_waypoints_topic", "/avoid_waypoints");
  local_path_topic_ =
    declare_parameter<std::string>("local_path_topic", "/local_planning/path");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  timing_diagnostics_enable_ =
    declare_parameter<bool>("timing_diagnostics_enable", false);
  timing_diagnostics_topic_ = declare_parameter<std::string>(
    "timing_diagnostics_topic", "/cma_timing/events");
  replay_diagnostics_enable_ =
    declare_parameter<bool>("replay_diagnostics_enable", false);
  replay_diagnostics_topic_ = declare_parameter<std::string>(
    "replay_diagnostics_topic", "/cma_replay/planner_events");
  lockstep_mode_ = declare_parameter<bool>("lockstep_mode", false);
  // TEST_ACTIVE(P3 주도 + P0 백업)가 기본 주행 모드입니다 (2026-08-12).
  const std::string p3_mode = declare_parameter<std::string>("p3_mode", "TEST_ACTIVE");
  p3_diagnostics_topic_ = declare_parameter<std::string>(
    "p3_diagnostics_topic", "/local_planning/p3_shadow");
  if (p3_mode == "OFF") {
    p3_mode_ = P3RuntimeMode::kOff;
  } else if (p3_mode == "SHADOW") {
    p3_mode_ = P3RuntimeMode::kShadow;
  } else if (p3_mode == "TEST_ACTIVE") {
    p3_mode_ = P3RuntimeMode::kTestActive;
  } else {
    throw std::invalid_argument("p3_mode must be OFF, SHADOW, or TEST_ACTIVE");
  }

  // 회피 후보 생성기는 P3 하나뿐이다(2026-08-15에 P0 quintic 격자 제거). P0의 안전 계층과
  // 안전정지 사다리는 P3의 토대라 그대로 살아 있다.
  RCLCPP_INFO(
    get_logger(), "회피 플래너: P3=%s (유일한 후보 생성기, P0 격자 없음)",
    p3RuntimeModeName(p3_mode_));

  const bool control_points_valid =
    planner_parameters_.pre_apex_distances_m.size() == 3U &&
    planner_parameters_.post_apex_distances_m.size() == 3U &&
    planner_parameters_.pre_apex_distances_m[0] > planner_parameters_.pre_apex_distances_m[1] &&
    planner_parameters_.pre_apex_distances_m[1] > planner_parameters_.pre_apex_distances_m[2] &&
    planner_parameters_.pre_apex_distances_m[2] > 0.0 &&
    planner_parameters_.post_apex_distances_m[0] > 0.0 &&
    planner_parameters_.post_apex_distances_m[1] > planner_parameters_.post_apex_distances_m[0] &&
    planner_parameters_.post_apex_distances_m[2] > planner_parameters_.post_apex_distances_m[1];
  const bool scales_valid =
    !planner_parameters_.entry_transition_fractions.empty() &&
    std::all_of(
    planner_parameters_.entry_transition_fractions.begin(),
    planner_parameters_.entry_transition_fractions.end(),
    [](double fraction) {
      return std::isfinite(fraction) && fraction > 0.0 && fraction <= 1.0;
    }) &&
    std::adjacent_find(
    planner_parameters_.entry_transition_fractions.begin(),
    planner_parameters_.entry_transition_fractions.end(),
    std::greater_equal<double>()) == planner_parameters_.entry_transition_fractions.end() &&
    !planner_parameters_.transition_distance_scales.empty() &&
    std::all_of(
    planner_parameters_.transition_distance_scales.begin(),
    planner_parameters_.transition_distance_scales.end(),
    [](double scale) {return std::isfinite(scale) && scale > 0.0;}) &&
    std::adjacent_find(
    planner_parameters_.transition_distance_scales.begin(),
    planner_parameters_.transition_distance_scales.end(),
    std::greater_equal<double>()) == planner_parameters_.transition_distance_scales.end();
  if (!control_points_valid || !scales_valid) {
    throw std::invalid_argument(
            "pre/post apex arrays must contain three ordered positive values and transition "
            "scales must be positive");
  }
  if (planning_period_ms_ <= 0 || merge_confirm_cycles_ <= 0 ||
    safe_stop_release_cycles_ <= 0 ||
    !std::isfinite(planner_parameters_.vehicle_length_m) ||
    !(planner_parameters_.vehicle_length_m > 0.0) ||
    !std::isfinite(planner_parameters_.vehicle_half_width_m) ||
    !(planner_parameters_.vehicle_half_width_m > 0.0) ||
    !std::isfinite(planner_parameters_.safety_margin_m) ||
    planner_parameters_.safety_margin_m < 0.0 ||
    !std::isfinite(planner_parameters_.tracking_error_reserve_m) ||
    planner_parameters_.tracking_error_reserve_m < 0.0 ||
    !planner_parameters_.trackingErrorLutValid() ||
    !planner_parameters_.avoidanceVelocityLimitValid() ||
    !std::isfinite(planner_parameters_.avoidance_minimum_speed_mps) ||
    planner_parameters_.avoidance_minimum_speed_mps < 0.0 ||
    !std::isfinite(planner_parameters_.margin_pass_speed_cap_mps) ||
    planner_parameters_.margin_pass_speed_cap_mps < 0.0 ||
    !std::isfinite(planner_parameters_.approach_feasibility_decel_mps2) ||
    !std::isfinite(planner_parameters_.commitment_retention_reserve_fraction) ||
    planner_parameters_.commitment_retention_reserve_fraction < 0.0 ||
    planner_parameters_.commitment_retention_reserve_fraction > 1.0 ||
    !std::isfinite(planner_parameters_.localization_reserve_m) ||
    planner_parameters_.localization_reserve_m < 0.0 ||
    !std::isfinite(planner_parameters_.wall_safety_margin_m) ||
    planner_parameters_.wall_safety_margin_m < 0.0 ||
    !std::isfinite(planner_parameters_.maximum_exit_length_m) ||
    planner_parameters_.post_merge_lookahead_m < 0.0 ||
    planner_parameters_.post_merge_min_time_sec < 0.0 ||
    !std::isfinite(planner_parameters_.merge_ramp_min_length_m) ||
    planner_parameters_.merge_ramp_min_length_m < 0.0 ||
    !std::isfinite(planner_parameters_.merge_ramp_time_sec) ||
    planner_parameters_.merge_ramp_time_sec < 0.0 ||
    !std::isfinite(state_handoff_tail_distance_m_) ||
    !(state_handoff_tail_distance_m_ > 0.0) ||
    !(state_handoff_speed_cap_mps_ > 0.0) ||
    initial_observation_count_ <= 0 ||
    !std::isfinite(initial_observation_min_duration_sec_) ||
    initial_observation_min_duration_sec_ < 0.0 ||
    !std::isfinite(initial_observation_max_wait_sec_) ||
    initial_observation_max_wait_sec_ < 0.0 ||
    initial_observation_max_wait_sec_ < initial_observation_min_duration_sec_ ||
    commitment_soft_violation_confirm_cycles_ <= 0 ||
    !std::isfinite(chain_release_distance_m_) ||
    chain_release_distance_m_ < 0.0 ||
    !std::isfinite(guard_parameters_.uncertainty_sigma_scale) ||
    guard_parameters_.uncertainty_sigma_scale < 0.0 ||
    !std::isfinite(guard_parameters_.minimum_longitudinal_inflation_m) ||
    guard_parameters_.minimum_longitudinal_inflation_m < 0.0 ||
    !std::isfinite(guard_parameters_.minimum_lateral_inflation_m) ||
    guard_parameters_.minimum_lateral_inflation_m < 0.0 ||
    guard_parameters_.maximum_lateral_inflation_m <
    guard_parameters_.minimum_lateral_inflation_m ||
    planner_parameters_.target_d_candidate_count <= 0 ||
    !std::isfinite(planner_parameters_.detection_lookahead_m) ||
    planner_parameters_.detection_lookahead_m <= 0.0 ||
    planner_parameters_.pre_apex_distances_m[0] >
    planner_parameters_.detection_lookahead_m ||
    commitment_lock_lateral_threshold_m_ < 0.0 ||
    commitment_lock_longitudinal_m_ < 0.0 ||
    planner_parameters_.minimum_path_points < 2)
  {
    throw std::invalid_argument(
            "planning periods, confirmation counts, tracking-error/velocity LUTs, lateral "
            "safety, handoff "
            "settings, observation/uncertainty guard settings, commitment chain release, "
            "commitment locks, and point counts must be valid");
  }
}

void LocalPlannerNode::initializeInterfaces()
{
  planning_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odometry_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const auto global_qos = rclcpp::QoS(1).reliable().transient_local();

  rclcpp::SubscriptionOptions planning_options;
  planning_options.callback_group = planning_callback_group_;
  rclcpp::SubscriptionOptions odometry_options;
  odometry_options.callback_group = odometry_callback_group_;
  global_waypoints_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
    global_waypoints_topic_, global_qos,
    std::bind(&LocalPlannerNode::onGlobalWaypoints, this, std::placeholders::_1), planning_options);
  obstacles_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    obstacles_topic_, volatile_qos,
    std::bind(&LocalPlannerNode::onObstacles, this, std::placeholders::_1), planning_options);
  frenet_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    frenet_odom_topic_, volatile_qos,
    std::bind(&LocalPlannerNode::onFrenetOdometry, this, std::placeholders::_1), odometry_options);
  state_sub_ = create_subscription<f110_msgs::msg::StateMachine>(
    state_topic_, global_qos,
    std::bind(&LocalPlannerNode::onState, this, std::placeholders::_1), planning_options);

  avoid_waypoints_pub_ =
    create_publisher<f110_msgs::msg::OTWpntArray>(ot_waypoints_topic_, volatile_qos);
  local_path_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, volatile_qos);
  if (timing_diagnostics_enable_) {
    timing_diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
      timing_diagnostics_topic_, rclcpp::QoS(100).reliable());
  }
  if (replay_diagnostics_enable_) {
    replay_diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
      replay_diagnostics_topic_, rclcpp::QoS(1000).reliable());
  }
  if (p3_mode_ != P3RuntimeMode::kOff) {
    p3_diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
      p3_diagnostics_topic_, rclcpp::QoS(1000).reliable());
  }

  if (!lockstep_mode_) {
    planning_timer_ = create_wall_timer(
      std::chrono::milliseconds(planning_period_ms_),
      std::bind(&LocalPlannerNode::onPlanningTimer, this), planning_callback_group_);
  }
}

bool LocalPlannerNode::sameReference(const f110_msgs::msg::WpntArray & message) const
{
  if (message.wpnts.size() != global_waypoints_.wpnts.size() || message.wpnts.empty()) {
    return false;
  }
  for (std::size_t i = 0; i < message.wpnts.size(); ++i) {
    const auto & first = message.wpnts[i];
    const auto & second = global_waypoints_.wpnts[i];
    // Compare every field the planner actually consumes, not just the centreline geometry.
    // d_left/d_right feed the track-bound gate and the footprint validator, psi_rad places each
    // shifted waypoint on its own normal, kappa_radpm and vx_mps drive the velocity limit and the
    // tracking-error LUT. A boundary-only recalibration (the control team's per-sector lidar wall
    // table, see docs/local_planner.md merge-ramp note) changes none of s/x/y, so comparing those
    // alone would silently keep the planner on the previous widths while obstacle_detector --
    // which already compares d_left/d_right -- switches to the new ones.
    if (std::abs(first.s_m - second.s_m) > 1.0e-9 ||
      std::abs(first.x_m - second.x_m) > 1.0e-9 ||
      std::abs(first.y_m - second.y_m) > 1.0e-9 ||
      std::abs(first.d_left - second.d_left) > 1.0e-9 ||
      std::abs(first.d_right - second.d_right) > 1.0e-9 ||
      std::abs(first.psi_rad - second.psi_rad) > 1.0e-9 ||
      std::abs(first.kappa_radpm - second.kappa_radpm) > 1.0e-9 ||
      std::abs(first.vx_mps - second.vx_mps) > 1.0e-9)
    {
      return false;
    }
  }
  return true;
}

void LocalPlannerNode::onGlobalWaypoints(
  const f110_msgs::msg::WpntArray::SharedPtr message)
{
  if (sameReference(*message)) {
    return;
  }
  std::string error;
  if (!planner_.setReference(*message, &error)) {
    has_global_waypoints_ = false;
    clearCommitment();
    if (p3_mode_ != P3RuntimeMode::kOff) {
      ++p3_source_epoch_;
      ++global_reference_generation_;
      p3_maneuver_lifecycle_.reset();
      resetP3SelectionEnvelope();
    }
    RCLCPP_ERROR(get_logger(), "Rejected /global_waypoints: %s", error.c_str());
    return;
  }
  global_waypoints_ = *message;
  has_global_waypoints_ = true;
  clearCommitment();
  if (p3_mode_ != P3RuntimeMode::kOff) {
    ++p3_source_epoch_;
    ++global_reference_generation_;
    p3_maneuver_lifecycle_.reset();
    resetP3SelectionEnvelope();
  }
  RCLCPP_INFO(
    get_logger(), "Loaded %zu ordered global race-line waypoints (track %.2f m).",
    global_waypoints_.wpnts.size(), planner_.trackLength());
}

void LocalPlannerNode::onObstacles(const f110_msgs::msg::ObstacleArray::SharedPtr message)
{
  if (!message->header.frame_id.empty() && message->header.frame_id != frame_id_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring obstacle array in frame '%s'; expected '%s'. Retaining the last valid snapshot.",
      message->header.frame_id.c_str(), frame_id_.c_str());
    return;
  }

  std::vector<f110_msgs::msg::Obstacle> accepted_obstacles;
  accepted_obstacles.reserve(message->obstacles.size());
  std::size_t rejected = 0;
  for (const auto & obstacle : message->obstacles) {
    if (validFrenetObstacle(obstacle, planner_.trackLength())) {
      accepted_obstacles.push_back(obstacle);
    } else {
      ++rejected;
    }
  }
  if (rejected > 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected %zu static obstacles with invalid detector-provided Frenet bounds.",
      rejected);
  }
  // An array whose every obstacle was rejected is degraded perception, not proof that the track is
  // clear. Accepting it would store an empty snapshot that is indistinguishable from an explicitly
  // empty array, and an explicitly empty array is contractually allowed to erase the retained
  // obstacle memory. Retain the last valid snapshot instead, exactly as the wrong-frame branch
  // above does, and do not advance the sequence, source stamp, or P3 epoch from it.
  if (!message->obstacles.empty() && accepted_obstacles.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Every obstacle in a %zu-entry %s array had invalid Frenet bounds; treating it as "
      "degraded perception and retaining the last valid snapshot.",
      message->obstacles.size(), obstacles_topic_.c_str());
    return;
  }
  const std::int64_t incoming_source_stamp_ns = stampNs(message->header.stamp);
  if (p3_mode_ != P3RuntimeMode::kOff && has_obstacles_message_ &&
    incoming_source_stamp_ns < latest_obstacle_source_stamp_ns_)
  {
    // 미세 역행(재발행/전송 순서 뒤섞임 수준)은 소스 재시작이 아니라 그냥 늦게 도착한
    // 옛 메시지다. epoch 리셋은 라이프사이클·선택 봉투를 전부 버리므로, 250 Hz 재발행
    // 시뮬처럼 3 ms 역행이 초당 수십 번 오면 플래너가 영구 마비된다 (2026-08-15 run24
    // 실측). 옛 메시지는 버리고 최신 상태를 유지한다. 큰 역행(소스 재시작·백 루프)만
    // 기존대로 epoch를 올린다.
    constexpr std::int64_t kSourceRestartRegressionNs = 500000000;  // 0.5 s
    if (latest_obstacle_source_stamp_ns_ - incoming_source_stamp_ns <
      kSourceRestartRegressionNs)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Out-of-order %s dropped (%" PRId64 " ns behind latest)",
        obstacles_topic_.c_str(),
        latest_obstacle_source_stamp_ns_ - incoming_source_stamp_ns);
      return;
    }
    ++p3_source_epoch_;
    p3_maneuver_lifecycle_.reset();
    resetP3SelectionEnvelope();
    RCLCPP_WARN(
      get_logger(),
      "P3 source epoch advanced after obstacle source-stamp regression: %" PRId64
      " -> %" PRId64,
      latest_obstacle_source_stamp_ns_, incoming_source_stamp_ns);
  }
  static_obstacles_ = std::move(accepted_obstacles);
  has_obstacles_message_ = true;
  last_obstacles_time_ = lockstep_mode_ ? rclcpp::Time(message->header.stamp) : now();
  ++obstacles_message_sequence_;
  latest_obstacle_source_stamp_ns_ = incoming_source_stamp_ns;
  if (timing_diagnostics_enable_ && !timing_t0_published_ && !static_obstacles_.empty()) {
    nav_msgs::msg::Odometry odometry;
    bool has_odometry = false;
    {
      std::lock_guard<std::mutex> lock(odometry_mutex_);
      has_odometry = has_odometry_;
      if (has_odometry) {
        odometry = latest_odometry_;
      }
    }
    std::ostringstream fields;
    fields << std::setprecision(17)
           << "\"source_stamp_ns\":" << stampNs(message->header.stamp)
           << ",\"obstacle_count\":" << static_obstacles_.size()
           << ",\"obstacle_ids\":[";
    for (std::size_t index = 0; index < static_obstacles_.size(); ++index) {
      if (index > 0U) {
        fields << ',';
      }
      fields << static_obstacles_[index].id;
    }
    fields << ']';
    if (has_odometry) {
      fields << ",\"ego_s\":" << odometry.pose.pose.position.x
             << ",\"ego_d\":" << odometry.pose.pose.position.y
             << ",\"speed_mps\":" << odometry.twist.twist.linear.x;
    }
    publishTimingEvent("T0_STATIC_OBS", fields.str());
    timing_t0_published_ = true;
  }
  if (obstacle_perception_degraded_) {
    obstacle_perception_degraded_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Static-obstacle perception recovered; replaced retained memory with a fresh snapshot.");
  }
  if (lockstep_mode_) {
    {
      std::lock_guard<std::mutex> lock(lockstep_mutex_);
      lockstep_obstacle_stamp_ns_ = stampNs(message->header.stamp);
    }
    tryRunLockstepCycle();
  }
}

void LocalPlannerNode::onFrenetOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (!finiteOdometry(*message)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Ignoring non-finite Frenet odometry.");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    // 미세 역행 샘플(재발행 순서 뒤섞임)은 버려 저장값을 단조롭게 유지한다 — 저장값이
    // 뒤로 가면 캡처 단계의 소스-스탬프 역행 감지가 epoch 리셋/사이클 스킵을 일으킨다
    // (onObstacles의 동일 처리 참고). 큰 역행(소스 재시작)은 통과시켜 기존 감지에 맡긴다.
    constexpr std::int64_t kSourceRestartRegressionNs = 500000000;  // 0.5 s
    const std::int64_t incoming_ns = stampNs(message->header.stamp);
    const std::int64_t latest_ns = stampNs(latest_odometry_.header.stamp);
    if (has_odometry_ && incoming_ns < latest_ns &&
      latest_ns - incoming_ns < kSourceRestartRegressionNs)
    {
      return;
    }
    latest_odometry_ = *message;
    last_odometry_time_ = lockstep_mode_ ? rclcpp::Time(message->header.stamp) : now();
    has_odometry_ = true;
  }
  if (lockstep_mode_) {
    {
      std::lock_guard<std::mutex> lock(lockstep_mutex_);
      lockstep_odometry_stamp_ns_ = stampNs(message->header.stamp);
    }
    tryRunLockstepCycle();
  }
}

void LocalPlannerNode::onState(const f110_msgs::msg::StateMachine::SharedPtr message)
{
  current_state_ = message->state;
  has_state_ = true;
  if (lockstep_mode_) {
    {
      std::lock_guard<std::mutex> lock(lockstep_mutex_);
      lockstep_state_stamp_ns_ = stampNs(message->header.stamp);
    }
    tryRunLockstepCycle();
  }
  if (has_commitment_ &&
    current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID)
  {
    avoid_state_observed_ = true;
  }
}

rclcpp::Time LocalPlannerNode::eventNow() const
{
  return lockstep_mode_ ? lockstep_event_time_ : now();
}

void LocalPlannerNode::tryRunLockstepCycle()
{
  std::int64_t cycle_stamp_ns = 0;
  {
    std::lock_guard<std::mutex> lock(lockstep_mutex_);
    if (!lockstep_mode_ || lockstep_obstacle_stamp_ns_ <= 0 ||
      lockstep_obstacle_stamp_ns_ != lockstep_odometry_stamp_ns_ ||
      lockstep_obstacle_stamp_ns_ <= lockstep_last_processed_stamp_ns_)
    {
      return;
    }
    if (lockstep_last_processed_stamp_ns_ > 0 &&
      lockstep_state_stamp_ns_ < lockstep_last_processed_stamp_ns_)
    {
      return;
    }
    cycle_stamp_ns = lockstep_obstacle_stamp_ns_;
    lockstep_last_processed_stamp_ns_ = cycle_stamp_ns;
    lockstep_event_time_ = rclcpp::Time(cycle_stamp_ns, RCL_ROS_TIME);
  }
  onPlanningTimer();
}

void LocalPlannerNode::clearCommitment()
{
  resetInitialStabilization();
  resetNextManeuverStabilization();
  resetCommitmentViolationConfirmation();
  completed_obstacle_ids_.clear();
  has_commitment_ = false;
  committed_result_ = RacelineSplineResult();
  clearSafeStopLatch();
  merge_complete_count_ = 0;
  merge_geometry_confirmed_ = false;
  handoff_active_ = false;
  avoid_state_observed_ = false;
  pre_engagement_side_switched_ = false;
  committed_obstacle_guards_.clear();
}

void LocalPlannerNode::resetInitialStabilization()
{
  initial_stabilization_active_ = false;
  initial_prepare_published_ = false;
  initial_has_counted_sequence_ = false;
  initial_cluster_union_.clear();
  initial_observation_counts_.clear();
}

void LocalPlannerNode::resetP3SelectionEnvelope()
{
  p3_selection_has_sequence_ = false;
  p3_selection_envelope_union_.clear();
  p3_selection_observation_counts_.clear();
  p3_selection_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void LocalPlannerNode::resetNextManeuverStabilization()
{
  next_stabilization_active_ = false;
  next_has_counted_sequence_ = false;
  next_cluster_union_.clear();
  next_observation_counts_.clear();
}

void LocalPlannerNode::resetCommitmentViolationConfirmation()
{
  commitment_soft_violation_count_ = 0;
}

std::vector<f110_msgs::msg::Obstacle>
LocalPlannerNode::buildInitialStabilizationInput() const
{
  std::map<int, f110_msgs::msg::Obstacle> conservative;
  for (const auto & obstacle : static_obstacles_) {
    if (completed_obstacle_ids_.count(obstacle.id) > 0U) {
      continue;
    }
    conservative[obstacle.id] = obstacle;
  }
  for (const auto & entry : initial_cluster_union_) {
    const int id = entry.first;
    const auto current = conservative.find(id);
    if (current == conservative.end()) {
      conservative[id] = entry.second;
      continue;
    }
    conservative[id] = mergeObstacleEnvelopes(
      current->second, entry.second, planner_.trackLength());
  }

  std::vector<f110_msgs::msg::Obstacle> result;
  result.reserve(conservative.size());
  for (const auto & entry : conservative) {
    result.push_back(entry.second);
  }
  return result;
}

std::vector<f110_msgs::msg::Obstacle> LocalPlannerNode::buildGuardedObstacles(
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  std::vector<f110_msgs::msg::Obstacle> guarded;
  guarded.reserve(obstacles.size());
  for (const auto & obstacle : obstacles) {
    guarded.push_back(
      buildUncertaintyGuard(obstacle, planner_.trackLength(), guard_parameters_));
  }
  return guarded;
}

bool LocalPlannerNode::updateInitialStabilization(
  const std::vector<int> & cluster_ids,
  const std::vector<f110_msgs::msg::Obstacle> & conservative_obstacles,
  const rclcpp::Time & update_time)
{
  if (cluster_ids.empty()) {
    resetInitialStabilization();
    return false;
  }

  const auto contains_id = [](const std::vector<int> & ids, int id) {
      return std::find(ids.begin(), ids.end(), id) != ids.end();
    };
  bool restart = !initial_stabilization_active_;
  if (!restart && !initial_cluster_union_.empty()) {
    restart = std::none_of(
      initial_cluster_union_.begin(), initial_cluster_union_.end(),
      [&cluster_ids, &contains_id](const auto & entry) {
        return contains_id(cluster_ids, entry.first);
      });
  }
  if (restart) {
    initial_stabilization_active_ = true;
    initial_stabilization_start_ = update_time;
    initial_has_counted_sequence_ = false;
    initial_cluster_union_.clear();
    initial_observation_counts_.clear();
    if (replay_diagnostics_enable_) {
      std::ostringstream fields;
      fields << "\"source_stamp_ns\":" << latest_obstacle_source_stamp_ns_
             << ",\"obstacle_sequence\":" << obstacles_message_sequence_
             << ",\"cluster_ids\":[";
      for (std::size_t index = 0; index < cluster_ids.size(); ++index) {
        if (index > 0U) {
          fields << ',';
        }
        fields << cluster_ids[index];
      }
      fields << ']';
      publishReplayEvent("INITIAL_STABILIZATION_START", fields.str());
    }
  }

  const bool new_obstacle_message =
    !initial_has_counted_sequence_ ||
    initial_last_counted_sequence_ != obstacles_message_sequence_;
  if (new_obstacle_message) {
    initial_has_counted_sequence_ = true;
    initial_last_counted_sequence_ = obstacles_message_sequence_;
    for (const int id : cluster_ids) {
      const auto current = std::find_if(
        static_obstacles_.begin(), static_obstacles_.end(),
        [id](const auto & candidate) {return candidate.id == id;});
      if (current == static_obstacles_.end()) {
        continue;
      }
      const auto conservative = std::find_if(
        conservative_obstacles.begin(), conservative_obstacles.end(),
        [id](const auto & candidate) {return candidate.id == id;});
      initial_cluster_union_[id] =
        conservative != conservative_obstacles.end() ? *conservative : *current;
      ++initial_observation_counts_[id];
    }
  }

  const bool observation_count_reached = std::all_of(
    cluster_ids.begin(), cluster_ids.end(),
    [this](int id) {
      const auto count = initial_observation_counts_.find(id);
      return count != initial_observation_counts_.end() &&
             count->second >= initial_observation_count_;
    });
  const double total_duration = (update_time - initial_stabilization_start_).seconds();
  const bool minimum_duration_reached =
    total_duration >= initial_observation_min_duration_sec_;
  const bool ready = (observation_count_reached && minimum_duration_reached) ||
    total_duration >= initial_observation_max_wait_sec_;
  if (ready && replay_diagnostics_enable_) {
    std::ostringstream fields;
    fields << std::setprecision(17)
           << "\"source_stamp_ns\":" << latest_obstacle_source_stamp_ns_
           << ",\"obstacle_sequence\":" << obstacles_message_sequence_
           << ",\"duration_sec\":" << total_duration
           << ",\"observation_count_reached\":"
           << (observation_count_reached ? "true" : "false")
           << ",\"minimum_duration_reached\":"
           << (minimum_duration_reached ? "true" : "false");
    publishReplayEvent("INITIAL_STABILIZATION_READY", fields.str());
  }
  return ready;
}

std::vector<f110_msgs::msg::Obstacle> LocalPlannerNode::buildNextManeuverInput() const
{
  std::set<int> excluded = completed_obstacle_ids_;
  if (has_commitment_) {
    excluded.insert(
      committed_result_.obstacle_ids.begin(), committed_result_.obstacle_ids.end());
    if (committed_result_.obstacle_id >= 0) {
      excluded.insert(committed_result_.obstacle_id);
    }
  }

  std::map<int, f110_msgs::msg::Obstacle> conservative;
  for (const auto & obstacle : static_obstacles_) {
    if (excluded.count(obstacle.id) == 0U) {
      conservative[obstacle.id] = obstacle;
    }
  }
  for (const auto & entry : next_cluster_union_) {
    if (excluded.count(entry.first) > 0U) {
      continue;
    }
    const auto current = conservative.find(entry.first);
    if (current == conservative.end()) {
      conservative[entry.first] = entry.second;
    } else {
      current->second = mergeObstacleEnvelopes(
        current->second, entry.second, planner_.trackLength());
    }
  }

  std::vector<f110_msgs::msg::Obstacle> result;
  result.reserve(conservative.size());
  for (const auto & entry : conservative) {
    result.push_back(entry.second);
  }
  return buildGuardedObstacles(result);
}

bool LocalPlannerNode::updateNextManeuverStabilization(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & next_obstacles,
  const rclcpp::Time & update_time)
{
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance) {
    resetNextManeuverStabilization();
    return false;
  }

  // Stabilize the next cluster from the current ego state. A long, smooth return to d=0 must not
  // hide an already-visible obstacle merely because it lies before the old maneuver's merge_s.
  const auto cluster_ids = planner_.blockingClusterIds(ego, next_obstacles);
  if (cluster_ids.empty()) {
    resetNextManeuverStabilization();
    return false;
  }
  const auto contains_id = [&cluster_ids](int id) {
      return std::find(cluster_ids.begin(), cluster_ids.end(), id) != cluster_ids.end();
    };
  bool restart = !next_stabilization_active_;
  if (!restart && !next_cluster_union_.empty()) {
    restart = std::none_of(
      next_cluster_union_.begin(), next_cluster_union_.end(),
      [&contains_id](const auto & entry) {return contains_id(entry.first);});
  }
  if (restart) {
    next_stabilization_active_ = true;
    next_stabilization_start_ = update_time;
    next_has_counted_sequence_ = false;
    next_cluster_union_.clear();
    next_observation_counts_.clear();
  }

  const bool new_obstacle_message =
    !next_has_counted_sequence_ ||
    next_last_counted_sequence_ != obstacles_message_sequence_;
  if (new_obstacle_message) {
    next_has_counted_sequence_ = true;
    next_last_counted_sequence_ = obstacles_message_sequence_;
    for (const int id : cluster_ids) {
      const auto current = std::find_if(
        static_obstacles_.begin(), static_obstacles_.end(),
        [id](const auto & candidate) {return candidate.id == id;});
      if (current == static_obstacles_.end()) {
        continue;
      }
      const auto previous = next_cluster_union_.find(id);
      next_cluster_union_[id] = previous == next_cluster_union_.end() ?
        *current :
        mergeObstacleEnvelopes(previous->second, *current, planner_.trackLength());
      ++next_observation_counts_[id];
    }
  }

  const bool observation_count_reached = std::all_of(
    cluster_ids.begin(), cluster_ids.end(),
    [this](int id) {
      const auto count = next_observation_counts_.find(id);
      return count != next_observation_counts_.end() &&
             count->second >= initial_observation_count_;
    });
  const double duration = (update_time - next_stabilization_start_).seconds();
  return obstacle_perception_degraded_ ||
         (observation_count_reached && duration >= initial_observation_min_duration_sec_) ||
         duration >= initial_observation_max_wait_sec_;
}

void LocalPlannerNode::promoteNextManeuverStabilization()
{
  if (next_stabilization_active_) {
    initial_stabilization_active_ = true;
    initial_stabilization_start_ = next_stabilization_start_;
    initial_has_counted_sequence_ = next_has_counted_sequence_;
    initial_last_counted_sequence_ = next_last_counted_sequence_;
    initial_cluster_union_ = next_cluster_union_;
    initial_observation_counts_ = next_observation_counts_;
  }
  resetNextManeuverStabilization();
}

double LocalPlannerNode::remainingDistanceToMerge(const EgoFrenetState & ego) const
{
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance) {
    return 0.0;
  }
  const double planned_distance =
    planner_.forwardDistance(commitment_start_s_, committed_result_.merge_s);
  const double driven_distance = planner_.forwardDistance(commitment_start_s_, ego.s);
  if (driven_distance >= 0.5 * planner_.trackLength() ||
    driven_distance + 1.0e-6 >= planned_distance)
  {
    return 0.0;
  }
  return planned_distance - driven_distance;
}

double LocalPlannerNode::maneuverCollisionHorizon(const EgoFrenetState & ego) const
{
  // 커밋 경로를 재검증할 때 쓰는 장애물 검사 범위. 후보 선택(`generateP3Candidates`)과
  // **같은 정의**여야 한다: 이 기동이 책임지는 클러스터 끝 + post_merge_lookahead.
  //
  // 예전에는 여기만 merge까지(remainingDistanceToMerge) 봤다. exit 램프가 길면 merge가
  // 클러스터 끝보다 10 m 넘게 뒤에 놓이는데, 그 사이에 다음 장애물이 있으면 선택기는
  // 통과시킨 경로를 재검증이 매번 기각한다. 2026-08-16 백에서 그 결과가 25 ms마다 같은
  // 후보를 다시 고르는 무한 재계획이었다(랩당 hard collision 41회). 두 범위가 어긋나는 것은
  // 더 엄격한 검사가 아니라 수렴하지 않는 루프다.
  //
  // 클러스터 끝은 이 커밋이 얼린 Guard들의 뒤쪽 경계로 잡는다 — Guard가 곧 이 기동이
  // 피하기로 한 확장 엔벨로프다. Guard가 없으면(핸드오프 루프 등) 종전 merge 기준을
  // 그대로 쓴다.
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance ||
    committed_obstacle_guards_.empty())
  {
    return remainingDistanceToMerge(ego);
  }
  double cluster_end_forward = 0.0;
  for (const auto & entry : committed_obstacle_guards_) {
    const double forward = planner_.forwardDistance(ego.s, entry.second.s_end);
    if (forward > 0.5 * planner_.trackLength()) {
      continue;   // 이미 지나친 Guard — 전방 범위에 기여하지 않는다.
    }
    cluster_end_forward = std::max(
      cluster_end_forward,
      forward + planner_parameters_.obstacle_longitudinal_padding_m);
  }
  return cluster_end_forward + planner_.postMergeLookaheadM();
}

bool LocalPlannerNode::activeManeuverObstacleCleared(const EgoFrenetState & ego) const
{
  if (!has_commitment_ || committed_obstacle_guards_.empty()) {
    return false;
  }
  const double driven_distance = planner_.forwardDistance(commitment_start_s_, ego.s);
  if (driven_distance >= 0.5 * planner_.trackLength()) {
    return false;
  }
  double active_rear_distance = 0.0;
  for (const auto & entry : committed_obstacle_guards_) {
    active_rear_distance = std::max(
      active_rear_distance,
      planner_.forwardDistance(commitment_start_s_, entry.second.s_end) +
      planner_parameters_.obstacle_longitudinal_padding_m);
  }
  return driven_distance + 1.0e-6 >= active_rear_distance + chain_release_distance_m_;
}

bool LocalPlannerNode::tryEarlyChainedManeuver(
  const EgoFrenetState & ego,
  std::vector<f110_msgs::msg::Obstacle> & next_obstacles)
{
  if (!has_commitment_ || merge_geometry_confirmed_) {
    return false;
  }
  next_obstacles = buildNextManeuverInput();
  const bool next_stable = updateNextManeuverStabilization(ego, next_obstacles, eventNow());
  if (!next_stable || !activeManeuverObstacleCleared(ego)) {
    return false;
  }

  RacelineSplineResult next_result = planner_.plan(ego, next_obstacles);
  if (next_result.kind != SplinePlanKind::kAvoidance) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Next static maneuver is stabilized but not yet feasible from ego "
      "(ego_s=%.3f ego_d=%.3f current_merge_s=%.3f): %s",
      ego.s, ego.d, committed_result_.merge_s, next_result.reason.c_str());
    return false;
  }

  const int completed_id = committed_result_.obstacle_id;
  const double previous_merge_s = committed_result_.merge_s;
  resetForChainedManeuver();
  commitAvoidance(std::move(next_result), ego, next_obstacles);
  resetNextManeuverStabilization();
  RCLCPP_INFO(
    get_logger(),
    "Preemptively chained completed obstacle %d to obstacle %d before the old merge "
    "(ego_s=%.3f ego_d=%.3f old_merge_s=%.3f); STATE_AVOID remains active.",
    completed_id, committed_result_.obstacle_id, ego.s, ego.d, previous_merge_s);
  return true;
}

std::vector<f110_msgs::msg::Obstacle> LocalPlannerNode::buildCurrentManeuverInput(
  const EgoFrenetState & ego,
  bool apply_uncertainty_guard) const
{
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance) {
    const auto initial = buildInitialStabilizationInput();
    return apply_uncertainty_guard ? buildGuardedObstacles(initial) : initial;
  }

  std::set<int> active_ids(
    committed_result_.obstacle_ids.begin(), committed_result_.obstacle_ids.end());
  if (committed_result_.obstacle_id >= 0) {
    active_ids.insert(committed_result_.obstacle_id);
  }
  const double merge_forward = planner_.forwardDistance(ego.s, committed_result_.merge_s);
  const bool merge_is_ahead = merge_forward < 0.5 * planner_.trackLength();

  std::vector<f110_msgs::msg::Obstacle> result;
  result.reserve(static_obstacles_.size() + committed_obstacle_guards_.size());
  std::set<int> observed_active_ids;
  for (const auto & obstacle : static_obstacles_) {
    if (completed_obstacle_ids_.count(obstacle.id) > 0U) {
      continue;
    }
    auto validation_obstacle = apply_uncertainty_guard ?
      buildUncertaintyGuard(obstacle, planner_.trackLength(), guard_parameters_) :
      obstacle;
    if (active_ids.count(obstacle.id) > 0U) {
      observed_active_ids.insert(obstacle.id);
      const auto committed_guard = committed_obstacle_guards_.find(obstacle.id);
      if (apply_uncertainty_guard &&
        committed_guard != committed_obstacle_guards_.end() &&
        obstacleEnvelopeContained(
          validation_obstacle, committed_guard->second, planner_.trackLength()))
      {
        validation_obstacle = committed_guard->second;
      }
    }

    if (!merge_is_ahead || active_ids.count(obstacle.id) > 0U) {
      result.push_back(validation_obstacle);
      continue;
    }

    const double center_forward = planner_.forwardDistance(ego.s, validation_obstacle.s_center);
    const double span_forward =
      planner_.forwardDistance(validation_obstacle.s_start, validation_obstacle.s_end);
    const double span_reverse =
      planner_.forwardDistance(validation_obstacle.s_end, validation_obstacle.s_start);
    double span = std::min(span_forward, span_reverse);
    if (!(span > 1.0e-6)) {
      span = std::max(0.05, std::abs(validation_obstacle.size));
    }
    const double start_forward = center_forward - 0.5 * span -
      planner_parameters_.obstacle_longitudinal_padding_m;
    if (start_forward <= merge_forward + 1.0e-6) {
      result.push_back(validation_obstacle);
    }
  }
  if (apply_uncertainty_guard) {
    for (const int id : active_ids) {
      if (observed_active_ids.count(id) > 0U) {
        continue;
      }
      const auto committed_guard = committed_obstacle_guards_.find(id);
      if (committed_guard != committed_obstacle_guards_.end()) {
        result.push_back(committed_guard->second);
      }
    }
  }
  return result;
}

void LocalPlannerNode::resetForChainedManeuver()
{
  completed_obstacle_ids_.insert(
    committed_result_.obstacle_ids.begin(), committed_result_.obstacle_ids.end());
  if (committed_result_.obstacle_id >= 0) {
    completed_obstacle_ids_.insert(committed_result_.obstacle_id);
  }
  const bool avoid_was_observed = has_state_ ?
    current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID :
    avoid_state_observed_;
  resetInitialStabilization();
  resetCommitmentViolationConfirmation();
  has_commitment_ = false;
  committed_result_ = RacelineSplineResult();
  clearSafeStopLatch();
  merge_complete_count_ = 0;
  merge_geometry_confirmed_ = false;
  handoff_active_ = false;
  avoid_state_observed_ = avoid_was_observed;
  pre_engagement_side_switched_ = false;
  committed_obstacle_guards_.clear();
}

bool LocalPlannerNode::beginChainedManeuverIfNeeded(
  const EgoFrenetState & ego,
  std::vector<f110_msgs::msg::Obstacle> & next_obstacles,
  const std::string & phase)
{
  if (!has_commitment_) {
    return false;
  }
  next_obstacles = buildNextManeuverInput();
  const auto next_cluster_ids = planner_.blockingClusterIds(ego, next_obstacles);
  if (next_cluster_ids.empty()) {
    return false;
  }

  std::string ids;
  for (const int id : next_cluster_ids) {
    ids += ids.empty() ? std::to_string(id) : "," + std::to_string(id);
  }

  // Plan-then-swap (2026-08-16): 다음 기동을 위해 기존 커밋을 지우기 **전에** 현재 ego에서
  // 새 회피가 성립하는지 먼저 본다. 성립하면 그 경로로 곧바로 교체 커밋한다 — 지우고 나서
  // 계획하던 예전 순서는 반대쪽 기동으로 넘어가는 전환부에서 매 랩 정지를 만들었다:
  // ego가 아직 이전 기동의 오프셋(d≈-0.5)에 있는 채로 준비-정지 경로부터 만들었고, 그
  // 지점의 준비-정지는 footprint_track_bound로 기각되어 안전정지 래치 + 8사이클 해제
  // 대기가 됐다(2026-08-16 백 3개 공통: s=32.4~33.4에서 랩당 0.26~0.35 s 감속, 그 뒤
  // 결국 같은 좌측 회피를 커밋). 측 잠금은 이미 해제된 상태의 계획이므로 양측을 다 본다.
  RacelineSplineResult next_result = planner_.plan(ego, next_obstacles);
  if (next_result.kind == SplinePlanKind::kAvoidance) {
    RCLCPP_INFO(
      get_logger(),
      "Chaining static avoidance during %s for new blocking cluster [%s]; "
      "swapping directly to a validated avoidance (plan-then-swap).",
      phase.c_str(), ids.c_str());
    resetForChainedManeuver();
    commitAvoidance(std::move(next_result), ego, next_obstacles);
    resetNextManeuverStabilization();
    return true;
  }

  // 새 회피가 아직 불가능하면 종전 순서로 간다: 커밋을 지우고 안정화/준비-정지/안전정지
  // 사다리에 맡긴다. 이때도 "안전한 제동 경로"가 다음 사이클에 즉시 만들어진다.
  RCLCPP_INFO(
    get_logger(),
    "Chaining static avoidance during %s for new blocking cluster [%s]; "
    "no immediate avoidance from ego (%s) — releasing the completed maneuver's side lock.",
    phase.c_str(), ids.c_str(), next_result.reason.c_str());
  resetForChainedManeuver();
  promoteNextManeuverStabilization();
  return true;
}

bool LocalPlannerNode::commitmentSideLocked(const EgoFrenetState & ego) const
{
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance) {
    return false;
  }
  // margin slow pass(d=0 경로)의 go_left는 임의값이다 — 라인을 그대로 달리는 경로에는
  // "측"이 없다. 이것이 잠금을 만들면, 장애물이 뒤늦게 라인을 막았을 때 재계획이 그 임의
  // 측에만 갇혀 가능한 반대측 회피를 영원히 못 본다 (run15 실측: 우측 창 0.17 m가 유효한데
  // 임의 "left" 잠금 때문에 해제 조건 B가 영구 false — 분 단위 크립 정체의 원인).
  if (committed_result_.margin_pass) {
    return false;
  }
  const bool lateral_engaged = committed_result_.go_left ?
    ego.d >= commitment_lock_lateral_threshold_m_ :
    ego.d <= -commitment_lock_lateral_threshold_m_;
  const double driven_distance = planner_.forwardDistance(commitment_start_s_, ego.s);
  const bool longitudinal_engaged =
    driven_distance >= commitment_lock_longitudinal_m_ &&
    driven_distance < 0.5 * planner_.trackLength();
  return lateral_engaged || longitudinal_engaged;
}

void LocalPlannerNode::commitAvoidance(
  RacelineSplineResult result,
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles)
{
  publishCandidateAudit(result, "commitment");
  const bool replacing = has_commitment_;
  const bool avoid_was_observed = avoid_state_observed_;
  if (replacing && committed_result_.kind == SplinePlanKind::kAvoidance &&
    result.go_left != committed_result_.go_left && !commitmentSideLocked(ego))
  {
    pre_engagement_side_switched_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Pre-engagement side switch to %s; further side switches are disabled until lateral "
      "engagement or the next maneuver.",
      result.go_left ? "left" : "right");
  }
  if (!replacing) {
    pre_engagement_side_switched_ = false;
  }
  std::set<int> committed_ids(result.obstacle_ids.begin(), result.obstacle_ids.end());
  if (result.obstacle_id >= 0) {
    committed_ids.insert(result.obstacle_id);
  }
  committed_obstacle_guards_.clear();
  for (const auto & obstacle : planning_obstacles) {
    if (committed_ids.count(obstacle.id) > 0U) {
      committed_obstacle_guards_[obstacle.id] = obstacle;
    }
  }
  resetInitialStabilization();
  resetCommitmentViolationConfirmation();
  committed_result_ = std::move(result);
  has_commitment_ = true;
  clearSafeStopLatch();
  commitment_start_s_ = ego.s;
  merge_complete_count_ = 0;
  merge_geometry_confirmed_ = false;
  handoff_active_ = false;
  avoid_state_observed_ = avoid_was_observed ||
    (has_state_ && current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID);
  if (!replacing) {
    resetNextManeuverStabilization();
  }
  if (replay_diagnostics_enable_) {
    std::ostringstream replay_fields;
    replay_fields << std::setprecision(17)
                  << "\"source_stamp_ns\":" << latest_obstacle_source_stamp_ns_
                  << ",\"obstacle_sequence\":" << obstacles_message_sequence_
                  << ",\"replacing\":" << (replacing ? "true" : "false")
                  << ",\"obstacle_id\":" << committed_result_.obstacle_id
                  << ",\"obstacle_ids\":[";
    for (std::size_t index = 0; index < committed_result_.obstacle_ids.size(); ++index) {
      if (index > 0U) {
        replay_fields << ',';
      }
      replay_fields << committed_result_.obstacle_ids[index];
    }
    replay_fields << ']'
                  << ",\"go_left\":" << (committed_result_.go_left ? "true" : "false")
                  << ",\"target_d\":" << committed_result_.target_d
                  << ",\"entry_transition_scale\":"
                  << committed_result_.entry_transition_scale
                  << ",\"exit_transition_scale\":"
                  << committed_result_.exit_transition_scale
                  << ",\"effective_entry_transition_scale\":"
                  << committed_result_.effective_entry_transition_scale
                  << ",\"effective_exit_transition_scale\":"
                  << committed_result_.effective_exit_transition_scale
                  << ",\"requested_entry_length_m\":"
                  << committed_result_.requested_entry_length_m
                  << ",\"effective_entry_length_m\":"
                  << committed_result_.effective_entry_length_m
                  << ",\"exit_length_m\":" << committed_result_.exit_length_m
                  << ",\"generated_candidate_count\":"
                  << committed_result_.candidate_audits.size()
                  << ",\"feasible_candidate_count\":"
                  << std::count_if(
      committed_result_.candidate_audits.begin(), committed_result_.candidate_audits.end(),
      [](const SplineCandidateAudit & audit) {return audit.feasible;})
                  << ",\"ego_s\":" << ego.s
                  << ",\"ego_d\":" << ego.d
                  << ",\"speed_mps\":" << ego.speed;
    publishReplayEvent("COMMITMENT", replay_fields.str());
  }
  RCLCPP_INFO(
    get_logger(), "%s %s d-offset spline around static obstacle %d (target d=%.2f).",
    replacing ? "Replaced commitment with" : "Committed",
    committed_result_.go_left ? "left" : "right",
    committed_result_.obstacle_id, committed_result_.target_d);
}

void LocalPlannerNode::clearSafeStopLatch()
{
  safe_stop_lifecycle_.reset();
  safe_stop_result_ = RacelineSplineResult();
}

bool LocalPlannerNode::activateGlobalHandoff(
  const EgoFrenetState & ego,
  SafeStopReleaseReason safe_stop_release_reason)
{
  if (safe_stop_lifecycle_.active() &&
    safe_stop_release_reason != SafeStopReleaseReason::kObstaclePassed &&
    safe_stop_release_reason != SafeStopReleaseReason::kStoppedCorridorClear)
  {
    RCLCPP_WARN(
      get_logger(),
      "Denied raceline_global_handoff: safe-stop hazard is still latched and no explicit "
      "global-handoff release condition was authorized.");
    return false;
  }
  auto handoff_path = planner_.buildGlobalHandoffPath(
    ego, state_handoff_tail_distance_m_, state_handoff_speed_cap_mps_);
  if (handoff_path.wpnts.empty()) {
    return false;
  }
  if (!has_commitment_) {
    committed_result_ = RacelineSplineResult();
    committed_result_.kind = SplinePlanKind::kAvoidance;
    has_commitment_ = true;
    commitment_start_s_ = ego.s;
  }
  committed_result_.path = std::move(handoff_path);
  committed_result_.merge_s = ego.s;
  committed_result_.control_points.clear();
  resetCommitmentViolationConfirmation();
  clearSafeStopLatch();
  merge_geometry_confirmed_ = true;
  handoff_active_ = true;
  avoid_state_observed_ = avoid_state_observed_ ||
    (has_state_ && current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID);
  return true;
}

void LocalPlannerNode::latchSafeStop(
  RacelineSplineResult result,
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles)
{
  std::set<int> trigger_ids(result.obstacle_ids.begin(), result.obstacle_ids.end());
  if (result.obstacle_id >= 0) {
    trigger_ids.insert(result.obstacle_id);
  }
  publishCandidateAudit(result, "safe_stop");
  resetCommitmentViolationConfirmation();
  if (has_commitment_ && committed_result_.kind == SplinePlanKind::kAvoidance) {
    auto committed_stop = planner_.buildCommittedPathStop(
      ego, committed_result_.path, planning_obstacles);
    if (committed_stop.kind == SplinePlanKind::kSafeStop) {
      committed_stop.reason += "; trigger: " + result.reason;
      result = std::move(committed_stop);
    }
  }
  if (result.path.wpnts.empty() && !last_valid_guidance_path_.wpnts.empty()) {
    // No stop prefix exists on the requested geometry (typically the commitment was just reset,
    // or the trigger obstacle appeared inside the stop buffer). Brake along the most recent
    // vetted guidance path instead of an in-place zero-speed hold: same deceleration authority,
    // but steering continuity on a line that was collision-checked when it was published.
    const std::string trigger_reason = result.reason;
    auto last_path_stop = planner_.buildCommittedPathStop(
      ego, last_valid_guidance_path_, planning_obstacles);
    if (last_path_stop.kind == SplinePlanKind::kSafeStop &&
      !last_path_stop.path.wpnts.empty())
    {
      last_path_stop.reason += "; trigger: " + trigger_reason;
      result = std::move(last_path_stop);
    } else {
      auto braked = planner_.buildLastPathBrake(ego, last_valid_guidance_path_);
      if (!braked.wpnts.empty()) {
        result.kind = SplinePlanKind::kSafeStop;
        result.path = std::move(braked);
        result.reason =
          "no collision-free stop prefix; braking along the last valid guidance path; trigger: " +
          trigger_reason;
      }
    }
  }
  if (result.path.wpnts.empty()) {
    result.kind = SplinePlanKind::kSafeStop;
    result.path = planner_.buildEmergencyStopPath(ego);
    result.reason = "no collision-free stop prefix; publishing a zero-speed emergency hold";
  }
  safe_stop_result_ = std::move(result);
  if (!safe_stop_lifecycle_.active()) {
    SafeStopActivation activation;
    activation.obstacle_ids.assign(trigger_ids.begin(), trigger_ids.end());
    activation.obstacle_sequence = obstacles_message_sequence_;
    activation.obstacle_source_stamp_ns = latest_obstacle_source_stamp_ns_;
    activation.activation_ego_s = ego.s;
    activation.activation_timestamp_ns = eventNow().nanoseconds();
    activation.safe_stop_target_s = safe_stop_result_.path.wpnts.empty() ?
      ego.s : safe_stop_result_.path.wpnts.back().s_m;
    activation.safe_stop_target_distance_m = planner_.forwardDistance(
      ego.s, activation.safe_stop_target_s);

    double danger_start = std::numeric_limits<double>::infinity();
    double danger_end = -std::numeric_limits<double>::infinity();
    for (const auto & obstacle : planning_obstacles) {
      if (!trigger_ids.empty() && trigger_ids.count(obstacle.id) == 0U) {
        continue;
      }
      const double center_forward = planner_.forwardDistance(ego.s, obstacle.s_center);
      if (center_forward >= 0.5 * planner_.trackLength()) {
        continue;
      }
      const double half_span = 0.5 * obstacleLongitudinalSpan(
        obstacle, planner_.trackLength());
      danger_start = std::min(danger_start, std::max(0.0, center_forward - half_span));
      danger_end = std::max(danger_end, center_forward + half_span);
      if (trigger_ids.empty()) {
        activation.obstacle_ids.push_back(obstacle.id);
      }
    }
    if (!std::isfinite(danger_start) || !std::isfinite(danger_end)) {
      // This fallback is intentionally conservative. Even if an upstream error omitted an ID,
      // an empty array still cannot release the latch before ego passes the stop target and the
      // configured safety buffer.
      danger_start = activation.safe_stop_target_distance_m +
        planner_parameters_.safe_stop_buffer_m;
      danger_end = danger_start;
    }
    activation.danger_start_distance_m = danger_start;
    activation.danger_end_distance_m = danger_end;
    activation.obstacle_s_start = wrapS(
      ego.s + danger_start, planner_.trackLength());
    activation.obstacle_s_end = wrapS(
      ego.s + danger_end, planner_.trackLength());
    safe_stop_lifecycle_.activate(std::move(activation));
  }
  merge_complete_count_ = 0;
  merge_geometry_confirmed_ = false;
  handoff_active_ = false;
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Static safe-stop latched: %s", safe_stop_result_.reason.c_str());
}

bool LocalPlannerNode::resultTargetsLatchedObstacle(
  const RacelineSplineResult & result) const
{
  if (!safe_stop_lifecycle_.active()) {
    return false;
  }
  const auto & latched_ids = safe_stop_lifecycle_.activation().obstacle_ids;
  if (latched_ids.empty()) {
    return false;
  }
  const std::set<int> latched(latched_ids.begin(), latched_ids.end());
  if (result.obstacle_id >= 0 && latched.count(result.obstacle_id) > 0U) {
    return true;
  }
  return std::any_of(
    result.obstacle_ids.begin(), result.obstacle_ids.end(),
    [&latched](int id) {return latched.count(id) > 0U;});
}

bool LocalPlannerNode::explicitForwardCorridorClear(const EgoFrenetState & ego) const
{
  if (!safe_stop_lifecycle_.active() || static_obstacles_.empty() ||
    obstacles_message_sequence_ <= safe_stop_lifecycle_.activation().obstacle_sequence)
  {
    return false;
  }
  for (const auto & obstacle : static_obstacles_) {
    const double center_forward = planner_.forwardDistance(ego.s, obstacle.s_center);
    if (center_forward >= 0.5 * planner_.trackLength()) {
      continue;
    }
    const double half_span = 0.5 * obstacleLongitudinalSpan(
      obstacle, planner_.trackLength());
    const double front_distance = std::max(0.0, center_forward - half_span);
    if (front_distance <=
      planner_parameters_.detection_lookahead_m + planner_parameters_.safe_stop_buffer_m)
    {
      return false;
    }
  }
  return true;
}

SafeStopCycleDecision LocalPlannerNode::evaluateSafeStopLifecycle(
  const EgoFrenetState & ego,
  const RacelineSplineResult & replanned_result,
  const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles)
{
  (void)planning_obstacles;
  SafeStopCycleInput input;
  input.ego_s = ego.s;
  input.ego_speed_mps = ego.speed;
  input.static_obstacles_empty = static_obstacles_.empty();
  // "Targets the latched obstacle" exists so a release cannot ignore the thing that made us
  // stop. That test is only meaningful while the latched obstacle is still being detected. When
  // it is gone from the current snapshot -- the car stopped just past it and it left the FOV --
  // requiring the escape to target it makes condition B unsatisfiable, and condition A cannot
  // fire either because clearing the danger range by safe_stop_buffer_m needs forward motion the
  // latch is preventing. That combination is a PERMANENT deadlock, reproducible within ~20 s of
  // driving regardless of which candidate generator is active (sim 2026-08-15: latched on
  // obstacle 0, a valid avoidance for obstacle 1 existed every cycle, car stopped indefinitely).
  // The replanned result is hard-valid against the current geometry before it reaches here, so
  // when the latched obstacle is no longer present that validated path is the stronger evidence
  // and the identity check is just a stale bookkeeping token.
  const auto & latched_ids = safe_stop_lifecycle_.activation().obstacle_ids;
  const bool latched_obstacle_still_present = std::any_of(
    latched_ids.begin(), latched_ids.end(), [this](int id) {
      return std::any_of(
        static_obstacles_.begin(), static_obstacles_.end(),
        [id](const auto & obstacle) {return obstacle.id == id;});
    });
  input.hard_valid_avoidance_for_latched_obstacle =
    replanned_result.kind == SplinePlanKind::kAvoidance &&
    (resultTargetsLatchedObstacle(replanned_result) || !latched_obstacle_still_present);
  input.state_can_select_avoidance = has_state_ &&
    current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID;
  input.explicit_forward_corridor_clear = explicitForwardCorridorClear(ego);
  input.replanned_no_obstacle = replanned_result.kind == SplinePlanKind::kNoObstacle;
  input.obstacle_sequence = obstacles_message_sequence_;
  const double stopped_speed_threshold =
    planner_parameters_.safe_stop_deceleration_mps2 *
    static_cast<double>(planning_period_ms_) / 1000.0;
  auto decision = safe_stop_lifecycle_.evaluate(
    input, planner_.trackLength(), planner_parameters_.safe_stop_buffer_m,
    stopped_speed_threshold, safe_stop_release_cycles_);
  if (!input.replanned_no_obstacle &&
    (decision.release_reason == SafeStopReleaseReason::kObstaclePassed ||
    decision.release_reason == SafeStopReleaseReason::kStoppedCorridorClear))
  {
    decision.raceline_global_handoff_allowed = false;
  }
  publishSafeStopLifecycleAudit(input, decision);
  return decision;
}

void LocalPlannerNode::publishSafeStopLifecycleAudit(
  const SafeStopCycleInput & input,
  const SafeStopCycleDecision & decision)
{
  if (!replay_diagnostics_enable_ || !safe_stop_lifecycle_.active()) {
    return;
  }
  const auto & activation = safe_stop_lifecycle_.activation();
  std::ostringstream fields;
  fields << std::setprecision(17)
         << "\"source_stamp_ns\":" << latest_obstacle_source_stamp_ns_
         << ",\"obstacle_sequence\":" << obstacles_message_sequence_
         << ",\"safe_stop_latched\":true"
         << ",\"latched_obstacle_ids\":[";
  for (std::size_t index = 0; index < activation.obstacle_ids.size(); ++index) {
    if (index > 0U) {
      fields << ',';
    }
    fields << activation.obstacle_ids[index];
  }
  fields << "]"
         << ",\"latched_obstacle_sequence\":" << activation.obstacle_sequence
         << ",\"latched_obstacle_source_stamp_ns\":"
         << activation.obstacle_source_stamp_ns
         << ",\"latched_obstacle_s_start\":" << jsonNumber(activation.obstacle_s_start)
         << ",\"latched_obstacle_s_end\":" << jsonNumber(activation.obstacle_s_end)
         << ",\"safe_stop_target_s\":" << jsonNumber(activation.safe_stop_target_s)
         << ",\"activation_ego_s\":" << jsonNumber(activation.activation_ego_s)
         << ",\"activation_timestamp_ns\":" << activation.activation_timestamp_ns
         << ",\"ego_s\":" << jsonNumber(input.ego_s)
         << ",\"ego_speed_mps\":" << jsonNumber(input.ego_speed_mps)
         << ",\"obstacle_passed\":" << (decision.obstacle_passed ? "true" : "false")
         << ",\"static_obs_empty\":"
         << (input.static_obstacles_empty ? "true" : "false")
         << ",\"release_condition_a_passed\":"
         << (decision.release_condition_a ? "true" : "false")
         << ",\"release_condition_b_hard_valid_avoidance\":"
         << (input.hard_valid_avoidance_for_latched_obstacle ? "true" : "false")
         << ",\"release_condition_b_state_selectable\":"
         << (input.state_can_select_avoidance ? "true" : "false")
         << ",\"release_condition_b_count\":" << decision.feasible_avoidance_count
         << ",\"release_condition_b_satisfied\":"
         << (decision.release_condition_b ? "true" : "false")
         << ",\"release_condition_c_vehicle_stopped\":"
         << (decision.vehicle_stopped ? "true" : "false")
         << ",\"release_condition_c_explicit_corridor_clear\":"
         << (input.explicit_forward_corridor_clear ? "true" : "false")
         << ",\"replanned_no_obstacle\":"
         << (input.replanned_no_obstacle ? "true" : "false")
         << ",\"release_condition_c_count\":" << decision.stopped_clear_count
         << ",\"release_condition_c_satisfied\":"
         << (decision.release_condition_c ? "true" : "false")
         << ",\"release_authorized\":"
         << (decision.release_authorized ? "true" : "false")
         << ",\"release_reason\":\""
         << safeStopReleaseReasonName(decision.release_reason) << "\""
         << ",\"raceline_global_handoff_allowed\":"
         << (decision.raceline_global_handoff_allowed ? "true" : "false")
         << ",\"handoff_reason\":\""
         << (decision.raceline_global_handoff_allowed ?
  "explicit_release_condition_authorized" :
  (decision.release_reason == SafeStopReleaseReason::kValidAvoidance ?
  "validated_avoidance_selected_instead_of_global_handoff" :
  (decision.release_authorized ?
  "new_hazard_or_avoidance_prevents_global_handoff" :
  "latched_danger_region_not_safely_released"))) << "\"";
  RCLCPP_INFO(get_logger(), "SAFE_STOP_LIFECYCLE {%s}", fields.str().c_str());
  publishReplayEvent("SAFE_STOP_LIFECYCLE", fields.str());
}

void LocalPlannerNode::handleSafeStopLatch(const EgoFrenetState & ego)
{
  const auto planning_obstacles = has_commitment_ ?
    buildCurrentManeuverInput(ego) :
    buildGuardedObstacles(buildInitialStabilizationInput());
  const std::optional<bool> locked_side =
    has_commitment_ && committed_result_.kind == SplinePlanKind::kAvoidance &&
    !committed_result_.margin_pass ?
    std::optional<bool>(committed_result_.go_left) : std::nullopt;
  // 횡진입 측 잠금은 주행 중 위빙을 막는 장치다. 안전정지 래치로 차가 서 있으면 위빙
  // 위험이 없으므로 잠금을 풀어 반대측 탈출을 허용한다 — 이것이 없으면 "우측 진입 중
  // 커밋 기각 → 래치 → 우측만 재계획 허용 → P3의 유효한 좌측 탈출을 영영 못 봄"이라는
  // 영구 정지가 된다 (2026-08-15 run23 실측: locked_side=R로 kNoSafePath 반복).
  const double stopped_speed_threshold =
    planner_parameters_.safe_stop_deceleration_mps2 *
    static_cast<double>(planning_period_ms_) / 1000.0;
  const bool vehicle_stopped = std::abs(ego.speed) <= stopped_speed_threshold;
  const bool allow_side_switch =
    !locked_side.has_value() || vehicle_stopped ||
    (!commitmentSideLocked(ego) && !pre_engagement_side_switched_);
  RacelineSplineResult result = planner_.plan(
    ego, planning_obstacles, locked_side, allow_side_switch);
  const bool replanned_safe_stop = result.kind == SplinePlanKind::kSafeStop;
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 3000,
    "LATCH_REPLAN kind=%d locked_side=%s allow_switch=%d obstacles=%zu reason=%s",
    static_cast<int>(result.kind),
    locked_side.has_value() ? (locked_side.value() ? "L" : "R") : "-",
    allow_side_switch ? 1 : 0, planning_obstacles.size(), result.reason.c_str());
  if (replanned_safe_stop) {
    latchSafeStop(std::move(result), ego, planning_obstacles);
  }

  const auto & evaluated_result = replanned_safe_stop ? safe_stop_result_ : result;
  const auto decision = evaluateSafeStopLifecycle(
    ego, evaluated_result, planning_obstacles);

  if (decision.release_reason == SafeStopReleaseReason::kValidAvoidance) {
    RCLCPP_INFO(
      get_logger(),
      "Safe-stop released to a selectable hard-valid avoidance path after %d confirmations.",
      decision.feasible_avoidance_count);
    commitAvoidance(std::move(result), ego, planning_obstacles);
    publishResult(committed_result_);
    return;
  }

  if (decision.release_reason == SafeStopReleaseReason::kObstaclePassed) {
    if (!replanned_safe_stop && result.kind == SplinePlanKind::kAvoidance &&
      has_state_ && current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID)
    {
      RCLCPP_INFO(
        get_logger(),
        "Latched obstacle passed; selecting the newly validated avoidance path.");
      commitAvoidance(std::move(result), ego, planning_obstacles);
      publishResult(committed_result_);
      return;
    }
    if (!replanned_safe_stop && result.kind == SplinePlanKind::kNoObstacle &&
      activateGlobalHandoff(ego, decision.release_reason))
    {
      RCLCPP_INFO(
        get_logger(),
        "Safe-stop released after ego passed the latched danger region with margin.");
      publishResult(committed_result_);
      return;
    }
    if (replanned_safe_stop) {
      // The old danger region is behind ego, but a new hard hazard still requires a stop. Replace
      // only the lifecycle metadata; the generated braking profile remains untouched.
      auto replacement_stop = safe_stop_result_;
      clearSafeStopLatch();
      latchSafeStop(std::move(replacement_stop), ego, planning_obstacles);
      (void)evaluateSafeStopLifecycle(ego, safe_stop_result_, planning_obstacles);
    }
  }

  if (decision.release_reason == SafeStopReleaseReason::kStoppedCorridorClear &&
    !replanned_safe_stop && result.kind == SplinePlanKind::kNoObstacle &&
    activateGlobalHandoff(ego, decision.release_reason))
  {
    RCLCPP_INFO(
      get_logger(),
      "Safe-stop released after the stopped vehicle observed a persistently clear corridor.");
    publishResult(committed_result_);
    return;
  }

  publishResult(safe_stop_result_);
}

bool LocalPlannerNode::commitmentComplete(const EgoFrenetState & ego)
{
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance) {
    return false;
  }
  const double planned_distance =
    planner_.forwardDistance(commitment_start_s_, committed_result_.merge_s);
  const double driven_distance = planner_.forwardDistance(commitment_start_s_, ego.s);
  const bool reached_tail = driven_distance + 0.20 >= planned_distance &&
    driven_distance < 0.5 * planner_.trackLength();
  if (reached_tail && std::abs(ego.d) <= merge_lateral_tolerance_m_) {
    ++merge_complete_count_;
  } else {
    merge_complete_count_ = 0;
  }
  return merge_complete_count_ >= merge_confirm_cycles_;
}

void LocalPlannerNode::logObstacleCollision(
  const std::string & severity,
  const PathValidationFailure & failure,
  int confirmation_count) const
{
  const std::string confirmation = confirmation_count > 0 ?
    " confirmation=" + std::to_string(confirmation_count) + "/" +
    std::to_string(commitment_soft_violation_confirm_cycles_) :
    "";
  RCLCPP_WARN(
    get_logger(),
    "%s%s: obstacle_id=%d waypoint[%zu]=(s=%.3f,d=%.3f) "
    "obstacle_s=[%.3f,%.3f] source_d=[%.3f,%.3f] tested_d=[%.3f,%.3f] "
    "clearance=%.3f",
    severity.c_str(), confirmation.c_str(), failure.obstacle_id,
    failure.waypoint_index, failure.waypoint_s, failure.waypoint_d,
    failure.obstacle_s_start, failure.obstacle_s_end,
    failure.obstacle_source_d_right, failure.obstacle_source_d_left,
    failure.obstacle_test_d_right, failure.obstacle_test_d_left,
    failure.obstacle_clearance);
}

P3CallbackSnapshot LocalPlannerNode::captureP3CallbackSnapshot()
{
  P3CallbackSnapshot snapshot;
  ++p3_callback_sequence_;
  {
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    snapshot.has_odometry = has_odometry_;
    snapshot.odometry = latest_odometry_;
    snapshot.odometry_receipt_time = last_odometry_time_;
  }
  snapshot.frenet_source_stamp_ns = stampNs(snapshot.odometry.header.stamp);
  if (snapshot.has_odometry && last_p3_frenet_source_stamp_ns_ != 0 &&
    snapshot.frenet_source_stamp_ns < last_p3_frenet_source_stamp_ns_)
  {
    ++p3_source_epoch_;
    p3_maneuver_lifecycle_.reset();
    resetP3SelectionEnvelope();
    snapshot.source_stamp_regressed = true;
    RCLCPP_WARN(
      get_logger(),
      "P3 source epoch advanced after Frenet source-stamp regression: %" PRId64
      " -> %" PRId64 " — skipping this planning cycle (holding previous output)",
      last_p3_frenet_source_stamp_ns_, snapshot.frenet_source_stamp_ns);
  }
  if (snapshot.has_odometry) {
    last_p3_frenet_source_stamp_ns_ = snapshot.frenet_source_stamp_ns;
    snapshot.maneuver.ego.s = snapshot.odometry.pose.pose.position.x;
    snapshot.maneuver.ego.d = snapshot.odometry.pose.pose.position.y;
    snapshot.maneuver.ego.speed = std::abs(snapshot.odometry.twist.twist.linear.x);
  }
  snapshot.maneuver.obstacles = static_obstacles_;
  snapshot.maneuver.raw_obstacles = static_obstacles_;
  snapshot.maneuver.source_stamp_ns = latest_obstacle_source_stamp_ns_;
  snapshot.maneuver.source_epoch = p3_source_epoch_;
  snapshot.maneuver.global_reference_generation = global_reference_generation_;
  snapshot.maneuver.obstacle_sequence = obstacles_message_sequence_;
  snapshot.maneuver.safe_stop_authority = safe_stop_lifecycle_.active();

  const rclcpp::Time capture_time = eventNow();
  snapshot.maneuver.source_stale = has_obstacles_message_ &&
    (capture_time - last_obstacles_time_).seconds() > obstacle_stale_timeout_sec_;
  const bool odometry_stale = snapshot.has_odometry &&
    (capture_time - snapshot.odometry_receipt_time).seconds() >
    odometry_stale_timeout_sec_;
  if (!has_global_waypoints_) {
    snapshot.not_ready_reason = "GLOBAL_REFERENCE_NOT_READY";
  } else if (!snapshot.has_odometry) {
    snapshot.not_ready_reason = "FRENET_ODOMETRY_NOT_READY";
  } else if (odometry_stale) {
    snapshot.not_ready_reason = "FRENET_ODOMETRY_STALE";
  } else if (require_obstacles_message_ && !has_obstacles_message_) {
    snapshot.not_ready_reason = "OBSTACLE_SOURCE_NOT_READY";
  } else if (snapshot.maneuver.source_stale) {
    snapshot.not_ready_reason = "OBSTACLE_SOURCE_STALE";
  } else if (snapshot.maneuver.safe_stop_authority) {
    snapshot.not_ready_reason = "SAFE_STOP_AUTHORITY";
  } else {
    snapshot.ready = true;
  }
  return snapshot;
}

bool LocalPlannerNode::prepareP3InitialSelectionSnapshot(P3CallbackSnapshot & snapshot)
{
  if (!snapshot.ready || !snapshot.has_odometry || !has_global_waypoints_) {
    return snapshot.ready;
  }

  const auto merge_current_snapshot = [&]() {
      std::map<int, f110_msgs::msg::Obstacle> conservative;
      for (const auto & obstacle : snapshot.maneuver.obstacles) {
        conservative[obstacle.id] = obstacle;
      }
      for (const auto & entry : p3_selection_envelope_union_) {
        const auto current = conservative.find(entry.first);
        if (current == conservative.end()) {
          conservative[entry.first] = entry.second;
        } else {
          current->second = mergeObstacleEnvelopes(
            current->second, entry.second, planner_.trackLength());
        }
      }
      std::vector<f110_msgs::msg::Obstacle> result;
      result.reserve(conservative.size());
      for (const auto & entry : conservative) {
        result.push_back(entry.second);
      }
      return result;
    };

  // Once an immutable maneuver is active, the Guard frozen at selection is the stable authority.
  // Do not permanently union later same-ID measurements into it: the lifecycle below applies the
  // pre-P3 exact-ID contract (contained -> frozen Guard, breach -> guarded exact validation, then
  // raw exact validation). A one-frame expansion therefore cannot poison the whole maneuver, and
  // a real raw collision still invalidates immediately.
  if (p3_maneuver_lifecycle_.active()) {
    snapshot.maneuver.selection_guard_ready = true;
    // Keep the pre-ownership union as a floor for every fresh active-plan evaluation, while adding
    // this callback's live geometry only transiently. The live expansion is therefore respected
    // by candidate generation and exact validation without becoming a permanent lifetime union.
    const auto conservative = merge_current_snapshot();
    snapshot.maneuver.selection_envelope_obstacles = conservative;
    snapshot.maneuver.obstacles = buildGuardedObstacles(conservative);
    return true;
  }

  const auto current_guarded = buildGuardedObstacles(snapshot.maneuver.obstacles);
  const auto cluster_ids = planner_.blockingClusterIds(
    snapshot.maneuver.ego, current_guarded);
  if (cluster_ids.empty()) {
    resetP3SelectionEnvelope();
    snapshot.maneuver.selection_guard_ready = true;
    return true;
  }

  const auto contains_id = [&cluster_ids](int id) {
      return std::find(cluster_ids.begin(), cluster_ids.end(), id) != cluster_ids.end();
    };
  const bool restart = p3_selection_envelope_union_.empty() || std::none_of(
    p3_selection_envelope_union_.begin(), p3_selection_envelope_union_.end(),
    [&contains_id](const auto & entry) {return contains_id(entry.first);});
  if (restart) {
    resetP3SelectionEnvelope();
    p3_selection_start_ = eventNow();
  }

  const bool new_obstacle_message =
    !p3_selection_has_sequence_ ||
    p3_selection_last_sequence_ != snapshot.maneuver.obstacle_sequence;
  if (new_obstacle_message) {
    p3_selection_has_sequence_ = true;
    p3_selection_last_sequence_ = snapshot.maneuver.obstacle_sequence;
    for (const int id : cluster_ids) {
      const auto current = std::find_if(
        snapshot.maneuver.obstacles.begin(), snapshot.maneuver.obstacles.end(),
        [id](const auto & obstacle) {return obstacle.id == id;});
      if (current == snapshot.maneuver.obstacles.end()) {
        continue;
      }
      const auto previous = p3_selection_envelope_union_.find(id);
      p3_selection_envelope_union_[id] = previous == p3_selection_envelope_union_.end() ?
        *current : mergeObstacleEnvelopes(previous->second, *current, planner_.trackLength());
      ++p3_selection_observation_counts_[id];
    }
  }

  const bool observation_count_reached = std::all_of(
    cluster_ids.begin(), cluster_ids.end(),
    [this](int id) {
      const auto count = p3_selection_observation_counts_.find(id);
      return count != p3_selection_observation_counts_.end() &&
             count->second >= initial_observation_count_;
    });
  const double duration = (eventNow() - p3_selection_start_).seconds();
  snapshot.maneuver.selection_guard_ready =
    (observation_count_reached && duration >= initial_observation_min_duration_sec_) ||
    duration >= initial_observation_max_wait_sec_;

  // Always run P3/M1 on the current conservative union so SHADOW/parity diagnostics observe the
  // candidate immediately. Only lifecycle ownership waits for the already-existing production
  // Guard contract above; no new parameter, TTL, or hysteresis is introduced.
  const auto conservative = merge_current_snapshot();
  snapshot.maneuver.selection_envelope_obstacles = conservative;
  snapshot.maneuver.obstacles = buildGuardedObstacles(conservative);
  return true;
}

P3ShadowResult LocalPlannerNode::evaluateP3Snapshot(
  const P3CallbackSnapshot & snapshot,
  const std::string & p0_context) const
{
  if (!snapshot.ready) {
    P3ShadowResult result;
    result.enabled = true;
    result.snapshot_source_stamp_ns = snapshot.maneuver.source_stamp_ns;
    result.snapshot_epoch = snapshot.maneuver.source_epoch;
    result.global_reference_generation = snapshot.maneuver.global_reference_generation;
    result.p0_failure_reason = p0_context;
    result.failure_classification = snapshot.not_ready_reason;
    return result;
  }
  return planner_.evaluateP3Shadow(
    snapshot.maneuver.ego, snapshot.maneuver.obstacles,
    snapshot.maneuver.source_stamp_ns, snapshot.maneuver.source_epoch,
    snapshot.maneuver.global_reference_generation, p0_context);
}

P3ManeuverLifecycleDecision LocalPlannerNode::advanceP3Lifecycle(
  const P3CallbackSnapshot & snapshot,
  const std::function<const P3ShadowResult &()> & evaluate)
{
  if (!p3_maneuver_lifecycle_.active() &&
    (p3_maneuver_lifecycle_.state() == P3ManeuverLifecycleState::kInvalidated ||
    p3_maneuver_lifecycle_.state() == P3ManeuverLifecycleState::kComplete))
  {
    // INVALIDATED/COMPLETE are one-callback transition events, not latched hysteresis states.
    p3_maneuver_lifecycle_.reset();
  }
  if (!snapshot.ready) {
    if (p3_maneuver_lifecycle_.active()) {
      if (snapshot.maneuver.source_stale || snapshot.maneuver.safe_stop_authority) {
        return p3_maneuver_lifecycle_.continueCurrent(
          snapshot.maneuver, planner_, commitment_soft_violation_confirm_cycles_);
      }
      return p3_maneuver_lifecycle_.invalidateExternal(snapshot.not_ready_reason);
    }
    P3ManeuverLifecycleDecision decision;
    decision.state = p3_maneuver_lifecycle_.state();
    decision.reason = snapshot.not_ready_reason;
    return decision;
  }
  // Continuation-first ordering (2026-08-12): while an active recorded maneuver still
  // hard-validates against the current obstacle envelopes, keep it frozen instead of
  // re-shaping the path from every fresh M1 evaluation. continueCurrent already owns the
  // safety story: the frozen guard keeps authority only while the live envelope stays
  // contained inside it, growth falls back to raw-geometry validation with the existing
  // soft-violation confirmation count, and every other failure invalidates immediately.
  // Fresh selection runs only when there is no active maneuver, or in the same callback in
  // which continuation just invalidated (so authority does not drop to P0 for one cycle).
  if (p3_maneuver_lifecycle_.active()) {
    if (!snapshot.has_odometry || !has_global_waypoints_) {
      return p3_maneuver_lifecycle_.invalidateExternal(snapshot.not_ready_reason);
    }
    auto continued = p3_maneuver_lifecycle_.continueCurrent(
      snapshot.maneuver, planner_, commitment_soft_violation_confirm_cycles_);
    // Return BEFORE touching evaluate(): this is the branch that makes continuation-first a
    // computation saving and not merely an output priority.
    if (continued.has_output || continued.complete) {
      return continued;
    }
    const P3ShadowResult & continuation_fallback = evaluate();
    if (!(continuation_fallback.invoked && continuation_fallback.would_recover)) {
      return continued;
    }
  }
  const P3ShadowResult & evaluation = evaluate();
  if (evaluation.invoked && evaluation.would_recover) {
    return p3_maneuver_lifecycle_.selectFresh(snapshot.maneuver, evaluation, planner_);
  }
  P3ManeuverLifecycleDecision decision;
  decision.state = p3_maneuver_lifecycle_.state();
  decision.reason = evaluation.failure_classification.empty() ?
    "NO_FRESH_P3_RESULT" : evaluation.failure_classification;
  return decision;
}

RacelineSplineResult LocalPlannerNode::makeP3ActiveResult(
  const P3ManeuverLifecycleDecision & decision) const
{
  RacelineSplineResult result;
  result.kind = SplinePlanKind::kAvoidance;
  result.path = decision.output_path;
  result.go_left = decision.go_left;
  result.obstacle_ids = decision.obstacle_ids;
  result.obstacle_id = result.obstacle_ids.empty() ? -1 : result.obstacle_ids.front();
  if (!result.path.wpnts.empty()) {
    result.merge_s = result.path.wpnts.back().s_m;
    result.target_d = result.path.wpnts.front().d_m;
    for (const auto & waypoint : result.path.wpnts) {
      result.target_d = decision.go_left ?
        std::max(result.target_d, waypoint.d_m) : std::min(result.target_d, waypoint.d_m);
    }
  }
  result.reason = "P3_M1_" + decision.reason;
  return result;
}

void LocalPlannerNode::publishP3CycleDiagnostic(
  const P3CallbackSnapshot & snapshot,
  const P3ShadowResult & evaluation,
  const P3ManeuverLifecycleDecision & lifecycle,
  const std::string & path_owner,
  bool p0_backup_only)
{
  // Ownership is a state, not an event: at the 25 ms planning period an unconditional INFO here
  // was 40 lines/second of identical text, which buries the transitions that actually matter.
  // Log every real change, and throttle the steady state so a stuck condition is still visible.
  const std::string ownership_state = path_owner + "|" +
    p3ManeuverLifecycleStateName(lifecycle.state) + "|" + (p0_backup_only ? "1" : "0");
  if (ownership_state != last_logged_ownership_state_) {
    last_logged_ownership_state_ = ownership_state;
    RCLCPP_INFO(
      get_logger(),
      "P3_PATH_OWNERSHIP callback=%" PRIu64
      " mode=%s owner=%s lifecycle=%s p0_backup_only=%s",
      p3_callback_sequence_, p3RuntimeModeName(p3_mode_),
      path_owner.c_str(), p3ManeuverLifecycleStateName(lifecycle.state),
      p0_backup_only ? "true" : "false");
  } else {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "P3_PATH_OWNERSHIP callback=%" PRIu64
      " mode=%s owner=%s lifecycle=%s p0_backup_only=%s (unchanged)",
      p3_callback_sequence_, p3RuntimeModeName(p3_mode_),
      path_owner.c_str(), p3ManeuverLifecycleStateName(lifecycle.state),
      p0_backup_only ? "true" : "false");
  }
  // Building the cycle JSON walks the whole candidate trace. With no subscriber that work is
  // discarded inside the publisher, so skip it entirely rather than paying for it every callback.
  // This mirrors what local_path_pub_ already does below.
  if (p3_diagnostics_pub_ == nullptr ||
    p3_diagnostics_pub_->get_subscription_count() == 0U)
  {
    return;
  }

  std_msgs::msg::String message;
  std::ostringstream json;
  json << std::setprecision(17)
       << "{\"schema\":\"local_planning_p3_cycle/1\""
       << ",\"callback_sequence\":" << p3_callback_sequence_
       << ",\"mode\":\"" << p3RuntimeModeName(p3_mode_) << "\""
       << ",\"timestamp_ns\":" << eventNow().nanoseconds()
       << ",\"source_epoch\":" << snapshot.maneuver.source_epoch
       << ",\"source_stamp_ns\":" << snapshot.maneuver.source_stamp_ns
       << ",\"frenet_source_stamp_ns\":" << snapshot.frenet_source_stamp_ns
       << ",\"global_reference_generation\":"
       << snapshot.maneuver.global_reference_generation
       << ",\"obstacle_sequence\":" << snapshot.maneuver.obstacle_sequence
       << ",\"snapshot_ready\":" << (snapshot.ready ? "true" : "false")
       << ",\"selection_guard_ready\":"
       << (snapshot.maneuver.selection_guard_ready ? "true" : "false")
       << ",\"snapshot_rejection\":\"" << jsonEscape(snapshot.not_ready_reason) << "\""
       << ",\"ego_s\":" << jsonNumber(snapshot.maneuver.ego.s)
       << ",\"ego_d\":" << jsonNumber(snapshot.maneuver.ego.d)
       << ",\"ego_speed_mps\":" << jsonNumber(snapshot.maneuver.ego.speed)
       << ",\"obstacles\":[";
  for (std::size_t index = 0U; index < snapshot.maneuver.obstacles.size(); ++index) {
    if (index > 0U) {
      json << ',';
    }
    const auto & obstacle = snapshot.maneuver.obstacles[index];
    json << "{\"id\":" << obstacle.id
         << ",\"s_start\":" << jsonNumber(obstacle.s_start)
         << ",\"s_end\":" << jsonNumber(obstacle.s_end)
         << ",\"d_right\":" << jsonNumber(obstacle.d_right)
         << ",\"d_left\":" << jsonNumber(obstacle.d_left) << '}';
  }
  json << ']'
       << ",\"cluster_obstacle_ids\":[";
  for (std::size_t index = 0U; index < evaluation.cluster_obstacle_ids.size(); ++index) {
    if (index > 0U) {
      json << ',';
    }
    json << evaluation.cluster_obstacle_ids[index];
  }
  json << ']'
       << ",\"cluster_start_forward_m\":"
       << jsonNumber(evaluation.cluster_start_forward_m)
       << ",\"cluster_end_forward_m\":"
       << jsonNumber(evaluation.cluster_end_forward_m)
       << ",\"selected_cluster_start_forward_m\":"
       << jsonNumber(evaluation.selected_cluster_start_forward_m)
       << ",\"selected_cluster_end_forward_m\":"
       << jsonNumber(evaluation.selected_cluster_end_forward_m)
       << ",\"candidate_count\":" << evaluation.candidate_count
       << ",\"m0_candidate_count\":" << evaluation.m0_candidate_count
       << ",\"m1_candidate_count\":" << evaluation.m1_candidate_count
       << ",\"m1_invoked\":" << (evaluation.m1_invoked ? "true" : "false")
       << ",\"selected_source\":\"" << jsonEscape(evaluation.selected_source) << "\""
       << ",\"selected_source_cell\":\""
       << jsonEscape(evaluation.selected_source_cell) << "\""
       << ",\"selected_branch\":\""
       << jsonEscape(evaluation.selected_source_branch_regime) << "\""
       << ",\"selected_candidate_identity\":\""
       << jsonEscape(evaluation.selected_candidate_identity) << "\""
       << ",\"selected_path_digest\":\""
       << jsonEscape(evaluation.selected_path_digest) << "\""
       << ",\"fresh_hard_valid\":" << (evaluation.would_recover ? "true" : "false")
       << ",\"fresh_rejection\":\""
       << jsonEscape(evaluation.failure_classification) << "\""
       << ",\"lifecycle_state\":\""
       << p3ManeuverLifecycleStateName(lifecycle.state) << "\""
       << ",\"lifecycle_reason\":\"" << jsonEscape(lifecycle.reason) << "\""
       << ",\"original_candidate_identity\":\""
       << jsonEscape(lifecycle.original_candidate_identity) << "\""
       << ",\"original_path_digest\":\""
       << jsonEscape(lifecycle.original_path_digest) << "\""
       << ",\"suffix_point_count\":" << lifecycle.suffix_point_count
       << ",\"suffix_revalidated\":"
       << (lifecycle.suffix_revalidated ? "true" : "false")
       << ",\"suffix_hard_valid\":"
       << (lifecycle.suffix_hard_valid ? "true" : "false")
       << ",\"guard_contained_same_id_count\":"
       << lifecycle.guard_contained_same_id_count
       << ",\"guard_raw_revalidated\":"
       << (lifecycle.guard_raw_revalidated ? "true" : "false")
       << ",\"guard_soft_violation_pending\":"
       << (lifecycle.guard_soft_violation_pending ? "true" : "false")
       << ",\"guard_soft_violation_count\":"
       << lifecycle.guard_soft_violation_count
       << ",\"suffix_rejection\":\""
       << jsonEscape(lifecycle.validation.rejection_reason) << "\""
       << ",\"completion_handoff_available\":"
       << (lifecycle.completion_handoff_available ? "true" : "false")
       << ",\"completion_handoff_decision_path_digest\":\""
       << jsonEscape(lifecycle.completion_handoff_path_digest) << "\""
       << ",\"completion_handoff_decision_point_count\":"
       << lifecycle.completion_handoff_path.wpnts.size()
       << ",\"production_selected_path_family\":\""
       << jsonEscape(last_selected_path_family_) << "\""
       << ",\"production_selected_path_digest\":\""
       << jsonEscape(last_selected_path_digest_) << "\""
       << ",\"path_owner\":\"" << jsonEscape(path_owner) << "\""
       << ",\"p0_backup_only\":" << (p0_backup_only ? "true" : "false")
       << ",\"safe_stop_active\":"
       << (safe_stop_lifecycle_.active() ? "true" : "false")
       << ",\"shadow_lifecycle_isolated_from_p0_output_authority\":"
       << (p3_mode_ == P3RuntimeMode::kShadow ? "true" : "false")
       << ",\"runtime_total_us\":" << jsonNumber(evaluation.runtime_total_us)
       << '}';
  message.data = json.str();
  p3_diagnostics_pub_->publish(message);
}

void LocalPlannerNode::onPlanningTimer()
{
  if (p3_mode_ == P3RuntimeMode::kOff) {
    // This is the entire OFF branch. It enters the pre-integration P0 body without capturing,
    // evaluating, logging, publishing, or mutating any P3 state.
    runP0PlanningCycle();
    return;
  }

  const P3CallbackSnapshot snapshot = captureP3CallbackSnapshot();
  if (snapshot.source_stamp_regressed) {
    // 역행 샘플로는 계획하지 않는다(스냅샷 구조체 주석 참고). 상태 리셋은 capture에서 이미
    // 끝났고, 다음 콜백(25 ms 뒤)이 일관된 샘플로 신선하게 재선택한다.
    return;
  }
  if (p3_mode_ == P3RuntimeMode::kShadow) {
    current_path_owner_ = "P0";
    runP0PlanningCycle(&snapshot);
    // SHADOW evaluates the same state/obstacle/reference snapshot without allowing the P0
    // output selected in this callback (including its safe-stop latch) to suppress the
    // observational P3 lifecycle. P0 publication remains unchanged, and the diagnostic still
    // reports the actual P0 safe-stop state separately.
    P3CallbackSnapshot shadow_snapshot = snapshot;
    shadow_snapshot.maneuver.safe_stop_authority = false;
    if (shadow_snapshot.not_ready_reason == "SAFE_STOP_AUTHORITY") {
      shadow_snapshot.ready = true;
      shadow_snapshot.not_ready_reason.clear();
    }
    (void)prepareP3InitialSelectionSnapshot(shadow_snapshot);
    // SHADOW is observational: it always evaluates, so the lazy hook is satisfied eagerly here.
    const P3ShadowResult evaluation = evaluateP3Snapshot(
      shadow_snapshot, "SHADOW_PRODUCTION_" + last_selected_path_family_);
    const auto lifecycle = advanceP3Lifecycle(
      shadow_snapshot, [&evaluation]() -> const P3ShadowResult & {return evaluation;});
    publishP3CycleDiagnostic(
      shadow_snapshot, evaluation, lifecycle, "P0_SHADOW_UNCHANGED", false);
    return;
  }

  P3CallbackSnapshot active_snapshot = snapshot;
  (void)prepareP3InitialSelectionSnapshot(active_snapshot);
  // Lazy by contract: advanceP3Lifecycle pulls this only after continuation fails to produce
  // output. While a frozen suffix keeps hard-validating, no candidate is constructed and no hard
  // validation runs on this callback.
  std::optional<P3ShadowResult> evaluation_storage;
  const auto evaluate = [&]() -> const P3ShadowResult & {
      if (!evaluation_storage.has_value()) {
        evaluation_storage = evaluateP3Snapshot(active_snapshot, "TEST_ACTIVE_PRIMARY");
      }
      return *evaluation_storage;
    };
  const auto lifecycle = advanceP3Lifecycle(active_snapshot, evaluate);
  // The previous same-callback replan branch lived here. It was unreachable by construction: the
  // lifecycle only surfaces CURRENT_RAW_OBSTACLE_COLLISION when the evaluation did NOT recover,
  // and re-running the evaluator on the same immutable snapshot cannot change that verdict, so the
  // retry could never select a fresh path. When the evaluation DOES recover, advanceP3Lifecycle
  // already falls through to selectFresh inside the very same call. Do not reintroduce it.
  if (!evaluation_storage.has_value()) {
    // Continuation held without ever consulting the evaluator. Report that explicitly instead of
    // publishing a default-constructed result that would read as a solver failure.
    P3ShadowResult held;
    held.enabled = true;
    held.snapshot_source_stamp_ns = active_snapshot.maneuver.source_stamp_ns;
    held.snapshot_epoch = active_snapshot.maneuver.source_epoch;
    held.global_reference_generation = active_snapshot.maneuver.global_reference_generation;
    held.failure_classification = "EVALUATOR_NOT_INVOKED_CONTINUATION_HELD";
    evaluation_storage = std::move(held);
  }
  const P3ShadowResult & evaluation = *evaluation_storage;
  if (lifecycle.has_output && lifecycle.suffix_hard_valid) {
    // A fresh/continuing valid maneuver always has authority over an older completed tail.
    current_path_owner_ = lifecycle.fresh_selected ? "P3_M1" : "P3_COMMITTED_SUFFIX";
    publishResult(makeP3ActiveResult(lifecycle));
    publishP3CycleDiagnostic(
      active_snapshot, evaluation, lifecycle, current_path_owner_, false);
    return;
  }

  if (lifecycle.complete) {
    // Route the post-obstacle handback through the SAME closed global handoff loop the P0 flow
    // uses: anchored at the current ego pose and released by the existing STATE_GLOBAL
    // confirmation in the P0 pipeline below. The previous immutable frozen-tail handoff was
    // published untrimmed until the FSM confirmed GLOBAL, but at speed its fixed tail fell
    // behind the ego within a few cycles, the FSM's tail-reach window was missed, and the
    // stale tail kept steering the car long after the obstacle (2026-08-12 20:49 run:
    // COMPLETION_HANDOFF held ~14 s with no merge confirmation).
    //
    // Register the completed maneuver's obstacle ids across the clearCommitment() wipe, exactly
    // like the P0 completion flow does via resetForChainedManeuver. The detector's static
    // occlusion hold keeps the just-passed track published, and without this exclusion it
    // re-enters buildNextManeuverInput while ego is still inside its padded span: the chained
    // cluster then starts at ego, the stop prefix is empty, and the safe-stop latch stalls the
    // car inside its own latched danger region (both v2 scenarios, .regression_check2).
    std::set<int> completed = completed_obstacle_ids_;
    completed.insert(lifecycle.obstacle_ids.begin(), lifecycle.obstacle_ids.end());
    clearCommitment();
    resetP3SelectionEnvelope();
    completed_obstacle_ids_ = std::move(completed);
    if (activateGlobalHandoff(active_snapshot.maneuver.ego)) {
      RCLCPP_INFO(
        get_logger(),
        "P3 maneuver complete; publishing the closed global handoff loop until STATE_GLOBAL "
        "confirmation.");
    }
  }

  current_path_owner_ = "P0_BACKUP_ONLY";
  if (lifecycle.invalidated) {
    std::ostringstream invalidation;
    invalidation << std::setprecision(17)
                 << "{\"schema\":\"p3_lifecycle_invalidation_observation/1\""
                 << ",\"callback_sequence\":" << p3_callback_sequence_
                 << ",\"lifecycle_reason\":\"" << jsonEscape(lifecycle.reason) << "\""
                 << ",\"original_candidate_identity\":\""
                 << jsonEscape(lifecycle.original_candidate_identity) << "\""
                 << ",\"original_path_digest\":\""
                 << jsonEscape(lifecycle.original_path_digest) << "\""
                 << ",\"source_lineage\":{\"record_epoch\":"
                 << lifecycle.record_source_epoch
                 << ",\"current_epoch\":" << lifecycle.current_source_epoch
                 << ",\"record_reference_generation\":"
                 << lifecycle.record_reference_generation
                 << ",\"current_reference_generation\":"
                 << lifecycle.current_reference_generation
                 << ",\"record_creation_stamp_ns\":"
                 << lifecycle.record_creation_source_stamp_ns
                 << ",\"record_last_stamp_ns\":"
                 << lifecycle.record_last_source_stamp_ns
                 << ",\"current_stamp_ns\":" << lifecycle.current_source_stamp_ns
                 << ",\"record_obstacle_sequence\":"
                 << lifecycle.record_obstacle_sequence
                 << ",\"current_obstacle_sequence\":"
                 << lifecycle.current_obstacle_sequence << '}'
                 << ",\"guarded_validator\":{\"attempted\":"
                 << (lifecycle.guarded_validation_attempted ? "true" : "false")
                 << ",\"hard_valid\":"
                 << (lifecycle.guarded_validation_hard_valid ? "true" : "false")
                 << ",\"rejection\":\""
                 << jsonEscape(lifecycle.guarded_validation_rejection) << "\"}"
                 << ",\"raw_validator\":{\"attempted\":"
                 << (lifecycle.raw_validation_attempted ? "true" : "false")
                 << ",\"hard_valid\":"
                 << (lifecycle.raw_validation_hard_valid ? "true" : "false")
                 << ",\"rejection\":\""
                 << jsonEscape(lifecycle.raw_validation_rejection) << "\"}"
                 << ",\"same_id_envelopes\":[";
    const auto append_envelope = [&invalidation](
      const f110_msgs::msg::Obstacle & obstacle)
      {
        invalidation << "{\"id\":" << obstacle.id
                     << ",\"has_cartesian\":"
                     << (obstacle.has_cartesian ? "true" : "false")
                     << ",\"x_min\":" << jsonNumber(obstacle.x_min)
                     << ",\"x_max\":" << jsonNumber(obstacle.x_max)
                     << ",\"y_min\":" << jsonNumber(obstacle.y_min)
                     << ",\"y_max\":" << jsonNumber(obstacle.y_max)
                     << ",\"s_start\":" << jsonNumber(obstacle.s_start)
                     << ",\"s_end\":" << jsonNumber(obstacle.s_end)
                     << ",\"d_right\":" << jsonNumber(obstacle.d_right)
                     << ",\"d_left\":" << jsonNumber(obstacle.d_left)
                     << ",\"s_var\":" << jsonNumber(obstacle.s_var)
                     << ",\"d_var\":" << jsonNumber(obstacle.d_var) << '}';
      };
    for (std::size_t index = 0U; index < lifecycle.guard_observations.size(); ++index) {
      if (index > 0U) {
        invalidation << ',';
      }
      const auto & observation = lifecycle.guard_observations[index];
      invalidation << "{\"id\":" << observation.obstacle_id
                   << ",\"obstacle_envelope_contained\":"
                   << (observation.contained_in_frozen_guard ? "true" : "false")
                   << ",\"live\":";
      if (observation.live_present) {
        append_envelope(observation.live_envelope);
      } else {
        invalidation << "null";
      }
      invalidation << ",\"accumulated_union\":";
      if (observation.accumulated_present) {
        append_envelope(observation.accumulated_envelope);
      } else {
        invalidation << "null";
      }
      invalidation << ",\"current_guarded\":";
      if (observation.guarded_present) {
        append_envelope(observation.guarded_envelope);
      } else {
        invalidation << "null";
      }
      invalidation << ",\"frozen_guard\":";
      append_envelope(observation.frozen_guard);
      invalidation << '}';
    }
    invalidation << "]}";
    RCLCPP_WARN(
      get_logger(), "P3_LIFECYCLE_INVALIDATION %s", invalidation.str().c_str());
    resetP3SelectionEnvelope();
  }
  // Reaching the P0 path is only a P3 SHORTFALL when P3 was actually asked for a maneuver: a
  // usable snapshot AND a blocking cluster to avoid. On a clear track every callback lands here
  // by design, and counting those made the ratio meaningless -- an obstacle-free lap reported
  // "1872/1924 콜백" as if P3 had failed 97% of the time, which is exactly backwards from what
  // this counter exists to measure ("is P3 alone sufficient"). Keep the fallback behaviour
  // unchanged; only the accounting and the warning are gated.
  const bool p3_was_asked_for_a_path =
    active_snapshot.ready && evaluation.invoked && !evaluation.cluster_obstacle_ids.empty();
  if (p3_was_asked_for_a_path) {
    ++p3_backup_fallback_count_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "P3 출력 없음 → 안전정지 (누적 %" PRIu64 "/%" PRIu64 " 콜백). 이유: %s",
      p3_backup_fallback_count_, p3_callback_sequence_,
      lifecycle.reason.empty() ? evaluation.failure_classification.c_str() :
      lifecycle.reason.c_str());
  } else {
    RCLCPP_DEBUG(
      get_logger(), "P3 미요청 콜백(장애물 없음/스냅샷 미준비): %s",
      lifecycle.reason.empty() ? evaluation.failure_classification.c_str() :
      lifecycle.reason.c_str());
  }
  runP0PlanningCycle(&snapshot);
  publishP3CycleDiagnostic(
    active_snapshot, evaluation, lifecycle, "P0_BACKUP_ONLY", true);
}

void LocalPlannerNode::runP0PlanningCycle(const P3CallbackSnapshot * snapshot)
{
  nav_msgs::msg::Odometry odometry;
  rclcpp::Time odometry_time(0, 0, RCL_ROS_TIME);
  bool has_odometry = false;
  if (snapshot != nullptr) {
    odometry = snapshot->odometry;
    odometry_time = snapshot->odometry_receipt_time;
    has_odometry = snapshot->has_odometry;
  } else {
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    odometry = latest_odometry_;
    odometry_time = last_odometry_time_;
    has_odometry = has_odometry_;
  }
  if (!has_global_waypoints_ || !has_odometry) {
    publishEmpty("waiting for global race line and Frenet odometry");
    return;
  }

  EgoFrenetState ego;
  ego.s = odometry.pose.pose.position.x;
  ego.d = odometry.pose.pose.position.y;
  ego.speed = std::abs(odometry.twist.twist.linear.x);

  if ((eventNow() - odometry_time).seconds() > odometry_stale_timeout_sec_) {
    RacelineSplineResult emergency_hold;
    emergency_hold.kind = SplinePlanKind::kSafeStop;
    emergency_hold.path = planner_.buildEmergencyStopPath(ego);
    emergency_hold.reason =
      "Frenet odometry is stale; publishing a zero-speed hold at the last known pose";
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s", emergency_hold.reason.c_str());
    publishResult(emergency_hold);
    return;
  }
  if (require_obstacles_message_ && !has_obstacles_message_) {
    publishEmpty("waiting for static-obstacle perception");
    return;
  }
  if (has_obstacles_message_ &&
    (eventNow() - last_obstacles_time_).seconds() > obstacle_stale_timeout_sec_)
  {
    if (!obstacle_perception_degraded_) {
      obstacle_perception_degraded_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Static-obstacle perception is stale; retaining the committed path and last valid "
        "obstacle snapshot until fresh perception arrives.");
    }
  }

  std::vector<f110_msgs::msg::Obstacle> planning_obstacles = static_obstacles_;
  bool chained_maneuver_started = false;

  // local planner가 먼저 빈 경로를 보내면 state_machine은 merge 판단에 사용할 tail을 잃는다.
  // 이번 commitment에서 STATE_AVOID를 실제로 관측했고, spline 합류가 확인된 뒤
  // state_machine이 STATE_GLOBAL을 발행한 경우에만 non-empty 경로 발행을 종료한다.
  if (has_commitment_ && merge_geometry_confirmed_) {
    chained_maneuver_started = beginChainedManeuverIfNeeded(
      ego, planning_obstacles, "global handoff");
    if (!chained_maneuver_started) {
      if (avoid_state_observed_ && has_state_ &&
        current_state_ == f110_msgs::msg::StateMachine::STATE_GLOBAL)
      {
        RCLCPP_INFO(
          get_logger(),
          "State machine confirmed GLOBAL after all static obstacles cleared; "
          "releasing committed tail.");
        clearCommitment();
        publishEmpty("state machine confirmed global handoff");
        return;
      }
      publishResult(committed_result_);
      return;
    }
  }
  if (safe_stop_lifecycle_.active()) {
    handleSafeStopLatch(ego);
    return;
  }
  if (has_commitment_ && !merge_geometry_confirmed_) {
    std::vector<f110_msgs::msg::Obstacle> early_next_obstacles;
    if (tryEarlyChainedManeuver(ego, early_next_obstacles)) {
      publishResult(committed_result_);
      return;
    }
  }
  if (has_commitment_ && !merge_geometry_confirmed_ && commitmentComplete(ego)) {
    chained_maneuver_started = beginChainedManeuverIfNeeded(
      ego, planning_obstacles, "merge completion");
    if (chained_maneuver_started) {
      // Fall through to the ordinary initial-stabilization path. This deliberately starts the
      // next obstacle with no preferred side, while the non-empty preparation path keeps AVOID.
    } else if (!activateGlobalHandoff(ego)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to build the global handoff loop; retaining the validated avoidance tail.");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Static avoidance geometry merged; publishing a closed global handoff loop until "
        "STATE_GLOBAL confirmation.");
    }
  }
  if (has_commitment_ && merge_geometry_confirmed_) {
    publishResult(committed_result_);
    return;
  }

  if (!has_commitment_) {
    auto conservative_obstacles = buildInitialStabilizationInput();
    planning_obstacles = buildGuardedObstacles(conservative_obstacles);
    if (obstacle_perception_degraded_) {
      // No new observation can improve stabilization while perception is stale. Reuse the
      // already accepted snapshot and its uncertainty guard immediately on a later lap.
      resetInitialStabilization();
    } else {
      auto preparation = planner_.buildPreparationStop(ego, planning_obstacles);
      if (preparation.kind == SplinePlanKind::kNoObstacle) {
        // 준비감속뿐 아니라 "안정화 중 조기회피"도 non-empty 발행이며, 그것만으로
        // state_machine은 STATE_AVOID로 넘어간다. 조기회피 분기가
        // initial_prepare_published_를 false로 지우므로, 그 경우에도 핸드오프 루프로
        // 돌려주려면 직전 발행이 non-empty였는지를 함께 봐야 한다. 빈 경로로 침묵하면
        // FSM은 복귀 판정 자체를 실행하지 못해 AVOID에 영구 고정된다.
        const bool published_guidance =
          initial_prepare_published_ || last_publication_non_empty_;
        resetInitialStabilization();
        if (published_guidance && activateGlobalHandoff(ego)) {
          publishResult(committed_result_);
          return;
        }
      } else if (preparation.kind == SplinePlanKind::kNoSafePath) {
        resetInitialStabilization();
        latchSafeStop(std::move(preparation), ego, planning_obstacles);
        (void)evaluateSafeStopLifecycle(ego, safe_stop_result_, planning_obstacles);
        publishResult(safe_stop_result_);
        return;
      } else {
        const bool stable = updateInitialStabilization(
          preparation.obstacle_ids, conservative_obstacles, eventNow());
        if (!stable) {
          // 🔴 2026-08-14 실차: 안정화가 끝날 때까지 무조건 정지 경로를 내보내면, 그 사이
          // 브레이크로 장애물까지 간극이 줄어 **정지 후 탈출이 물리적으로 불가능해지는**
          // 함정에 빠진다(하니스 실측: 정지 상태 탈출은 간극 2.5 m 이상에서만 가능, 그
          // 아래는 곡률 한계로 전 후보 거부). 실제로 0814 백에서 자율 정지 11건 중 8건이
          // 이 경로로 들어가 사람이 E-stop으로 꺼내야 했고, 같은 순간을 오프라인으로
          // 재현하면 회피 후보가 36개 중 6개나 실현 가능했다.
          //
          // 그래서 안정화 중이라도 **지금까지 모은 보수적 합집합 엔벨로프**에 대해 하드
          // 검증을 통과하는 회피가 있으면 정지 대신 그것을 발행한다. 안전 성질은 유지된다:
          // 판단 근거가 단일 메시지가 아니라 누적 최악 엔벨로프이고, plan()은 하드 검증을
          // 통과한 후보만 kAvoidance로 돌려준다. 커밋은 하지 않으므로 관측이 흔들리면
          // 다음 사이클에 다시 판단하고, 회피가 성립하지 않으면 종전대로 정지 준비로 간다.
          const auto stabilizing_obstacles =
            buildGuardedObstacles(buildInitialStabilizationInput());
          if (!stabilizing_obstacles.empty()) {
            auto early_avoidance = planner_.plan(ego, stabilizing_obstacles);
            if (early_avoidance.kind == SplinePlanKind::kAvoidance) {
              initial_prepare_published_ = false;
              publishResult(early_avoidance);
              return;
            }
          }
          initial_prepare_published_ = true;
          publishResult(preparation);
          return;
        }
        // The final guard is frozen from the conservative multi-message union and its worst
        // positional variance. Subsequent same-ID observations inside it cannot move the path.
        conservative_obstacles = buildInitialStabilizationInput();
        planning_obstacles = buildGuardedObstacles(conservative_obstacles);
        resetInitialStabilization();
      }
    }
  } else {
    // Obstacles whose expanded front face starts after this maneuver's merge belong to the next
    // maneuver. They must not invalidate the current path merely because its controller tail
    // extends beyond the merge point.
    planning_obstacles = buildCurrentManeuverInput(ego);
  }

  std::string commitment_error;
  PathValidationFailure commitment_failure;
  if (has_commitment_ && committed_result_.margin_pass) {
    // A margin slow pass intentionally rides inside the inflated margin band, so the
    // margin-based validator would replace it on every callback. It remains valid while the
    // cluster is still only margin-blocking; the moment any raw envelope (plus physical
    // clearance) reaches the line, fall through to the ordinary replacement machinery, which
    // re-plans a true avoidance or escalates to a stop.
    if (!planner_.obstaclesPhysicallyBlockRaceline(ego, planning_obstacles)) {
      publishResult(committed_result_);
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "Margin slow pass invalidated: an obstacle now physically blocks the race line; "
      "replanning.");
  }
  if (has_commitment_) {
    const double collision_horizon = maneuverCollisionHorizon(ego);
    const bool commitment_valid = planner_.validatePath(
      ego, committed_result_.path, planning_obstacles,
      &commitment_error, &commitment_failure, collision_horizon);
    if (commitment_valid) {
      if (commitment_soft_violation_count_ > 0) {
        RCLCPP_INFO(
          get_logger(),
          "Soft commitment violation cleared after %d/%d confirmation cycles; "
          "keeping the frozen path.",
          commitment_soft_violation_count_, commitment_soft_violation_confirm_cycles_);
      }
      resetCommitmentViolationConfirmation();
      // The committed geometry is still safe. Rebuilding six spline candidates here only makes
      // perception jitter visible downstream and repeats all geometry/curvature work.
      publishResult(committed_result_);
      return;
    }

    if (commitment_failure.kind == PathValidationFailureKind::kObstacleCollision) {
      // Retention band (2026-08-12): the hard check validates the committed geometry against
      // the raw envelopes with only the retained reserve fraction (physical clearance intact).
      // While it passes, the commitment is held frozen indefinitely — a guard/margin-level
      // violation alone no longer replaces the path, which re-shaped an almost identical
      // geometry on every progressive envelope reveal. Replacement requires an actual
      // retention-margin violation or a non-obstacle failure.
      const auto hard_collision_obstacles = buildCurrentManeuverInput(ego, false);
      PathValidationFailure hard_failure;
      const bool hard_collision_free = planner_.validatePath(
        ego, committed_result_.path, hard_collision_obstacles,
        nullptr, &hard_failure, collision_horizon,
        planner_.commitmentRetentionReserveFraction());
      const bool hard_collision =
        !hard_collision_free &&
        hard_failure.kind == PathValidationFailureKind::kObstacleCollision;
      if (hard_collision) {
        resetCommitmentViolationConfirmation();
        logObstacleCollision("Hard commitment collision; replanning immediately", hard_failure);
      } else {
        ++commitment_soft_violation_count_;
        if (commitment_soft_violation_count_ == 1) {
          logObstacleCollision(
            "Soft commitment collision pending; holding the frozen geometry inside the "
            "retention margin",
            commitment_failure, commitment_soft_violation_count_);
        }
        if (commitment_soft_violation_count_ >=
          commitment_soft_violation_confirm_cycles_)
        {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Committed path held by the retention margin; keeping the frozen geometry.");
          resetCommitmentViolationConfirmation();
        }
        publishResult(committed_result_);
        return;
      }
    } else {
      resetCommitmentViolationConfirmation();
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Committed path needs replacement: %s", commitment_error.c_str());
  }

  const std::optional<bool> preferred_side =
    has_commitment_ && committed_result_.kind == SplinePlanKind::kAvoidance &&
    !committed_result_.margin_pass ?
    std::optional<bool>(committed_result_.go_left) : std::nullopt;
  const bool allow_side_switch =
    !preferred_side.has_value() ||
    (!commitmentSideLocked(ego) && !pre_engagement_side_switched_);
  RacelineSplineResult result = planner_.plan(
    ego, planning_obstacles, preferred_side, allow_side_switch);

  if (result.kind == SplinePlanKind::kAvoidance) {
    commitAvoidance(std::move(result), ego, planning_obstacles);
    publishResult(committed_result_);
    return;
  }

  if (has_commitment_ && result.kind == SplinePlanKind::kNoObstacle) {
    // Perception commonly drops the passed obstacle before the spline tail is reached. Keep the
    // already race-line-locked commitment until its geometric merge is complete.
    publishResult(committed_result_);
    return;
  }
  if (result.kind == SplinePlanKind::kSafeStop) {
    // 🔴 탈출 불가 정지는 반드시 드러낸다 (2026-08-14). 정지점에서도, 자차 위치까지
    // 물러난 모든 지점에서도 회피 후보가 0개면 전진 계획으로는 재출발할 수 없다.
    // 이전에는 이 상태가 아무 로그 없이 30초씩 매달려 있어 현장에서 원인을 알 수 없었다.
    if (!result.safe_stop_escape_verified) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "탈출 불가 안전정지 — 정지점(자차 +%.2f m)에서도, 더 뒤로 물려도 회피 후보가 "
        "하나도 생성되지 않는다. 전진 계획으로는 재출발 불가(후진 필요). ego s=%.2f d=%+.2f",
        result.safe_stop_forward_m, ego.s, ego.d);
    }
    latchSafeStop(std::move(result), ego, planning_obstacles);
    (void)evaluateSafeStopLifecycle(ego, safe_stop_result_, planning_obstacles);
    publishResult(safe_stop_result_);
    return;
  }
  if (has_commitment_) {
    latchSafeStop(std::move(result), ego, planning_obstacles);
    (void)evaluateSafeStopLifecycle(ego, safe_stop_result_, planning_obstacles);
    publishResult(safe_stop_result_);
    return;
  }
  if (result.kind == SplinePlanKind::kNoSafePath && result.obstacle_id >= 0) {
    latchSafeStop(std::move(result), ego, planning_obstacles);
    (void)evaluateSafeStopLifecycle(ego, safe_stop_result_, planning_obstacles);
    publishResult(safe_stop_result_);
    return;
  }
  // 🔴 STATE_AVOID일 때 빈 경로를 내보내면 FSM은 영구히 AVOID에 갇힌다. state_machine의
  // 복귀 판정(enter_to_global)은 "최신 /avoid_waypoints가 non-empty"를 전제로만 실행되고,
  // 빈 메시지에는 어떤 타임아웃도 대안 경로도 없다. 유령 장애물(검출기의 provisional
  // 객체·벽 조각)로 AVOID에 들어갔다가 그 장애물이 사라지는 흔한 경우가 정확히 여기다.
  // 트랙이 실제로 비었음이 확인된 kNoObstacle에서만, 침묵 대신 ego에 앵커된 닫힌 글로벌
  // 핸드오프 루프를 발행한다 — 그 루프는 tail이 ego에 놓이고 d=0이라 FSM의 tail/횡오차
  // 게이트를 곧바로 만족시켜 정상 경로로 GLOBAL 복귀를 확정시킨다. GLOBAL이 확인되면
  // 위쪽 handoff 릴리즈 분기가 커밋을 지우고 다시 빈 경로로 돌아간다.
  if (result.kind == SplinePlanKind::kNoObstacle && has_state_ &&
    current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID &&
    activateGlobalHandoff(ego))
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No blocking obstacle remains while /state is still AVOID; publishing the closed global "
      "handoff loop instead of an empty path so the FSM can confirm GLOBAL.");
    publishResult(committed_result_);
    return;
  }
  clearCommitment();
  publishEmpty(result.reason);
}

nav_msgs::msg::Path LocalPlannerNode::makePath(
  const std::vector<f110_msgs::msg::Wpnt> & waypoints,
  const std_msgs::msg::Header & header) const
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(waypoints.size());
  for (const auto & waypoint : waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = waypoint.x_m;
    pose.pose.position.y = waypoint.y_m;
    pose.pose.orientation = yawQuaternion(waypoint.psi_rad);
    path.poses.push_back(pose);
  }
  return path;
}

void LocalPlannerNode::publishResult(const RacelineSplineResult & result)
{
  if (p3_mode_ != P3RuntimeMode::kOff) {
    if (current_path_owner_.rfind("P3_", 0U) == 0U) {
      last_selected_path_family_ = current_path_owner_;
    } else if (result.kind == SplinePlanKind::kSafeStop) {
      last_selected_path_family_ = "P0_SAFE_STOP";
    } else if (result.kind == SplinePlanKind::kPreparation) {
      last_selected_path_family_ = "P0_PREPARATION";
    } else if (handoff_active_) {
      last_selected_path_family_ = "P0_GLOBAL_HANDOFF";
    } else {
      last_selected_path_family_ = "P0_AVOIDANCE";
    }
    last_selected_path_digest_ = pathDigest(result.path);
  }
  if (result.kind == SplinePlanKind::kAvoidance && !result.path.wpnts.empty()) {
    last_valid_guidance_path_ = result.path;
  }
  f110_msgs::msg::OTWpntArray output;
  output.header.stamp = eventNow();
  output.header.frame_id = frame_id_;
  output.wpnts = result.path.wpnts;
  const bool stop_like =
    result.kind == SplinePlanKind::kSafeStop ||
    result.kind == SplinePlanKind::kPreparation;
  output.ot_side = stop_like ?
    "stop" : (result.go_left ? "left" : "right");
  const bool p3_owned = p3_mode_ != P3RuntimeMode::kOff &&
    current_path_owner_.rfind("P3_", 0U) == 0U;
  output.ot_line = p3_owned ? "raceline_local_d_offset_spline" :
    (result.kind == SplinePlanKind::kPreparation ? "raceline_static_prepare" :
    (result.kind == SplinePlanKind::kSafeStop ? "raceline_static_safe_stop" :
    (handoff_active_ ? "raceline_global_handoff" : "raceline_local_d_offset_spline")));
  const std::optional<bool> current_side = result.kind == SplinePlanKind::kAvoidance ?
    std::optional<bool>(result.go_left) : std::nullopt;
  output.side_switch = current_side.has_value() &&
    (!last_published_side_.has_value() || current_side.value() != last_published_side_.value());
  if (output.side_switch) {
    last_side_switch_time_ = eventNow();
  }
  output.last_switch_time = last_side_switch_time_;
  last_published_side_ = current_side;
  const std::int64_t publish_steady_ns = steadyNowNs();
  last_publication_non_empty_ = !output.wpnts.empty();
  avoid_waypoints_pub_->publish(output);

  if (timing_diagnostics_enable_ && !timing_t1_published_ && !output.wpnts.empty()) {
    nav_msgs::msg::Odometry odometry;
    bool has_odometry = false;
    {
      std::lock_guard<std::mutex> lock(odometry_mutex_);
      has_odometry = has_odometry_;
      if (has_odometry) {
        odometry = latest_odometry_;
      }
    }
    std::ostringstream fields;
    fields << std::setprecision(17)
           << "\"steady_override_ns\":" << publish_steady_ns
           << ",\"path_stamp_ns\":" << stampNs(output.header.stamp)
           << ",\"waypoint_count\":" << output.wpnts.size()
           << ",\"plan_kind\":" << static_cast<int>(result.kind)
           << ",\"obstacle_id\":" << result.obstacle_id
           << ",\"committed_target_d\":" << result.target_d;
    if (has_odometry) {
      fields << ",\"ego_s\":" << odometry.pose.pose.position.x
             << ",\"ego_d\":" << odometry.pose.pose.position.y
             << ",\"speed_mps\":" << odometry.twist.twist.linear.x;
    }
    publishTimingEvent("T1_AVOID_WAYPOINTS", fields.str());
    timing_t1_published_ = true;
  }

  if (local_path_pub_->get_subscription_count() > 0U) {
    local_path_pub_->publish(makePath(result.path.wpnts, output.header));
  }
}

void LocalPlannerNode::publishTimingEvent(
  const std::string & event, const std::string & fields)
{
  if (!timing_diagnostics_enable_ || timing_diagnostics_pub_ == nullptr) {
    return;
  }
  const std::string override_key = "\"steady_override_ns\":";
  std::int64_t steady_time_ns = steadyNowNs();
  const auto override_position = fields.find(override_key);
  if (override_position != std::string::npos) {
    const auto value_begin = override_position + override_key.size();
    const auto value_end = fields.find(',', value_begin);
    try {
      steady_time_ns = std::stoll(fields.substr(value_begin, value_end - value_begin));
    } catch (const std::exception &) {
      steady_time_ns = steadyNowNs();
    }
  }
  std_msgs::msg::String message;
  std::ostringstream json;
  json << "{\"schema\":\"cma_timing_event/1\","
       << "\"event\":\"" << event << "\","
       << "\"node\":\"local_planner_node\","
       << "\"steady_time_ns\":" << steady_time_ns << ','
       << "\"ros_time_ns\":" << eventNow().nanoseconds();
  if (!fields.empty()) {
    json << ',' << fields;
  }
  json << '}';
  message.data = json.str();
  timing_diagnostics_pub_->publish(message);
}

void LocalPlannerNode::publishReplayEvent(
  const std::string & event, const std::string & fields)
{
  if (!replay_diagnostics_enable_ || replay_diagnostics_pub_ == nullptr) {
    return;
  }
  std_msgs::msg::String message;
  std::ostringstream json;
  json << "{\"schema\":\"cma_replay_planner_event/1\",\"event\":\""
       << event << "\"";
  if (!fields.empty()) {
    json << ',' << fields;
  }
  json << '}';
  message.data = json.str();
  replay_diagnostics_pub_->publish(message);
}

void LocalPlannerNode::publishCandidateAudit(
  const RacelineSplineResult & result, const std::string & decision)
{
  if (!replay_diagnostics_enable_ || result.candidate_audits.empty()) {
    return;
  }
  const auto selected = std::find_if(
    result.candidate_audits.begin(), result.candidate_audits.end(),
    [](const SplineCandidateAudit & audit) {return audit.selected;});
  const auto feasible_count = std::count_if(
    result.candidate_audits.begin(), result.candidate_audits.end(),
    [](const SplineCandidateAudit & audit) {return audit.feasible;});
  if (selected != result.candidate_audits.end()) {
    RCLCPP_INFO(
      get_logger(),
      "Candidate audit [%s]: generated=%zu feasible=%zu rank=%d side=%s target=%.6f "
      "entry_requested/effective=%.6f/%.6f exit=%.6f wall/obstacle=%.6f/%.6f "
      "footprint_wall=%.6f peak_curvature/rate=%.6f/%.6f speed_loss=%.6f min_slack=%.6f",
      decision.c_str(), result.candidate_audits.size(), feasible_count, selected->final_rank,
      selected->go_left ? "left" : "right", selected->target_d,
      selected->requested_entry_length_m, selected->effective_entry_length_m,
      selected->exit_length_m, selected->wall_clearance_m, selected->obstacle_clearance_m,
      selected->rectangular_footprint_wall_clearance_m,
      selected->peak_curvature_radpm, selected->peak_curvature_rate_radpm2,
      selected->velocity_loss, selected->minimum_normalized_safety_slack);
  } else {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Candidate audit [%s]: generated=%zu feasible=0; all hard-invalid",
      decision.c_str(), result.candidate_audits.size());
  }
  for (const auto & audit : result.candidate_audits) {
    std::ostringstream fields;
    fields << std::setprecision(17)
           << "\"decision\":\"" << jsonEscape(decision) << "\""
           << ",\"source_stamp_ns\":" << latest_obstacle_source_stamp_ns_
           << ",\"obstacle_sequence\":" << obstacles_message_sequence_
           << ",\"generation_index\":" << audit.generation_index
           << ",\"feasible\":" << (audit.feasible ? "true" : "false")
           << ",\"selected\":" << (audit.selected ? "true" : "false")
           << ",\"side\":\"" << (audit.go_left ? "left" : "right") << "\""
           << ",\"target_d\":" << jsonNumber(audit.target_d)
           << ",\"entry_fraction\":" << jsonNumber(audit.entry_fraction)
           << ",\"exit_transition_scale\":" << jsonNumber(audit.exit_transition_scale)
           << ",\"requested_entry_length_m\":"
           << jsonNumber(audit.requested_entry_length_m)
           << ",\"effective_entry_length_m\":"
           << jsonNumber(audit.effective_entry_length_m)
           << ",\"exit_length_m\":" << jsonNumber(audit.exit_length_m)
           << ",\"minimum_centerline_wall_clearance_m\":"
           << jsonNumber(audit.centerline_wall_clearance_m)
           << ",\"minimum_rectangular_footprint_wall_clearance_m\":"
           << jsonNumber(audit.rectangular_footprint_wall_clearance_m)
           << ",\"footprint_invalid\":"
           << (audit.footprint_invalid ? "true" : "false")
           << ",\"footprint_violation_side\":\""
           << jsonEscape(audit.footprint_violation_side) << "\""
           << ",\"footprint_violation_waypoint_index\":"
           << audit.footprint_violation_waypoint_index
           << ",\"footprint_violation_s_m\":"
           << jsonNumber(audit.footprint_violation_s_m)
           << ",\"footprint_violation_x_m\":"
           << jsonNumber(audit.footprint_violation_x_m)
           << ",\"footprint_violation_y_m\":"
           << jsonNumber(audit.footprint_violation_y_m)
           << ",\"footprint_violation_yaw_rad\":"
           << jsonNumber(audit.footprint_violation_yaw_rad)
           << ",\"heading_relative_to_reference_rad\":"
           << jsonNumber(audit.footprint_heading_relative_to_reference_rad)
           << ",\"wallward_corner_protrusion_m\":"
           << jsonNumber(audit.wallward_corner_protrusion_m)
           << ",\"wall_clearance_m\":" << jsonNumber(audit.wall_clearance_m)
           << ",\"obstacle_clearance_m\":" << jsonNumber(audit.obstacle_clearance_m)
           << ",\"peak_curvature_radpm\":" << jsonNumber(audit.peak_curvature_radpm)
           << ",\"peak_curvature_rate_radpm2\":"
           << jsonNumber(audit.peak_curvature_rate_radpm2)
           << ",\"velocity_loss\":" << jsonNumber(audit.velocity_loss)
           << ",\"global_path_deviation_m\":"
           << jsonNumber(audit.global_path_deviation_m)
           << ",\"minimum_normalized_safety_slack\":"
           << jsonNumber(audit.minimum_normalized_safety_slack)
           << ",\"rejection_reason\":\"" << jsonEscape(audit.rejection_reason) << "\""
           << ",\"final_rank\":" << audit.final_rank
           << ",\"exit_reaches_next_obstacle\":"
           << (audit.exit_reaches_next_obstacle ? "true" : "false")
           << ",\"rank_without_exit_demotion\":" << audit.rank_without_exit_demotion;
    // The replay event remains machine-readable on its diagnostic topic. Mirror the payload to
    // the node log as well because tuning runners may deliberately keep their rosbag topic list
    // minimal; this guarantees that every generated/rejected candidate remains post-hoc auditable
    // without changing the production path (replay diagnostics are disabled by default).
    RCLCPP_INFO(get_logger(), "PLAN_CANDIDATE {%s}", fields.str().c_str());
    publishReplayEvent("PLAN_CANDIDATE", fields.str());
  }
}

void LocalPlannerNode::publishEmpty(const std::string & reason)
{
  if (p3_mode_ != P3RuntimeMode::kOff) {
    last_selected_path_family_ = "P0_EMPTY";
    last_selected_path_digest_ = "NONE";
  }
  f110_msgs::msg::OTWpntArray output;
  output.header.stamp = eventNow();
  output.header.frame_id = frame_id_;
  output.last_switch_time = last_side_switch_time_;
  output.ot_line = reason;
  last_publication_non_empty_ = false;
  avoid_waypoints_pub_->publish(output);

  if (local_path_pub_->get_subscription_count() > 0U) {
    nav_msgs::msg::Path empty_path;
    empty_path.header = output.header;
    local_path_pub_->publish(empty_path);
  }
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason.c_str());
}

}  // namespace local_planning
