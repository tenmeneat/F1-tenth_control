#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "f110_msgs/msg/gap_data.hpp"
#include "f110_msgs/msg/obstacle.hpp"
#include "f110_msgs/msg/obstacle_array.hpp"
#include "f110_msgs/msg/state_machine.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "f1tenth_control/cruise_controller.hpp"

namespace f1tenth_control
{

class CruiseControllerNode : public rclcpp::Node
{
public:
  CruiseControllerNode()
  : Node("cruise_controller_node"), controller_(loadControllerConfig())
  {
    opponent_topic_ = declare_parameter<std::string>("opponent_topic", "/opp_obs");
    ego_frenet_topic_ =
      declare_parameter<std::string>("ego_frenet_topic", "/car_state/frenet/odom");
    global_waypoints_topic_ =
      declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
    speed_limit_topic_ =
      declare_parameter<std::string>("speed_limit_topic", "/cruise_speed_limit");
    gap_data_topic_ = declare_parameter<std::string>("gap_data_topic", "/cruise/gap_data");
    state_topic_ = declare_parameter<std::string>("state_topic", "/state");

    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 50.0));
    maximum_speed_ = std::max(0.0, get_parameter("maximum_speed").as_double());
    trailing_mode_distance_ =
      declare_parameter<bool>("trailing_mode_distance", true);
    trailing_gap_ = std::max(0.0, declare_parameter<double>("trailing_gap", 5.0));
    minimum_gap_ = std::max(0.0, declare_parameter<double>("minimum_gap", 0.8));
    max_desired_gap_ = std::max(0.0, declare_parameter<double>("max_desired_gap", 0.0));
    ego_front_offset_ = std::max(0.0, declare_parameter<double>("ego_front_offset", 0.25));
    opponent_timeout_ = std::max(0.01, declare_parameter<double>("opponent_timeout", 0.15));
    ego_timeout_ = std::max(0.01, declare_parameter<double>("ego_timeout", 0.20));
    state_timeout_ = std::max(0.01, declare_parameter<double>("state_timeout", 0.30));
    clear_confirm_sec_ =
      std::max(opponent_timeout_, declare_parameter<double>("clear_confirm_sec", 0.50));
    blind_trailing_speed_ =
      std::clamp(declare_parameter<double>("blind_trailing_speed", 1.5), 0.0, maximum_speed_);

    const auto latched_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_sub_ = create_subscription<f110_msgs::msg::StateMachine>(
      state_topic_, latched_qos,
      std::bind(&CruiseControllerNode::onState, this, std::placeholders::_1));
    opponent_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
      opponent_topic_, 10,
      std::bind(&CruiseControllerNode::onOpponent, this, std::placeholders::_1));
    ego_frenet_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ego_frenet_topic_, 10,
      std::bind(&CruiseControllerNode::onEgoFrenet, this, std::placeholders::_1));
    global_waypoints_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
      global_waypoints_topic_, latched_qos,
      std::bind(&CruiseControllerNode::onGlobalWaypoints, this, std::placeholders::_1));

    speed_limit_pub_ = create_publisher<std_msgs::msg::Float64>(speed_limit_topic_, 10);
    gap_data_pub_ = create_publisher<f110_msgs::msg::GapData>(gap_data_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(period, std::bind(&CruiseControllerNode::onTimer, this));
    last_cycle_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Cruise controller started: %s + %s -> %s (gap %.2f %s, blind %.2f m/s)",
      opponent_topic_.c_str(), ego_frenet_topic_.c_str(), speed_limit_topic_.c_str(),
      trailing_gap_, trailing_mode_distance_ ? "m" : "s", blind_trailing_speed_);
  }

private:
  CruiseControllerConfig loadControllerConfig()
  {
    CruiseControllerConfig config;
    config.maximum_speed =
      std::max(0.0, declare_parameter<double>("maximum_speed", 12.0));
    config.emergency_stop_distance =
      std::max(0.0, declare_parameter<double>("emergency_stop_distance", 0.45));
    config.relative_deceleration =
      std::max(0.01, declare_parameter<double>("relative_deceleration", 2.5));
    config.proportional_gain = declare_parameter<double>("trailing_p_gain", 1.0);
    config.integral_gain = declare_parameter<double>("trailing_i_gain", 0.0);
    config.derivative_gain = declare_parameter<double>("trailing_d_gain", 0.2);
    config.integral_limit =
      std::max(0.0, declare_parameter<double>("integral_limit", 2.0));
    config.uncertainty_sigma =
      std::max(0.0, declare_parameter<double>("uncertainty_sigma", 2.0));
    config.allow_acceleration = declare_parameter<bool>("allow_accel_trailing", true);
    config.gap_uncertainty_horizon_max =
      std::max(0.0, declare_parameter<double>("gap_uncertainty_horizon_max", 0.0));
    config.opp_speed_confidence_z =
      std::max(0.0, declare_parameter<double>("opp_speed_confidence_z", 0.0));
    // 0.0 (default) = fall back to relative_deceleration / no latency = old braking cap exactly.
    config.ego_deceleration = declare_parameter<double>("ego_deceleration", 0.0);
    config.opponent_deceleration = declare_parameter<double>("opponent_deceleration", 0.0);
    config.actuation_latency =
      std::max(0.0, declare_parameter<double>("actuation_latency", 0.0));
    return config;
  }

  static const char * constraintName(uint8_t constraint)
  {
    switch (constraint) {
      case CRUISE_CONSTRAINT_BRAKING: return "braking";
      case CRUISE_CONSTRAINT_EMERGENCY: return "emergency";
      case CRUISE_CONSTRAINT_MAX_SPEED: return "max_speed";
      case CRUISE_CONSTRAINT_NO_ACCEL: return "no_accel";
      default: return "pid";
    }
  }

  static double forwardDelta(double target_s, double source_s, double track_length)
  {
    if (!(track_length > 0.0)) {
      return target_s - source_s;
    }
    double delta = std::fmod(target_s - source_s, track_length);
    if (delta < 0.0) {
      delta += track_length;
    }
    return delta;
  }

  void onGlobalWaypoints(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    if (msg->wpnts.size() < 2) {
      return;
    }
    const auto & first = msg->wpnts.front();
    const auto & last = msg->wpnts.back();
    const double closing_distance = std::hypot(last.x_m - first.x_m, last.y_m - first.y_m);
    const double length = last.s_m - first.s_m + closing_distance;
    if (std::isfinite(length) && length > 1.0) {
      track_length_ = length;
    }
  }

  void onEgoFrenet(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double s = msg->pose.pose.position.x;
    const double vs = msg->twist.twist.linear.x;
    if (!std::isfinite(s) || !std::isfinite(vs)) {
      return;
    }
    ego_s_ = s;
    ego_vs_ = vs;
    ego_time_ = now();
    ego_seen_ = true;
  }

  void onOpponent(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    if (msg->obstacles.empty()) {
      if (opponent_active_ && !empty_seen_) {
        empty_since_ = now();
        empty_seen_ = true;
      }
      return;
    }

    const auto & obstacle = msg->obstacles.front();
    const bool valid =
      !obstacle.is_static && std::isfinite(obstacle.s_center) &&
      std::isfinite(obstacle.s_start) && std::isfinite(obstacle.s_end) &&
      std::isfinite(obstacle.vs) && std::isfinite(obstacle.s_var) &&
      std::isfinite(obstacle.vs_var) && std::isfinite(obstacle.s_vs_cov);
    if (!valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring invalid/static obstacle received on %s", opponent_topic_.c_str());
      return;
    }

    if (!opponent_active_ || obstacle.id != opponent_.id) {
      controller_.reset();
      RCLCPP_INFO(
        get_logger(), "Cruise target acquired: id=%d, s=%.2f, vs=%.2f",
        obstacle.id, obstacle.s_center, obstacle.vs);
    }
    opponent_ = obstacle;
    opponent_time_ = now();
    opponent_active_ = true;
    empty_seen_ = false;
  }

  void onState(const f110_msgs::msg::StateMachine::SharedPtr msg)
  {
    const bool was_cruise = state_seen_ &&
      state_ == f110_msgs::msg::StateMachine::STATE_CRUISE;
    state_ = msg->state;
    state_time_ = now();
    state_seen_ = true;
    const bool is_cruise = state_ == f110_msgs::msg::StateMachine::STATE_CRUISE;
    if (was_cruise != is_cruise) {
      controller_.reset();
    }
  }

  void publishLimit(double speed_limit)
  {
    std_msgs::msg::Float64 msg;
    msg.data = std::clamp(speed_limit, 0.0, maximum_speed_);
    speed_limit_pub_->publish(msg);
  }

  void onTimer()
  {
    const auto current_time = now();
    double dt = (current_time - last_cycle_time_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0) {
      dt = 1.0 / publish_rate_hz_;
    }
    last_cycle_time_ = current_time;

    if (!state_seen_ || state_ != f110_msgs::msg::StateMachine::STATE_CRUISE) {
      publishLimit(maximum_speed_);
      return;
    }
    if ((current_time - state_time_).seconds() > state_timeout_) {
      publishLimit(blind_trailing_speed_);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "STATE_CRUISE heartbeat stale: blind cap %.2f m/s", blind_trailing_speed_);
      return;
    }

    if (opponent_active_ && empty_seen_ &&
      (current_time - empty_since_).seconds() >= clear_confirm_sec_)
    {
      RCLCPP_INFO(get_logger(), "Cruise target cleared after %.2f s empty confirmation",
          clear_confirm_sec_);
      opponent_active_ = false;
      empty_seen_ = false;
      controller_.reset();
    }

    if (!opponent_active_) {
      // STATE_CRUISE is itself evidence that the state machine selected an opponent. If this
      // process has not acquired that target yet (startup ordering, dropped first sample, or a
      // detector failure), treating it as a clear road is fail-open. Stay at the blind cap until
      // either a valid target arrives or the state machine leaves CRUISE.
      publishLimit(blind_trailing_speed_);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "STATE_CRUISE without an acquired opponent: blind cap %.2f m/s",
        blind_trailing_speed_);
      return;
    }

    const bool opponent_fresh =
      (current_time - opponent_time_).seconds() <= opponent_timeout_;
    const bool ego_fresh = ego_seen_ && (current_time - ego_time_).seconds() <= ego_timeout_;
    if (!opponent_fresh || !ego_fresh || !track_length_.has_value()) {
      publishLimit(blind_trailing_speed_);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Active cruise input stale/missing (opp=%s ego=%s track=%s): blind cap %.2f m/s",
        opponent_fresh ? "ok" : "stale", ego_fresh ? "ok" : "stale",
        track_length_.has_value() ? "ok" : "missing", blind_trailing_speed_);
      return;
    }

    const double length = track_length_.value();
    const double center_ahead = forwardDelta(opponent_.s_center, ego_s_, length);
    double longitudinal_span = forwardDelta(opponent_.s_end, opponent_.s_start, length);
    if (!std::isfinite(longitudinal_span) || longitudinal_span > 0.5 * length) {
      longitudinal_span = 0.0;
    }
    const double raw_gap = std::max(
      0.0, center_ahead - 0.5 * longitudinal_span - ego_front_offset_);
    const double desired_gap = desiredGap(
      trailing_mode_distance_, trailing_gap_, minimum_gap_, ego_vs_, max_desired_gap_);

    CruiseControllerInput input;
    input.gap = raw_gap;
    input.desired_gap = desired_gap;
    input.ego_speed = ego_vs_;
    input.opponent_speed = opponent_.vs;
    input.opponent_s_variance = opponent_.s_var;
    input.opponent_vs_variance = opponent_.vs_var;
    input.opponent_s_vs_cov = opponent_.s_vs_cov;
    input.dt = dt;
    const auto output = controller_.update(input);
    publishLimit(output.speed_limit);

    f110_msgs::msg::GapData gap;
    gap.header.stamp = current_time;
    gap.header.frame_id = "frenet";
    gap.gap_diff = output.gap_error;
    gap.vs_diff = output.relative_speed;
    gap.gap_int = output.gap_integral;
    gap.active_constraint = output.active_constraint;
    gap.raw_gap = output.raw_gap;
    gap.effective_gap = output.effective_gap;
    gap.desired_gap = output.desired_gap;
    gap.sigma_gap = output.sigma_gap;
    gap.horizon_tau = output.horizon_tau;
    gap.ego_speed = ego_vs_;
    gap.opponent_speed = opponent_.vs;
    gap.feedback_speed = output.feedback_speed;
    gap.braking_speed = output.braking_speed;
    gap.speed_limit = output.speed_limit;
    gap_data_pub_->publish(gap);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "CRUISE id=%d gap=%.2f/%.2f m ego=%.2f opp=%.2f limit=%.2f m/s by=%s visible=%s",
      opponent_.id, output.effective_gap, desired_gap, ego_vs_, opponent_.vs,
      output.speed_limit, constraintName(output.active_constraint),
      opponent_.is_visible ? "yes" : "predicted");
  }

  CruiseLongitudinalController controller_;

  std::string opponent_topic_;
  std::string ego_frenet_topic_;
  std::string global_waypoints_topic_;
  std::string speed_limit_topic_;
  std::string gap_data_topic_;
  std::string state_topic_;
  double publish_rate_hz_{50.0};
  double maximum_speed_{12.0};
  bool trailing_mode_distance_{true};
  double trailing_gap_{5.0};
  double minimum_gap_{0.8};
  double max_desired_gap_{0.0};
  double ego_front_offset_{0.25};
  double opponent_timeout_{0.15};
  double ego_timeout_{0.20};
  double state_timeout_{0.30};
  double clear_confirm_sec_{0.50};
  double blind_trailing_speed_{1.5};

  std::optional<double> track_length_;
  f110_msgs::msg::Obstacle opponent_;
  bool opponent_active_{false};
  bool empty_seen_{false};
  bool ego_seen_{false};
  bool state_seen_{false};
  uint8_t state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};
  double ego_s_{0.0};
  double ego_vs_{0.0};
  rclcpp::Time opponent_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ego_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time empty_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cycle_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr opponent_sub_;
  rclcpp::Subscription<f110_msgs::msg::StateMachine>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_frenet_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_waypoints_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_limit_pub_;
  rclcpp::Publisher<f110_msgs::msg::GapData>::SharedPtr gap_data_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace f1tenth_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<f1tenth_control::CruiseControllerNode>());
  rclcpp::shutdown();
  return 0;
}
