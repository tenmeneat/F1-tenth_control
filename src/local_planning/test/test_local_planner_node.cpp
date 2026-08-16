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

// Node-level tests: these exercise LocalPlannerNode through its real ROS interfaces, which is the
// only way to cover the subscription callbacks themselves. The algorithm classes have their own
// unit tests; what is verified here is the wiring those tests cannot see -- which incoming
// messages are accepted as authoritative and which are refused.
//
// Observables come from the P3 cycle diagnostic, which reports the accepted snapshot's identity:
//   obstacle_sequence            advances only when an obstacle array is accepted
//   source_stamp_ns              the accepted array's stamp
//   global_reference_generation  advances only when a NEW reference is adopted

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "local_planning/local_planner_node.hpp"

namespace
{

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    // These node-level tests publish on the same well-known topics as the other
    // package's node test, and colcon runs packages in parallel. Without an isolated
    // domain they cross-talk (a foreign /global_waypoints made this suite flaky).
    setenv("ROS_DOMAIN_ID", "91", 1);
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {rclcpp::shutdown();}
};

::testing::Environment * const kRclcppEnvironment =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

f110_msgs::msg::WpntArray ringReference(double d_left = 3.0, double d_right = 3.0)
{
  f110_msgs::msg::WpntArray reference;
  reference.header.frame_id = "map";
  constexpr int kCount = 240;
  constexpr double kRadius = 12.0;
  const double spacing = 2.0 * M_PI * kRadius / static_cast<double>(kCount);
  for (int i = 0; i < kCount; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kCount);
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = i;
    waypoint.s_m = static_cast<double>(i) * spacing;
    waypoint.x_m = kRadius * std::cos(angle);
    waypoint.y_m = kRadius * std::sin(angle);
    waypoint.psi_rad = angle + 0.5 * M_PI;
    waypoint.kappa_radpm = 1.0 / kRadius;
    waypoint.vx_mps = 3.0;
    waypoint.d_left = d_left;
    waypoint.d_right = d_right;
    reference.wpnts.push_back(waypoint);
  }
  return reference;
}

f110_msgs::msg::Obstacle validObstacle(int id, double s_center)
{
  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = id;
  obstacle.s_center = s_center;
  obstacle.s_start = s_center - 0.2;
  obstacle.s_end = s_center + 0.2;
  obstacle.d_right = -0.2;
  obstacle.d_left = 0.2;
  obstacle.d_center = 0.0;
  obstacle.size = 0.4;
  obstacle.is_static = true;
  return obstacle;
}

// Non-finite Frenet bounds are exactly what validFrenetObstacle refuses.
f110_msgs::msg::Obstacle invalidObstacle(int id)
{
  auto obstacle = validObstacle(id, 8.0);
  obstacle.d_left = std::numeric_limits<double>::quiet_NaN();
  obstacle.d_right = std::numeric_limits<double>::quiet_NaN();
  return obstacle;
}

double jsonField(const std::string & json, const std::string & key)
{
  const std::string needle = "\"" + key + "\":";
  const auto position = json.find(needle);
  if (position == std::string::npos) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::strtod(json.c_str() + position + needle.size(), nullptr);
}

class LocalPlannerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"planning_period_ms", 10},
        // Keep the staleness guards out of the way: these tests are about which messages are
        // accepted, not about how long an accepted one survives.
        {"obstacle_stale_timeout_sec", 30.0},
        {"odometry_stale_timeout_sec", 30.0},
        {"p3_mode", std::string("TEST_ACTIVE")},
      });
    planner_ = std::make_shared<local_planning::LocalPlannerNode>(options);
    helper_ = std::make_shared<rclcpp::Node>("local_planner_node_test_helper");

    const auto global_qos = rclcpp::QoS(1).reliable().transient_local();
    const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    waypoints_pub_ =
      helper_->create_publisher<f110_msgs::msg::WpntArray>("/global_waypoints", global_qos);
    obstacles_pub_ = helper_->create_publisher<f110_msgs::msg::ObstacleArray>(
      "/confirmed_static_obs", volatile_qos);
    odometry_pub_ =
      helper_->create_publisher<nav_msgs::msg::Odometry>("/car_state/frenet/odom", volatile_qos);
    diagnostics_sub_ = helper_->create_subscription<std_msgs::msg::String>(
      "/local_planning/p3_shadow", rclcpp::QoS(1000).reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) {last_diagnostic_ = message->data;});

    executor_.add_node(planner_);
    executor_.add_node(helper_);
  }

  void TearDown() override
  {
    executor_.remove_node(helper_);
    executor_.remove_node(planner_);
  }

  void spin(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(std::chrono::milliseconds(2));
    }
  }

  void publishOdometry(double s)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = helper_->now();
    odometry.header.frame_id = "map";
    odometry.pose.pose.position.x = s;
    odometry.pose.pose.position.y = 0.0;
    odometry.twist.twist.linear.x = 2.0;
    odometry_pub_->publish(odometry);
  }

  void publishObstacles(
    const std::vector<f110_msgs::msg::Obstacle> & obstacles, std::int32_t stamp_sec)
  {
    f110_msgs::msg::ObstacleArray array;
    array.header.frame_id = "map";
    array.header.stamp.sec = stamp_sec;
    array.obstacles = obstacles;
    obstacles_pub_->publish(array);
  }

  // Drives the node until the diagnostic reports a snapshot carrying this obstacle stamp, so the
  // assertions below never race the planning timer.
  bool waitForAcceptedStamp(std::int32_t stamp_sec, std::chrono::milliseconds timeout)
  {
    const double expected = static_cast<double>(stamp_sec) * 1.0e9;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(std::chrono::milliseconds(2));
      if (!last_diagnostic_.empty() &&
        std::abs(jsonField(last_diagnostic_, "source_stamp_ns") - expected) < 1.0)
      {
        return true;
      }
    }
    return false;
  }

  std::shared_ptr<local_planning::LocalPlannerNode> planner_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diagnostics_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::string last_diagnostic_;
};

// An array whose every entry fails the Frenet validity check is degraded perception. Accepting it
// would store an empty snapshot indistinguishable from the explicitly-empty array that IS allowed
// to erase the retained obstacle memory.
TEST_F(LocalPlannerNodeTest, AllInvalidObstacleArrayRetainsThePreviousSnapshot)
{
  waypoints_pub_->publish(ringReference());
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(150));

  publishObstacles({validObstacle(11, 8.0)}, 5);
  ASSERT_TRUE(waitForAcceptedStamp(5, std::chrono::milliseconds(1500)))
    << "the valid array was never accepted; diagnostic=" << last_diagnostic_;
  const double accepted_sequence = jsonField(last_diagnostic_, "obstacle_sequence");
  ASSERT_TRUE(std::isfinite(accepted_sequence));

  publishObstacles({invalidObstacle(12), invalidObstacle(13)}, 9);
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(300));

  EXPECT_NEAR(jsonField(last_diagnostic_, "source_stamp_ns"), 5.0e9, 1.0)
    << "an all-invalid array advanced the accepted obstacle stamp";
  EXPECT_DOUBLE_EQ(jsonField(last_diagnostic_, "obstacle_sequence"), accepted_sequence)
    << "an all-invalid array advanced the accepted obstacle sequence";
}

// The contrast case: an explicitly empty array is a valid statement that the track is clear and
// must still replace the retained snapshot.
TEST_F(LocalPlannerNodeTest, ExplicitlyEmptyObstacleArrayIsStillAccepted)
{
  waypoints_pub_->publish(ringReference());
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(150));

  publishObstacles({validObstacle(21, 8.0)}, 5);
  ASSERT_TRUE(waitForAcceptedStamp(5, std::chrono::milliseconds(1500)))
    << "the valid array was never accepted; diagnostic=" << last_diagnostic_;

  publishObstacles({}, 9);
  EXPECT_TRUE(waitForAcceptedStamp(9, std::chrono::milliseconds(1500)))
    << "an explicitly empty array was refused; diagnostic=" << last_diagnostic_;
}

// obstacle_detector rebuilds its CLCS when d_left/d_right change. If the planner compared only
// s/x/y the two nodes would run on different track widths after a boundary-only recalibration.
TEST_F(LocalPlannerNodeTest, BoundaryOnlyReferenceChangeIsAdoptedAsNewReference)
{
  waypoints_pub_->publish(ringReference(3.0, 3.0));
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(300));
  ASSERT_FALSE(last_diagnostic_.empty());
  const double first_generation = jsonField(last_diagnostic_, "global_reference_generation");
  ASSERT_TRUE(std::isfinite(first_generation));

  // Same centreline geometry, different track widths: s/x/y are byte-identical.
  waypoints_pub_->publish(ringReference(1.4, 1.4));
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(400));

  EXPECT_GT(jsonField(last_diagnostic_, "global_reference_generation"), first_generation)
    << "a d_left/d_right-only change was treated as the same reference";
}

// The other half of the same contract: an identical retransmission must NOT churn the reference,
// because adopting it would clear the commitment and the P3 lifecycle every time the global
// planner republishes.
TEST_F(LocalPlannerNodeTest, IdenticalReferenceRetransmissionIsIgnored)
{
  waypoints_pub_->publish(ringReference(3.0, 3.0));
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(300));
  ASSERT_FALSE(last_diagnostic_.empty());
  const double first_generation = jsonField(last_diagnostic_, "global_reference_generation");

  waypoints_pub_->publish(ringReference(3.0, 3.0));
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(400));

  EXPECT_DOUBLE_EQ(
    jsonField(last_diagnostic_, "global_reference_generation"), first_generation)
    << "an identical reference retransmission was adopted as a new reference";
}

}  // namespace
