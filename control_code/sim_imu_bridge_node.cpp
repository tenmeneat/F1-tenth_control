#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

// sim_imu_bridge_node — 시뮬 전용 odom → IMU 중계
//   gym_bridge가 /imu/data를 안 내보내므로 odom의 요레이트만 sensor_msgs/Imu로 옮긴다.
//   orientation은 identity, linear_acceleration은 0 고정(2D 시뮬).
//   ⚠️ 실차 런치에는 넣지 말 것 — 실제 VESC IMU와 토픽이 충돌한다.

class SimImuBridgeNode : public rclcpp::Node {
public:
    SimImuBridgeNode() : Node("sim_imu_bridge_node") {
        this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&SimImuBridgeNode::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "sim_imu_bridge_node 시작 — %s(요레이트) → %s 중계 (roll=0 고정, 시뮬 전용)",
            odom_topic.c_str(), imu_topic.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";

        imu_msg.orientation.w = 1.0;
        imu_msg.angular_velocity.z = msg->twist.twist.angular.z;

        imu_pub_->publish(imu_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimImuBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
