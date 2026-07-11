#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

// ============================================================================
// sim_imu_bridge_node — 시뮬레이터용 odom → IMU 변환 노드 (시뮬 전용 유틸리티)
// ============================================================================
// f1tenth_gym(gym_bridge)은 /imu/data를 발행하지 않지만 odom.twist.twist.angular.z에
// 요레이트가 실려 나온다. control_map_node의 use_imu 경로(요레이트 카운터스티어 등)를
// 시뮬에서도 실제 데이터로 동작시키기 위해, odom 요레이트를 sensor_msgs/Imu로 중계한다.
// ⚠️ 2D 물리 시뮬이라 롤 각도/롤레이트는 항상 0으로 발행(orientation identity,
// angular_velocity.x=0) — 롤 인지 ESC는 시뮬에서 항상 비활성(정상, 실차 전용 검증 항목).
// 종가속도(linear_acceleration)도 0 고정 — 조향 스케일러의 acc_mean 경로는 시뮬에서 중립.
// use_imu=false(IMU 없다고 가정)로 두면 이 노드를 안 띄워도 무해하다.

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

        // orientation identity → roll=0 (2D 시뮬 한계, 실차에서는 실제 IMU가 대체)
        imu_msg.orientation.w = 1.0;

        // 요레이트만 실측 중계 (control_map_node의 imu_callback이 angular_velocity.z를 사용)
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
