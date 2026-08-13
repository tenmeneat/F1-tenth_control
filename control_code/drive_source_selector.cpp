#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

// drive_source_selector — 자율 명령 포워더 (sim/real 공용)
//   구독 /drive_autonomous → 재스탬프 → 발행 /drive
//   E-stop·수동 전환은 f1tenth_stack(drive_mode_manager + ackermann_mux)이 담당한다.

class DriveSourceSelector : public rclcpp::Node {
public:
    DriveSourceSelector() : Node("drive_source_selector") {
        auto_drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_autonomous", 10,
            std::bind(&DriveSourceSelector::auto_drive_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        RCLCPP_INFO(this->get_logger(),
            "drive_source_selector 시작 — /drive_autonomous를 /drive로 포워딩합니다.");
    }

private:
    void auto_drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        auto drive_msg = *msg;
        drive_msg.header.stamp = this->now();
        drive_pub_->publish(drive_msg);
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_drive_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveSourceSelector>());
    rclcpp::shutdown();
    return 0;
}
