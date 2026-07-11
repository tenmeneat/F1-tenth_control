#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// ============================================================================
// teleop_dashboard_node — 조이스틱/제어 상태 대시보드 뷰어 (별도 프로세스)
// ============================================================================
// joy_teleop_monitor(Mux)가 /teleop_dashboard(std_msgs/String)로 발행하는
// 대시보드 텍스트를 받아, "이 노드 자기 터미널"에서 화면을 지우고(\033[2J\033[H)
// 렌더링한다. 화면 클리어를 Mux 밖으로 빼서, 공용 런치 터미널의 다른 노드 로그를
// 10Hz로 덮어쓰던 문제를 없앤다.
//
// 사용: 별도 터미널에서
//   ros2 run f1tenth_control teleop_dashboard_node
//   (또는 ros2 launch f1tenth_control dashboard.launch.py)
// 전제: joy_teleop_monitor가 실행 중이어야 /teleop_dashboard가 발행됨.

class TeleopDashboard : public rclcpp::Node {
public:
    TeleopDashboard() : Node("teleop_dashboard_node") {
        dashboard_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/teleop_dashboard", 10,
            std::bind(&TeleopDashboard::dashboard_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "teleop_dashboard_node 시작 — /teleop_dashboard 구독. 화면 대기 중...");
    }

private:
    void dashboard_callback(const std_msgs::msg::String::ConstSharedPtr msg) {
        // 화면 클리어 + 커서 홈 이동 후 대시보드 내용 출력 (이 터미널 전용)
        std::cout << "\033[2J\033[H" << msg->data << std::flush;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dashboard_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopDashboard>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
