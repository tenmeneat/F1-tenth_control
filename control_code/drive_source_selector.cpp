#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

// ============================================================================
// drive_source_selector — 실차 전용 MAP/MPPI 알고리즘 셀렉터 (슬림 Mux)
// ============================================================================
// 실차는 조이스틱 수동/자율/E-stop Mux를 팀 공용 f1tenth_stack의 drive_mode_manager +
// ackermann_mux가 담당한다(우리 joy_teleop_monitor는 실차 런치에서 제외됨). 하지만
// 그 스택엔 MAP/MPPI 개념이 없어서(자율 입력은 mux의 navigation 채널 'drive' 하나뿐),
// 두 컨트롤러(control_map_node→/drive_autonomous, control_mppi_node→/drive_mppi) 중
// 하나를 골라 그 navigation 채널로 흘려보내는 역할이 필요하다. 이 노드가 딱 그것만 한다.
//
// joy_teleop_monitor에서 알고리즘 선택(RB) 부분만 떼어낸 것 — 수동 조종/E-stop/대시보드는
// 없다. E-stop은 drive_mode_manager가 estop_lock으로 mux 입력 전체를 마스킹하므로, 이
// 노드가 계속 /drive를 발행해도 제동 중엔 mux에서 차단된다(별도 처리 불필요).
//
// 구독: /joy(RB 토글), /drive_autonomous(MAP), /drive_mppi(MPPI)
// 발행: /drive(= ackermann_mux navigation 채널, 우선순위 10 — 자율모드에서 teleop 침묵 시 통과),
//       /mppi_active(latched — control_mppi_node가 이걸 보고 활성/워밍업 전환)
// ============================================================================

class DriveSourceSelector : public rclcpp::Node {
public:
    enum class ControlAlgorithm { MAP, MPPI };

    DriveSourceSelector() : Node("drive_source_selector") {
        // RB 버튼(기본 5) — drive_mode_manager가 안 쓰는 버튼이라 충돌 없음.
        this->declare_parameter<int>("algorithm_button", 5);
        this->get_parameter("algorithm_button", algorithm_button_);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&DriveSourceSelector::joy_callback, this, std::placeholders::_1));

        // MAP: control_map_node → /drive_autonomous
        auto_drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_autonomous", 10,
            std::bind(&DriveSourceSelector::auto_drive_callback, this, std::placeholders::_1));

        // MPPI: control_mppi_node → /drive_mppi
        mppi_drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_mppi", 10,
            std::bind(&DriveSourceSelector::mppi_drive_callback, this, std::placeholders::_1));

        // 최종 자율 구동 명령 → ackermann_mux navigation 채널('drive')
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        // 현재 활성 알고리즘(MPPI 여부) — latched. control_mppi_node가 비활성일 때 워밍업만
        // 유지(젯슨 연산 절약), RB 전환 시 즉시 복귀. 나중에 떠도 최신 상태를 바로 수신.
        mppi_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/mppi_active", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        publish_mppi_active_state();

        RCLCPP_INFO(this->get_logger(),
            "drive_source_selector 시작 — 기본 알고리즘 [MAP], RB(%d) 버튼으로 MAP↔MPPI 전환.",
            algorithm_button_);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        if (msg->buttons.size() <= static_cast<size_t>(algorithm_button_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "조이스틱 버튼 개수가 부족합니다(RB 미검출). 컨트롤러 연결을 확인하세요.");
            return;
        }

        // RB 상승 엣지 감지 → MAP/MPPI 토글
        bool current_algorithm_button_state = (msg->buttons[algorithm_button_] == 1);
        if (current_algorithm_button_state && !last_algorithm_button_state_) {
            if (current_algorithm_ == ControlAlgorithm::MAP) {
                current_algorithm_ = ControlAlgorithm::MPPI;
                RCLCPP_INFO(this->get_logger(), "🔀 제어 알고리즘 전환: [MPPI]");
            } else {
                current_algorithm_ = ControlAlgorithm::MAP;
                RCLCPP_INFO(this->get_logger(), "🔀 제어 알고리즘 전환: [MAP]");
            }
            publish_mppi_active_state();
        }
        last_algorithm_button_state_ = current_algorithm_button_state;
    }

    void auto_drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        // MAP이 활성일 때만 포워딩(MPPI 활성 시 이 소스 무시).
        if (current_algorithm_ != ControlAlgorithm::MAP) return;
        forward(msg);
    }

    void mppi_drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        if (current_algorithm_ != ControlAlgorithm::MPPI) return;
        forward(msg);
    }

    // 활성 자율 소스를 그대로 /drive(navigation 채널)로 포워딩(재스탬프만).
    // E-stop/수동 게이트는 여기서 하지 않는다 — drive_mode_manager + ackermann_mux 담당.
    void forward(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr& msg) {
        auto drive_msg = *msg;
        drive_msg.header.stamp = this->now();
        drive_pub_->publish(drive_msg);
    }

    void publish_mppi_active_state() {
        auto msg = std_msgs::msg::Bool();
        msg.data = (current_algorithm_ == ControlAlgorithm::MPPI);
        mppi_active_pub_->publish(msg);
    }

    int algorithm_button_;
    ControlAlgorithm current_algorithm_ = ControlAlgorithm::MAP;
    bool last_algorithm_button_state_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_drive_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr mppi_drive_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mppi_active_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveSourceSelector>());
    rclcpp::shutdown();
    return 0;
}
