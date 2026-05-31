#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <iomanip>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

// 수학 상수 정의
const double PI = 3.14159265358979323846;

class JoyTeleopMonitor : public rclcpp::Node {
public:
    enum class ControlMode { MANUAL, AUTONOMOUS };

    JoyTeleopMonitor() : Node("joy_teleop_monitor") {
        // ==========================================
        // 1. 파라미터 정의 및 설정
        // ==========================================
        this->declare_parameter<double>("max_steering_angle", 0.41); // 최대 조향각 (rad, 약 23.5도)
        this->declare_parameter<double>("max_speed", 6.0);           // 최대 제어 속도 (m/s)
        this->declare_parameter<int>("steering_axis", 0);            // 좌측 스틱 가로 (기본 0)
        this->declare_parameter<int>("throttle_axis", 1);            // 좌측 스틱 세로 (기본 1)
        this->declare_parameter<bool>("use_trigger_throttle", true); // 트리거(RT/LT) 가감속 사용 여부
        this->declare_parameter<int>("emergency_button", 1);         // B 버튼 (기본 1)
        this->declare_parameter<int>("boost_button", 5);             // RB 버튼 (기본 5)
        this->declare_parameter<bool>("is_simulation", false);       // 시뮬레이터 환경 모드 여부
        this->declare_parameter<bool>("force_autonomous", false);     // 조이스틱 연결 없이 자율주행 모드 즉시 기동 여부

        this->get_parameter("max_steering_angle", max_steering_angle_);
        this->get_parameter("max_speed", max_speed_);
        this->get_parameter("steering_axis", steering_axis_);
        this->get_parameter("throttle_axis", throttle_axis_);
        this->get_parameter("use_trigger_throttle", use_trigger_throttle_);
        this->get_parameter("emergency_button", emergency_button_);
        this->get_parameter("boost_button", boost_button_);
        this->get_parameter("is_simulation", is_simulation_);
        this->get_parameter("force_autonomous", force_autonomous_);

        // 기본 제어 모드 설정 (시뮬레이터이면 MANUAL로 대기, 실차이면 AUTONOMOUS로 시작)
        if (force_autonomous_) {
            current_mode_ = ControlMode::AUTONOMOUS;
        } else if (is_simulation_) {
            current_mode_ = ControlMode::MANUAL;
        } else {
            current_mode_ = ControlMode::AUTONOMOUS;
        }

        // ==========================================
        // 2. 통신 및 타이머 바인딩
        // ==========================================
        // 조이스틱 토픽 구독
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyTeleopMonitor::joy_callback, this, std::placeholders::_1));

        // 자율주행 제어 토픽 구독
        auto_drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_autonomous", 10,
            std::bind(&JoyTeleopMonitor::auto_drive_callback, this, std::placeholders::_1));

        // AEB 활성화 여부 토픽 구독 추가
        aeb_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/aeb_active", 10,
            std::bind(&JoyTeleopMonitor::aeb_active_callback, this, std::placeholders::_1));

        // 최종 구동 토픽 발행
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        // 모니터 대시보드 출력용 타이머 (10Hz)
        display_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JoyTeleopMonitor::display_dashboard, this));

        RCLCPP_INFO(this->get_logger(), "RoboRacer 자율/수동 토글 및 중계(Mux) 모니터가 시작되었습니다.");
        RCLCPP_INFO(this->get_logger(), " - 초기 구동 모드: %s", 
                    is_simulation_ ? "SIMULATION (기본 수동 제어)" : "REAL CAR (기본 자율주행)");
    }

private:
    void aeb_active_callback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
        is_aeb_active_ = msg->data;
    }

    void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        size_t required_axes = use_trigger_throttle_ ? 6 : static_cast<size_t>(std::max(steering_axis_, throttle_axis_) + 1);
        if (msg->axes.size() < required_axes ||
            msg->buttons.size() <= static_cast<size_t>(std::max({emergency_button_, boost_button_, 4, 6, 7}))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "조이스틱 축(Axes) 또는 버튼 개수가 부족합니다. 컨트롤러 연결을 확인하세요.");
            return;
        }

        // 1. 모드 토글 전환 (LB 버튼: msg->buttons[4] 상승 엣지 감지)
        bool current_lb_state = (msg->buttons[4] == 1);
        if (current_lb_state && !last_lb_state_) {
            if (current_mode_ == ControlMode::MANUAL) {
                current_mode_ = ControlMode::AUTONOMOUS;
                RCLCPP_INFO(this->get_logger(), "🔄 제어 권한 전환: [AUTONOMOUS] 자율주행 활성화");
            } else {
                current_mode_ = ControlMode::MANUAL;
                RCLCPP_INFO(this->get_logger(), "🔄 제어 권한 전환: [MANUAL] 수동 조작 활성화");
            }
        }
        last_lb_state_ = current_lb_state;

        // 2. 토글식 비상 정지 (B 버튼: emergency_button_)
        if (msg->buttons[emergency_button_] == 1) {
            if (!is_emergency_stop_) {
                is_emergency_stop_ = true;
                RCLCPP_ERROR(this->get_logger(), "🚨 비상 제동 활성화 (B 버튼 감지)! Latch 상태 유지.");
            }
        }

        // 3. 비상 정지 수동 해제 (X 버튼: msg->buttons[2] 입력)
        if (msg->buttons[2] == 1) {
            if (is_emergency_stop_) {
                is_emergency_stop_ = false;
                current_mode_ = is_simulation_ ? ControlMode::MANUAL : ControlMode::AUTONOMOUS;
                RCLCPP_INFO(this->get_logger(), "🔄 비상 제동 해제. 시스템이 [%s] 모드로 안전 복귀합니다.", 
                            is_simulation_ ? "MANUAL" : "AUTONOMOUS");
            }
        }

        // 4. 수동 조작 조향 및 속도 계산
        double steer_input = msg->axes[steering_axis_];
        target_steering_angle_ = steer_input * max_steering_angle_;

        double current_max_speed = max_speed_;
        if (msg->buttons[boost_button_] == 1) {
            current_max_speed *= 1.5;
            is_boost_active_ = true;
        } else {
            is_boost_active_ = false;
        }

        if (use_trigger_throttle_) {
            double rt_val = msg->axes[5];
            double lt_val = msg->axes[2];

            double throttle = 0.0;
            if (rt_val != 0.0 || rt_pressed_once_) {
                rt_pressed_once_ = true;
                throttle = (1.0 - rt_val) / 2.0;
            }

            double brake = 0.0;
            if (lt_val != 0.0 || lt_pressed_once_) {
                lt_pressed_once_ = true;
                brake = (1.0 - lt_val) / 2.0;
            }

            target_speed_ = (throttle - brake) * current_max_speed;
        } else {
            double throttle_input = msg->axes[throttle_axis_];
            target_speed_ = throttle_input * current_max_speed;
        }

        raw_axes_ = msg->axes;
        raw_buttons_ = msg->buttons;

        // 5. 비상 정지 또는 Lidar AEB가 트리거된 경우 제동 명령 최우선 송출
        if (is_emergency_stop_ || is_aeb_active_) {
            publish_brake_command();
            return;
        }

        // 6. MANUAL 모드인 경우 조이스틱 명령 퍼블리시
        // 단, is_simulation 파라미터가 true일 때만 수동 명령 송출을 실시간 포워딩합니다.
        if (current_mode_ == ControlMode::MANUAL) {
            if (is_simulation_) {
                auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
                drive_msg.header.stamp = this->now();
                drive_msg.header.frame_id = "base_link";
                drive_msg.drive.steering_angle = target_steering_angle_;
                drive_msg.drive.speed = target_speed_;
                drive_pub_->publish(drive_msg);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "⚠️ [실물 차량 모드] 수동 조작 조종이 비활성화되어 있습니다. 수동 조종은 is_simulation이 true여야 합니다.");
                publish_brake_command();
            }
        }
    }

    void auto_drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        // 비상 정지 또는 Lidar AEB가 트리거된 경우 즉각 주행 출력을 차단하고 브레이크 송출
        if (is_emergency_stop_ || is_aeb_active_) {
            publish_brake_command();
            return;
        }

        // AUTONOMOUS 모드일 때만 자율주행 명령을 최종 /drive 토픽으로 포워딩(Pass-through)
        if (current_mode_ == ControlMode::AUTONOMOUS) {
            auto drive_msg = *msg;
            drive_msg.header.stamp = this->now();
            drive_pub_->publish(drive_msg);
        }
    }

    void publish_brake_command() {
        auto brake_msg = ackermann_msgs::msg::AckermannDriveStamped();
        brake_msg.header.stamp = this->now();
        brake_msg.header.frame_id = "base_link";
        brake_msg.drive.speed = 0.0;
        brake_msg.drive.acceleration = -9.0;     // 최대 감속 가속도
        brake_msg.drive.steering_angle = 0.0;     // 제동 중 스핀 방지 정렬
        drive_pub_->publish(brake_msg);
    }

    void display_dashboard() {
        std::cout << "\033[2J\033[H";
        std::cout << "=========================================================\n";
        std::cout << "        ROBORACER CONTROL MULTIPLEXER & TELEMETRY        \n";
        std::cout << "=========================================================\n";
        
        // 제어 모드 및 비상 상태 표시
        std::cout << " [System State & Config] \n";
        std::cout << "  * Operating Mode      : " 
                  << (is_simulation_ ? "\033[1;33m[SIMULATION (시뮬레이터용)]\033[0m" : "\033[1;32m[REAL CAR (대회 실차용)]\033[0m") 
                  << "\n";
        
        std::string mode_str = "";
        if (current_mode_ == ControlMode::MANUAL) {
            mode_str = "\033[1;32m[MANUAL (수동 조작)]\033[0m";
        } else {
            mode_str = "\033[1;36m[AUTONOMOUS (자율주행)]\033[0m";
        }
        std::cout << "  * Active Control Mode : " << mode_str << "\n";
        
        std::cout << "  * Joystick E-Stop     : " 
                  << (is_emergency_stop_ ? "\033[1;31m[ACTIVE - BRAKE LATCHED]\033[0m" : "\033[1;32m[NORMAL]\033[0m") 
                  << "\n";
                  
        std::cout << "  * Lidar AEB State     : " 
                  << (is_aeb_active_ ? "\033[1;31m[TRIGGERED - BRAKING]\033[0m" : "\033[1;32m[SAFE]\033[0m") 
                  << "\n";

        std::cout << "  * Boost Mode (RB)     : " << (is_boost_active_ ? "\033[1;33m[BOOST ON]\033[0m" : "[OFF]") << "\n";
        std::cout << "\n";

        // 명령 전송 상태 표시
        std::cout << " [Current Telemetry] \n";
        std::cout << std::fixed << std::setprecision(3);
        if (is_emergency_stop_ || is_aeb_active_) {
            std::cout << "  * Status              : \033[1;31mEMERGENCY BRAKING ACTIVE\033[0m\n";
            std::cout << "  * Target Speed        : 0.000 m/s (Braking)\n";
            std::cout << "  * Target Steering     : 0.000 rad\n";
        } else if (current_mode_ == ControlMode::MANUAL) {
            std::cout << "  * Target Speed (Joy)  : " << target_speed_ << " m/s\n";
            std::cout << "  * Target Steering(Joy): " << target_steering_angle_ << " rad\n";
        } else {
            std::cout << "  * Control Source      : Redirecting /drive_autonomous to /drive...\n";
        }
        std::cout << "\n";

        // 조이스틱 버튼 상태
        std::cout << " [XBox Key Mapping Guides] \n";
        std::cout << "  * LB Button           : Toggle AUTO / MANUAL Mode\n";
        std::cout << "  * B Button            : Emergency Stop (Latch)\n";
        std::cout << "  * X Button            : Reset Emergency Stop Latch\n";
        std::cout << "=========================================================\n";
        std::flush(std::cout);
    }

    // 설정 파라미터
    double max_steering_angle_;
    double max_speed_;
    int steering_axis_;
    int throttle_axis_;
    bool use_trigger_throttle_;
    int emergency_button_;
    int boost_button_;
    bool is_simulation_;
    bool force_autonomous_;

    // 제어 목표 변수
    double target_steering_angle_ = 0.0;
    double target_speed_ = 0.0;
    
    // 상태 변수
    ControlMode current_mode_ = ControlMode::MANUAL;
    bool last_lb_state_ = false;
    bool is_emergency_stop_ = false;
    bool is_boost_active_ = false;
    bool rt_pressed_once_ = false;
    bool lt_pressed_once_ = false;
    bool is_aeb_active_ = false;

    // 조이스틱 상태 캐싱
    std::vector<float> raw_axes_;
    std::vector<int> raw_buttons_;

    // ROS 2 통신 개체
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_drive_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aeb_active_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr display_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyTeleopMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
