#include <chrono>
#include <memory>
#include <iomanip>
#include <sstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

class JoyTeleopMonitor : public rclcpp::Node {
public:
    enum class ControlMode { MANUAL, AUTONOMOUS };
    enum class ControlAlgorithm { MAP, MPPI };

    JoyTeleopMonitor() : Node("joy_teleop_monitor") {
        // 1. 파라미터 정의 및 설정
        // 버튼/축 매핑은 실차 젯슨 f1tenth_stack의 drive_mode_manager와 일치시킨다(2026-07-17) —
        // 시뮬(이 노드)과 실차의 조이스틱 조작감을 같게 해 근육기억이 그대로 전이되게 하기 위함.
        //   A(0)=자율, B(1)=비상정지, X(2)=수동, 좌스틱 세로(axis1)=속도, 우스틱 가로(axis3)=조향.
        //   RB(5)=MAP/MPPI 전환은 drive_mode_manager가 안 쓰는 버튼이라 충돌 없이 유지.
        this->declare_parameter<double>("max_steering_angle", 0.41); // 수동 조향 풀스틱 출력 [rad] (launch에서 drive_mode_manager steering_scale=0.34로 정렬)
        this->declare_parameter<double>("max_speed", 4.0);           // 수동 속도 풀스틱 출력 [m/s] (launch에서 speed_scale=5.0로 정렬)
        this->declare_parameter<int>("steering_axis", 3);            // 우측 스틱 가로 (drive_mode_manager steering_axis)
        this->declare_parameter<int>("throttle_axis", 1);            // 좌측 스틱 세로 (drive_mode_manager speed_axis)
        this->declare_parameter<bool>("use_trigger_throttle", false);// 트리거(RT/LT) 대신 좌스틱 세로 속도 사용(실차 정렬)
        this->declare_parameter<int>("autonomous_button", 0);        // A 버튼 — AUTONOMOUS 전환(+E-stop 해제)
        this->declare_parameter<int>("emergency_button", 1);         // B 버튼 — 비상정지 Latch
        this->declare_parameter<int>("manual_button", 2);            // X 버튼 — MANUAL 전환(+E-stop 해제)
        this->declare_parameter<int>("algorithm_button", 5);         // RB 버튼 — MAP/MPPI 알고리즘 전환
        this->declare_parameter<bool>("is_simulation", false);       // 시뮬레이터 환경 모드 여부
        this->declare_parameter<bool>("force_autonomous", false);     // 조이스틱 연결 없이 자율주행 모드 즉시 기동 여부
        // 속도[m/s]→VESC ERPM 환산 게인. **표시 전용** — 실제 변환은 f1tenth_stack의
        // ackermann_to_vesc_node가 젯슨 vesc.yaml 값으로 수행한다(이 저장소 밖).
        // 대시보드 RPM이 실제와 맞으려면 그쪽 값과 같아야 한다.
        this->declare_parameter<double>("speed_to_erpm_gain", 4232.0);

        this->get_parameter("max_steering_angle", max_steering_angle_);
        this->get_parameter("max_speed", max_speed_);
        this->get_parameter("steering_axis", steering_axis_);
        this->get_parameter("throttle_axis", throttle_axis_);
        this->get_parameter("use_trigger_throttle", use_trigger_throttle_);
        this->get_parameter("autonomous_button", autonomous_button_);
        this->get_parameter("emergency_button", emergency_button_);
        this->get_parameter("manual_button", manual_button_);
        this->get_parameter("algorithm_button", algorithm_button_);
        this->get_parameter("is_simulation", is_simulation_);
        this->get_parameter("force_autonomous", force_autonomous_);
        this->get_parameter("speed_to_erpm_gain", speed_to_erpm_gain_);

        // 기본 제어 모드 설정 (시뮬레이터이면 MANUAL로 대기, 실차이면 AUTONOMOUS로 시작)
        if (force_autonomous_) {
            current_mode_ = ControlMode::AUTONOMOUS;
        } else if (is_simulation_) {
            current_mode_ = ControlMode::MANUAL;
        } else {
            current_mode_ = ControlMode::AUTONOMOUS;
        }

        // 2. 통신 및 타이머 바인딩
        // 조이스틱 토픽 구독
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyTeleopMonitor::joy_callback, this, std::placeholders::_1));

        // 자율주행 제어 토픽 구독 (MAP: control_map_node → /drive_autonomous)
        auto_drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_autonomous", 10,
            std::bind(&JoyTeleopMonitor::auto_drive_callback, this, std::placeholders::_1));

        // 자율주행 제어 토픽 구독 (MPPI: control_mppi_node → /drive_mppi)
        mppi_drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_mppi", 10,
            std::bind(&JoyTeleopMonitor::mppi_drive_callback, this, std::placeholders::_1));

        // 최종 구동 토픽 발행
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        // 현재 활성 알고리즘(MPPI 여부) 발행 — control_mppi_node가 이를 구독해 비활성일 때
        // 매 사이클 풀 연산 대신 워밍업만 유지(젯슨 CPU/GPU 절약), RB 전환 시 즉시 복귀.
        // transient_local(latched)이라 control_mppi_node가 나중에 떠도 최신 상태를 바로 수신.
        mppi_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/mppi_active", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        publish_mppi_active_state();

        // 대시보드(조이스틱/모드 상태) 텍스트 발행 — 별도 뷰어 노드(teleop_dashboard_node)가
        // /teleop_dashboard를 구독해 자기 터미널에서 렌더링. Mux는 화면을 직접 지우지 않으므로
        // 공용 런치 터미널의 다른 노드 로그를 덮어쓰지 않는다.
        dashboard_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/teleop_dashboard", 10);

        // 대시보드 텍스트 발행용 타이머 (10Hz)
        display_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JoyTeleopMonitor::display_dashboard, this));

        RCLCPP_INFO(this->get_logger(), "RoboRacer 자율/수동 토글 및 중계(Mux) 모니터가 시작되었습니다.");
        // is_simulation_이 아니라 실제로 확정된 current_mode_를 그대로 찍는다 — is_simulation_은
        // sim/real 런치 양쪽 모두 true로 고정돼 있어(의도된 기본 MANUAL 시작), 이 값 자체로
        // 메시지를 만들면 실차 구동 중에도 "SIMULATION"이 찍혀 혼동을 준다.
        RCLCPP_INFO(this->get_logger(), " - 초기 구동 모드: %s",
                    current_mode_ == ControlMode::MANUAL ? "MANUAL (조이스틱 수동 대기)" : "AUTONOMOUS (자율주행)");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        size_t required_axes = use_trigger_throttle_ ? 6 : static_cast<size_t>(std::max(steering_axis_, throttle_axis_) + 1);
        if (msg->axes.size() < required_axes ||
            msg->buttons.size() <= static_cast<size_t>(std::max({autonomous_button_, emergency_button_, manual_button_, algorithm_button_}))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "조이스틱 축(Axes) 또는 버튼 개수가 부족합니다. 컨트롤러 연결을 확인하세요.");
            return;
        }

        // 1. 모드/비상정지 전환 (A/B/X) — 실차 drive_mode_manager와 동일 시맨틱.
        //    B(비상정지)를 최우선 검사하고, 한 콜백당 하나의 전환만 반영(상승 엣지).
        //    A·X는 각각 자율/수동으로 전환하며 걸려 있던 E-stop Latch도 함께 해제한다
        //    (drive_mode_manager는 A/X로만 E-stop을 빠져나옴 — 별도 해제 버튼 없음).
        bool estop_pressed = (msg->buttons[emergency_button_] == 1);
        bool auto_pressed = (msg->buttons[autonomous_button_] == 1);
        bool manual_pressed = (msg->buttons[manual_button_] == 1);
        if (estop_pressed && !last_emergency_button_state_) {
            if (!is_emergency_stop_) {
                is_emergency_stop_ = true;
                RCLCPP_ERROR(this->get_logger(), "🚨 비상 제동 활성화 (B 버튼)! Latch 상태 유지.");
            }
        } else if (auto_pressed && !last_autonomous_button_state_) {
            is_emergency_stop_ = false;
            if (current_mode_ != ControlMode::AUTONOMOUS) current_mode_ = ControlMode::AUTONOMOUS;
            RCLCPP_INFO(this->get_logger(), "🔄 제어 권한 전환: [AUTONOMOUS] 자율주행 (A 버튼)");
        } else if (manual_pressed && !last_manual_button_state_) {
            is_emergency_stop_ = false;
            if (current_mode_ != ControlMode::MANUAL) current_mode_ = ControlMode::MANUAL;
            RCLCPP_INFO(this->get_logger(), "🔄 제어 권한 전환: [MANUAL] 수동 조작 (X 버튼)");
        }
        last_emergency_button_state_ = estop_pressed;
        last_autonomous_button_state_ = auto_pressed;
        last_manual_button_state_ = manual_pressed;

        // 1-1. 제어 알고리즘 전환 (RB 버튼: algorithm_button_ 상승 엣지 감지)
        // current_algorithm_에 따라 auto_drive_callback(/drive_autonomous=MAP)과
        // mppi_drive_callback(/drive_mppi=MPPI)이 각자 자기 차례일 때만 /drive로 포워딩한다.
        // 두 컨트롤러 노드는 항상 켜져 있으므로 전환은 즉시 반영된다.
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

        // 2. 수동 조작 조향 및 속도 계산 (우스틱 가로=조향, 좌스틱 세로=속도 — 실차 정렬)
        double steer_input = msg->axes[steering_axis_];
        target_steering_angle_ = steer_input * max_steering_angle_;
        input_steer_pct_ = steer_input * 100.0; // 조향 입력 퍼센티지 (+좌/-우)

        double current_max_speed = max_speed_;

        if (use_trigger_throttle_) {
            double rt_val = msg->axes[5];  // RT: 1.0(놓음) → -1.0(꽉 누름)
            double lt_val = msg->axes[2];  // LT: 1.0(놓음) → -1.0(꽉 누름)

            // 트리거가 처음으로 실제 눌렸는지(1.0이 아닌 값) 감지
            // 일부 드라이버에서는 입력 전에 0.0을 보내는데, 이를 "눌린 것"으로 처리하지 않음
            if (!rt_pressed_once_ && rt_val != 0.0 && rt_val != 1.0) {
                rt_pressed_once_ = true;
            }
            if (!lt_pressed_once_ && lt_val != 0.0 && lt_val != 1.0) {
                lt_pressed_once_ = true;
            }

            // 트리거 입력이 한 번도 없었으면 해당 축은 0으로 처리
            double throttle = 0.0;
            if (rt_pressed_once_) {
                // rt_val: 1.0(놓음)→0, -1.0(꽉 누름)→1.0
                throttle = std::max(0.0, (1.0 - rt_val) / 2.0);
            }

            double brake = 0.0;
            if (lt_pressed_once_) {
                brake = std::max(0.0, (1.0 - lt_val) / 2.0);
            }

            target_speed_ = (throttle - brake) * current_max_speed;
            input_throttle_pct_ = throttle * 100.0; // RT 스로틀 입력 퍼센티지
            input_brake_pct_ = brake * 100.0;       // LT 브레이크 입력 퍼센티지
        } else {
            double throttle_input = msg->axes[throttle_axis_];
            target_speed_ = throttle_input * current_max_speed;
            input_throttle_pct_ = throttle_input * 100.0;
            input_brake_pct_ = 0.0;
        }

        // 5. 비상 정지(수동 B버튼)가 트리거된 경우 제동 명령 최우선 송출
        if (is_emergency_stop_) {
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
                last_drive_speed_ = drive_msg.drive.speed;
                drive_pub_->publish(drive_msg);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "⚠️ [실물 차량 모드] 수동 조작 조종이 비활성화되어 있습니다. 수동 조종은 is_simulation이 true여야 합니다.");
                publish_brake_command();
            }
        }
    }

    void auto_drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        // MAP이 활성 알고리즘일 때만 처리(MPPI 활성 시 이 소스는 무시). 알고리즘 게이트를
        // E-stop보다 먼저 두어 비활성 소스 콜백이 브레이크를 중복 발행하지 않게 한다 —
        // 활성 소스(항상 흐름)가 E-stop 브레이크를 담당.
        if (current_algorithm_ != ControlAlgorithm::MAP) return;
        forward_autonomous(msg);
    }

    void mppi_drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        // MPPI가 활성 알고리즘일 때만 처리
        if (current_algorithm_ != ControlAlgorithm::MPPI) return;
        forward_autonomous(msg);
    }

    // 활성 자율주행 소스를 최종 /drive로 포워딩(E-stop·모드 게이트 공통 처리).
    void forward_autonomous(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr& msg) {
        // 비상 정지(수동 B버튼)가 트리거된 경우 즉각 주행 출력을 차단하고 브레이크 송출
        if (is_emergency_stop_) {
            publish_brake_command();
            return;
        }

        // AUTONOMOUS 모드일 때만 자율주행 명령을 최종 /drive 토픽으로 포워딩(Pass-through)
        if (current_mode_ == ControlMode::AUTONOMOUS) {
            auto drive_msg = *msg;
            drive_msg.header.stamp = this->now();
            last_drive_speed_ = drive_msg.drive.speed;
            drive_pub_->publish(drive_msg);
        }
    }

    // control_mppi_node에게 현재 활성 알고리즘이 MPPI인지 알려준다(latched).
    void publish_mppi_active_state() {
        auto msg = std_msgs::msg::Bool();
        msg.data = (current_algorithm_ == ControlAlgorithm::MPPI);
        mppi_active_pub_->publish(msg);
    }

    void publish_brake_command() {
        auto brake_msg = ackermann_msgs::msg::AckermannDriveStamped();
        brake_msg.header.stamp = this->now();
        brake_msg.header.frame_id = "base_link";
        brake_msg.drive.speed = 0.0;
        brake_msg.drive.acceleration = -9.0;     // 최대 감속 가속도
        brake_msg.drive.steering_angle = 0.0;     // 제동 중 스핀 방지 정렬
        last_drive_speed_ = brake_msg.drive.speed;
        drive_pub_->publish(brake_msg);
    }

    void display_dashboard() {
        // 화면 클리어(\033[2J\033[H)는 뷰어 노드(teleop_dashboard_node)가 담당한다.
        // 여기서는 대시보드 "내용"만 조립해 /teleop_dashboard(String)로 발행한다.
        std::ostringstream oss;
        oss << "=========================================================\n";
        oss << "        ROBORACER CONTROL MULTIPLEXER & TELEMETRY        \n";
        oss << "=========================================================\n";

        // 제어 모드 및 비상 상태 표시
        oss << " [System State & Config] \n";
        oss << "  * Operating Mode      : "
                  << (is_simulation_ ? "\033[1;33m[SIMULATION (시뮬레이터용)]\033[0m" : "\033[1;32m[REAL CAR (대회 실차용)]\033[0m")
                  << "\n";

        std::string mode_str = "";
        if (current_mode_ == ControlMode::MANUAL) {
            mode_str = "\033[1;32m[MANUAL (수동 조작)]\033[0m";
        } else {
            mode_str = "\033[1;36m[AUTONOMOUS (자율주행)]\033[0m";
        }
        oss << "  * Active Control Mode : " << mode_str << "\n";

        oss << "  * Joystick E-Stop     : "
                  << (is_emergency_stop_ ? "\033[1;31m[ACTIVE - BRAKE LATCHED]\033[0m" : "\033[1;32m[NORMAL]\033[0m")
                  << "\n";

        std::string algo_str = (current_algorithm_ == ControlAlgorithm::MAP)
                                    ? "\033[1;36m[MAP]\033[0m"
                                    : "\033[1;35m[MPPI]\033[0m";
        oss << "  * Active Algorithm    : " << algo_str << "\n";
        oss << "\n";

        // 명령 전송 상태 표시
        oss << " [Current Telemetry] \n";
        oss << std::fixed << std::setprecision(3);
        if (is_emergency_stop_) {
            oss << "  * Status              : \033[1;31mEMERGENCY BRAKING ACTIVE\033[0m\n";
            oss << "  * Target Speed        : 0.000 m/s (Braking)\n";
            oss << "  * Target Steering     : 0.000 rad\n";
        } else if (current_mode_ == ControlMode::MANUAL) {
            oss << "  * Target Speed (Joy)  : " << target_speed_ << " m/s\n";
            oss << "  * Target Steering(Joy): " << target_steering_angle_ << " rad\n";
        } else {
            oss << "  * Control Source      : Redirecting /drive_autonomous to /drive...\n";
        }

        // 실제 /drive에 나가는 속도(수동/자율/E-stop 어느 경로든)를 ERPM으로 환산해 표시.
        // 값 자체가 아니라 표시용 환산이라, 실제 VESC 변환(ackermann_to_vesc_node)과는 별개 계산.
        oss << "  * Commanded RPM(ERPM) : " << (last_drive_speed_ * speed_to_erpm_gain_)
                  << "  (= /drive speed " << last_drive_speed_ << " m/s x " << speed_to_erpm_gain_ << ")\n";

        // 조이스틱 원입력 퍼센티지 (모드와 무관하게 항상 표시)
        oss << "\n [Joystick Input] \n";
        oss << std::setprecision(1);
        oss << "  * Throttle (RT)       : " << input_throttle_pct_ << " %\n";
        oss << "  * Brake (LT)          : " << input_brake_pct_ << " %\n";
        oss << "  * Steering (L stick)  : " << std::showpos << input_steer_pct_ << std::noshowpos
                  << " %   (+ Left / - Right)\n";
        oss << std::setprecision(3);
        // target_speed_는 모드와 무관하게 joy_callback에서 항상 계산되므로, AUTONOMOUS 중에도
        // "지금 트리거를 그대로 쓰면 몇 m/s가 나갈지" 미리 확인 가능하도록 여기서 항상 표시한다.
        oss << "  * Commanded Speed     : " << target_speed_ << " m/s (limit " << max_speed_ << " m/s)\n";
        oss << "\n";

        // 조이스틱 버튼 상태
        oss << " [XBox Key Mapping Guides] (실차 drive_mode_manager와 정렬) \n";
        oss << "  * A Button            : AUTONOMOUS Mode (clears E-stop)\n";
        oss << "  * B Button            : Emergency Stop (Latch)\n";
        oss << "  * X Button            : MANUAL Mode (clears E-stop)\n";
        oss << "  * L-Stick Vert (ax1)  : Speed   |   R-Stick Horiz (ax3) : Steering\n";
        oss << "  * RB Button           : Toggle MAP / MPPI Algorithm\n";
        oss << "=========================================================\n";

        auto dash_msg = std_msgs::msg::String();
        dash_msg.data = oss.str();
        dashboard_pub_->publish(dash_msg);
    }

    // 설정 파라미터
    double max_steering_angle_;
    double max_speed_;
    int steering_axis_;
    int throttle_axis_;
    bool use_trigger_throttle_;
    int autonomous_button_;
    int emergency_button_;
    int manual_button_;
    int algorithm_button_;
    bool is_simulation_;
    bool force_autonomous_;
    double speed_to_erpm_gain_;

    // 제어 목표 변수
    double target_steering_angle_ = 0.0;
    double target_speed_ = 0.0;

    // 대시보드 표시용 — 실제 /drive로 나간 마지막 속도(ERPM 환산 대상)
    double last_drive_speed_ = 0.0;

    // 조이스틱 원입력 퍼센티지 (터미널 표시용)
    double input_throttle_pct_ = 0.0;
    double input_brake_pct_ = 0.0;
    double input_steer_pct_ = 0.0;

    // 상태 변수
    ControlMode current_mode_ = ControlMode::MANUAL;
    ControlAlgorithm current_algorithm_ = ControlAlgorithm::MAP;
    bool last_autonomous_button_state_ = false;
    bool last_manual_button_state_ = false;
    bool last_emergency_button_state_ = false;
    bool last_algorithm_button_state_ = false;
    bool is_emergency_stop_ = false;
    bool rt_pressed_once_ = false;
    bool lt_pressed_once_ = false;

    // ROS 2 통신 개체
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_drive_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr mppi_drive_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dashboard_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mppi_active_pub_;
    rclcpp::TimerBase::SharedPtr display_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyTeleopMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
