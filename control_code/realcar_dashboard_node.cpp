#include <iostream>
#include <iomanip>
#include <sstream>
#include <memory>
#include <cstdlib>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

// ============================================================================
// realcar_dashboard_node — 실차(젯슨) 원격 모니터링 대시보드 (우리 컴에서 실행)
// ============================================================================
// 젯슨은 자기 원시 토픽만 내보내고, 이 노드가 그것들을 구독해 "자기 터미널"에서
// 조립·렌더링한다 → 젯슨 렌더 연산 0. teleop_dashboard_node(시뮬용, 완성된
// /teleop_dashboard 문자열을 그대로 그리는 뷰어)와 달리, 실차엔 그 문자열을 만드는
// joy_teleop_monitor가 없으므로(시뮬 전용) 이 노드가 원시 토픽에서 직접 대시보드를
// 구성한다.
//
// 우리 컴↔젯슨은 같은 ROS_DOMAIN_ID + DDS 디스커버리(무선은 유니캐스트 프로파일 필요,
// dashboard.launch.py mode:=real 참고)로 연결돼야 토픽이 넘어온다.
//
// 구독(실차 발행원):
//   /drive_mode   (std_msgs/String)                — drive_mode_manager: estop/manual/autonomous
//   /mppi_active  (std_msgs/Bool, transient_local)  — drive_source_selector: MAP/MPPI
//   /drive        (ackermann_msgs/AckermannDriveStamped) — 최종 VESC 명령
//   <odom_topic>  (nav_msgs/Odometry, 기본 /pf/pose/odom) — 실측 속도
//   /joy          (sensor_msgs/Joy)                 — 버튼/축
// 발행: 없음(표시 전용). 원격 wifi 뷰라 각 토픽의 마지막 수신 경과(age)도 함께 표시.
// ============================================================================

class RealcarDashboard : public rclcpp::Node {
public:
    RealcarDashboard() : Node("realcar_dashboard_node") {
        this->declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
        std::string odom_topic = this->get_parameter("odom_topic").as_string();

        never_ = this->now();  // "한 번도 못 받음" 판정 기준(각 stamp 초기값)
        last_drive_mode_t_ = last_mppi_t_ = last_drive_t_ = last_odom_t_ = last_joy_t_ = never_;

        drive_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/drive_mode", 10,
            [this](const std_msgs::msg::String::ConstSharedPtr m) {
                drive_mode_ = m->data; got_drive_mode_ = true; last_drive_mode_t_ = this->now();
            });

        // /mppi_active는 drive_source_selector가 latched(transient_local+reliable)로 발행 →
        // 구독도 동일 QoS라야 늦게 떠도 최신값을 받는다.
        mppi_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mppi_active", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            [this](const std_msgs::msg::Bool::ConstSharedPtr m) {
                mppi_active_ = m->data; got_mppi_ = true; last_mppi_t_ = this->now();
            });

        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10,
            [this](const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr m) {
                drive_speed_ = m->drive.speed; drive_steer_ = m->drive.steering_angle;
                drive_accel_ = m->drive.acceleration; got_drive_ = true; last_drive_t_ = this->now();
            });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr m) {
                double vx = m->twist.twist.linear.x, vy = m->twist.twist.linear.y;
                odom_speed_ = std::sqrt(vx * vx + vy * vy); got_odom_ = true; last_odom_t_ = this->now();
            });

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr m) {
                joy_ = *m; got_joy_ = true; last_joy_t_ = this->now();
            });

        odom_topic_ = odom_topic;
        // 10Hz 렌더(원본 대시보드와 동일). 데이터가 안 와도 화면은 갱신되어 "연결 끊김"이 보인다.
        render_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&RealcarDashboard::render, this));

        RCLCPP_INFO(this->get_logger(),
            "realcar_dashboard_node 시작 — 젯슨 원시 토픽 구독(odom=%s). 화면 대기 중...",
            odom_topic_.c_str());
    }

private:
    // 마지막 수신 이후 경과 표시. 신선(<0.5s)이면 초록, 오래(>1.5s)면 빨강.
    std::string age(bool ever, const rclcpp::Time& t) {
        if (!ever) return "\033[1;31m --  \033[0m";
        double a = (this->now() - t).seconds();
        const char* col = (a < 0.5) ? "\033[1;32m" : (a < 1.5 ? "\033[1;33m" : "\033[1;31m");
        std::ostringstream o;
        o << col << std::fixed << std::setprecision(1) << a << "s\033[0m";
        return o.str();
    }

    std::string btn(int idx, const char* name) {
        std::ostringstream o;
        bool pressed = (static_cast<int>(joy_.buttons.size()) > idx && joy_.buttons[idx] == 1);
        o << name << ":" << (pressed ? "\033[1;32m1\033[0m" : "0");
        return o.str();
    }

    void render() {
        std::ostringstream oss;
        oss << "=========================================================\n";
        oss << "     ROBORACER REAL-CAR DASHBOARD  (remote @ laptop)     \n";
        oss << "=========================================================\n";
        const char* dom = std::getenv("ROS_DOMAIN_ID");
        oss << " [Link] ROS_DOMAIN_ID=" << (dom ? dom : "0")
            << "   (age 색: 초록=신선 노랑=지연 빨강=끊김/미수신)\n\n";

        // 모드 (drive_mode_manager)
        std::string mode_col = "[N/A]";
        if (got_drive_mode_) {
            if (drive_mode_ == "autonomous") mode_col = "\033[1;36m[AUTONOMOUS]\033[0m";
            else if (drive_mode_ == "manual") mode_col = "\033[1;32m[MANUAL]\033[0m";
            else if (drive_mode_ == "estop") mode_col = "\033[1;31m[ESTOP - BRAKE]\033[0m";
            else mode_col = "[" + drive_mode_ + "]";
        }
        oss << " [Drive Mode Manager]  Mode : " << mode_col
            << "   (/drive_mode " << age(got_drive_mode_, last_drive_mode_t_) << ")\n";

        // 알고리즘 (drive_source_selector)
        std::string algo = got_mppi_ ? (mppi_active_ ? "\033[1;35m[MPPI]\033[0m" : "\033[1;36m[MAP]\033[0m")
                                     : "[N/A]";
        oss << " [Algorithm]           Algo : " << algo
            << "   (/mppi_active " << age(got_mppi_, last_mppi_t_) << ")\n";

        // 최종 /drive
        oss << std::fixed << std::setprecision(3);
        oss << " [Final /drive->VESC]  ";
        if (got_drive_)
            oss << "Speed " << drive_speed_ << " m/s | Steer " << drive_steer_
                << " rad | Accel " << drive_accel_;
        else
            oss << "\033[1;31m(수신 없음)\033[0m";
        oss << "   (" << age(got_drive_, last_drive_t_) << ")\n";

        // 실측 속도 (odom)
        oss << " [State]               Odom : ";
        if (got_odom_) oss << odom_speed_ << " m/s";
        else oss << "\033[1;31m(수신 없음)\033[0m";
        oss << "   (" << odom_topic_ << " " << age(got_odom_, last_odom_t_) << ")\n";

        // 조이스틱 (실차 매핑: A=0, B=1, X=2, RB=5 / 속도축1, 조향축3)
        oss << " [Joystick /joy]       ";
        if (got_joy_) {
            oss << btn(0, "A") << " " << btn(1, "B") << " " << btn(2, "X") << " " << btn(5, "RB");
            double sp = (joy_.axes.size() > 1) ? joy_.axes[1] : 0.0;
            double st = (joy_.axes.size() > 3) ? joy_.axes[3] : 0.0;
            oss << std::setprecision(2) << "  speed_ax " << std::showpos << sp
                << "  steer_ax " << st << std::noshowpos;
        } else {
            oss << "\033[1;31m(수신 없음)\033[0m";
        }
        oss << "   (" << age(got_joy_, last_joy_t_) << ")\n";
        oss << "=========================================================\n";

        std::cout << "\033[2J\033[H" << oss.str() << std::flush;
    }

    // 수신 데이터
    std::string drive_mode_, odom_topic_;
    bool mppi_active_ = false;
    double drive_speed_ = 0.0, drive_steer_ = 0.0, drive_accel_ = 0.0, odom_speed_ = 0.0;
    sensor_msgs::msg::Joy joy_;

    bool got_drive_mode_ = false, got_mppi_ = false, got_drive_ = false, got_odom_ = false, got_joy_ = false;
    rclcpp::Time never_, last_drive_mode_t_, last_mppi_t_, last_drive_t_, last_odom_t_, last_joy_t_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mppi_active_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr render_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealcarDashboard>());
    rclcpp::shutdown();
    return 0;
}
