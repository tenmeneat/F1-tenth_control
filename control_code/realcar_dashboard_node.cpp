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
#include "nav_msgs/msg/odometry.hpp"

// ============================================================================
// realcar_dashboard_node — 실차(젯슨) 원격 모니터링 대시보드 (우리 컴에서 실행)
// ============================================================================
// 젯슨은 자기 원시 토픽만 내보내고, 이 노드가 그것들을 구독해 "자기 터미널"에서
// 조립·렌더링한다 → 젯슨 렌더 연산 0. teleop_dashboard_node(시뮬용, 완성된
// /teleop_dashboard 문자열을 그대로 그리는 뷰어)와 달리, 실차엔 그 문자열을 만드는
// joy_teleop_monitor가 없으므로(시뮬 전용) 이 노드가 원시 토픽에서 직접 대시보드를 만든다.
//
// 우리 컴↔젯슨은 같은 ROS_DOMAIN_ID + DDS 디스커버리(무선은 Fast DDS Discovery Server,
// dashboard.launch.py mode:=real 참고)로 연결돼야 토픽이 넘어온다.
//
// 표시 항목(2026-07-18):
//   - E-Stop on/off + 주행 모드 (/drive_mode) : drive_mode_manager estop/manual/autonomous
//   - 주행 알고리즘 (/mppi_active)             : MAP/MPPI
//   - 스로틀/조향 % (/joy 축)                  : 운전자 입력(좌스틱 세로/우스틱 가로)
//   - 현재 속도 (odom twist.linear.x)
//   - ERPM = 속도 × speed_to_erpm_gain          : 환산치(실 VESC 피드백 sensors/core는
//              vesc_msgs 의존이라 우리 컴 빌드에 없음 → 표시 전용 환산)
//   - 종가속도 = d(vx)/dt (odom 미분, EMA 평활)
//   - 횡가속도 = vx × yaw_rate (odom angular.z) : 원심가속, SI 단위 안전
//   ※ 가속도를 IMU에서 직접 뽑을 수도 있으나 VESC IMU 축/단위 미확정이라 odom 파생으로 둔다.
// 발행: 없음(표시 전용). 원격 wifi 뷰라 각 토픽의 마지막 수신 경과(age)도 함께 표시.
// ============================================================================

class RealcarDashboard : public rclcpp::Node {
public:
    RealcarDashboard() : Node("realcar_dashboard_node") {
        this->declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
        // 속도[m/s]→VESC ERPM 환산 게인 (ackermann_to_vesc_node의 speed_to_erpm_gain과 동일값).
        this->declare_parameter<double>("speed_to_erpm_gain", 4614.0);
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        speed_to_erpm_gain_ = this->get_parameter("speed_to_erpm_gain").as_double();

        never_ = this->now();  // "한 번도 못 받음" 판정 기준(각 stamp 초기값)
        last_drive_mode_t_ = last_mppi_t_ = last_odom_t_ = last_joy_t_ = never_;

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

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&RealcarDashboard::odom_callback, this, std::placeholders::_1));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr m) {
                joy_ = *m; got_joy_ = true; last_joy_t_ = this->now();
            });

        // 10Hz 렌더(원본 대시보드와 동일). 데이터가 안 와도 화면은 갱신되어 "연결 끊김"이 보인다.
        render_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&RealcarDashboard::render, this));

        RCLCPP_INFO(this->get_logger(),
            "realcar_dashboard_node 시작 — 젯슨 원시 토픽 구독(odom=%s). 화면 대기 중...",
            odom_topic_.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr m) {
        rclcpp::Time now_t = this->now();
        double vx = m->twist.twist.linear.x;               // 전방 속도(base_link, REP-105)
        double yaw_rate = m->twist.twist.angular.z;         // 요레이트 [rad/s]

        // 종가속도: 속도 미분. odom 미분은 노이즈가 있어 EMA로 평활. 큰 시간간격(끊김 후 재개)은 스킵.
        if (got_odom_) {
            double dt = (now_t - last_odom_t_).seconds();
            if (dt > 1e-3 && dt < 0.5) {
                double raw = (vx - prev_vx_) / dt;
                long_accel_ = kEmaAlpha * long_accel_ + (1.0 - kEmaAlpha) * raw;
            }
        }
        prev_vx_ = vx;
        speed_ = vx;
        lat_accel_ = vx * yaw_rate;                         // 원심(횡) 가속도
        got_odom_ = true; last_odom_t_ = now_t;
    }

    // 마지막 수신 이후 경과 표시. 신선(<0.5s)이면 초록, 지연(<1.5s) 노랑, 그 이상/미수신 빨강.
    std::string age(bool ever, const rclcpp::Time& t) {
        if (!ever) return "\033[1;31m --  \033[0m";
        double a = (this->now() - t).seconds();
        const char* col = (a < 0.5) ? "\033[1;32m" : (a < 1.5 ? "\033[1;33m" : "\033[1;31m");
        std::ostringstream o;
        o << col << std::fixed << std::setprecision(1) << a << "s\033[0m";
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

        // ── 안전 / 모드 ──
        bool estop = (got_drive_mode_ && drive_mode_ == "estop");
        oss << " [Safety / Mode]\n";
        oss << "   E-Stop     : "
            << (estop ? "\033[1;31m[ ON  - BRAKE ]\033[0m" : "\033[1;32m[ OFF ]\033[0m")
            << "        (/drive_mode " << age(got_drive_mode_, last_drive_mode_t_) << ")\n";

        std::string mode_col = "\033[1;33m[N/A]\033[0m";
        if (got_drive_mode_) {
            if (drive_mode_ == "autonomous") mode_col = "\033[1;36m[AUTONOMOUS]\033[0m";
            else if (drive_mode_ == "manual") mode_col = "\033[1;32m[MANUAL]\033[0m";
            else if (drive_mode_ == "estop") mode_col = "\033[1;31m[ESTOP]\033[0m";
            else mode_col = "[" + drive_mode_ + "]";
        }
        oss << "   Drive Mode : " << mode_col << "\n";

        std::string algo = got_mppi_ ? (mppi_active_ ? "\033[1;35m[MPPI]\033[0m" : "\033[1;36m[MAP]\033[0m")
                                     : "\033[1;33m[N/A]\033[0m";
        oss << "   Algorithm  : " << algo
            << "        (/mppi_active " << age(got_mppi_, last_mppi_t_) << ")\n\n";

        // ── 운전자 입력(조이스틱) ──
        oss << " [Driver Input /joy]        (" << age(got_joy_, last_joy_t_) << ")\n";
        oss << std::fixed << std::setprecision(1);
        if (got_joy_) {
            double thr = (joy_.axes.size() > 1) ? joy_.axes[1] * 100.0 : 0.0;
            double str = (joy_.axes.size() > 3) ? joy_.axes[3] * 100.0 : 0.0;
            oss << "   Throttle : " << std::showpos << thr << " %"
                << "     Steering : " << str << " %" << std::noshowpos << "\n\n";
        } else {
            oss << "   \033[1;31m(수신 없음)\033[0m\n\n";
        }

        // ── 차량 텔레메트리(odom 파생) ──
        oss << " [Vehicle Telemetry]        (odom " << age(got_odom_, last_odom_t_) << ")\n";
        if (got_odom_) {
            double erpm = speed_ * speed_to_erpm_gain_;
            oss << std::setprecision(3) << std::showpos;
            oss << "   Speed    : " << speed_ << " m/s"
                << "      ERPM : " << std::setprecision(0) << erpm << std::setprecision(2) << "\n";
            oss << "   Long Acc : " << long_accel_ << " m/s^2"
                << "    Lat Acc : " << lat_accel_ << " m/s^2" << std::noshowpos << "\n";
        } else {
            oss << "   \033[1;31m(수신 없음)\033[0m\n";
        }
        oss << "=========================================================\n";

        std::cout << "\033[2J\033[H" << oss.str() << std::flush;
    }

    // 파라미터
    std::string odom_topic_;
    double speed_to_erpm_gain_ = 4614.0;

    // 수신/파생 데이터
    std::string drive_mode_;
    bool mppi_active_ = false;
    double speed_ = 0.0, prev_vx_ = 0.0, long_accel_ = 0.0, lat_accel_ = 0.0;
    sensor_msgs::msg::Joy joy_;
    static constexpr double kEmaAlpha = 0.6;  // 종가속도 평활 계수(클수록 부드럽게)

    bool got_drive_mode_ = false, got_mppi_ = false, got_odom_ = false, got_joy_ = false;
    rclcpp::Time never_, last_drive_mode_t_, last_mppi_t_, last_odom_t_, last_joy_t_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mppi_active_sub_;
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
