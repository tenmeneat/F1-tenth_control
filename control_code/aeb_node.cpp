#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

// 안전한 수학 상수 정의
const double PI = 3.14159265358979323846;

class AutonomousEmergencyBraking : public rclcpp::Node {
public:
    AutonomousEmergencyBraking() : Node("aeb_node") {
        // ==========================================
        // 1. 파라미터 정의 및 주입
        // ==========================================
        this->declare_parameter<double>("ttc_threshold", 0.35);       // 비상 제동 임계 시간 (초)
        this->declare_parameter<double>("scan_fov_deg", 120.0);       // 전방 충돌 감지 각도 범위 (도, +-60도)
        this->declare_parameter<double>("min_safe_dist", 0.15);       // 초근접 시 강제 제동 최소 거리 (m)
        this->declare_parameter<bool>("latch_aeb", true);             // 비상 제동 발생 후 잠금(Latching) 여부

        this->get_parameter("ttc_threshold", ttc_threshold_);
        this->get_parameter("scan_fov_deg", scan_fov_deg_);
        this->get_parameter("min_safe_dist", min_safe_dist_);
        this->get_parameter("latch_aeb", latch_aeb_);

        // ==========================================
        // 2. 통신 구성 (Pub/Sub)
        // ==========================================
        // 센서 및 상태 데이터 구독
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&AutonomousEmergencyBraking::scan_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&AutonomousEmergencyBraking::odom_callback, this, std::placeholders::_1));

        // 수동 리셋 구독
        reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/aeb/reset", 10,
            std::bind(&AutonomousEmergencyBraking::reset_callback, this, std::placeholders::_1));

        // 제어권 제동 명령 및 경보 플래그 발행
        aeb_drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10); 

        aeb_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/aeb_active", 10);

        RCLCPP_INFO(this->get_logger(), "=================================================");
        RCLCPP_INFO(this->get_logger(), "F1TENTH 자율 비상제동(AEB) 시스템 구성 정보:");
        RCLCPP_INFO(this->get_logger(), " - TTC Threshold: %.3f sec", ttc_threshold_);
        RCLCPP_INFO(this->get_logger(), " - FOV Angle: %.1f deg", scan_fov_deg_);
        RCLCPP_INFO(this->get_logger(), " - Min Safe Distance: %.2f m", min_safe_dist_);
        RCLCPP_INFO(this->get_logger(), " - Latch AEB: %s", latch_aeb_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(this->get_logger(), "=================================================");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        // 현재 차량의 종방향 속도(Forward Speed) 갱신
        current_speed_ = msg->twist.twist.linear.x;
    }

    void reset_callback(const std_msgs::msg::Empty::ConstSharedPtr msg) {
        (void)msg; // 사용하지 않는 매개변수 경고 방지
        if (is_latched_) {
            is_latched_ = false;
            RCLCPP_INFO(this->get_logger(), "🔄 AEB 잠금 상태가 사용자에 의해 리셋되었습니다. 정상 주행이 가능합니다.");
            publish_aeb_status(false);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "AEB 잠금 상태가 아니므로 리셋 명령을 무시합니다.");
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        // 1. 3-point Median Filter 적용
        std::vector<float> filtered_ranges = msg->ranges;
        if (msg->ranges.size() >= 3) {
            for (size_t i = 1; i < msg->ranges.size() - 1; ++i) {
                float r_prev = msg->ranges[i-1];
                float r_curr = msg->ranges[i];
                float r_next = msg->ranges[i+1];

                std::vector<float> window;
                auto is_valid = [msg](float r) {
                    return !std::isnan(r) && !std::isinf(r) && r >= msg->range_min && r <= msg->range_max;
                };

                if (is_valid(r_prev)) window.push_back(r_prev);
                if (is_valid(r_curr)) window.push_back(r_curr);
                if (is_valid(r_next)) window.push_back(r_next);

                if (window.size() >= 2) {
                    std::sort(window.begin(), window.end());
                    filtered_ranges[i] = window[window.size() / 2];
                } else if (window.size() == 1) {
                    filtered_ranges[i] = window[0];
                } else {
                    filtered_ranges[i] = msg->ranges[i];
                }
            }
        }

        // 2. 이미 수동 비상제동 상태가 유지(Latch) 중이고 latch_aeb_가 true인 경우
        if (latch_aeb_ && is_latched_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "🚨 AEB 잠금 유지 중! 주행을 재개하려면 '/aeb/reset' 토픽을 발행하십시오.");
            execute_emergency_brake();
            publish_aeb_status(true);
            return;
        }

        // 3. 자율 복구 상태 머신 처리 (latch_aeb_가 false인 상태에서 is_latched_가 true인 경우)
        // 아래 루프를 통해 현재 시점에서도 위험 요소가 존재하는지 확인해야 하므로 계속 진행함.
        
        // 단, 평상시(위험 상태가 아닌 상태)에 차량이 정지해 있거나 후진 중일 때 AEB 감지 연산을 건너뛰는 기존 안전 필터
        if (!is_latched_ && current_speed_ <= 0.01) {
            publish_aeb_status(false);
            return;
        }

        bool trigger_aeb = false;
        double min_ttc = std::numeric_limits<double>::max();
        double fov_rad = (scan_fov_deg_ * PI) / 180.0;
        
        // 라이다 각도 인덱스별 분석
        for (size_t i = 0; i < filtered_ranges.size(); ++i) {
            double range = filtered_ranges[i];

            // 노이즈 또는 빈 측정값 무시
            if (std::isnan(range) || std::isinf(range) || range < msg->range_min || range > msg->range_max) {
                continue;
            }

            // 라이다 빔의 각도 계산 (차량 중심선 기준 좌우 각도)
            double angle = msg->angle_min + i * msg->angle_increment;

            // 전방 특정 FOV (예: +-60도) 바깥 영역은 충돌 회피 연산에서 배제
            if (std::abs(angle) > (fov_rad / 2.0)) {
                continue;
            }

            // 1단계: 초근접 거리에 물체가 감지되면 즉각 비상제동 트리거
            if (range < min_safe_dist_) {
                trigger_aeb = true;
                min_ttc = 0.0; // 즉각 충돌 상황
                break;
            }

            // 2단계: TTC (Time-To-Collision) 계산
            // 차량 정방향 속도를 해당 빔 방향으로 사영(Projection)
            double projected_velocity = current_speed_ * std::cos(angle);

            // 해당 빔 방향으로 물체가 가까워지고 있을 때만 연산 (projected_velocity > 0)
            if (projected_velocity > 0.001) {
                double ttc = range / projected_velocity;
                if (ttc < min_ttc) {
                    min_ttc = ttc;
                }

                // 충돌 예상 시간이 임계치 미만인 경우 제동 트리거
                if (ttc < ttc_threshold_) {
                    trigger_aeb = true;
                }
            }
        }

        // 4. AEB 상태 및 복구 결정 로직
        if (trigger_aeb) {
            // 충돌 위험 감지 시 무조건 Latch 상태로 설정하고 브레이크
            is_latched_ = true;
            last_unsafe_time_ = this->now(); // 최종 위험 시간 갱신

            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "⚠️ 충돌 위험 감지! 긴급 제동 발동 (TTC: %.3f s, 속도: %.2f m/s)", min_ttc, current_speed_);
            execute_emergency_brake();
            publish_aeb_status(true);
        } else {
            // 충돌 위험이 감지되지 않은 상황
            if (is_latched_) {
                // latch_aeb_가 false인 자율 복구 상태
                double elapsed = (this->now() - last_unsafe_time_).seconds();
                if (elapsed >= 1.0) {
                    // 1초간 위험이 감지되지 않은 경우 Latch 해제 및 복구
                    is_latched_ = false;
                    RCLCPP_INFO(this->get_logger(), "🔄 [Self-Recovery] AEB 자율 복구 완료! 전방 위험 해제 1.0초 경과로 주행을 자동 재개합니다.");
                    publish_aeb_status(false);
                } else {
                    // 안전 대기 시간(1.0초)이 지나기 전까지는 정지 상태 유지
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "⚠️ [Self-Recovery] 위험 요인은 사라졌으나 안전을 위해 대기 중... (경과 시간: %.2f s)", elapsed);
                    execute_emergency_brake();
                    publish_aeb_status(true);
                }
            } else {
                // 평상시 안전한 주행 상태
                publish_aeb_status(false);
            }
        }
    }

    void execute_emergency_brake() {
        // 감속 한계에 가까운 속도 감속 프로파일 전송 (속도 0, 급감속 적용)
        auto brake_msg = ackermann_msgs::msg::AckermannDriveStamped();
        brake_msg.header.stamp = this->now();
        brake_msg.header.frame_id = "base_link";
        
        brake_msg.drive.speed = 0.0;             // 즉시 정지 목표
        brake_msg.drive.acceleration = -9.0;     // 최대 제동력 (m/s^2)
        brake_msg.drive.steering_angle = 0.0;     // 제동 시 스핀 방지를 위한 중립 조향 조치
        
        aeb_drive_pub_->publish(brake_msg);
    }

    void publish_aeb_status(bool active) {
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = active;
        aeb_active_pub_->publish(status_msg);
    }

    // 파라미터 변수
    double ttc_threshold_;
    double scan_fov_deg_;
    double min_safe_dist_;
    bool latch_aeb_;

    // 상태 변수
    double current_speed_ = 0.0;
    bool is_latched_ = false;
    rclcpp::Time last_unsafe_time_;

    // ROS 2 통신 개체
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr aeb_drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr aeb_active_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousEmergencyBraking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
