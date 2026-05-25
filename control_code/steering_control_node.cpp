#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// 신규 모듈 헤더
#include "f1tenth_control/types.hpp"
#include "f1tenth_control/geometry_utils.hpp"
#include "f1tenth_control/gap_follower.hpp"
#include "f1tenth_control/velocity_profiler.hpp"
#include "f1tenth_control/imu_stability_controller.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"

using namespace f1tenth_control;

class SteeringControlNode : public rclcpp::Node {
public:
    SteeringControlNode() : Node("steering_control_node") {
        // ==========================================
        // 1. ROS 2 파라미터 선언 및 초기화
        // ==========================================
        this->declare_parameter<double>("wheelbase", 0.33);
        this->declare_parameter<double>("min_lookahead", 0.8);
        this->declare_parameter<double>("max_lookahead", 3.0);
        this->declare_parameter<double>("lookahead_ratio", 0.35);
        this->declare_parameter<double>("base_steering_gain", 1.0);
        
        // 과도기 응답 보상 파라미터 (Roll Rate 피드백)
        this->declare_parameter<double>("roll_rate_gain", 0.4);
        this->declare_parameter<double>("max_gain_attenuation", 0.5);
        
        // 롤 상태 인지형 가변 감속 파라미터 (Roll Angle 피드백)
        this->declare_parameter<double>("max_roll_limit", 0.15);
        this->declare_parameter<double>("decel_attenuation", 0.6);
        this->declare_parameter<double>("base_max_accel", 4.0);
        this->declare_parameter<double>("base_max_decel", 8.0);

        // IMU 센서 실물 장착 여부 스위치 및 횡슬립 방지 피드백 게인
        this->declare_parameter<bool>("use_imu", false);               // IMU 미장착 상태에 대비한 안전 토글 (기본 OFF)
        this->declare_parameter<double>("yaw_rate_gain", 0.1);         // 횡슬립 방지 조향 복원 게인

        // BEXCO 우레탄 바닥 낮은 마찰계수(0.4~0.6)에 대응하기 위해 최대 허용 횡가속도 하향 조정
        this->declare_parameter<double>("max_lat_accel", 4.0);         // 최대 허용 횡가속도 (m/s^2)
        this->declare_parameter<double>("max_speed", 7.0);             // 직선 코스 최대 한계 속도 (m/s)
        this->declare_parameter<double>("min_speed", 2.0);             // 급코너 및 장애물 우회 최소 속도 (m/s)
        this->declare_parameter<std::string>("odom_topic", "/pf/pose/odom"); // 오도메트리 토픽명

        // 파라미터 값 로드
        this->get_parameter("wheelbase", wheelbase_);
        this->get_parameter("min_lookahead", min_lookahead_);
        this->get_parameter("max_lookahead", max_lookahead_);
        this->get_parameter("lookahead_ratio", lookahead_ratio_);
        this->get_parameter("base_steering_gain", base_steering_gain_);
        this->get_parameter("roll_rate_gain", roll_rate_gain_);
        this->get_parameter("max_gain_attenuation", max_gain_attenuation_);
        this->get_parameter("max_roll_limit", max_roll_limit_);
        this->get_parameter("decel_attenuation", decel_attenuation_);
        this->get_parameter("base_max_accel", base_max_accel_);
        this->get_parameter("base_max_decel", base_max_decel_);
        this->get_parameter("use_imu", use_imu_);
        this->get_parameter("yaw_rate_gain", yaw_rate_gain_);
        this->get_parameter("max_lat_accel", max_lat_accel_);
        this->get_parameter("max_speed", max_speed_);
        this->get_parameter("min_speed", min_speed_);
        this->get_parameter("odom_topic", odom_topic_);

        // ==========================================
        // 2. 글로벌 경로(Waypoints) 구독 설정 (플래닝 팀 연동)
        // ==========================================
        auto qos_gl = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        global_path_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/global_waypoints", qos_gl,
            std::bind(&SteeringControlNode::global_path_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "플래닝 팀의 글로벌 경로 토픽(/global_waypoints) 구독 설정 완료.");

        // ==========================================
        // 3. 알고리즘 인스턴스 초기화 및 통신 채널 설정
        // ==========================================
        gap_follower_ = std::make_unique<GapFollower>(180.0, 0.38, 3.0, 0.41);
        stability_controller_ = std::make_unique<StabilityController>(0.15, 0.2, 0.2);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&SteeringControlNode::odom_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&SteeringControlNode::imu_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SteeringControlNode::scan_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_autonomous", 10);

        // 실시간 50Hz (20ms) 주기 타이머 가동
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SteeringControlNode::control_loop, this));

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "RoboRacer 곡률 및 로컬 회피 연계 제어 노드가 시작되었습니다.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // Quaternion -> Yaw 변환
        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        // 선속도 추출
        current_speed_ = msg->twist.twist.linear.x;
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        if (!use_imu_) {
            stability_controller_->reset();
            return;
        }
        stability_controller_->update_imu(msg->orientation, msg->angular_velocity.x, msg->angular_velocity.z);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        latest_scan_ = msg;
    }

    void global_path_callback(const f110_msgs::msg::WpntArray::ConstSharedPtr msg) {
        if (msg->wpnts.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty global waypoints.");
            return;
        }

        waypoints_.clear();
        waypoints_.reserve(msg->wpnts.size());

        for (const auto& wp : msg->wpnts) {
            Waypoint interp_wp;
            interp_wp.x = wp.x_m;
            interp_wp.y = wp.y_m;
            interp_wp.speed = wp.vx_mps;
            interp_wp.curvature = wp.kappa_radpm;
            interp_wp.raw_speed_limit = wp.vx_mps;
            waypoints_.push_back(interp_wp);
        }

        last_target_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "🔄 플래닝 팀의 글로벌 경로 수신 완료! 웨이포인트 개수: %zu", waypoints_.size());
    }

    /**
     * O(1) Local Window Search & 보간(Interpolation) 목표점 로딩
     */
    Waypoint get_interpolated_lookahead_point(double lookahead_dist) {
        size_t n = waypoints_.size();
        if (n == 0) return Waypoint{0.0, 0.0, 0.0, 0.0, 0.0};

        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = last_target_idx_;

        for (int i = -15; i <= 15; ++i) {
            size_t idx = (last_target_idx_ + i + n) % n;
            double dx = waypoints_[idx].x - current_x_;
            double dy = waypoints_[idx].y - current_y_;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = idx;
            }
        }

        size_t idx_a = closest_idx;
        size_t idx_b = (closest_idx + 1) % n;
        
        double dist_a = std::hypot(waypoints_[idx_a].x - current_x_, waypoints_[idx_a].y - current_y_);
        double dist_b = std::hypot(waypoints_[idx_b].x - current_x_, waypoints_[idx_b].y - current_y_);

        while (dist_b < lookahead_dist) {
            idx_a = idx_b;
            idx_b = (idx_b + 1) % n;
            dist_a = dist_b;
            dist_b = std::hypot(waypoints_[idx_b].x - current_x_, waypoints_[idx_b].y - current_y_);
            if (idx_a == closest_idx) break;
        }

        last_target_idx_ = idx_a;

        double segment_len = dist_b - dist_a;
        double t = 0.0;
        if (segment_len > 0.001) {
            t = (lookahead_dist - dist_a) / segment_len;
            t = std::max(0.0, std::min(t, 1.0));
        }

        Waypoint interp_wp;
        interp_wp.x = waypoints_[idx_a].x + t * (waypoints_[idx_b].x - waypoints_[idx_a].x);
        interp_wp.y = waypoints_[idx_a].y + t * (waypoints_[idx_b].y - waypoints_[idx_a].y);
        interp_wp.speed = waypoints_[idx_a].speed + t * (waypoints_[idx_b].speed - waypoints_[idx_a].speed);
        interp_wp.curvature = waypoints_[idx_a].curvature + t * (waypoints_[idx_b].curvature - waypoints_[idx_a].curvature);
        interp_wp.raw_speed_limit = waypoints_[idx_a].raw_speed_limit + t * (waypoints_[idx_b].raw_speed_limit - waypoints_[idx_a].raw_speed_limit);

        return interp_wp;
    }

    void control_loop() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) dt = 0.02;
        last_time_ = current_time;

        // 글로벌 경로가 없는 경우 -> 순수 라이다 기반 Gap Follower로 무한 회피 주행
        if (waypoints_.empty()) {
            double avoid_steering_angle = 0.0;
            double min_obstacle_dist = 999.0;
            bool obstacle_detected = gap_follower_->process_scan(latest_scan_, avoid_steering_angle, min_obstacle_dist);

            // 항상 gap 방향으로 조향 (장애물 유무 무관하게 가장 열린 방향으로) + 조향 안정화 LPF 적용
            double final_steering_angle = avoid_steering_angle;
            const double steer_filter_alpha = 0.70; 
            final_steering_angle = steer_filter_alpha * final_steering_angle + (1.0 - steer_filter_alpha) * last_steering_angle_;
            last_steering_angle_ = final_steering_angle;

            // 최소 장애물 거리 기반 속도 스케일링
            // dist >= 4.0m → max_speed(3.5), dist <= 1.0m → min_speed(1.2), 선형 보간
            const double speed_dist_max = 4.0;
            const double speed_dist_min = 1.0;
            const double max_speed = 3.5;
            const double target_min_speed = 1.2; // 코너 안정 속도 하향
            double speed_ratio = (min_obstacle_dist - speed_dist_min) / (speed_dist_max - speed_dist_min);
            speed_ratio = std::max(0.0, std::min(1.0, speed_ratio));
            double final_speed = target_min_speed + speed_ratio * (max_speed - target_min_speed);

            // 조향각이 클수록 추가 감속 (급커브 안정성)
            double steer_ratio = std::abs(final_steering_angle) / 0.41;
            final_speed *= (1.0 - 0.50 * steer_ratio); // 최대 조향 시 50% 감속

            double speed_error = final_speed - current_speed_;
            double cmd_speed = last_target_speed_;
            double max_accel = base_max_accel_;
            double max_decel = base_max_decel_;

            if (speed_error > 0.0) {
                cmd_speed += std::min(speed_error, max_accel * dt);
                if (cmd_speed > final_speed) cmd_speed = final_speed;
            } else {
                cmd_speed += std::max(speed_error, -max_decel * dt);
                if (cmd_speed < final_speed) cmd_speed = final_speed;
            }
            last_target_speed_ = cmd_speed;

            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.header.stamp = this->now();
            drive_msg.header.frame_id = "base_link";
            drive_msg.drive.steering_angle = final_steering_angle;
            drive_msg.drive.speed = cmd_speed;
            drive_msg.drive.acceleration = (cmd_speed - current_speed_) / dt;
            drive_pub_->publish(drive_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[Pure GapFollower Mode] Steering: %.2f rad, Speed: %.2f m/s, MinDist: %.2f m",
                final_steering_angle, cmd_speed, min_obstacle_dist);
            return;
        }

        // --------------------------------------------------------
        // Step 1. 속도 비례 동적 룩어헤드 계산 (Dynamic Look-ahead)
        // --------------------------------------------------------
        double lookahead_dist = lookahead_ratio_ * current_speed_ + min_lookahead_;
        lookahead_dist = std::max(min_lookahead_, std::min(lookahead_dist, max_lookahead_));

        // 최적화된 경로점 로드
        Waypoint target_wp = get_interpolated_lookahead_point(lookahead_dist);

        // --------------------------------------------------------
        // Step 1-1. 장애물 감지 및 로컬 회피 조향(Reactive Gap Follower) 연산
        // --------------------------------------------------------
        double avoid_steering_angle = 0.0;
        double avoid_min_dist = 999.0;
        bool obstacle_detected = gap_follower_->process_scan(latest_scan_, avoid_steering_angle, avoid_min_dist);

        // --------------------------------------------------------
        // Step 2. Pure Pursuit 조향각 산출
        // --------------------------------------------------------
        double dx = target_wp.x - current_x_;
        double dy = target_wp.y - current_y_;
        
        // 로컬 차량 기준 좌표계 횡변위 y 계산
        double local_y = -dx * std::sin(current_yaw_) + dy * std::cos(current_yaw_);
        
        // 기하학 조향각 계산
        double raw_steering_angle = std::atan2(2.0 * wheelbase_ * local_y, std::pow(lookahead_dist, 2));

        // --------------------------------------------------------
        // Step 3. 과도기 응답 보상 제어 (IMU Roll Rate 적용)
        // --------------------------------------------------------
        double attenuation = 0.0;
        if (use_imu_) {
            attenuation = stability_controller_->calculate_steering_attenuation(roll_rate_gain_, max_gain_attenuation_);
        }
        
        double current_steering_gain = base_steering_gain_ * (1.0 - attenuation);
        double final_steering_angle = raw_steering_angle * current_steering_gain;

        // --------------------------------------------------------
        // IMU Yaw Rate 피드백을 이용한 횡슬립(오버/언더스티어) 능동 조향 보정
        // --------------------------------------------------------
        if (use_imu_) {
            double correction = stability_controller_->calculate_yaw_rate_correction(
                current_speed_, final_steering_angle, wheelbase_, yaw_rate_gain_);
            final_steering_angle += correction;
        }

        // 조향각 물리 한계 적용 (+-0.41 rad)
        final_steering_angle = std::max(-0.41, std::min(final_steering_angle, 0.41));

        // 장애물 감지 시 Gap Follower 회피 조향으로 덮어씀
        if (obstacle_detected) {
            final_steering_angle = avoid_steering_angle;
        }

        // --------------------------------------------------------
        // Step 4. 롤 상태 인지형 가변 감속 제어 (ESC 가감속 필터 적용)
        // --------------------------------------------------------
        double roll_ratio = 0.0;
        if (use_imu_) {
            roll_ratio = stability_controller_->calculate_roll_ratio(max_roll_limit_);
        }

        // 롤 각도 비례하여 차량 동적 허용 가감속 폭 스케일링 (접지 불균형에 의한 스핀아웃 예방)
        double max_accel = base_max_accel_ * (1.0 - roll_ratio * decel_attenuation_);
        double max_decel = base_max_decel_ * (1.0 - roll_ratio * decel_attenuation_);

        // --------------------------------------------------------
        // Step 5. 속도 명령 추종 (Rate Limiting 필터 적용)
        // --------------------------------------------------------
        double target_speed = target_wp.speed;

        // 장애물 감지 시 안전 감속 모드 적용
        if (obstacle_detected) {
            target_speed = min_speed_; // 급코너 속도 상한인 2.0 m/s로 제약
        }

        // 주행 상황에 따라 롤링 한계에 극도로 가까워진 경우 ESC 감속 보정 적용
        if (roll_ratio > 0.8) {
            target_speed = std::max(min_speed_, target_speed * (1.0 - (roll_ratio - 0.8)));
        }

        double speed_error = target_speed - current_speed_;
        double final_speed = last_target_speed_;

        if (speed_error > 0.0) {
            double speed_change = std::min(speed_error, max_accel * dt);
            final_speed += speed_change;
            if (final_speed > target_speed) final_speed = target_speed;
        } else {
            double speed_change = std::max(speed_error, -max_decel * dt);
            final_speed += speed_change;
            if (final_speed < target_speed) final_speed = target_speed;
        }
        last_target_speed_ = final_speed;

        // --------------------------------------------------------
        // Step 6. Ackermann 제어 명령 토픽 발행
        // --------------------------------------------------------
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";

        drive_msg.drive.steering_angle = final_steering_angle;
        drive_msg.drive.speed = final_speed;
        drive_msg.drive.acceleration = (final_speed - current_speed_) / dt;

        drive_pub_->publish(drive_msg);
    }

    // ==========================================
    // 4. 멤버 변수 선언
    // ==========================================
    // 제어 파라미터 변수
    double wheelbase_;
    double min_lookahead_;
    double max_lookahead_;
    double lookahead_ratio_;
    double base_steering_gain_;
    double roll_rate_gain_;
    double max_gain_attenuation_;
    double max_roll_limit_;
    double decel_attenuation_;
    double base_max_accel_;
    double base_max_decel_;
    bool use_imu_;
    double yaw_rate_gain_;
    std::string odom_topic_;

    // 곡률 제어 관련 파라미터
    double max_lat_accel_;
    double max_speed_;
    double min_speed_;

    // 차량 상태값 변수
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double current_speed_ = 0.0;

    double last_target_speed_ = 0.0;
    double last_steering_angle_ = 0.0;
    rclcpp::Time last_time_;

    // 최적화 로컬 윈도우 인덱스
    size_t last_target_idx_ = 0;

    std::vector<Waypoint> waypoints_;

    // 알고리즘 인스턴스 독점 소유
    std::unique_ptr<GapFollower> gap_follower_;
    std::unique_ptr<StabilityController> stability_controller_;

    // ROS 2 통신 개체
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_path_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 최신 라이다 데이터
    sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_ = nullptr;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SteeringControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
