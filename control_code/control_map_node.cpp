#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "f1tenth_control/types.hpp"
#include "f1tenth_control/gap_follower.hpp"
#include "f1tenth_control/imu_stability_controller.hpp"
#include "f1tenth_control/steering_lookup_table.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace f1tenth_control;

class ControlMapNode : public rclcpp::Node {
public:
    ControlMapNode() : Node("control_map_node") {
        // 1. ROS 2 파라미터 선언 및 초기화
        this->declare_parameter<double>("wheelbase", 0.33);

        // L1 Guidance Control 파라미터
        this->declare_parameter<double>("l1_gain", 0.5);
        this->declare_parameter<double>("l1_distance", 0.3); // Python's m_l1
        this->declare_parameter<double>("t_clip_min", 0.8);
        this->declare_parameter<double>("t_clip_max", 5.0);
        this->declare_parameter<double>("lateral_error_coeff", 1.0);

        // Heading-error 댐핑 게인 (Stanley형 정렬항): 순수 L1 cross-track 제어의 복구 시
        // heading 오버슈트(라인 가로지름→외벽 충돌)를 억제. 0이면 기존 순수 L1 동작.
        // 런타임 파라미터로 A/B 튜닝 가능. 기본 0.0(비활성).
        this->declare_parameter<double>("heading_damping_gain", 0.0);

        // 조향각 스케일러 파라미터 (가속 시 조향 급격화 방지를 위해 완화)
        this->declare_parameter<double>("acceleration_scaler_for_steering", 1.0);
        this->declare_parameter<double>("deceleration_scaler_for_steering", 0.95);
        this->declare_parameter<double>("start_scale_speed", 7.0);
        this->declare_parameter<double>("end_scale_speed", 8.0);
        this->declare_parameter<double>("downscale_factor", 0.10);

        // 속도 예측 룩어헤드 파라미터
        this->declare_parameter<double>("speed_lookahead", 0.15);
        this->declare_parameter<double>("speed_lookahead_for_steering", 0.0);
        this->declare_parameter<std::string>("lookup_table_file", "");

        // 기존 롤 상태 인지형 가변 감속 파라미터 (Roll Angle 피드백)
        this->declare_parameter<double>("max_roll_limit", 0.15);
        this->declare_parameter<double>("decel_attenuation", 0.6);
        this->declare_parameter<double>("base_max_accel", 4.0);
        this->declare_parameter<double>("base_max_decel", 8.0);

        // IMU 센서 안전 토글 및 횡슬립 방지
        this->declare_parameter<bool>("use_imu", true);
        // IMU 각속도 단위 보정. 실제 값은 런치가 넘긴다(_control_common.py IMU_ANGULAR_SCALE).
        this->declare_parameter<double>("imu_angular_scale", 1.0);
        this->declare_parameter<double>("yaw_rate_gain", 0.1);
        this->declare_parameter<double>("max_speed", 12.0);
        this->declare_parameter<double>("min_speed", 2.0);

        // 곡률 룩어헤드 감속 파라미터
        this->declare_parameter<int>("curvature_lookahead_count", 20);
        this->declare_parameter<double>("max_lateral_accel", 6.0);
        this->declare_parameter<double>("curvature_ff_blend", 0.0); // 곡률 FF 비활성: 검증된 순수 L1 격리 (원본 MAP 컨트롤러 미보유 항목)
        this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");

        // 안전라인 시프트: 플래너 최적라인이 벽에 과도하게 붙은(클리어런스 부족) 구간에서
        // 차체(길이 0.58m)가 벽을 스치는 충돌을 방지하기 위해, 메시지의 d_left/d_right(트랙 경계까지
        // 거리)를 이용해 해당 웨이포인트를 트랙 중심 쪽으로 밀어 최소 벽 클리어런스 C를 확보한다.
        // C는 차량 반폭+자세/추종 마진. 0이면 원본 라인 그대로(비활성).
        this->declare_parameter<double>("wall_safety_margin", 0.6);

        this->get_parameter("wheelbase", wheelbase_);
        this->get_parameter("l1_gain", l1_gain_);
        this->get_parameter("l1_distance", l1_distance_);
        this->get_parameter("t_clip_min", t_clip_min_);
        this->get_parameter("t_clip_max", t_clip_max_);
        this->get_parameter("lateral_error_coeff", lateral_error_coeff_);
        this->get_parameter("heading_damping_gain", heading_damping_gain_);
        this->get_parameter("acceleration_scaler_for_steering", acceleration_scaler_for_steering_);
        this->get_parameter("deceleration_scaler_for_steering", deceleration_scaler_for_steering_);
        this->get_parameter("start_scale_speed", start_scale_speed_);
        this->get_parameter("end_scale_speed", end_scale_speed_);
        this->get_parameter("downscale_factor", downscale_factor_);
        this->get_parameter("speed_lookahead", speed_lookahead_);
        this->get_parameter("speed_lookahead_for_steering", speed_lookahead_for_steering_);
        
        std::string lut_file;
        this->get_parameter("lookup_table_file", lut_file);

        this->get_parameter("max_roll_limit", max_roll_limit_);
        this->get_parameter("decel_attenuation", decel_attenuation_);
        this->get_parameter("base_max_accel", base_max_accel_);
        this->get_parameter("base_max_decel", base_max_decel_);
        this->get_parameter("use_imu", use_imu_);
        this->get_parameter("imu_angular_scale", imu_angular_scale_);
        this->get_parameter("yaw_rate_gain", yaw_rate_gain_);
        this->get_parameter("max_speed", max_speed_);
        this->get_parameter("min_speed", min_speed_);
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("wall_safety_margin", wall_safety_margin_);

        int cl_count;
        this->get_parameter("curvature_lookahead_count", cl_count);
        curvature_lookahead_count_ = static_cast<size_t>(cl_count);
        this->get_parameter("max_lateral_accel", max_lateral_accel_);
        this->get_parameter("curvature_ff_blend", curvature_ff_blend_);

        acc_now_ = std::vector<double>(10, 0.0);

        // 룩업 테이블 로딩 (다중 경로 Fallback 확보)
        bool loaded = false;
        if (!lut_file.empty()) {
            loaded = lookup_table_.load(lut_file);
        }
        
        // 1차 시도 (ament index 기반 share 폴더)
        if (!loaded) {
            try {
                std::string share_dir = ament_index_cpp::get_package_share_directory("steering_lookup");
                lut_file = share_dir + "/cfg/NUC6_glc_pacejka_lookup_table.csv";
                loaded = lookup_table_.load(lut_file);
            } catch (...) {}
        }
        
        // 2차 시도 (f1tenth_control 패키지 자체 share/cfg — 이식성 확보. 하드코딩 홈 경로 제거)
        if (!loaded) {
            try {
                std::string share_dir = ament_index_cpp::get_package_share_directory("f1tenth_control");
                lut_file = share_dir + "/cfg/NUC6_glc_pacejka_lookup_table.csv";
                loaded = lookup_table_.load(lut_file);
            } catch (...) {}
        }

        if (!loaded) {
            RCLCPP_ERROR(this->get_logger(), "❌ [ControlMapNode] 모든 경로에서 룩업 테이블(LUT) 로드 실패! 조향각이 0.0으로 고정됩니다.");
        } else {
            RCLCPP_INFO(this->get_logger(), "🟢 [ControlMapNode] 룩업 테이블(LUT) 로드 성공: %s", lut_file.c_str());
        }

        // 2. 글로벌 경로(Waypoints) 구독 설정
        auto qos_gl = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        global_path_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/global_waypoints", qos_gl,
            std::bind(&ControlMapNode::global_path_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "플래닝 팀의 글로벌 경로 토픽(/global_waypoints) 구독 설정 완료.");

        // 2.5 로컬 경로(Local Waypoints) 구독 + 장애물 회피 폴백 파라미터
        // wpnt_publisher가 발행하는 짧은 열린 전방 구간(~50점). 글로벌과 달리 non-latched(VOLATILE).
        // 신선한 로컬이 있으면 글로벌보다 우선 추종하고, 끊기면 글로벌로 폴백한다.
        local_fresh_timeout_ = this->declare_parameter<double>("local_fresh_timeout", 0.3);
        // 로컬 회피경로가 없을 때, 글로벌 추종 중 앞이 막히면 GapFollower로 회피 폴백하는 파라미터
        obstacle_avoid_enable_ = this->declare_parameter<bool>("obstacle_avoid_enable", false);
        obstacle_cone_halfangle_ = this->declare_parameter<double>("obstacle_cone_halfangle", 0.14);
        obstacle_trigger_dist_ = this->declare_parameter<double>("obstacle_trigger_dist", 1.5);
        obstacle_margin_ = this->declare_parameter<double>("obstacle_margin", 0.3);
        obstacle_avoid_hold_cycles_ = this->declare_parameter<int>("obstacle_avoid_hold_cycles", 15);
        auto qos_local = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // 로컬 퍼블리셔에 맞춰 volatile
        local_path_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/local_waypoints", qos_local,
            std::bind(&ControlMapNode::local_path_callback, this, std::placeholders::_1));
        local_last_recv_time_ = this->now(); // 노드 클럭 타입으로 초기화(비교 시 clock mismatch 방지)
        RCLCPP_INFO(this->get_logger(), "로컬 경로 토픽(/local_waypoints) 구독 설정 완료.");

        // 3. 알고리즘 인스턴스 초기화 및 통신 채널 설정
        gap_follower_ = std::make_unique<GapFollower>(180.0, 0.38, 3.0, 0.41);
        stability_controller_ = std::make_unique<StabilityController>(0.15, 0.2);  // alpha_roll, alpha_yaw_rate

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&ControlMapNode::odom_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ControlMapNode::imu_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ControlMapNode::scan_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_autonomous", 10);

        // 실시간 50Hz (20ms) 주기 타이머 가동
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControlMapNode::control_loop, this));

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "RoboRacer L1 Guidance & Steer LUT 제어 노드가 시작되었습니다.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        current_speed_ = msg->twist.twist.linear.x;
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        if (use_imu_) {
            // VESC 자이로가 deg/s로 발행하는 것이 실차에서 확인되어(2026-07-19) 여기서 rad/s로
            // 환산한다. 보정 안 하면 실측 요레이트가 57.3배 → 카운터스티어가 즉시 반대로 포화.
            // 값의 근거·재확인 절차는 launch/_control_common.py의 IMU_ANGULAR_SCALE 주석 참고.
            stability_controller_->update_imu(msg->orientation,
                                              msg->angular_velocity.z * imu_angular_scale_);
        }
        // 가속도 rolling buffer 업데이트 (longitudinal acceleration)
        // VESC의 장착 방향 회전(90도)에 맞춰 -linear_acceleration.y 값을 적용
        std::rotate(acc_now_.rbegin(), acc_now_.rbegin() + 1, acc_now_.rend());
        acc_now_[0] = -msg->linear_acceleration.y;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        latest_scan_ = msg;
    }

    void global_path_callback(const f110_msgs::msg::WpntArray::ConstSharedPtr msg) {
        if (msg->wpnts.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty global waypoints.");
            return;
        }

        // global_republisher_node 등 플래너는 동일 경로를 주기적으로 재발행할 수 있음
        // (예: publish_period_sec=2.0). 최초 수신 여부를 먼저 기록해 아래 인덱스 재초기화
        // 범위를 최초 1회로 제한한다(이유는 아래 주석 참고).
        const bool first_reception = !waypoints_initialized_;

        waypoints_.clear();
        waypoints_.reserve(msg->wpnts.size());

        for (const auto& wp : msg->wpnts) {
            Waypoint interp_wp;
            interp_wp.x = wp.x_m;
            interp_wp.y = wp.y_m;
            interp_wp.speed = wp.vx_mps;
            interp_wp.curvature = wp.kappa_radpm;
            interp_wp.yaw = wp.psi_rad;

            // 안전라인 시프트: 벽에 너무 붙은 점을 트랙 중심 쪽으로 이동시켜 최소 클리어런스 C 확보.
            // d_left/d_right = 경로점에서 좌/우 트랙 경계까지의 거리. normal_left=(-sin psi, cos psi)는
            // 진행방향 좌측. +방향 이동 시 d_left↓·d_right↑ (즉 우벽에서 멀어짐).
            if (wall_safety_margin_ > 1e-3) {
                const double C = wall_safety_margin_;
                const double dl = wp.d_left;
                const double dr = wp.d_right;
                double shift = 0.0; // +면 좌측(우벽에서 멀어짐), -면 우측(좌벽에서 멀어짐)
                if (dr < C && dl > C) {
                    shift = std::min(C - dr, dl - C);        // 우벽이 가까움 → 좌측으로
                } else if (dl < C && dr > C) {
                    shift = -std::min(C - dl, dr - C);       // 좌벽이 가까움 → 우측으로
                } else if (dr < C && dl < C) {
                    shift = (dl - dr) / 2.0;                 // 양쪽 다 좁음 → 통로 중앙 정렬
                }
                if (std::abs(shift) > 1e-4) {
                    const double nx = -std::sin(wp.psi_rad);
                    const double ny =  std::cos(wp.psi_rad);
                    interp_wp.x += shift * nx;
                    interp_wp.y += shift * ny;
                }
            }

            waypoints_.push_back(interp_wp);
        }

        // 경로 수신 시 최단 거리 인덱스로 초기화 — 단 "최초 수신"이거나 기존 인덱스가
        // 새 배열 범위를 벗어났을 때만 전체 재탐색을 수행한다. 매 재발행마다 전체
        // 재탐색하면 스타트/피니시처럼 유클리드 거리는 가깝지만 인덱스는 트랙 반대편인
        // 구간에서 엉뚱한 인덱스로 스냅되어 조향 포화·속도 붕괴로 이어질 수 있다
        // (global_republisher_node의 주기 재발행 시 상시 발생 가능). 최초 수신 이후엔
        // control_loop이 매 사이클 윈도우 탐색으로 인덱스를 계속 추적하므로 재초기화가
        // 필요 없다.
        if (first_reception || last_target_idx_ >= waypoints_.size()) {
            double min_dist = std::numeric_limits<double>::max();
            size_t closest_idx = 0;
            for (size_t i = 0; i < waypoints_.size(); ++i) {
                double dx = waypoints_[i].x - current_x_;
                double dy = waypoints_[i].y - current_y_;
                double dist = std::hypot(dx, dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_idx = i;
                }
            }
            last_target_idx_ = closest_idx;
            waypoints_initialized_ = true;
        }

        // 전체 경로의 평균 곡률을 한 번만 계산 (control_loop에서 매 사이클 반복 제거)
        double sum_kappa = 0.0;
        for (const auto& wp : waypoints_) {
            sum_kappa += std::abs(wp.curvature);
        }
        mean_track_curvature_ = waypoints_.empty() ? 0.0 : (sum_kappa / waypoints_.size());

        // 평균 웨이포인트 간격(닫힌 루프 둘레/개수) — closest_idx 윈도우 탐색 크기와 아래
        // 곡률 평활 창 크기를 물리 거리 기준으로 산출하는 데 사용(웨이포인트 밀도가 소스마다
        // 크게 다를 수 있음).
        double total_path_length = 0.0;
        for (size_t i = 0; i + 1 < waypoints_.size(); ++i) {
            total_path_length += std::hypot(waypoints_[i + 1].x - waypoints_[i].x,
                                             waypoints_[i + 1].y - waypoints_[i].y);
        }
        if (waypoints_.size() > 1) {
            total_path_length += std::hypot(waypoints_.front().x - waypoints_.back().x,
                                             waypoints_.front().y - waypoints_.back().y);
        }
        avg_waypoint_spacing_ = waypoints_.empty() ? 0.36
                                                    : std::max(0.01, total_path_length / waypoints_.size());

        // 곡률 사전감속(1.5절)용 물리거리 창 평활 곡률 계산.
        //
        // wp.curvature(kappa_radpm)는 인접점 헤딩차분으로 산출되어, 웨이포인트가 촘촘할수록
        // 짧은 구간의 헤딩 노이즈가 증폭돼 개별 포인트 kappa가 실제 지속 곡률보다 훨씬 크게
        // 튈 수 있다. 사전감속이 "윈도우 내 최대 단일점 kappa"를 그대로 쓰면 노이즈 스파이크
        // 하나로 오프라인 최적화된 프로파일 속도보다 훨씬 낮게 순간 과잉감속된다. 물리거리
        // ±0.3m(총 0.6m) 창으로 |kappa| 평균을 내면 순간 노이즈는 눌리되 실제 지속 곡률(헤어핀
        // 등)은 거의 그대로 반영된다. 원본 wp.curvature 필드는 FF 조향(curvature_ff_blend_,
        // 기본 비활성) 등 다른 용도를 위해 그대로 둔다.
        {
            const double window_half_m = 0.3;
            const int half_n = std::max(1, static_cast<int>(std::round(window_half_m / avg_waypoint_spacing_)));
            const int n = static_cast<int>(waypoints_.size());
            for (int i = 0; i < n; ++i) {
                double sum = 0.0;
                int cnt = 0;
                for (int off = -half_n; off <= half_n; ++off) {
                    int idx = ((i + off) % n + n) % n;
                    sum += std::abs(waypoints_[idx].curvature);
                    ++cnt;
                }
                waypoints_[i].smoothed_curvature = (cnt > 0) ? (sum / cnt) : std::abs(waypoints_[i].curvature);
            }
        }

        RCLCPP_INFO(this->get_logger(), "🔄 플래닝 팀의 글로벌 경로 수신 완료! 웨이포인트 개수: %zu, 초기 인덱스: %zu", waypoints_.size(), last_target_idx_);
    }

    // 로컬 경로 콜백: 짧은 열린 전방 구간을 그대로 저장(닫힌 루프 아님).
    // wall_safety_margin 시프트는 적용하지 않는다(회피/추월 경로의 원본 기하 유지, 참조 컨트롤러와 동일).
    void local_path_callback(const f110_msgs::msg::WpntArray::ConstSharedPtr msg) {
        if (msg->wpnts.empty()) {
            local_waypoints_.clear(); // 빈 로컬 → 다음 사이클에 글로벌로 폴백
            return;
        }
        local_waypoints_.clear();
        local_waypoints_.reserve(msg->wpnts.size());
        for (const auto& wp : msg->wpnts) {
            Waypoint w;
            w.x = wp.x_m;
            w.y = wp.y_m;
            w.speed = wp.vx_mps;
            w.curvature = wp.kappa_radpm;
            w.smoothed_curvature = wp.kappa_radpm; // 로컬(짧은 회피경로)은 창 평활 미적용, 원본 그대로
            w.yaw = wp.psi_rad;
            local_waypoints_.push_back(w);
        }
        local_last_recv_time_ = this->now();
    }

    // GapFollower 기반 순수 LiDAR 회피 주행 계산·발행 (failsafe + 장애물 차단 폴백 공용).
    void publish_gap_follower(double dt) {
        double avoid_steering_angle = 0.0;
        double min_obstacle_dist = 999.0;
        gap_follower_->process_scan(latest_scan_, avoid_steering_angle, min_obstacle_dist);

        double final_steering_angle = avoid_steering_angle;
        const double steer_filter_alpha = 0.70;
        final_steering_angle = steer_filter_alpha * final_steering_angle + (1.0 - steer_filter_alpha) * last_steering_angle_;
        last_steering_angle_ = final_steering_angle;

        const double max_speed = 3.5;
        const double target_min_speed = 1.2;
        double speed_ratio = (min_obstacle_dist - 1.0) / (4.0 - 1.0);
        speed_ratio = std::max(0.0, std::min(1.0, speed_ratio));
        double final_speed = target_min_speed + speed_ratio * (max_speed - target_min_speed);

        double steer_ratio = std::abs(final_steering_angle) / 0.41;
        final_speed *= (1.0 - 0.50 * steer_ratio);

        double speed_error = final_speed - current_speed_;
        double cmd_speed = last_target_speed_;
        if (speed_error > 0.0) {
            cmd_speed += std::min(speed_error, base_max_accel_ * dt);
            if (cmd_speed > final_speed) cmd_speed = final_speed;
        } else {
            cmd_speed += std::max(speed_error, -base_max_decel_ * dt);
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
    }

    // L1 목표점 방향의 좁은 콘 안에서, 목표점보다 (margin 이상) 가깝고 절대 근접 임계 이내인
    // 물체가 잡히면 "경로가 막혔다"고 판단. 벽은 콘 밖(측면)이라 대체로 걸러지지만, 헤어핀/잘록
    // 구간에선 오검출 여지 → 파라미터(콘 각도/트리거 거리)로 튜닝.
    bool is_path_blocked(double L1_vec_x, double L1_vec_y, double L1_norm) const {
        if (!latest_scan_ || latest_scan_->ranges.empty() || L1_norm < 1e-3) return false;
        // 차량 프레임에서 L1 목표 방위각 (0 = 정면)
        double forward = std::cos(current_yaw_) * L1_vec_x + std::sin(current_yaw_) * L1_vec_y;
        double left    = -std::sin(current_yaw_) * L1_vec_x + std::cos(current_yaw_) * L1_vec_y;
        double bearing = std::atan2(left, forward);
        const auto& s = *latest_scan_;
        double min_r = std::numeric_limits<double>::max();
        for (size_t k = 0; k < s.ranges.size(); ++k) {
            double ang = s.angle_min + static_cast<double>(k) * s.angle_increment;
            double da = ang - bearing;
            while (da > PI) da -= 2.0 * PI;
            while (da < -PI) da += 2.0 * PI;
            if (std::abs(da) > obstacle_cone_halfangle_) continue;
            double r = s.ranges[k];
            if (std::isfinite(r) && r > 0.05 && r < min_r) min_r = r;
        }
        double trigger = std::min(L1_norm - obstacle_margin_, obstacle_trigger_dist_);
        return min_r < trigger;
    }

    void control_loop() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) dt = 0.02;
        last_time_ = current_time;

        // 0. 경로 소스 3-tier 중재: 로컬(신선) → 글로벌 → GapFollower(둘 다 없을 때만)
        bool local_fresh = !local_waypoints_.empty() &&
                           (current_time - local_last_recv_time_).seconds() < local_fresh_timeout_;
        bool global_avail = !waypoints_.empty();

        if (!local_fresh && !global_avail) {
            // 글로벌·로컬 둘 다 없을 시(초기/전체 소실) 순수 Lidar 기반 Gap Follower 동작 (failsafe)
            publish_gap_follower(dt);
            return;
        }

        // 활성 경로 선택: 신선한 로컬 우선(열린 구간), 없으면 글로벌(닫힌 루프)
        const std::vector<Waypoint>& wps = local_fresh ? local_waypoints_ : waypoints_;
        const bool closed = !local_fresh;

        // 1. 차량 위치 기준 최단 거리 인덱스 (closest_idx) 스캔
        size_t n = wps.size();
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;

        if (closed) {
            // 글로벌(닫힌 루프): 직전 인덱스 주변 윈도우 스캔 + 이탈 시 전역 재탐색
            //
            // 윈도우 크기는 고정 인덱스 개수가 아니라 물리 거리(후방 1m·전방 3m) 기준으로
            // 웨이포인트 밀도에 맞춰 동적 산출한다. 고정 개수였다면 웨이포인트 간격이 촘촘한
            // 소스에서 물리적 탐색 반경이 크게 줄어, 트랙이 스스로에게 가까워지는 구간(스타트/
            // 피니시 등)에서 윈도우가 진짜 최근접점을 놓치고 엉뚱한 인덱스에 잠길 수 있다.
            // 이때 min_dist가 fail-safe 임계(2.5m) 밑이면 전역 재탐색도 발동하지 않아
            // 인덱스가 역행/진동하며 조향 포화·속도 붕괴로 이어진다.
            const double spacing = std::max(0.01, avg_waypoint_spacing_);
            int back_count = std::max(2, static_cast<int>(std::ceil(1.0 / spacing)));
            int fwd_count = std::max(8, static_cast<int>(std::ceil(3.0 / spacing)));
            const int half_n = static_cast<int>(n / 2);
            back_count = std::min(back_count, half_n);
            fwd_count = std::min(fwd_count, half_n);

            closest_idx = last_target_idx_;
            for (int i = -back_count; i <= fwd_count; ++i) {
                size_t idx = (last_target_idx_ + i + n) % n;
                double dx = wps[idx].x - current_x_;
                double dy = wps[idx].y - current_y_;
                double dist = std::hypot(dx, dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_idx = idx;
                }
            }
            // Fail-safe recovery: 경로와 2.5m 초과하여 멀어지면 전체 탐색 (U턴 옆차선 점프 방지)
            if (min_dist > 2.5) {
                min_dist = std::numeric_limits<double>::max();
                for (size_t i = 0; i < n; ++i) {
                    double dx = wps[i].x - current_x_;
                    double dy = wps[i].y - current_y_;
                    double dist = std::hypot(dx, dy);
                    if (dist < min_dist) {
                        min_dist = dist;
                        closest_idx = i;
                    }
                }
            }
            last_target_idx_ = closest_idx;
        } else {
            // 로컬(짧은 열린 구간): 전체 최근접 스캔(~50점이라 저렴, wrap 인덱스 미사용)
            for (size_t i = 0; i < n; ++i) {
                double dx = wps[i].x - current_x_;
                double dy = wps[i].y - current_y_;
                double dist = std::hypot(dx, dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_idx = i;
                }
            }
        }
        double lateral_error = min_dist;

        // 1.5 곡률 룩어헤드 사전 감속 (Curvature Lookahead Pre-deceleration)
        // 속도비례 룩어헤드: 현재속도 제동거리(v^2/2a)만큼 전방 곡률을 미리 스캔.
        // 고정 2m는 고속 진입 시 감속 개시가 늦어 헤어핀 오버스피드 → 제동거리만큼 확장.
        double brake_dist = (current_speed_ * current_speed_) / (2.0 * std::max(0.1, base_max_decel_));
        double min_lookahead_dist = static_cast<double>(curvature_lookahead_count_) * 0.1; // 기존 고정값을 하한으로 유지
        double curv_lookahead_dist = std::max(min_lookahead_dist, brake_dist);

        // 프로파일 신뢰형 사전감속 (backward-pass): 오프라인 최적화된 프로파일 vx_mps는 이미
        // 각 지점의 최적 속도(코너 감속 램프 포함)를 담고 있다는 전제로, 전방 각 지점의 그립
        // 제한 목표속도 v_cap[i] = min(vx_profile[i], √(a_lat/κ_smoothed[i]))까지
        // base_max_decel로 감속 가능한 현재 최대 속도 v_reach = √(v_cap[i]² + 2·a_decel·d_i)의
        // 최소값을 사전감속 캡으로 쓴다(accum=0인 현재 위치 항이 순간 그립 클램프 역할도 겸함).
        // 직선·완만구간은 κ≈0 → v_cap=프로파일이라 안 눌리고, 코너는 제동거리만큼 앞에서부터
        // 정확히 그립속도로 선제동된다. (구 방식인 "창 내 최대 κ로 √(a_lat/κ) 블랭킷 재캡"은
        // 프로파일보다 낮은 속도로 전 구간을 과잉감속시켜 폐기 — 상세 비교는 CLAUDE.md 참고.)
        double curvature_speed_limit = std::numeric_limits<double>::max();
        double accum_curv_dist = 0.0;
        size_t curv_scan_idx = closest_idx;
        while (accum_curv_dist < curv_lookahead_dist) {
            double v_cap_i = wps[curv_scan_idx].speed;
            double k_i = std::abs(wps[curv_scan_idx].smoothed_curvature);
            if (k_i > 0.01) {
                v_cap_i = std::min(v_cap_i, std::sqrt(max_lateral_accel_ / k_i));
            }
            double v_reach = std::sqrt(v_cap_i * v_cap_i + 2.0 * base_max_decel_ * accum_curv_dist);
            if (v_reach < curvature_speed_limit) {
                curvature_speed_limit = v_reach;
            }
            size_t next_idx;
            if (closed) {
                next_idx = (curv_scan_idx + 1) % n;
            } else {
                if (curv_scan_idx + 1 >= n) break; // 열린 경로: 끝에 도달하면 종료
                next_idx = curv_scan_idx + 1;
            }
            double dx = wps[next_idx].x - wps[curv_scan_idx].x;
            double dy = wps[next_idx].y - wps[curv_scan_idx].y;
            accum_curv_dist += std::hypot(dx, dy);
            curv_scan_idx = next_idx;
            if (closed && curv_scan_idx == closest_idx) break; // 한바퀴 방지
        }
        curvature_speed_limit = std::max(min_speed_, curvature_speed_limit);

        // 2. L1 Guidance Distance 계산 및 L1 Point 스캔
        double L1_distance = l1_gain_ + current_speed_ * l1_distance_;
        double lower_bound = std::max(t_clip_min_, std::sqrt(2.0) * lateral_error);
        L1_distance = std::max(lower_bound, std::min(L1_distance, t_clip_max_));

        // closest_idx로부터 물리적으로 L1_distance만큼 전방에 위치한 목표 인덱스 스캔
        size_t idx_a = closest_idx;
        double accum_dist = 0.0;
        while (accum_dist < L1_distance) {
            size_t next_idx;
            if (closed) {
                next_idx = (idx_a + 1) % n;
            } else {
                if (idx_a + 1 >= n) break; // 열린 경로: 끝점을 L1 타깃으로 사용(뒤로 감기 방지)
                next_idx = idx_a + 1;
            }
            double dx = wps[next_idx].x - wps[idx_a].x;
            double dy = wps[next_idx].y - wps[idx_a].y;
            accum_dist += std::hypot(dx, dy);
            idx_a = next_idx;
            if (closed && idx_a == closest_idx) break; // 한바퀴 도는 것 방지
        }

        double L1_x = wps[idx_a].x;
        double L1_y = wps[idx_a].y;

        // 3. sin(eta) 직접 계산 (차량 헤딩과 L1 point 간의 횡방향 sin 오차)
        // asin() 후 sin() 호출은 항등식: sin(asin(x)) == x — 중간 삼각함수 2회를 제거
        double L1_vector_x = L1_x - current_x_;
        double L1_vector_y = L1_y - current_y_;
        double L1_norm = std::hypot(L1_vector_x, L1_vector_y);
        double sin_eta = 0.0;
        if (L1_norm > 1e-5) {
            double dot_prod = -std::sin(current_yaw_) * L1_vector_x + std::cos(current_yaw_) * L1_vector_y;
            sin_eta = std::max(-1.0, std::min(dot_prod / L1_norm, 1.0));
        }

        // 3.5 장애물 차단 감지 → GapFollower 회피 폴백
        // 로컬 회피경로(팀원 planner)가 아직 없을 때, 글로벌 라인이 장애물로 막히면 그대로 박으므로
        // L1 목표 방향이 근접 물체로 차단되면 GapFollower로 회피한다.
        // 로컬 추종 중(open=로컬 회피경로 존재)이면 상류 회피를 신뢰하고 이 폴백을 끈다.
        if (obstacle_avoid_enable_ && closed) {
            if (is_path_blocked(L1_vector_x, L1_vector_y, L1_norm)) {
                avoid_hold_counter_ = obstacle_avoid_hold_cycles_; // 차단 감지 → 홀드 재충전(채터링 방지)
            }
            if (avoid_hold_counter_ > 0) {
                avoid_hold_counter_--;
                publish_gap_follower(dt);
                return;
            }
        } else {
            avoid_hold_counter_ = 0; // 로컬 추종/비활성 시 홀드 리셋
        }

        // 4. 조향 속도 룩어헤드 예측 위치 기준 속도 (speed_for_lu) 결정
        double speed_la_for_lu = wps[find_lookahead_wp_idx(wps, closed, closest_idx, speed_lookahead_for_steering_)].speed;

        // 횡오차 정규화 및 가변 곡률 반영 속도
        double max_lat_e = 0.5;
        double min_lat_e = 0.01;
        double lat_e_clip = std::max(min_lat_e, std::min(lateral_error, max_lat_e));
        double lat_e_norm = 0.5 * ((lat_e_clip - min_lat_e) / (max_lat_e - min_lat_e));

        double curv_factor = std::max(0.0, std::min(2.0 * (mean_track_curvature_ / 0.8) - 2.0, 1.0));

        double speed_for_lu = speed_la_for_lu * (1.0 - lateral_error_coeff_ + lateral_error_coeff_ * std::exp(-lat_e_norm * 2.0 * curv_factor));

        // 5. 목표 횡가속도 및 Steer 룩업 테이블(LUT) 조향각 조회
        double lat_acc = 0.0;
        if (L1_distance > 0.0) {
            // speed_for_lu도 곡률 제한 적용
            speed_for_lu = std::min(speed_for_lu, curvature_speed_limit);
            lat_acc = 2.0 * std::pow(speed_for_lu, 2) / L1_distance * sin_eta;
        }
        double steering_angle = lookup_table_.lookup_steer_angle(lat_acc, speed_for_lu);

        // 6. 조향각 물리 및 가변 스케일러 보정 (Dynamic Scalers)
        // 1) 가감속 스케일링
        double acc_mean = 0.0;
        for (double a : acc_now_) acc_mean += a;
        acc_mean /= acc_now_.size();
        if (acc_mean >= 1.0) {
            steering_angle *= acceleration_scaler_for_steering_;
        } else if (acc_mean <= -1.0) {
            steering_angle *= deceleration_scaler_for_steering_;
        }

        // 2) 속도 스케일링
        double speed_diff = std::max(0.1, end_scale_speed_ - start_scale_speed_);
        double clip_factor = std::max(0.0, std::min((speed_for_lu - start_scale_speed_) / speed_diff, 1.0));
        double factor = 1.0 - clip_factor * downscale_factor_;
        steering_angle *= factor;

        // 3) 속도 비례 추가 튜닝
        steering_angle *= std::max(1.0, std::min(1.0 + (current_speed_ / 10.0), 1.4));

        // 3.5) 곡률 피드포워드 조향 보정 (Curvature Feedforward)
        // closest 웨이포인트의 곡률로부터 Ackermann 기하학 기반 피드포워드 조향각 산출
        double kappa_closest = wps[closest_idx].curvature;
        double steer_ff = std::atan(wheelbase_ * kappa_closest);
        // L1 조향과 피드포워드를 블렌딩 (curvature_ff_blend_ 비율만큼 피드포워드 혼합)
        steering_angle = (1.0 - curvature_ff_blend_) * steering_angle + curvature_ff_blend_ * steer_ff;

        // 3.7) Heading-error 댐핑 (Stanley형 정렬항)
        // 순수 L1 cross-track 제어는 횡오차 복구 시 경로 접선을 지나쳐 heading이 오버슈트(라인을
        // 비스듬히 가로질러 외벽 충돌)하는 약점이 있다. 경로 접선(psi)과 차량 헤딩의 정렬 오차에
        // 비례하는 보정을 더해 PD형 거동으로 만들어 오버슈트/진동을 억제한다.
        // 부호 검증: 정상 추종 시 (psi - yaw) 중앙값 ≈ 0, 좌측 정렬 필요 시 양수 → +조향(좌) 규약 일치.
        double path_heading = wps[closest_idx].yaw;
        double heading_err = path_heading - current_yaw_;
        while (heading_err > PI) heading_err -= 2.0 * PI;
        while (heading_err < -PI) heading_err += 2.0 * PI;
        steering_angle += heading_damping_gain_ * heading_err;

        // 3.8) 요레이트 피드백 카운터스티어 (횡슬립/언더스티어 보정)
        // 방금 확정한 명령 조향각이 기하학적으로 의도하는 기대 요레이트(v·tanδ/L) 대비
        // IMU 실측 요레이트의 오차에 비례해 조향을 보정한다. 언더스티어(실측<기대) 시
        // +방향으로 더 꺾어 슬립을 상쇄. rate limit·물리 클리핑 이전에 더해 보정분까지
        // 안전 한계(±0.41, rate 0.4) 안으로 함께 수렴시킨다. IMU 미장착 시(use_imu=false)
        // 무효. 저속(<0.5m/s) 특이점은 함수 내부에서 0으로 게이트됨.
        if (use_imu_) {
            steering_angle += stability_controller_->calculate_yaw_rate_correction(
                current_speed_, steering_angle, wheelbase_, yaw_rate_gain_);
        }

        // 4) Rate limit
        double threshold = 0.4;
        steering_angle = std::max(last_steering_angle_ - threshold, std::min(steering_angle, last_steering_angle_ + threshold));

        // 5) 물리 한계 적용
        steering_angle = std::max(-0.41, std::min(steering_angle, 0.41));
        last_steering_angle_ = steering_angle;

        // 7. 종방향 제어 명령 (Target Speed) 산출
        // 속도용 룩어헤드 예측
        double global_speed = wps[find_lookahead_wp_idx(wps, closed, closest_idx, speed_lookahead_)].speed;
        // 곡률 룩어헤드 제한 적용
        global_speed = std::min(global_speed, curvature_speed_limit);
        // 직선 최고속도 캡. 곡률 제한은 코너에서만 걸리므로(직선은 kappa~0 → 사실상 무제한)
        // 이 줄이 없으면 속도가 플래너 프로파일의 vx를 그대로 따라가 컨트롤러 쪽 상한이 없다.
        // 2026-07-19: 파라미터·런치 배선은 있는데 이 clamp만 빠져 있어 max_speed:=X가 무효였다
        // (실차 셰이크다운에서 "일단 천천히"가 통하지 않는 상태였음).
        global_speed = std::min(global_speed, max_speed_);
        double target_speed = global_speed * (1.0 - lateral_error_coeff_ + lateral_error_coeff_ * std::exp(-lat_e_norm * 2.0 * curv_factor));

        // 헤딩 에러 감속 보정 (speed_adjust_heading)
        double heading = current_yaw_;
        double map_heading = wps[closest_idx].yaw;
        double heading_error = std::abs(heading - map_heading);
        if (heading_error > PI) {
            heading_error = 2.0 * PI - heading_error;
        }
        if (heading_error >= PI / 9.0) { // 20도 이상
            double scaler = 0.5;
            if (heading_error < PI / 2.0) {
                scaler = 1.0 - 0.5 * (heading_error / (PI / 2.0));
            }
            target_speed *= scaler;
        }

        // 8. 롤링 가변 가감속 필터링 (ESC) 및 최종 구동 발행
        double roll_ratio = 0.0;
        if (use_imu_) {
            roll_ratio = stability_controller_->calculate_roll_ratio(max_roll_limit_);
        }
        double max_accel = base_max_accel_ * (1.0 - roll_ratio * decel_attenuation_);
        double max_decel = base_max_decel_ * (1.0 - roll_ratio * decel_attenuation_);

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

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";

        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = final_speed;
        drive_msg.drive.acceleration = (final_speed - current_speed_) / dt;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Pose: (%.2f, %.2f, %.2f) | Target WP: (%.2f, %.2f), Idx: %zu -> %zu | Steer: %.4f | Speed: %.2f / %.2f | L1_dist: %.2f",
            current_x_, current_y_, current_yaw_, L1_x, L1_y, closest_idx, idx_a, steering_angle, final_speed, current_speed_, L1_distance);

        drive_pub_->publish(drive_msg);
    }

    // 8.5 헬퍼: 룩어헤드 투영점 기준 최근접 웨이포인트 인덱스 반환
    size_t find_lookahead_wp_idx(const std::vector<Waypoint>& wps, bool closed, size_t base_idx, double lookahead_time) const {
        size_t nn = wps.size();
        double la_x = current_x_ + std::cos(current_yaw_) * current_speed_ * lookahead_time;
        double la_y = current_y_ + std::sin(current_yaw_) * current_speed_ * lookahead_time;
        size_t best_idx = base_idx;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = -5; i <= 15; ++i) {
            size_t idx;
            if (closed) {
                idx = (base_idx + static_cast<size_t>(i + static_cast<int>(nn))) % nn;
            } else {
                // 열린 경로: 인덱스를 [0, nn-1]로 clamp(뒤로 감기 방지)
                long t = static_cast<long>(base_idx) + i;
                if (t < 0) t = 0;
                if (t >= static_cast<long>(nn)) t = static_cast<long>(nn) - 1;
                idx = static_cast<size_t>(t);
            }
            double dist = std::hypot(wps[idx].x - la_x, wps[idx].y - la_y);
            if (dist < min_dist) {
                min_dist = dist;
                best_idx = idx;
            }
        }
        return best_idx;
    }

    // 9. 멤버 변수 선언
    double wheelbase_;

    // L1 Guidance
    double l1_gain_;
    double l1_distance_;
    double t_clip_min_;
    double t_clip_max_;
    double lateral_error_coeff_;
    double heading_damping_gain_;

    // Steer Scaling
    double acceleration_scaler_for_steering_;
    double deceleration_scaler_for_steering_;
    double start_scale_speed_;
    double end_scale_speed_;
    double downscale_factor_;

    // Speed lookaheads
    double speed_lookahead_;
    double speed_lookahead_for_steering_;

    // Stability
    double max_roll_limit_;
    double decel_attenuation_;
    double base_max_accel_;
    double base_max_decel_;
    bool use_imu_;
    double imu_angular_scale_;
    double yaw_rate_gain_;
    double max_speed_;
    double min_speed_;
    std::string odom_topic_;
    double wall_safety_margin_;

    // 곡률 룩어헤드 감속
    size_t curvature_lookahead_count_;
    double max_lateral_accel_;
    double curvature_ff_blend_;

    // IMU Rolling Buffer
    std::vector<double> acc_now_;

    // 룩업 테이블
    SteeringLookupTable lookup_table_;

    // 차량 상태
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double current_speed_ = 0.0;

    double last_target_speed_ = 0.0;
    double last_steering_angle_ = 0.0;
    rclcpp::Time last_time_;

    size_t last_target_idx_ = 0;
    bool waypoints_initialized_ = false; // 최초 global_waypoints 수신 여부(재발행 시 인덱스 오초기화 방지)
    double avg_waypoint_spacing_ = 0.36; // global_path_callback에서 실측 갱신, 수신 전 보수적 기본값

    std::vector<Waypoint> waypoints_;          // 글로벌 경로 (닫힌 루프)
    double mean_track_curvature_ = 0.0;

    // 로컬 경로 (짧은 열린 구간, 회피/추월 포함). 신선하면 글로벌보다 우선.
    std::vector<Waypoint> local_waypoints_;
    rclcpp::Time local_last_recv_time_;
    double local_fresh_timeout_ = 0.3;         // 이 시간(s) 넘게 로컬 미수신 시 글로벌로 폴백

    // 장애물 차단 시 GapFollower 회피 폴백 (글로벌 추종 중, 로컬 회피경로 없을 때)
    bool obstacle_avoid_enable_ = false;
    double obstacle_cone_halfangle_ = 0.14;    // L1 방향 콘 반각 [rad] (~8도)
    double obstacle_trigger_dist_ = 1.5;       // 이 거리[m] 이내 근접 시 차단 판정
    double obstacle_margin_ = 0.3;             // 목표점 거리 대비 최소 여유[m]
    int obstacle_avoid_hold_cycles_ = 15;      // 회피 유지 사이클(50Hz→0.3s), 채터링 방지
    int avoid_hold_counter_ = 0;

    std::unique_ptr<GapFollower> gap_follower_;
    std::unique_ptr<StabilityController> stability_controller_;

    // ROS 2 통신
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_path_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr local_path_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_ = nullptr;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
