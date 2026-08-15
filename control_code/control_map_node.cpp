#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <string>
#include <tuple>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "f1tenth_control/types.hpp"
#include "f1tenth_control/steering_lookup_table.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "f110_msgs/msg/state_machine.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace f1tenth_control;

namespace {

constexpr double MAX_STEERING_ANGLE = 0.41;

// 전 구간 최근접 웨이포인트 스캔. 반환 {최단거리, 인덱스}.
std::pair<double, size_t> scan_closest(const std::vector<Waypoint>& wps, double x, double y) {
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    for (size_t i = 0; i < wps.size(); ++i) {
        double dist = std::hypot(wps[i].x - x, wps[i].y - y);
        if (dist < min_dist) { min_dist = dist; closest_idx = i; }
    }
    return {min_dist, closest_idx};
}

inline double wrap_pi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

std::tuple<double, size_t, bool> scan_closest_heading_gated(
    const std::vector<Waypoint>& wps, double x, double y, double yaw, double max_heading_err) {
    if (max_heading_err <= 0.0) {   // 0이면 게이트 비활성 = 구 거동
        auto [d, i] = scan_closest(wps, x, y);
        return {d, i, false};
    }
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    bool found = false;
    for (size_t i = 0; i < wps.size(); ++i) {
        if (std::abs(wrap_pi(wps[i].yaw - yaw)) > max_heading_err) continue;
        double dist = std::hypot(wps[i].x - x, wps[i].y - y);
        if (dist < min_dist) { min_dist = dist; closest_idx = i; found = true; }
    }
    if (found) return {min_dist, closest_idx, true};
    auto [d, i] = scan_closest(wps, x, y);
    return {d, i, false};
}

// start_idx에서 경로를 따라 호 길이 max_dist만큼 전진하며 각 웨이포인트를 방문한다.
// visit(idx, accum_dist)가 false면 중단. 닫힌 경로는 한 바퀴에서, 열린 경로는 끝점에서 멈춘다.
// 반환값은 마지막으로 도달한 인덱스. (곡률 사전감속 / L1 목표점 탐색 공용)
template <typename Visitor>
size_t walk_forward(const std::vector<Waypoint>& wps, size_t start_idx,
                    double max_dist, bool closed, Visitor&& visit) {
    const size_t n = wps.size();
    if (n == 0) return start_idx;

    size_t idx = start_idx;
    double accum = 0.0;
    while (accum < max_dist) {
        if (!visit(idx, accum)) break;

        size_t next_idx;
        if (closed) {
            next_idx = (idx + 1) % n;
        } else {
            if (idx + 1 >= n) break;  // 열린 경로: 끝점에서 종료(뒤로 감기 방지)
            next_idx = idx + 1;
        }
        accum += std::hypot(wps[next_idx].x - wps[idx].x, wps[next_idx].y - wps[idx].y);
        idx = next_idx;
        if (closed && idx == start_idx) break;
    }
    return idx;
}

void smooth_curvature(std::vector<Waypoint>& wps, bool closed, double window_half_m = 0.3) {
    const int n = static_cast<int>(wps.size());
    if (n < 2) return;

    double total_len = 0.0;
    for (int i = 1; i < n; ++i) {
        total_len += std::hypot(wps[i].x - wps[i - 1].x, wps[i].y - wps[i - 1].y);
    }
    const double avg_spacing = total_len / static_cast<double>(n - 1);
    if (avg_spacing <= 1e-6) return;

    const int half_n = std::max(1, static_cast<int>(std::round(window_half_m / avg_spacing)));
    std::vector<double> smoothed(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        double sum = 0.0;
        int cnt = 0;
        for (int off = -half_n; off <= half_n; ++off) {
            int idx = i + off;
            if (closed) {
                idx = ((idx % n) + n) % n;
            } else {
                if (idx < 0 || idx >= n) continue;  // 열린 경로: 창을 배열 안으로 자른다
            }
            sum += std::abs(wps[static_cast<size_t>(idx)].curvature);
            ++cnt;
        }
        smoothed[static_cast<size_t>(i)] =
            (cnt > 0) ? (sum / cnt) : std::abs(wps[static_cast<size_t>(i)].curvature);
    }
    for (int i = 0; i < n; ++i) {
        wps[static_cast<size_t>(i)].smoothed_curvature = smoothed[static_cast<size_t>(i)];
    }
}

}  // namespace

class ControlMapNode : public rclcpp::Node {
public:
    ControlMapNode() : Node("control_map_node") {
        // ── 1. 파라미터 (전부 생성자에서 1회만 읽음 — 콜백 없음, 변경하려면 노드 재시작) ──
        wheelbase_ = declare_parameter<double>("wheelbase", 0.33);

        l1_offset_ = declare_parameter<double>("l1_offset", 0.5);
        l1_speed_gain_ = declare_parameter<double>("l1_speed_gain", 0.3);
        t_clip_min_ = declare_parameter<double>("t_clip_min", 0.8);
        t_clip_max_ = declare_parameter<double>("t_clip_max", 5.0);
        // L1 횡가속 분모의 하한 [m]. ⚠️ 예전엔 t_clip_min을 그대로 재사용했는데, t_clip_min은
        // **룩어헤드 튜닝 노브**라 그걸 낮추면 횡가속 명령 상한이 조용히 올라갔다
        // (0.6 → 6 m/s에서 최대 lat_acc 120 m/s²). 발산 방지용 수치 하한은 따로 둔다.
        l1_min_denom_ = std::max(0.05, declare_parameter<double>("l1_min_denom", 0.6));
        // Stanley형 heading 정렬항. 0이면 순수 L1(시뮬 검증 상태). 실차 튜닝용으로만 보존.
        heading_damping_gain_ = declare_parameter<double>("heading_damping_gain", 0.0);

        // 조향각 가변 스케일러
        acceleration_scaler_for_steering_ =
            declare_parameter<double>("acceleration_scaler_for_steering", 1.0);
        deceleration_scaler_for_steering_ =
            declare_parameter<double>("deceleration_scaler_for_steering", 0.95);
        steering_scaler_accel_ref_ =
            std::max(0.05, declare_parameter<double>("steering_scaler_accel_ref", 1.0));
        start_scale_speed_ = declare_parameter<double>("start_scale_speed", 7.0);
        end_scale_speed_ = declare_parameter<double>("end_scale_speed", 8.0);
        downscale_factor_ = declare_parameter<double>("downscale_factor", 0.10);

        steering_reach_ratio_ =
            std::clamp(declare_parameter<double>("steering_reach_ratio", 0.74), 0.3, 1.0);

        speed_lookahead_ = declare_parameter<double>("speed_lookahead", 0.15);
        speed_lookahead_for_steering_ =
            declare_parameter<double>("speed_lookahead_for_steering", 0.0);
        std::string lut_file = declare_parameter<std::string>("lookup_table_file", "");

        base_max_accel_ = declare_parameter<double>("base_max_accel", 4.0);
        base_max_decel_ = declare_parameter<double>("base_max_decel", 8.0);
        prebrake_decel_ = declare_parameter<double>("prebrake_decel", 1.5);

        ramp_lead_max_ = declare_parameter<double>("ramp_lead_max", 2.4);

        // 런치 킥(자율 정지출발 시 센서리스 데드존 관통) — control_loop 8-c
        launch_boost_enable_ = declare_parameter<bool>("launch_boost_enable", true);
        launch_boost_speed_ = declare_parameter<double>("launch_boost_speed", 2.2);
        launch_boost_time_ = declare_parameter<double>("launch_boost_time", 0.6);
        launch_exit_speed_ = declare_parameter<double>("launch_exit_speed", 0.8);
        launch_standstill_speed_ = declare_parameter<double>("launch_standstill_speed", 0.3);

        use_imu_ = declare_parameter<bool>("use_imu", true);
        imu_linear_scale_ = declare_parameter<double>("imu_linear_scale", 1.0);
        imu_angular_scale_ = declare_parameter<double>("imu_angular_scale", 1.0);

        steering_trim_gain_ =
            std::max(0.0, declare_parameter<double>("steering_trim_adapt_gain", 0.0));
        steering_trim_limit_ =
            std::clamp(declare_parameter<double>("steering_trim_limit", 0.06), 0.0, 0.15);
        steering_trim_max_steer_ =
            std::max(0.01, declare_parameter<double>("steering_trim_max_steer", 0.10));
        steering_trim_min_speed_ =
            std::max(0.5, declare_parameter<double>("steering_trim_min_speed", 2.0));
        steering_trim_max_lat_acc_ =
            std::max(0.1, declare_parameter<double>("steering_trim_max_lat_acc", 2.0));
        steering_trim_lag_ =
            std::clamp(declare_parameter<double>("steering_trim_lag", 0.14), 0.0, 0.5);

        max_speed_ = declare_parameter<double>("max_speed", 12.0);
        min_speed_ = declare_parameter<double>("min_speed", 2.0);

        // 곡률 룩어헤드 사전감속
        curvature_lookahead_count_ =
            static_cast<size_t>(declare_parameter<int>("curvature_lookahead_count", 60));
        max_lateral_accel_ = declare_parameter<double>("max_lateral_accel", 6.0);
        understeer_gradient_ = declare_parameter<double>("understeer_gradient", 0.019);
        // δ_max 중 곡률 추종에 배정할 비율. 나머지는 횡오차 보정·요레이트 피드백 여유.
        steer_authority_ratio_ = declare_parameter<double>("steer_authority_ratio", 0.85);
        curvature_ff_blend_ = declare_parameter<double>("curvature_ff_blend", 0.0);
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");

        sector_scale_enable_ = declare_parameter<bool>("sector_scale_enable", false);
        sector_scale_topic_ = declare_parameter<std::string>("sector_scale_topic", "/sector_scales");
        sector_scale_max_ = declare_parameter<double>("sector_scale_max", 1.5);
        sector_scale_blend_ = declare_parameter<double>("sector_scale_blend", 0.5);
        sector_scale_track_len_tol_ = declare_parameter<double>("sector_scale_track_len_tol", 0.02);
        // 회피/추월 중에는 스케일을 끄고 보수적 전역 MLA로 돌아간다. scale의 근거인 벽 여유는
        // **차가 라인 위에 있을 때** 잰 값이라, 라인에서 0.5 m 밀려나면 그 여유가 성립하지 않는다.
        // (0810 섹터2: 여유 p5 0.453인데 벽 쪽으로 0.4 m 밀리면 0.05가 된다)
        sector_scale_global_only_ = declare_parameter<bool>("sector_scale_global_only", true);
        sector_scale_state_topic_ = declare_parameter<std::string>("sector_scale_state_topic", "/state");
        sector_scale_state_timeout_ = declare_parameter<double>("sector_scale_state_timeout", 1.0);
        sector_scale_timeout_ = declare_parameter<double>("sector_scale_timeout", 3.0);

        l1_use_actual_distance_ = declare_parameter<bool>("l1_use_actual_distance", true);

        steering_speed_cap_measured_ =
            declare_parameter<bool>("steering_speed_cap_measured", true);

        // 상태 한 줄 로그 주기 [ms]. 0이면 끈다. 예전엔 500ms 고정이라 실차 로그의 61%가
        // 이 줄이었다(0807 로그 1007줄 중 617줄 / 317초).
        const auto status_log_param = declare_parameter<int>("status_log_period_ms", 2000);
        status_log_period_ms_ = static_cast<int>(status_log_param > 0 ? status_log_param : 0);

        // L1 목표점이 차량보다 이만큼 더 많이 튀면 진단 경고 + 카운트. 0이면 검출 끔.
        // 주행 개입은 전혀 없다(순수 관측). 상세는 control_loop의 검출부 주석 참고.
        l1_jump_warn_m_ = declare_parameter<double>("l1_jump_warn_m", 1.0);

        max_steering_rate_ = std::max(0.5, declare_parameter<double>("max_steering_rate", 20.0));
        pose_lpf_alpha_ = std::clamp(declare_parameter<double>("pose_lpf_alpha", 0.30), 0.01, 1.0);

        // 좌우 조향 한계. 둘 다 같으면 기존 대칭 거동과 100% 동일.
        max_steering_left_ =
            std::max(0.05, std::abs(declare_parameter<double>("max_steering_left", MAX_STEERING_ANGLE)));
        max_steering_right_ =
            std::max(0.05, std::abs(declare_parameter<double>("max_steering_right", MAX_STEERING_ANGLE)));
        // 조향 권한 캡·갭팔로워는 **작은 쪽**을 쓴다 — 큰 쪽을 쓰면 우선회에서 8% 낙관이 된다.
        steer_limit_min_ = std::min(max_steering_left_, max_steering_right_);
        if (std::abs(max_steering_left_ - max_steering_right_) > 1e-6) {
            RCLCPP_WARN(this->get_logger(),
                "조향 한계 좌우 비대칭: 좌 %.3f / 우 %.3f rad — 속도 캡은 작은 쪽(%.3f)을 사용. "
                "근본 해결은 서보 트림 재정렬(링키지)임",
                max_steering_left_, max_steering_right_, steer_limit_min_);
        }

        engage_gate_enable_ = declare_parameter<bool>("engage_gate_enable", true);
        drive_mode_topic_ = declare_parameter<std::string>("drive_mode_topic", "/drive_mode");
        engaged_mode_value_ = declare_parameter<std::string>("engaged_mode_value", "autonomous");
        drive_mode_timeout_ = declare_parameter<double>("drive_mode_timeout", 1.0);

        // 경로 소스 중재
        local_fresh_timeout_ = declare_parameter<double>("local_fresh_timeout", 0.3);

        closest_idx_max_heading_err_ =
            declare_parameter<double>("closest_idx_max_heading_err", 1.40);

        acc_now_ = std::vector<double>(10, 0.0);

        bool loaded = !lut_file.empty() && lookup_table_.load(lut_file);
        for (const char* pkg : {"f1tenth_control", "steering_lookup"}) {
            if (loaded) break;
            try {
                lut_file = ament_index_cpp::get_package_share_directory(pkg)
                           + "/cfg/LUT_calibrated.csv";
                loaded = lookup_table_.load(lut_file);
            } catch (...) {}
        }
        if (!loaded) {
            RCLCPP_ERROR(this->get_logger(),
                "❌ 모든 경로에서 룩업 테이블(LUT) 로드 실패! 조향각이 0.0으로 고정됩니다.");
        } else {
            RCLCPP_INFO(this->get_logger(), "🟢 룩업 테이블(LUT) 로드 성공: %s", lut_file.c_str());
        }

        // ── 3. 통신 채널 ──
        // 글로벌은 latched(transient_local), 로컬은 퍼블리셔에 맞춰 volatile.
        global_path_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/global_waypoints", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(&ControlMapNode::global_path_callback, this, std::placeholders::_1));
        local_path_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/local_waypoints", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            std::bind(&ControlMapNode::local_path_callback, this, std::placeholders::_1));
        local_last_recv_time_ = this->now();  // 노드 클럭 타입으로 초기화(clock mismatch 방지)

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&ControlMapNode::odom_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&ControlMapNode::imu_callback, this, std::placeholders::_1));

        // 자율 체결 상태(실차 f1tenth_stack drive_mode_manager가 estop/manual/autonomous 발행).
        if (engage_gate_enable_) {
            drive_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
                drive_mode_topic_, 10,
                std::bind(&ControlMapNode::drive_mode_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),
                        "engage 게이트 활성 — %s == \"%s\"일 때만 속도 램프 진행 "
                        "(미수신 %.1fs 초과 시 자동 비활성)",
                        drive_mode_topic_.c_str(), engaged_mode_value_.c_str(), drive_mode_timeout_);
        }
        drive_mode_last_recv_time_ = this->now();
        sector_state_last_recv_time_ = this->now();   // 노드 클럭 타입으로 초기화
        sector_table_last_recv_time_ = this->now();

        // 섹터 스케일 구독. transient_local이라 컨트롤러가 늦게 떠도 마지막 테이블을 받는다.
        if (sector_scale_enable_) {
            sector_scale_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                sector_scale_topic_,
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
                std::bind(&ControlMapNode::sector_scale_callback, this, std::placeholders::_1));
            if (sector_scale_global_only_) {
                sector_state_sub_ = this->create_subscription<f110_msgs::msg::StateMachine>(
                    sector_scale_state_topic_, 10,
                    [this](const f110_msgs::msg::StateMachine::SharedPtr msg) {
                        const bool was = sector_on_global_ && sector_state_seen_;
                        sector_on_global_ = (msg->state == f110_msgs::msg::StateMachine::STATE_GLOBAL);
                        sector_state_last_recv_time_ = this->now();
                        sector_state_seen_ = true;
                        if (was != sector_on_global_) {
                            // 상태가 바뀌면 이미 받아둔 경로의 mla를 즉시 다시 해소해야 한다 —
                            // 안 그러면 회피에 들어갔는데 다음 경로 메시지까지 옛 스케일이 남는다.
                            apply_sector_scales(waypoints_);
                            apply_sector_scales(local_waypoints_);
                        }
                    });
            }
            // 테이블 데드맨 감시. 스케일은 **수신 콜백에서 웨이포인트에 구워 두는** 구조라
            // (50 Hz 루프를 가볍게 유지하려는 설계), 신선도가 끊긴 순간 누가 다시 굽지 않으면
            // 옛 스케일이 배열에 그대로 남는다. 그래서 전이를 감시해 한 번만 되굽는다.
            if (sector_scale_timeout_ > 0.0) {
                sector_deadman_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(500), [this]() {
                        // ⚠️ 테이블을 **한 번도 못 받은** 상태는 "끊김"이 아니다(발행자가 아직
                        //    안 떴을 뿐). 이걸 안 거르면 기동 직후 매번 가짜 경고가 뜬다.
                        if (!sector_table_seen_) return;
                        const bool fresh = sector_table_fresh();
                        if (fresh == sector_table_fresh_prev_) return;
                        sector_table_fresh_prev_ = fresh;
                        if (!fresh) {
                            apply_sector_scales(waypoints_);
                            apply_sector_scales(local_waypoints_);
                            RCLCPP_WARN(this->get_logger(),
                                "🔴 섹터 테이블이 %.1fs 넘게 끊겼다 — 전역 MLA %.2f로 복귀. "
                                "발행자(sector_pub/sector_learner)가 죽었는지 확인할 것",
                                sector_scale_timeout_, max_lateral_accel_);
                        }
                    });
            }
            RCLCPP_INFO(this->get_logger(),
                        "섹터 횡가속 스케일 활성 — %s 구독 (scale ∈ [1.0, %.2f], 블렌딩 %.2f m, "
                        "전역 MLA %.2f, %s, 데드맨 %s)",
                        sector_scale_topic_.c_str(), sector_scale_max_, sector_scale_blend_,
                        max_lateral_accel_,
                        sector_scale_global_only_
                            ? "회피/추월 중 자동 1.0 복귀" : "⚠️ 회피 중에도 적용(global_only=false)",
                        sector_scale_timeout_ > 0.0
                            ? (std::to_string(sector_scale_timeout_).substr(0, 4) + "s").c_str()
                            : "꺼짐(테이블 영구 유지)");
        }

        // 조향 트림 자동 보상 설정을 기동 시 1회 남긴다 — 나중에 로그만 보고 "그때 켜져
        // 있었나 / 단위 계수가 맞았나"를 확인할 수 있어야 한다. imu_angular_scale이
        // 1.0으로 남아 있으면(real인데 pi/180이 아니면) 요레이트가 57.3배가 되므로 경고한다.
        if (steering_trim_gain_ > 0.0) {
            RCLCPP_INFO(this->get_logger(),
                        "조향 트림 자동 보상 활성 — gain %.2f (τ=%.1fs), 한계 ±%.2f°, "
                        "학습게이트: v≥%.1f m/s, |δ|≤%.1f°, |a_lat|≤%.1f m/s², lag %.0f ms | "
                        "imu_angular_scale %.6f",
                        steering_trim_gain_, 1.0 / steering_trim_gain_,
                        steering_trim_limit_ * 180.0 / M_PI, steering_trim_min_speed_,
                        steering_trim_max_steer_ * 180.0 / M_PI, steering_trim_max_lat_acc_,
                        steering_trim_lag_ * 1000.0, imu_angular_scale_);
            if (std::abs(imu_angular_scale_ - 1.0) < 1e-6) {
                RCLCPP_WARN(this->get_logger(),
                    "⚠️ imu_angular_scale=1.0 — 시뮬(sim_imu_bridge_node)이면 정상이지만 "
                    "실차면 VESC가 deg/s로 발행하므로 요레이트가 57.3배가 된다. "
                    "그 경우 트림이 즉시 한계(±%.1f°)에 붙는다.",
                    steering_trim_limit_ * 180.0 / M_PI);
            }
        }

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_autonomous", 10);
        l1_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/debug/l1_lookahead", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&ControlMapNode::control_loop, this));

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "RoboRacer L1 Guidance & Steer LUT 제어 노드가 시작되었습니다.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        // 웨이포인트와 pose가 같은 프레임에서 직접 뺄셈되므로 L1 마커도 이 프레임에 그린다.
        if (!msg->header.frame_id.empty()) odom_frame_ = msg->header.frame_id;

        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        const auto q = msg->pose.pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        const double v = msg->twist.twist.linear.x;

        // ⚠️ NaN/Inf 게이트. MCL이 붕괴하면 pose에 NaN이 실려 오고, 그게 들어오면 L1 기하부터
        //    조향·속도까지 전 파이프라인이 NaN이 되어 **NaN 조향각이 그대로 발행**된다.
        //    받아들이지 않고 버린다.
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) || !std::isfinite(v)) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "odom에 비유한값(NaN/Inf) 수신 — 이 샘플을 버린다(위치추정 붕괴 의심)");
            return;
        }

        if (!odom_seen_) {
            current_x_ = x;
            current_y_ = y;
            current_yaw_ = yaw;
            current_speed_ = v;
            odom_seen_ = true;
        } else {
            const double a = pose_lpf_alpha_;
            current_x_ = (1.0 - a) * current_x_ + a * x;
            current_y_ = (1.0 - a) * current_y_ + a * y;
            const double dyaw = wrap_pi(yaw - current_yaw_);
            current_yaw_ = wrap_pi(current_yaw_ + a * dyaw);
            current_speed_ = (1.0 - a) * current_speed_ + a * v;
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        // use_imu=false는 "IMU를 신뢰하지 않는다"는 뜻이므로 파생값을 전부 쓰지 않는다.
        // acc_now_는 0 초기화 상태로 남아 acc_mean=0 → 스케일러 중립(1.0)으로 안전히 떨어진다.
        if (!use_imu_) return;

        std::rotate(acc_now_.rbegin(), acc_now_.rbegin() + 1, acc_now_.rend());
        acc_now_[0] = -msg->linear_acceleration.x * imu_linear_scale_;

        yaw_rate_now_ = msg->angular_velocity.z * imu_angular_scale_;
        yaw_rate_last_recv_ = this->now();
        yaw_rate_seen_ = true;
    }

    void drive_mode_callback(const std_msgs::msg::String::ConstSharedPtr msg) {
        bool engaged = (msg->data == engaged_mode_value_);
        if (engaged != is_engaged_) {
            RCLCPP_INFO(this->get_logger(), "자율 체결 상태 변경: %s → %s (램프 %s)",
                        is_engaged_ ? "체결" : "미체결", engaged ? "체결" : "미체결",
                        engaged ? "진행" : "실측 속도로 고정");
        }
        is_engaged_ = engaged;
        drive_mode_last_recv_time_ = this->now();
        drive_mode_seen_ = true;
    }

    // 곡률 추종에 쓸 수 있는 **실제 도달** 조향각 [rad].
    //   좌우 중 작은 한계 × 도달각 비율 × 곡률 추종 배정 비율.
    // 나머지(1 − steer_authority_ratio)는 횡오차 보정·요레이트 피드백 여유로 남긴다.
    double steer_avail() const {
        return steer_authority_ratio_ * steer_limit_min_ * steering_reach_ratio_;
    }

    void update_steering_trim(double dt, double published) {
        // 발행 이력은 게이트와 무관하게 항상 쌓는다(게이트가 열린 순간 lag만큼 과거가 필요).
        const double tnow = this->now().seconds();
        steer_hist_.emplace_back(tnow, published);
        while (steer_hist_.size() > 2 && tnow - steer_hist_.front().first > steering_trim_lag_ + 0.3)
            steer_hist_.pop_front();

        if (steering_trim_gain_ <= 0.0 || !use_imu_ || !yaw_rate_seen_) return;
        if (!is_engaged_ && engage_gate_active()) return;
        if (launch_active_) return;
        const double v = std::abs(current_speed_);
        if (v < steering_trim_min_speed_) return;
        if ((this->now() - yaw_rate_last_recv_).seconds() > 0.2) return;
        if (std::abs(published) > steering_trim_max_steer_) return;
        if (std::abs(v * yaw_rate_now_) > steering_trim_max_lat_acc_) return;

        // lag만큼 과거의 발행값 (선형보간). 이력이 아직 짧으면 학습을 미룬다.
        const double t_target = tnow - steering_trim_lag_;
        if (steer_hist_.front().first > t_target) return;
        double past = steer_hist_.back().second;
        for (size_t i = 1; i < steer_hist_.size(); ++i) {
            if (steer_hist_[i].first >= t_target) {
                const auto &a = steer_hist_[i - 1], &b = steer_hist_[i];
                const double w = (b.first > a.first) ? (t_target - a.first) / (b.first - a.first) : 0.0;
                past = a.second + w * (b.second - a.second);
                break;
            }
        }

        // 실측 요레이트가 함의하는 바퀴각 → 명령 공간으로 환산.
        const double delta_wheel = yaw_rate_now_ * (wheelbase_ / v + understeer_gradient_ * v);
        const double e = past - delta_wheel / std::max(0.3, steering_reach_ratio_);
        steering_trim_ += steering_trim_gain_ * (e - steering_trim_) * dt;
        steering_trim_ = std::clamp(steering_trim_, -steering_trim_limit_, steering_trim_limit_);
        steering_trim_samples_++;
    }

    // engage 게이트가 지금 실제로 작동 중인가. /drive_mode를 한 번도 못 받았거나 timeout 넘게
    // 끊겼으면 false(= 구 거동 유지).
    bool engage_gate_active() const {
        if (!engage_gate_enable_ || !drive_mode_seen_) return false;
        return (this->now() - drive_mode_last_recv_time_).seconds() < drive_mode_timeout_;
    }

    void global_path_callback(const f110_msgs::msg::WpntArray::ConstSharedPtr msg) {
        if (msg->wpnts.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty global waypoints.");
            return;
        }

        // 플래너는 동일 경로를 주기적으로 재발행할 수 있다 → 최초 수신 여부를 먼저 기록해
        // 아래 인덱스 재초기화를 최초 1회로 제한한다.
        const bool first_reception = !waypoints_initialized_;

        waypoints_.clear();
        waypoints_.reserve(msg->wpnts.size());
        for (const auto& wp : msg->wpnts) {
            Waypoint w;
            w.x = wp.x_m;
            w.y = wp.y_m;
            w.speed = wp.vx_mps;
            w.curvature = wp.kappa_radpm;
            w.yaw = wp.psi_rad;
            w.s = wp.s_m;   // Frenet 호길이(장애물 감속의 에고 s 기준)

            waypoints_.push_back(w);
        }

        // ⚠️ 매 재발행마다 전체 재탐색하면 스타트/피니시처럼 유클리드 거리는 가깝지만 인덱스는
        //    트랙 반대편인 구간에서 엉뚱한 인덱스로 스냅되어 조향 포화·속도 붕괴로 이어진다.
        //    최초 수신 이후엔 control_loop이 매 사이클 윈도우 탐색으로 계속 추적한다.
        if (first_reception || last_target_idx_ >= waypoints_.size()) {
            last_target_idx_ = scan_closest(waypoints_, current_x_, current_y_).second;
            waypoints_initialized_ = true;
        }

        // 평균 웨이포인트 간격 — 인덱스 윈도우/곡률 평활 창을 물리 거리 기준으로 잡는 데 쓴다
        // (웨이포인트 밀도가 소스마다 크게 다를 수 있음).
        double total_path_length = 0.0;
        for (size_t i = 0; i + 1 < waypoints_.size(); ++i) {
            total_path_length += std::hypot(waypoints_[i + 1].x - waypoints_[i].x,
                                            waypoints_[i + 1].y - waypoints_[i].y);
        }
        if (waypoints_.size() > 1) {
            total_path_length += std::hypot(waypoints_.front().x - waypoints_.back().x,
                                            waypoints_.front().y - waypoints_.back().y);
        }
        avg_waypoint_spacing_ =
            waypoints_.empty() ? 0.36 : std::max(0.01, total_path_length / waypoints_.size());

        smooth_curvature(waypoints_, /*closed=*/true);

        // 섹터 테이블이 이 라인 기준인지 확인한 뒤 mla를 해소한다. 순서가 중요하다 —
        // 검증 결과(sector_len_ok_)가 apply_sector_scales의 활성 조건에 들어간다.
        validate_sector_track_length(total_path_length);
        apply_sector_scales(waypoints_);

        // 플래너가 **같은 경로를 2초마다 재발행**하므로 매번 찍으면 로그의 16%가 이 줄이 된다
        // (0807 실차 로그 1007줄 중 158줄). 내용이 실제로 바뀐 경우에만 찍는다.
        const size_t path_sig = waypoints_.size() ^
            (std::hash<double>{}(waypoints_.front().x) << 1) ^
            (std::hash<double>{}(waypoints_.back().y) << 2) ^
            (std::hash<double>{}(total_path_length) << 3);
        if (path_sig != last_global_sig_) {
            last_global_sig_ = path_sig;
            RCLCPP_INFO(this->get_logger(), "🔄 글로벌 경로 수신! 웨이포인트 %zu개, 길이 %.2f m, 초기 인덱스 %zu",
                        waypoints_.size(), total_path_length, last_target_idx_);
        }
    }

    void local_path_callback(const f110_msgs::msg::WpntArray::ConstSharedPtr msg) {
        if (msg->wpnts.empty()) {
            local_waypoints_.clear();   // 빈 로컬 → 다음 사이클에 글로벌로 폴백
            local_is_closed_ = false;
            return;
        }
        const size_t prev_size = local_waypoints_.size();
        local_waypoints_.clear();
        local_waypoints_.reserve(msg->wpnts.size());
        for (const auto& wp : msg->wpnts) {
            Waypoint w;
            w.x = wp.x_m;
            w.y = wp.y_m;
            w.speed = wp.vx_mps;
            w.curvature = wp.kappa_radpm;
            w.smoothed_curvature = wp.kappa_radpm;   // 아래 smooth_curvature가 덮어씀
            w.yaw = wp.psi_rad;
            w.s = wp.s_m;
            local_waypoints_.push_back(w);
        }

        // 닫힘 판정: 끝점→시작점 간격이 평균 간격의 2배 이내면 닫힌 루프. 짧은 회피 세그먼트는
        // 양 끝이 경로 길이만큼 떨어져 있어 확실히 구분된다.
        const size_t n = local_waypoints_.size();
        local_is_closed_ = false;

        // 총 길이 (닫힘 판정용).
        double total_len = 0.0;
        for (size_t i = 1; i < n; ++i) {
            total_len += std::hypot(local_waypoints_[i].x - local_waypoints_[i - 1].x,
                                    local_waypoints_[i].y - local_waypoints_[i - 1].y);
        }

        if (n >= 8) {
            const double avg_spacing = total_len / static_cast<double>(n - 1);
            const double closing_gap = std::hypot(local_waypoints_[n - 1].x - local_waypoints_[0].x,
                                                  local_waypoints_[n - 1].y - local_waypoints_[0].y);
            local_is_closed_ = (avg_spacing > 1e-6) && (closing_gap <= 2.0 * avg_spacing);
        }

        smooth_curvature(local_waypoints_, local_is_closed_);

        apply_sector_scales(local_waypoints_);

        // 배열이 교체되면 로컬 추적기를 초기화.
        if (n != prev_size) last_local_idx_ = 0;

        if (local_is_closed_ != last_logged_local_closed_) {
            RCLCPP_INFO(this->get_logger(), "로컬 경로 기하: %s (웨이포인트 %zu개)",
                        local_is_closed_ ? "닫힌 루프(wrap 적용)" : "열린 구간", n);
            last_logged_local_closed_ = local_is_closed_;
        }
        local_last_recv_time_ = this->now();
    }

    // 경로를 모를 때의 안전 정지. 발행을 멈추지 않고 명시적 0을 보내는 이유는, 침묵하면
    // 하류(ackermann_mux→VESC)가 **직전 명령을 그대로 유지**해 타력주행이 되기 때문이다.
    void publish_safe_stop() {
        last_steering_angle_ = 0.0;
        last_target_speed_ = std::max(0.0, current_speed_);
        last_published_speed_ = 0.0;
        publish_drive(0.0, 0.0, 0.0);
    }

    double ramp_speed(double last_cmd, double target, double dt,
                      double max_accel, double max_decel) const {
        double speed_error = target - current_speed_;
        double out = last_cmd;
        if (speed_error > 0.0) {
            out += std::min(speed_error, max_accel * dt);
            if (out > target) out = target;
        } else {
            out += std::max(speed_error, -max_decel * dt);
            if (out < target) out = target;
        }
        return out;
    }

    void publish_drive(double steering_angle, double speed, double accel) {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.drive.steering_angle = steering_angle;
        msg.drive.speed = speed;
        msg.drive.acceleration = accel;
        drive_pub_->publish(msg);
    }

    void control_loop() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) dt = 0.02;
        dt = std::clamp(dt, 0.001, 0.1);
        last_time_ = current_time;

        // 0-a. odom 미수신 — 위치추정이 뜨기 전에는 주행하지 않는다.
        if (!odom_seen_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "odom(%s) 미수신 — 위치추정이 뜨기 전에는 주행하지 않는다", odom_topic_.c_str());
            publish_safe_stop();
            return;
        }

        // 0. 경로 소스 중재: 로컬(신선) → 글로벌 → 둘 다 없으면 안전 정지
        bool local_fresh = !local_waypoints_.empty() &&
                           (current_time - local_last_recv_time_).seconds() < local_fresh_timeout_;

        if (local_fresh && closest_idx_max_heading_err_ > 0.0) {
            auto [ld, li, lgated] = scan_closest_heading_gated(
                local_waypoints_, current_x_, current_y_, current_yaw_,
                closest_idx_max_heading_err_);
            (void)ld; (void)li;
            if (!lgated) {
                ++local_heading_reject_count_;
                if (!waypoints_.empty()) {
                    local_fresh = false;   // 글로벌로 폴백
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "로컬 경로가 차량 헤딩과 ±%.0f° 안에서 정합하는 점이 하나도 없다 — "
                        "상류(state_machine/frenet) 경로 반전 의심, 글로벌로 폴백 (누적 %u회)",
                        closest_idx_max_heading_err_ * 180.0 / M_PI, local_heading_reject_count_);
                } else {
                    // 글로벌이 없으면 재획득 불능을 만들지 않기 위해 로컬을 그대로 쓴다.
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "로컬 경로 진행방향 불일치(±%.0f°) — 글로벌이 없어 그대로 추종한다 (누적 %u회)",
                        closest_idx_max_heading_err_ * 180.0 / M_PI, local_heading_reject_count_);
                }
            }
        }

        if (!local_fresh && waypoints_.empty()) {
            publish_safe_stop();
            return;
        }

        const std::vector<Waypoint>& wps = local_fresh ? local_waypoints_ : waypoints_;

        const bool following_local = local_fresh;
        const bool path_closed = local_fresh ? local_is_closed_ : true;

        // 1. 최근접 웨이포인트 인덱스
        const size_t n = wps.size();
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;

        // 추적기는 경로 소스별로 따로 둔다 — 로컬/글로벌은 배열 길이·인덱싱이 달라 하나를
        // 공유하면 소스가 바뀔 때 엉뚱한 인덱스에서 시작한다.
        size_t& idx_tracker = following_local ? last_local_idx_ : last_target_idx_;
        if (idx_tracker >= n) idx_tracker = 0;

        if (path_closed) {
            const double spacing = std::max(0.01, avg_waypoint_spacing_);
            const int half_n = static_cast<int>(n / 2);
            int back_count = std::min(std::max(2, static_cast<int>(std::ceil(1.0 / spacing))), half_n);
            int fwd_count  = std::min(std::max(8, static_cast<int>(std::ceil(3.0 / spacing))), half_n);

            closest_idx = idx_tracker;
            for (int i = -back_count; i <= fwd_count; ++i) {
                size_t idx = (idx_tracker + i + n) % n;
                double dist = std::hypot(wps[idx].x - current_x_, wps[idx].y - current_y_);
                if (dist < min_dist) { min_dist = dist; closest_idx = idx; }
            }
            // Fail-safe: 경로와 2.5m 넘게 멀어지면 전역 재탐색.
            // ⚠️ 이 전역 재탐색이 pose 붕괴 시 인덱스 텔레포트의 통로다 — 헤딩 게이트를 건다.
            if (min_dist > 2.5) {
                std::tie(min_dist, closest_idx, std::ignore) = scan_closest_heading_gated(
                    wps, current_x_, current_y_, current_yaw_, closest_idx_max_heading_err_);
            }
        } else {
            // 열린 구간(짧은 회피경로): 전체 최근접 스캔(저렴, wrap 인덱스 미사용)
            std::tie(min_dist, closest_idx, std::ignore) = scan_closest_heading_gated(
                wps, current_x_, current_y_, current_yaw_, closest_idx_max_heading_err_);
        }

        if (closest_idx_max_heading_err_ > 0.0 && n > 0 &&
            std::abs(wrap_pi(wps[closest_idx].yaw - current_yaw_)) > closest_idx_max_heading_err_) {
            auto [d, i, gated] = scan_closest_heading_gated(
                wps, current_x_, current_y_, current_yaw_, closest_idx_max_heading_err_);
            if (gated) {
                min_dist = d; closest_idx = i;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "전역 재탐색: 헤딩 정합(±%.0f°) 후보가 없어 게이트 없이 선택 — "
                    "차량이 경로 반대 방향이거나 pose가 깨졌을 수 있음",
                    closest_idx_max_heading_err_ * 180.0 / M_PI);
            }
        }

        // ⚠️ 추적기 갱신은 두 분기 공통이어야 한다. 예전엔 닫힌 분기에서만 되써서 로컬 추종 중
        //    last_target_idx_가 0에 얼어붙었고, 로컬→글로벌 폴백 시 stale 인덱스에서 탐색을
        //    시작해 2.5m failsafe에만 의존했다.
        idx_tracker = closest_idx;
        const double lateral_error = min_dist;

        // 1.5 곡률 룩어헤드 사전 감속 —────────────────────────────────────────────────
        // 스캔 거리 = max(하한, 제동거리 v²/2a, 형상 적응). 형상 적응은 전방 12m 안의 고곡률
        // 피크까지 창을 늘려 시케인/헤어핀을 놓치지 않게 한다.
        double brake_dist = (current_speed_ * current_speed_) / (2.0 * std::max(0.1, prebrake_decel_));
        double min_lookahead_dist = static_cast<double>(curvature_lookahead_count_) * 0.1;

        double adaptive_lookahead_dist = min_lookahead_dist;
        walk_forward(wps, closest_idx, 12.0, path_closed, [&](size_t i, double accum) {
            if (std::abs(wps[i].smoothed_curvature) > 0.4) {
                adaptive_lookahead_dist = std::max(adaptive_lookahead_dist, accum + 1.0);
            }
            return true;
        });
        double curv_lookahead_dist = std::max({min_lookahead_dist, brake_dist, adaptive_lookahead_dist});

        double curvature_speed_limit = std::numeric_limits<double>::max();
        double steer_bound_k = 0.0, steer_bound_v = 0.0;   // 진단 로그용
        walk_forward(wps, closest_idx, curv_lookahead_dist, path_closed, [&](size_t i, double accum) {
            double v_cap_i = wps[i].speed;
            double k_i = std::abs(wps[i].smoothed_curvature);
            if (k_i > 0.01) {
                const double mla_i = (wps[i].mla > 0.0) ? wps[i].mla : max_lateral_accel_;
                v_cap_i = std::min(v_cap_i, std::sqrt(mla_i / k_i));                // (a) 그립
                if (understeer_gradient_ > 1e-6) {                                  // (b) 조향 권한
                    // ⚠️ 좌우 중 **작은** 한계를 쓰고, 거기에 도달각 비율까지 곱한다 —
                    //    캡은 "바퀴가 실제로 꺾이는 각"으로 계산해야 의미가 있다(0.379를 다
                    //    낸다고 보면 코너 진입 속도를 그만큼 과대 허용한다).
                    double steer_budget = steer_avail() - wheelbase_ * k_i;
                    double v_steer = (steer_budget > 0.0)
                        ? std::sqrt(steer_budget / (understeer_gradient_ * k_i))
                        : 0.0;
                    if (v_steer < v_cap_i) {
                        v_cap_i = v_steer;
                        if (k_i > steer_bound_k) { steer_bound_k = k_i; steer_bound_v = v_steer; }
                    }
                }
            }
            curvature_speed_limit = std::min(
                curvature_speed_limit, std::sqrt(v_cap_i * v_cap_i + 2.0 * prebrake_decel_ * accum));
            return true;
        });
        // ⚠️ min_speed 하한이 조향 캡보다 높으면 캡이 무력화된다(07-26 헤어핀 조향 한계 0.87 m/s
        //    vs min_speed 2.5). 고곡률 트랙에선 min_speed를 함께 낮출 것.
        if (steer_bound_k > 0.0 && steer_bound_v < min_speed_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "조향 권한 한계 %.2f m/s (κ=%.3f, R=%.2fm) < min_speed %.2f — 하한이 캡을 무력화 중",
                steer_bound_v, steer_bound_k, 1.0 / steer_bound_k, min_speed_);
        }
        curvature_speed_limit = std::max(min_speed_, curvature_speed_limit);

        // 2. L1 룩어헤드 거리 + 목표점. 고곡률 진입에서는 L1을 최대 25% 줄여 반응성을 올린다.
        double curv_closest = std::abs(wps[closest_idx].smoothed_curvature);
        double L1_distance = l1_offset_ + current_speed_ * l1_speed_gain_;
        if (curv_closest > 0.3) {
            L1_distance *= (1.0 - 0.25 * std::min(1.0, (curv_closest - 0.3) / 1.0));
        }
        // ⚠️ 하한이 상한보다 클 수 있으므로(큰 횡오차) std::clamp를 쓰면 안 된다 — 하한 우선.
        double lower_bound = std::max(t_clip_min_, std::sqrt(2.0) * lateral_error);
        L1_distance = std::max(lower_bound, std::min(L1_distance, t_clip_max_));

        size_t idx_a = walk_forward(wps, closest_idx, L1_distance, path_closed,
                                    [](size_t, double) { return true; });

        const double L1_x = wps[idx_a].x, L1_y = wps[idx_a].y;
        publish_l1_marker(L1_x, L1_y);   // 표시 전용

        if (l1_jump_warn_m_ > 0.0 && l1_prev_valid_) {
            const double veh_move = std::hypot(current_x_ - prev_pose_x_, current_y_ - prev_pose_y_);
            const double tgt_move = std::hypot(L1_x - prev_l1_x_, L1_y - prev_l1_y_);
            if (tgt_move - veh_move > l1_jump_warn_m_) {
                ++l1_jump_count_;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "L1 목표점 순간이동: 목표점 %.2f m 이동 / 차량 %.2f m (초과 %.2f m). "
                    "누적 점프 %lu회, 뒤쪽 %lu회 / 주행 %lu 사이클",
                    tgt_move, veh_move, tgt_move - veh_move,
                    l1_jump_count_, l1_behind_count_, l1_cycle_count_);
            }
            // 목표점이 차량 **뒤**에 찍히면 sin_eta 부호가 뒤집혀 조향이 반대로 나간다.
            const double fwd = std::cos(current_yaw_) * (L1_x - current_x_) +
                               std::sin(current_yaw_) * (L1_y - current_y_);
            if (fwd < 0.0) ++l1_behind_count_;
            if (current_speed_ > 0.5) ++l1_cycle_count_;
        }
        prev_pose_x_ = current_x_; prev_pose_y_ = current_y_;
        prev_l1_x_ = L1_x; prev_l1_y_ = L1_y; l1_prev_valid_ = true;

        // 3. sin(eta) — 차량 헤딩과 L1 목표점 사이의 횡방향 성분
        double L1_vector_x = L1_x - current_x_;
        double L1_vector_y = L1_y - current_y_;
        double L1_norm = std::hypot(L1_vector_x, L1_vector_y);
        double sin_eta = 0.0;
        if (L1_norm > 1e-5) {
            double lat = -std::sin(current_yaw_) * L1_vector_x + std::cos(current_yaw_) * L1_vector_y;
            sin_eta = std::clamp(lat / L1_norm, -1.0, 1.0);
        }

        double speed_for_lu =
            wps[find_lookahead_wp_idx(wps, path_closed, closest_idx, speed_lookahead_for_steering_)].speed;

        // 5. 목표 횡가속도 → LUT 조향각
        double lat_acc = 0.0;
        speed_for_lu = std::min(speed_for_lu, curvature_speed_limit);
        // 🔴 실측 속도 상한 (선언부 주석 참고). L1_distance가 실측 속도로 계산되므로 게인도
        //    같은 속도를 써야 한다 — 안 그러면 정지/저속에서 게인이 (v_prof/v_meas)²배로 뛴다.
        //    ⚠️ 반드시 lat_acc 계산과 LUT 조회 **앞**에 둘 것(둘 다 speed_for_lu를 쓴다).
        if (steering_speed_cap_measured_) {
            speed_for_lu = std::min(speed_for_lu, current_speed_);
        }
        double l1_denom = l1_use_actual_distance_ ? std::max(L1_norm, l1_min_denom_)
                                                  : std::max(L1_distance, l1_min_denom_);
        lat_acc = 2.0 * speed_for_lu * speed_for_lu / l1_denom * sin_eta;

        bool lut_saturated = false;
        double steering_angle = lookup_table_.lookup_steer_angle(lat_acc, speed_for_lu, &lut_saturated);

        const double lut_v_max = lookup_table_.max_velocity();
        if (lut_v_max > 1e-3 && speed_for_lu > lut_v_max) {
            const double a_abs = std::abs(lat_acc);
            const double dk = a_abs / (lut_v_max * lut_v_max) - a_abs / (speed_for_lu * speed_for_lu);
            const double corr = wheelbase_ * dk;   // ≥ 0
            const double sgn = (steering_angle >= 0.0) ? 1.0 : -1.0;
            // 부호를 넘어가진 않게(과보정 방지) 크기에서만 뺀다.
            steering_angle = sgn * std::max(0.0, std::abs(steering_angle) - corr);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "속도 %.2f m/s가 LUT 속도축 상한 %.2f를 초과 — 기구학 보정 %.4f rad 적용. "
                "LUT를 max_speed 이상까지 재생성할 것", speed_for_lu, lut_v_max, corr);
        }

        if (lut_saturated && std::abs(sin_eta) > 0.05 && speed_for_lu > 2.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "LUT 그립 포화: 요구 a_lat %.2f m/s² @ %.2f m/s (조향 %.3f rad에서 saturate) — "
                "조향 피드백 개루프 상태. 코너 진입 속도/prebrake_decel 확인",
                std::abs(lat_acc), speed_for_lu, std::abs(steering_angle));
        }

        double acc_mean = 0.0;
        for (double a : acc_now_) acc_mean += a;
        acc_mean /= acc_now_.size();
        {
            const double w = std::clamp(std::abs(acc_mean) / steering_scaler_accel_ref_, 0.0, 1.0);
            const double target_scaler = (acc_mean >= 0.0) ? acceleration_scaler_for_steering_
                                                           : deceleration_scaler_for_steering_;
            steering_angle *= (1.0 - w) + w * target_scaler;
        }

        double speed_diff = std::max(0.1, end_scale_speed_ - start_scale_speed_);
        double clip_factor = std::clamp((speed_for_lu - start_scale_speed_) / speed_diff, 0.0, 1.0);
        steering_angle *= (1.0 - clip_factor * downscale_factor_);

        // 6-3) 곡률 피드포워드 블렌딩 (curvature_ff_blend=0이면 순수 L1 격리)
        double steer_ff = std::atan(wheelbase_ * wps[closest_idx].curvature);
        steering_angle = (1.0 - curvature_ff_blend_) * steering_angle + curvature_ff_blend_ * steer_ff;

        // 6-4) Stanley형 heading 정렬 댐핑. 순수 L1은 횡오차 복구 시 경로 접선을 지나쳐 heading이
        //      오버슈트(라인을 비스듬히 가로질러 외벽 충돌)하는 약점이 있다. 부호 규약: 좌측 정렬이
        //      필요하면 (psi − yaw)가 양수 → +조향(좌).
        double heading_err = wrap_pi(wps[closest_idx].yaw - current_yaw_);
        steering_angle += heading_damping_gain_ * heading_err;

        if (steering_reach_ratio_ < 0.999) steering_angle /= steering_reach_ratio_;

        steering_angle += steering_trim_;

        const double steer_step = max_steering_rate_ * dt;
        steering_angle = std::clamp(steering_angle,
                                    last_steering_angle_ - steer_step,
                                    last_steering_angle_ + steer_step);
        steering_angle = std::clamp(steering_angle, -max_steering_right_, max_steering_left_);
        last_steering_angle_ = steering_angle;

        // 6-8) 트림 추정기 갱신 — **최종 발행값**으로 학습해야 한다. rate limit/클램프에
        //      걸린 값을 안 쓰고 원래 명령을 쓰면 포화 구간에서 있지도 않은 오차를 학습한다.
        update_steering_trim(dt, steering_angle);

        // 7. 목표 속도 ───────────────────────────────────────────────────────────────
        double global_speed =
            wps[find_lookahead_wp_idx(wps, path_closed, closest_idx, speed_lookahead_)].speed;
        global_speed = std::min(global_speed, curvature_speed_limit);
        global_speed = std::min(global_speed, max_speed_);
        double target_speed = global_speed;

        // 8. 명령 속도 램프
        double final_speed = ramp_speed(last_target_speed_, target_speed, dt,
                                        base_max_accel_, base_max_decel_);
        last_target_speed_ = final_speed;

        if (ramp_lead_max_ > 0.0) {
            const double lead_cap = std::max(0.0, current_speed_) + ramp_lead_max_;
            if (final_speed > lead_cap) {
                final_speed = lead_cap;
                last_target_speed_ = final_speed;
            }
        }

        // 8-a2. 자율 미체결 중 램프 고정 — bumpless transfer (선언부 주석 참고).
        const bool disengaged = engage_gate_active() && !is_engaged_;
        if (disengaged) {
            final_speed = std::max(0.0, current_speed_);
            last_target_speed_ = final_speed;
            // 런치 상태도 누적하지 않는다(애초에 출발 명령이 하류로 나가지 않는 구간이다).
            // ⚠️ launch_latched_off_는 건드리지 않는다 — 매 사이클 false로 되돌리면 아래 8-c의
            //    킥이 무한 재무장돼 미체결 중에도 발행값이 부스트 값으로 덮인다.
            launch_active_ = false;
            launch_time_ = 0.0;
            // 조향 트림 추정도 리셋한다 — 미체결 중엔 발행이 하류로 안 나가므로 그 구간의
            // "명령 vs 요레이트"는 물리적 의미가 없고, 재체결 시 남은 값이 계단으로 나간다.
            steering_trim_ = 0.0;
            steering_trim_samples_ = 0;
            steer_hist_.clear();
            // ⚠️ 2초 throttle이면 대기 중 계속 찍힌다(0807 로그 1007줄 중 156줄). 상태 전이는
            //    이미 위 "자율 체결 상태 변경"이 1회 찍으므로, 여기선 30초 하트비트로 충분하다.
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                "자율 미체결 — 속도 램프를 실측(%.2f m/s)에 고정 중(engage 시 무충격 전환)",
                current_speed_);
        }

        double publish_speed = final_speed;
        if (launch_boost_enable_ && !disengaged) {
            bool moving = std::abs(current_speed_) > launch_exit_speed_;
            bool standstill = std::abs(current_speed_) < launch_standstill_speed_;
            if (moving) launch_latched_off_ = false;   // 움직였으면 다음 정지서 다시 킥
            if (target_speed > 0.1) {                  // 갈 의도가 있을 때만
                if (!launch_active_ && standstill && !launch_latched_off_) {
                    launch_active_ = true; launch_time_ = 0.0;
                }
                if (launch_active_) {
                    launch_time_ += dt;
                    if (moving) {
                        launch_active_ = false;                        // 관통 성공
                    } else if (launch_time_ > launch_boost_time_) {
                        launch_active_ = false; launch_latched_off_ = true;
                        RCLCPP_WARN(this->get_logger(),
                            "런치 킥 %.2fs 관통 실패 → 포기. 데드존 심함 — 푸시스타트 필요",
                            launch_time_);
                    } else {
                        publish_speed = std::max(publish_speed, launch_boost_speed_);
                        last_target_speed_ = publish_speed; // 런치 킥 해제 후 속도 절벽 멈칫거림 방지 (무구간 부드러운 가속 이음)
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                            "런치 킥: 실측 %.2f → 발행 %.2f m/s (t=%.2fs)",
                            current_speed_, publish_speed, launch_time_);
                    }
                }
            } else {
                launch_active_ = false;   // 정지 명령 중엔 킥 안 함
            }
        }

        if (status_log_period_ms_ > 0) {
            // ⚠️ Idx 앞의 L/G는 **어느 배열의 인덱스인지**다. 로컬(L)과 글로벌(G)은 배열이
            //    달라 소스가 바뀌면 번호가 크게 튀는데(실측 L53→G134) 실제 목표점 좌표는
            //    연속이다. 이 표기가 없으면 "룩어헤드가 트랙 반대쪽으로 튀었다"로 오독된다.
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), status_log_period_ms_,
                "Pose: (%.2f, %.2f, %.2f) | Target WP: (%.2f, %.2f), Idx: %c%zu -> %c%zu | Steer: %.4f | "
                "Speed: %.2f / %.2f | L1_dist: %.2f | acc_mean: %.2f | 점프 %lu/뒤쪽 %lu/경로반전 %u"
                " | trim: %+.2f° (n=%ld)%s",
                current_x_, current_y_, current_yaw_, L1_x, L1_y,
                following_local ? 'L' : 'G', closest_idx, following_local ? 'L' : 'G', idx_a,
                steering_angle, final_speed, current_speed_, L1_distance, acc_mean,
                l1_jump_count_, l1_behind_count_, local_heading_reject_count_,
                steering_trim_ * 180.0 / M_PI, steering_trim_samples_,
                // 섹터가 켜져 있으면 **지금 이 지점에 실제로 적용된 MLA**를 같이 찍는다.
                // 파라미터가 아니라 적용값을 찍어야 "켜졌는데 왜 안 빨라지나"를 로그만으로 가른다.
                sector_status_suffix(wps[closest_idx].mla).c_str());
        }

        const double cmd_accel = (publish_speed - last_published_speed_) / dt;
        last_published_speed_ = publish_speed;
        publish_drive(steering_angle, publish_speed, cmd_accel);
    }

    // 디버그: L1 목표점(초록 구) + 룩어헤드 벡터(노란 선)를 웨이포인트/pose와 같은 프레임에
    // 그린다. 제어 경로와 완전 분리된 표시 전용.
    void publish_l1_marker(double L1_x, double L1_y) {
        visualization_msgs::msg::MarkerArray arr;
        auto stamp = this->now();

        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = odom_frame_;
        sphere.header.stamp = stamp;
        sphere.ns = "l1_lookahead";
        sphere.id = 0;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = L1_x;
        sphere.pose.position.y = L1_y;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.30;
        sphere.color.g = 1.0f; sphere.color.a = 1.0f;
        arr.markers.push_back(sphere);

        visualization_msgs::msg::Marker line;
        line.header.frame_id = odom_frame_;
        line.header.stamp = stamp;
        line.ns = "l1_lookahead";
        line.id = 1;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.05;
        line.color.r = 1.0f; line.color.g = 1.0f; line.color.a = 0.8f;
        geometry_msgs::msg::Point p0, p1;
        p0.x = current_x_; p0.y = current_y_;
        p1.x = L1_x;       p1.y = L1_y;
        line.points.push_back(p0);
        line.points.push_back(p1);
        arr.markers.push_back(line);

        l1_marker_pub_->publish(arr);
    }

    struct Sector { double s0, s1, scale; };
    struct SectorTrans { double s, before, after; };   // 값이 실제로 바뀌는 지점만

    void sector_scale_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        const auto& d = msg->data;
        // ⚠️ 하나라도 이상하면 **메시지 전체를 버리고 직전 값을 유지**한다. 부분 적용하면
        //    어느 구간이 새 값이고 어느 구간이 옛 값인지 알 수 없는 상태가 된다.
        if (d.empty() || (d.size() - 1) % 3 != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "섹터 테이블 무시: 길이 %zu — [track_length, (s0,s1,scale)×N] 형식이 아님", d.size());
            return;
        }
        const double decl_len = d[0];
        if (!std::isfinite(decl_len) || decl_len <= 1.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "섹터 테이블 무시: track_length %.3f가 비정상", decl_len);
            return;
        }
        std::vector<Sector> parsed;
        parsed.reserve((d.size() - 1) / 3);
        for (size_t i = 1; i + 2 < d.size(); i += 3) {
            const double s0 = d[i], s1 = d[i + 1], sc = d[i + 2];
            if (!std::isfinite(s0) || !std::isfinite(s1) || !std::isfinite(sc) ||
                s1 <= s0 || s0 < 0.0 || s1 > decl_len + 1e-6) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "섹터 테이블 무시: %zu번 항목이 비정상 (s %.2f→%.2f, 랩길이 %.2f, scale %.2f)",
                    i / 3, s0, s1, decl_len, sc);
                return;
            }
            if (sc < 1.0 || sc > sector_scale_max_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "섹터 테이블 무시: %zu번 scale %.2f이 [1.0, %.2f] 밖 — 1.0 미만은 "
                    "설계상 금지다(미수신 시 위험 코너가 빨라지는 방향이 된다)",
                    i / 3, sc, sector_scale_max_);
                return;
            }
            parsed.push_back({s0, s1, sc});
        }
        // 내용 서명 — 재발행이 잦으므로 "실제로 바뀌었나"를 싸게 판정하기 위한 것.
        double sig = decl_len * 7919.0 + static_cast<double>(parsed.size());
        for (const auto& p : parsed) sig += p.s0 * 31.0 + p.s1 * 131.0 + p.scale * 1009.0;

        sectors_ = std::move(parsed);
        sector_decl_track_len_ = decl_len;
        rebuild_sector_profile();
        const bool was_stale = sector_table_seen_ && !sector_table_fresh();
        sector_table_seen_ = true;
        sector_table_last_recv_time_ = this->now();
        if (was_stale) {
            RCLCPP_INFO(this->get_logger(), "섹터 테이블 재개 — 발행자가 돌아왔다");
        }
        if (sector_track_len_ > 0.0) validate_sector_track_length(sector_track_len_);
        apply_sector_scales(waypoints_);
        apply_sector_scales(local_waypoints_);
        if (sig != sector_table_sig_ || was_stale) {
            sector_table_sig_ = sig;
            RCLCPP_INFO(this->get_logger(), "섹터 테이블 갱신: %zu구간 / 랩길이 %.2f m%s",
                        sectors_.size(), decl_len,
                        sector_len_ok_ ? "" : " (⚠️ 라인 길이 미검증 — 글로벌 경로 수신 대기)");
        }
    }

    // 블렌딩 없는 계단 함수. 구간이 겹치면 큰 쪽(발행 쪽에서 겹침을 금지하지만 방어적으로).
    double sector_scale_step(double s) const {
        double v = 1.0;
        for (const auto& sec : sectors_) {
            if (s >= sec.s0 && s < sec.s1) v = std::max(v, static_cast<double>(sec.scale));
        }
        return v;
    }

    void rebuild_sector_profile() {
        sector_trans_.clear();
        const double L = sector_decl_track_len_;
        if (sectors_.empty() || L <= 0.0) return;
        std::vector<double> bounds;
        bounds.reserve(sectors_.size() * 2);
        for (const auto& sec : sectors_) { bounds.push_back(sec.s0); bounds.push_back(sec.s1); }
        std::sort(bounds.begin(), bounds.end());
        bounds.erase(std::unique(bounds.begin(), bounds.end()), bounds.end());
        const double eps = 1e-4;
        for (double b : bounds) {
            const double before = sector_scale_step(std::fmod(b - eps + L, L));
            const double after = sector_scale_step(std::fmod(b + eps, L));
            if (std::abs(after - before) > 1e-9) sector_trans_.push_back({b, before, after});
        }
    }

    // s에 적용할 스케일. 전이점 ±blend/2에서만 선형 램프.
    double sector_scale_at(double s) const {
        const double L = sector_decl_track_len_;
        if (sectors_.empty() || L <= 0.0) return 1.0;
        s = std::fmod(std::fmod(s, L) + L, L);
        const double v = sector_scale_step(s);
        const double h = 0.5 * sector_scale_blend_;
        if (h <= 1e-9) return v;
        for (const auto& t : sector_trans_) {
            double d = s - t.s;
            if (d > L * 0.5) d -= L; else if (d < -L * 0.5) d += L;
            if (std::abs(d) >= h) continue;
            // 전이 간격이 blend보다 좁으면 겹치지만, 그건 발행 쪽에서 금지한다
            // (bag_analyzer는 경계를 κ 최소점으로 스냅해 충분히 벌려서 준다).
            return t.before + (t.after - t.before) * std::clamp((d + h) / (2.0 * h), 0.0, 1.0);
        }
        return v;
    }

    void apply_sector_scales(std::vector<Waypoint>& wps) {
        const bool active = sector_active();
        for (auto& w : wps) {
            w.mla = active ? max_lateral_accel_ * sector_scale_at(w.s) : max_lateral_accel_;
        }
    }

    // 스케일을 지금 적용해도 되는가. ⚠️ "모르겠으면 끈다"가 원칙이다 — 여기서 애매한 걸
    // 켜는 쪽으로 처리하면 불변식(모든 실패는 느려지는 방향)이 깨진다.
    // 테이블이 신선한가. 데드맨이 꺼져 있으면(0) 항상 참 = 구 거동.
    bool sector_table_fresh() const {
        if (sector_scale_timeout_ <= 0.0) return true;
        if (!sector_table_seen_) return false;
        return (this->now() - sector_table_last_recv_time_).seconds() <= sector_scale_timeout_;
    }

    bool sector_active() const {
        if (!sector_scale_enable_ || !sector_table_seen_ || !sector_len_ok_) return false;
        if (!sector_table_fresh()) return false;
        if (!sector_scale_global_only_) return true;
        // /state를 한 번도 못 받았거나 끊겼으면 회피 중인지 알 수 없다 → 끈다.
        // (state_machine을 안 띄우는 시뮬에서는 이 경로로 항상 비활성이 된다. 의도된 것 —
        //  검증 근거인 벽 여유가 실차 bag에서만 나오므로 시뮬에서 켤 이유가 없다.)
        if (!sector_state_seen_) return false;
        if ((this->now() - sector_state_last_recv_time_).seconds() > sector_scale_state_timeout_)
            return false;
        return sector_on_global_;
    }

    // 상태 로그 꼬리표. 꺼져 있으면 빈 문자열이라 기존 로그 형식이 그대로 유지된다.
    std::string sector_status_suffix(double applied_mla) const {
        if (!sector_scale_enable_) return "";
        char buf[96];
        if (!sector_active()) {
            // ⚠️ "/state 끊김"과 "회피 중"을 구분해서 찍는다. 둘 다 비활성이지만 원인이
            //    정반대다 — 전자는 배선 문제(고쳐야 함), 후자는 의도된 동작(정상)이다.
            const bool state_stale =
                sector_state_seen_ &&
                (this->now() - sector_state_last_recv_time_).seconds() > sector_scale_state_timeout_;
            const char* why = !sector_table_seen_     ? "테이블 미수신"
                            : !sector_len_ok_         ? "라인 불일치"
                            : !sector_table_fresh()   ? "테이블 끊김"
                            : !sector_state_seen_     ? "/state 미수신"
                            : state_stale             ? "/state 끊김"
                                                      : "회피/추월 중";
            std::snprintf(buf, sizeof(buf), " | 섹터: 비활성(%s)", why);
        } else {
            const double m = (applied_mla > 0.0) ? applied_mla : max_lateral_accel_;
            std::snprintf(buf, sizeof(buf), " | 섹터 MLA %.2f (×%.2f)", m, m / max_lateral_accel_);
        }
        return std::string(buf);
    }

    // 섹터 테이블이 지금 들어온 글로벌 라인과 같은 라인 기준인지 확인한다.
    // 다르면 s가 다른 코너를 가리키고, scale ≥ 1.0이라 그 오적용은 "빨라지는" 쪽이다.
    void validate_sector_track_length(double track_len) {
        if (!sector_scale_enable_) return;
        sector_track_len_ = track_len;
        if (sector_decl_track_len_ <= 0.0) {
            // 발행 쪽이 길이를 안 알려준 경우 — 검증 없이 쓰는 건 위험하니 비활성.
            sector_len_ok_ = false;
            if (sector_table_seen_)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "섹터 스케일 비활성: 발행 쪽이 track_length를 안 줬다 "
                    "(sector_scale_track_length 파라미터 또는 테이블 헤더 필요)");
            return;
        }
        const bool ok = std::abs(track_len - sector_decl_track_len_) <= sector_scale_track_len_tol_;
        if (ok != sector_len_ok_) {
            if (ok) RCLCPP_INFO(this->get_logger(),
                        "섹터 스케일 라인 검증 통과 (테이블 %.2f m / 현재 %.2f m)",
                        sector_decl_track_len_, track_len);
            else RCLCPP_ERROR(this->get_logger(),
                        "🔴 섹터 스케일 폐기: 테이블이 다른 라인 기준이다 (%.2f m vs 현재 %.2f m, "
                        "허용 %.2f) — 전 구간 scale 1.0으로 되돌린다. 라인을 재생성했으면 "
                        "섹터 파일도 새 bag으로 다시 뽑을 것",
                        sector_decl_track_len_, track_len, sector_scale_track_len_tol_);
        }
        sector_len_ok_ = ok;
    }

    size_t find_lookahead_wp_idx(const std::vector<Waypoint>& wps, bool closed, size_t base_idx,
                                 double lookahead_time) const {
        const size_t nn = wps.size();
        double la_x = current_x_ + std::cos(current_yaw_) * current_speed_ * lookahead_time;
        double la_y = current_y_ + std::sin(current_yaw_) * current_speed_ * lookahead_time;
        size_t best_idx = base_idx;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = -5; i <= 15; ++i) {
            size_t idx;
            if (closed) {
                idx = (base_idx + static_cast<size_t>(i + static_cast<int>(nn))) % nn;
            } else {   // 열린 경로: [0, nn-1]로 clamp(뒤로 감기 방지)
                idx = static_cast<size_t>(
                    std::clamp<long>(static_cast<long>(base_idx) + i, 0, static_cast<long>(nn) - 1));
            }
            double dist = std::hypot(wps[idx].x - la_x, wps[idx].y - la_y);
            if (dist < min_dist) { min_dist = dist; best_idx = idx; }
        }
        return best_idx;
    }

    // ── 멤버 변수 ────────────────────────────────────────────────────────────────
    // 차량/L1
    // l1_offset_[m] = L1 거리의 절편, l1_speed_gain_[s] = 속도 계수 (L1 = offset + v·gain)
    double wheelbase_, l1_offset_, l1_speed_gain_, t_clip_min_, t_clip_max_;
    double l1_min_denom_ = 0.6;              // L1 횡가속 분모 하한 [m] (t_clip_min과 분리)
    double heading_damping_gain_;
    bool l1_use_actual_distance_ = true;
    bool steering_speed_cap_measured_ = true;  // 조향용 속도를 실측 속도로 상한(정지 시 LUT 포화 차단)
    int status_log_period_ms_ = 2000;          // 상태 한 줄 로그 주기 [ms], 0 = 끔
    size_t last_global_sig_ = 0;               // 글로벌 경로 재발행 중복 로그 억제용 서명

    // L1 목표점 점프 검출 (진단 전용)
    double l1_jump_warn_m_ = 1.0;
    double prev_pose_x_ = 0.0, prev_pose_y_ = 0.0, prev_l1_x_ = 0.0, prev_l1_y_ = 0.0;
    bool l1_prev_valid_ = false;
    unsigned long l1_jump_count_ = 0, l1_behind_count_ = 0, l1_cycle_count_ = 0;

    // 조향 스케일러 / 속도 룩어헤드
    double acceleration_scaler_for_steering_, deceleration_scaler_for_steering_;
    double steering_scaler_accel_ref_ = 1.0;  // 가감속 스케일러 완전 적용 기준 |a| [m/s²]
    double start_scale_speed_, end_scale_speed_, downscale_factor_;
    double speed_lookahead_, speed_lookahead_for_steering_;
    // 명령각 중 바퀴가 실제로 내는 비율. 조향 명령 보상(1/ratio)과 조향 권한 캡의 δ_avail을
    // 동시에 지배한다(steer_avail()). 1.0이면 둘 다 구 낙관 거동.
    double steering_reach_ratio_ = 0.74;
    double max_steering_rate_ = 20.0;         // 조향 rate limit [rad/s] (dt 비례)
    double pose_lpf_alpha_ = 0.30;            // MCL 포즈/헤딩 저역통과 필터 알파 (0.01~1.0)

    // 종방향
    double base_max_accel_;
    double ramp_lead_max_ = 2.4;   // 램프 안티와인드업 선행 상한 [m/s], 0이면 비활성
    double base_max_decel_;                  // 명령 속도 하강 rate limit [m/s²]
    double prebrake_decel_ = 1.5;            // 곡률 사전감속용 실측 감속 권한 [m/s²]
    double max_speed_, min_speed_;

    // 런치 킥
    bool launch_boost_enable_ = true;
    double launch_boost_speed_ = 2.2, launch_boost_time_ = 0.6;
    double launch_exit_speed_ = 0.8, launch_standstill_speed_ = 0.3;
    bool launch_active_ = false;
    double launch_time_ = 0.0;
    bool launch_latched_off_ = false;        // 관통 실패로 포기(차가 실제로 움직일 때까지 재시도 안 함)

    bool use_imu_;
    double imu_linear_scale_ = 1.0;
    double imu_angular_scale_ = 1.0;         // deg/s → rad/s (real=pi/180, sim=1.0)
    std::vector<double> acc_now_;            // 종가속 rolling buffer
    double yaw_rate_now_ = 0.0;              // 실측 요레이트 [rad/s] (트림 추정 전용)
    rclcpp::Time yaw_rate_last_recv_{0, 0, RCL_ROS_TIME};
    bool yaw_rate_seen_ = false;

    double steering_trim_ = 0.0;             // 추정된 트림 [rad], 발행 명령에 더해진다
    double steering_trim_gain_ = 0.0;        // 1/τ [1/s], 0 = 비활성
    double steering_trim_limit_ = 0.06;      // |trim| 상한 [rad] (≈3.4°)
    double steering_trim_max_steer_ = 0.10;  // 이 각을 넘는 조향 중엔 학습 정지 [rad]
    double steering_trim_min_speed_ = 2.0;   // 이 속도 미만에선 학습 정지 [m/s]
    double steering_trim_max_lat_acc_ = 2.0; // 이 횡가속을 넘으면 학습 정지 [m/s²]
    double steering_trim_lag_ = 0.14;        // 조향→요레이트 지연 [s] (0810 bag 실측 140 ms)
    long   steering_trim_samples_ = 0;       // 학습 샘플 수 (로그용)
    std::deque<std::pair<double, double>> steer_hist_;   // (t, 발행 조향) — lag 조회용

    // 곡률 사전감속
    size_t curvature_lookahead_count_;
    double max_lateral_accel_;
    double understeer_gradient_ = 0.019;     // K_us [rad/(m/s²)] — 조향 권한 캡, 0이면 비활성
    double steer_authority_ratio_ = 0.85;
    double curvature_ff_blend_;

    // 좌우 조향 한계 [rad]. 둘 다 같으면 기존 대칭 거동과 동일.
    double max_steering_left_ = MAX_STEERING_ANGLE;
    double max_steering_right_ = MAX_STEERING_ANGLE;
    double steer_limit_min_ = MAX_STEERING_ANGLE;   // 속도 캡용 보수값

    SteeringLookupTable lookup_table_;

    // 차량 상태 / 출력 이력
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0, current_speed_ = 0.0;
    double last_target_speed_ = 0.0, last_steering_angle_ = 0.0;
    double last_published_speed_ = 0.0;      // 발행 acceleration(명령 속도 미분)의 기준
    rclcpp::Time last_time_;

    // odom 수신 여부 — 위치추정이 뜨기 전에는 주행하지 않는다
    bool odom_seen_ = false;

    // 경로 & 인덱스 추적
    std::vector<Waypoint> waypoints_;        // 글로벌 (닫힌 루프)
    std::vector<Waypoint> local_waypoints_;  // 로컬 (회피/추월 포함, 신선하면 우선)
    double avg_waypoint_spacing_ = 0.36;     // 수신 전 보수적 기본값
    size_t last_target_idx_ = 0, last_local_idx_ = 0;
    bool waypoints_initialized_ = false;
    bool local_is_closed_ = false, last_logged_local_closed_ = false;
    rclcpp::Time local_last_recv_time_;
    double local_fresh_timeout_ = 0.3;
    double closest_idx_max_heading_err_ = 1.40;  // 경로 진행방향 게이트 [rad], 0=비활성
    uint32_t local_heading_reject_count_ = 0;    // 로컬 경로 반전 거부 누적(진단용)
    // 자율 체결 게이트 (bumpless transfer)
    bool engage_gate_enable_ = true;
    // ── 섹터 스케일 상태 ──
    bool sector_scale_enable_ = false, sector_scale_global_only_ = true;
    std::string sector_scale_topic_ = "/sector_scales", sector_scale_state_topic_ = "/state";
    double sector_scale_max_ = 1.5, sector_scale_blend_ = 0.5, sector_scale_track_len_tol_ = 0.02;
    std::vector<Sector> sectors_;
    std::vector<SectorTrans> sector_trans_;
    bool sector_table_seen_ = false;   // 테이블을 한 번이라도 받았나
    bool sector_len_ok_ = false;       // 그 테이블이 지금 라인과 같은 라인 기준인가
    // ⚠️ 기본 false다. "글로벌 추종 중"이 확인되기 전에는 켜지 않는다 — true로 두면
    //    /state 발행자가 없을 때 회피 여부를 모르는 채로 스케일이 먹는다.
    bool sector_on_global_ = false, sector_state_seen_ = false;
    double sector_scale_state_timeout_ = 1.0;
    double sector_scale_timeout_ = 3.0;              // 테이블 데드맨, 0이면 비활성
    rclcpp::Time sector_table_last_recv_time_;
    bool sector_table_fresh_prev_ = true;            // 데드맨 전이 검출용
    double sector_table_sig_ = 0.0;                  // 재발행 로그 억제용 내용 서명
    rclcpp::TimerBase::SharedPtr sector_deadman_timer_;
    double sector_decl_track_len_ = 0.0;  // 테이블이 선언한 랩 길이
    double sector_track_len_ = 0.0;       // 실제 글로벌 경로에서 잰 랩 길이
    rclcpp::Time sector_state_last_recv_time_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sector_scale_sub_;
    rclcpp::Subscription<f110_msgs::msg::StateMachine>::SharedPtr sector_state_sub_;

    std::string drive_mode_topic_ = "/drive_mode", engaged_mode_value_ = "autonomous";
    double drive_mode_timeout_ = 1.0;
    bool is_engaged_ = false, drive_mode_seen_ = false;
    rclcpp::Time drive_mode_last_recv_time_;

    // ROS 2 통신
    std::string odom_topic_;
    std::string odom_frame_ = "map";   // odom header.frame_id (L1 마커 프레임)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_mode_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_path_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr local_path_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr l1_marker_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlMapNode>());
    rclcpp::shutdown();
    return 0;
}
