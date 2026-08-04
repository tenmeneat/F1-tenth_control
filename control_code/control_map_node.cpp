#include <chrono>
#include <cmath>
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

#include "f1tenth_control/types.hpp"
#include "f1tenth_control/imu_stability_controller.hpp"
#include "f1tenth_control/steering_lookup_table.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace f1tenth_control;

namespace {

// 조향각 물리 한계 [rad] — 좌우 한계 파라미터의 기본값(대칭 하드웨어 기준).
// ⚠️ 실차는 서보 트림이 기계 중심에서 밀려 좌 0.41 / 우 0.379로 비대칭이다. 이 값은 젯슨
//    vesc.yaml의 servo_min/servo_max와 **반드시 한 쌍**으로 움직인다 — 컨트롤러만 올리면
//    vesc_driver가 조용히 자르고 컨트롤러는 "꺾었다"고 착각한다.
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

// 곡률 사전감속용 물리거리 창 평활 곡률.
// wp.curvature는 인접점 헤딩차분이라 웨이포인트가 촘촘하면 단일점 kappa가 실제 지속 곡률보다
// 크게 튄다. 그대로 쓰면 노이즈 스파이크 하나로 프로파일 속도보다 훨씬 낮게 과잉감속한다.
// ⚠️ 글로벌·로컬 **양쪽 모두**에 적용할 것 — 팀 플래너의 /local_waypoints는 짧은 회피
//    세그먼트가 아니라 191점 풀랩이라 "로컬은 짧으니 불필요" 가정이 깨진다(2026-07-21).
// 원본 wp.curvature는 FF 조향 등 다른 용도로 그대로 둔다.
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

        // L1 Guidance: L1 = clamp(l1_offset + v·l1_speed_gain,
        //                        max(t_clip_min, √2·lat_err), t_clip_max)
        // 이름이 역할과 반대였던 구 파라미터(l1_gain=절편, l1_distance=속도계수)를 2026-07-30에
        // 개명했다 — l1_offset[m] = 절편, l1_speed_gain[s] = 속도 계수(원본 Python MAP의 q_l1/m_l1).
        l1_offset_ = declare_parameter<double>("l1_offset", 0.5);
        l1_speed_gain_ = declare_parameter<double>("l1_speed_gain", 0.3);
        // 구 이름 호환 shim: 명시적으로 넘어온 경우에만 신 이름을 덮는다(기본은 NaN = 미지정).
        // ⚠️ 조용히 무시하면 "튜닝했는데 안 바뀐다"가 되므로 반드시 경고를 띄운다.
        {
            const double kUnset = std::numeric_limits<double>::quiet_NaN();
            double legacy_offset = declare_parameter<double>("l1_gain", kUnset);
            double legacy_slope  = declare_parameter<double>("l1_distance", kUnset);
            if (!std::isnan(legacy_offset)) {
                l1_offset_ = legacy_offset;
                RCLCPP_WARN(this->get_logger(),
                    "구 파라미터 l1_gain=%.3f 사용 중 — l1_offset[m]으로 개명됐다(절편). "
                    "런치/명령줄을 l1_offset으로 바꿀 것", legacy_offset);
            }
            if (!std::isnan(legacy_slope)) {
                l1_speed_gain_ = legacy_slope;
                RCLCPP_WARN(this->get_logger(),
                    "구 파라미터 l1_distance=%.3f 사용 중 — l1_speed_gain[s]으로 개명됐다(속도 계수). "
                    "런치/명령줄을 l1_speed_gain으로 바꿀 것", legacy_slope);
            }
        }
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
        // 위 두 스케일러의 **완전 적용 기준 종가속도** [m/s²]. 예전엔 acc_mean이 ±1.0을 넘는
        // 순간 스케일러가 계단으로 붙었다(0.95배 = 조향 5% 점프). 실측 coast 감속이 −0.4라
        // 감속측은 급제동 스파이크에서만 드물게 튀는, 가장 나쁜 형태였다 → 0~ref 구간
        // 선형 블렌딩으로 바꿨다(ref 이상에서 구 거동과 동일).
        steering_scaler_accel_ref_ =
            std::max(0.05, declare_parameter<double>("steering_scaler_accel_ref", 1.0));
        start_scale_speed_ = declare_parameter<double>("start_scale_speed", 7.0);
        end_scale_speed_ = declare_parameter<double>("end_scale_speed", 8.0);
        downscale_factor_ = declare_parameter<double>("downscale_factor", 0.10);

        // 조향 도달각 비율 — 명령한 조향각 중 **바퀴가 실제로 내는** 비율.
        // 🔴 2026-07-28 실차 3회 재현: 0.41 rad을 명령해도 실측 ~0.30 rad(74%)뿐이고,
        //    당시 횡가속 1.09 m/s²라 슬립으로 설명이 안 된다 = 기계적(링키지/서보 트림).
        // 이 상수 하나가 두 곳을 동시에 지배한다(예전엔 둘이 어긋나 있었다):
        //   ① 조향 명령 보상: LUT가 낸 각을 바퀴가 실제로 내도록 1/ratio를 곱한다(control_loop 6-6).
        //      ⚠️ 예전에는 이 자리에 `*= clamp(1 + v/10, 1.0, 1.4)`가 **하드코딩**돼 있었다.
        //         1/0.74 = 1.35 ≈ 1.4라 사실상 이 도달각 보상이었지만, 기계적 손실은 속도와
        //         무관한데 속도 램프로 만들어놔서 4 m/s에서 천장에 붙는 이상한 모양이었고
        //         이름·문서·파라미터가 전부 없었다(바로 윗줄 downscale_factor와도 싸웠다).
        //   ② 조향 권한 속도 캡(②-b)의 δ_avail: 캡은 명령각이 아니라 **도달각**으로 계산해야
        //      한다. 예전엔 0.379를 다 낼 수 있다고 보고 코너 진입 속도를 과대 허용했다.
        // 1.0으로 두면 보상·캡 모두 구 낙관 거동(각도기 실측 후 조정할 값).
        steering_reach_ratio_ =
            std::clamp(declare_parameter<double>("steering_reach_ratio", 0.74), 0.3, 1.0);

        speed_lookahead_ = declare_parameter<double>("speed_lookahead", 0.15);
        speed_lookahead_for_steering_ =
            declare_parameter<double>("speed_lookahead_for_steering", 0.0);
        std::string lut_file = declare_parameter<std::string>("lookup_table_file", "");

        base_max_accel_ = declare_parameter<double>("base_max_accel", 4.0);
        // ⚠️ 감속도 파라미터가 둘인 이유 — 튜닝 방향이 정반대다.
        //   base_max_decel : 명령 속도의 하강 rate limit. 낮추면 감속 명령이 늦게 도달 → 높게 유지.
        //   prebrake_decel : 차가 **실제로 낼 수 있는** 감속도(제동거리 v²/2a 산출용).
        //     07-25 실차 실측은 -0.4 m/s²(VESC 속도모드는 회생제동이 거의 없어 사실상 coast).
        //     8.0을 쓰면 4 m/s에서 제동거리를 1.0m로 착각한다(실제 필요 ~8m) → 시케인 크래시.
        base_max_decel_ = declare_parameter<double>("base_max_decel", 8.0);
        prebrake_decel_ = declare_parameter<double>("prebrake_decel", 1.5);

        // 런치 킥(자율 정지출발 시 센서리스 데드존 관통) — control_loop 8-c
        launch_boost_enable_ = declare_parameter<bool>("launch_boost_enable", true);
        launch_boost_speed_ = declare_parameter<double>("launch_boost_speed", 2.2);
        launch_boost_time_ = declare_parameter<double>("launch_boost_time", 0.6);
        launch_exit_speed_ = declare_parameter<double>("launch_exit_speed", 0.8);
        launch_standstill_speed_ = declare_parameter<double>("launch_standstill_speed", 0.3);

        // IMU. 단위 보정 계수의 실제 값은 런치가 넘긴다(_control_common.py IMU_*_SCALE).
        use_imu_ = declare_parameter<bool>("use_imu", true);
        imu_angular_scale_ = declare_parameter<double>("imu_angular_scale", 1.0);
        imu_linear_scale_ = declare_parameter<double>("imu_linear_scale", 1.0);
        yaw_rate_gain_ = declare_parameter<double>("yaw_rate_gain", 0.1);

        max_speed_ = declare_parameter<double>("max_speed", 12.0);
        min_speed_ = declare_parameter<double>("min_speed", 2.0);

        // 곡률 룩어헤드 사전감속
        curvature_lookahead_count_ =
            static_cast<size_t>(declare_parameter<int>("curvature_lookahead_count", 60));
        max_lateral_accel_ = declare_parameter<double>("max_lateral_accel", 6.0);
        // 조향 권한 캡 — 곡률 캡이 **그립만** 보던 구멍을 메운다. 그립("타이어가 그 횡가속을
        // 낼 수 있나")과 조향("바퀴가 그만큼 꺾일 수 있나")은 다른 물리다.
        //   정상상태 자전거 모델 δ = L·κ + K_us·κ·v² ≤ δ_avail  ⇒  v ≤ √((δ_avail − L·κ)/(K_us·κ))
        // 07-26 실차 κ=1.190(R=0.84m) 헤어핀: 그립 한계 2.11 m/s vs 조향 한계 0.87 m/s —
        // 조향이 먼저 걸린다. 그립만 보고 2배 빠르게 진입해 풀락에도 안 돌아가고 이탈했다.
        // 0이면 이 항 전체 비활성(구 거동).
        understeer_gradient_ = declare_parameter<double>("understeer_gradient", 0.019);
        // δ_max 중 곡률 추종에 배정할 비율. 나머지는 횡오차 보정·요레이트 피드백 여유.
        steer_authority_ratio_ = declare_parameter<double>("steer_authority_ratio", 0.85);
        curvature_ff_blend_ = declare_parameter<double>("curvature_ff_blend", 0.0);
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");

        // L1 횡가속 분모로 목표점까지의 **실제** 직선거리를 쓸지. 목표점은 호 길이 기준으로
        // 고르므로 |목표점−차량| != L1_distance다(07-27 bag 실측 비율 중앙 1.06~1.31, p95 1.72).
        // 명목값을 분모로 쓰면 횡가속 명령이 최대 +70% 과대해지고, 경로에서 벗어날수록
        // = 복귀가 필요한 바로 그 순간에 더 심해진다. false면 구 거동(즉시 롤백용).
        l1_use_actual_distance_ = declare_parameter<bool>("l1_use_actual_distance", true);

        // 조향 rate limit [rad/s]. ⚠️ 예전엔 "사이클당 0.4 rad" 하드코딩이었다 — 50Hz에서
        // 20 rad/s = 풀락까지 2 사이클(40ms)이라 제한이 있는 척만 하고 아무것도 안 막았고,
        // dt와 무관해서 루프가 밀리면 실효 제한이 더 느슨해졌다. 기본값 20.0은 구 거동과 동일
        // (0.4/0.02). 서보 물리 속도(~7 rad/s 추정)로 낮추면 고주파 조향 채터링을 막을 수 있으나
        // 실측 전이라 기본은 중립으로 둔다.
        max_steering_rate_ = std::max(0.5, declare_parameter<double>("max_steering_rate", 20.0));

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

        // 자율 미체결 중 속도 램프 고정 (bumpless transfer).
        // 이 노드는 /drive_mode를 모른 채 상시 돌기 때문에 MANUAL/E-stop으로 서 있는 동안에도
        // 램프가 감겨 올라가고, engage 순간 그 값이 계단으로 VESC에 꽂힌다(07-27 bag 8개 전부:
        // 정차 중 명령 1.50~3.98 → engage 시 0→6348 ERPM 한 스텝 → 모터전류 60~62A 포화).
        // ⚠️ 체결 중에는 아무것도 하지 않는다 — 07-22에 금지한 일반 lead-clamp와 다르다.
        // ⚠️ /drive_mode 미수신·끊김 시 게이트 자동 비활성(시뮬 호환).
        engage_gate_enable_ = declare_parameter<bool>("engage_gate_enable", true);
        drive_mode_topic_ = declare_parameter<std::string>("drive_mode_topic", "/drive_mode");
        engaged_mode_value_ = declare_parameter<std::string>("engaged_mode_value", "autonomous");
        drive_mode_timeout_ = declare_parameter<double>("drive_mode_timeout", 1.0);

        // 경로 소스 중재
        local_fresh_timeout_ = declare_parameter<double>("local_fresh_timeout", 0.3);

        acc_now_ = std::vector<double>(10, 0.0);

        // ── 2. LUT 로드 (다중 경로 Fallback — 전부 ament 경로, 하드코딩 홈 경로 없음) ──
        // f1tenth_control 자신의 cfg를 먼저 본다 — steering_lookup은 서드파티 패키지라
        // 파일명이 우연히 겹치면(과거 NUC6_glc_pacejka_lookup_table.csv가 그랬다) 우리
        // 보정본이 조용히 안 먹히고 그쪽의 미보정 원본이 로드된다. 이름을 LUT_calibrated.csv로
        // 바꾼 것도 그 충돌을 피하기 위함(steering_lookup엔 이 이름의 파일이 없다) — 순서까지
        // 같이 뒤집어 어느 한쪽만 믿어도 되게 한다.
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

        stability_controller_ = std::make_unique<StabilityController>(0.2);  // alpha_yaw_rate

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

        current_x_ = x;
        current_y_ = y;
        current_yaw_ = yaw;
        current_speed_ = v;
        odom_seen_ = true;
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        // use_imu=false는 "IMU를 신뢰하지 않는다"는 뜻이므로 파생값을 전부 쓰지 않는다.
        // acc_now_는 0 초기화 상태로 남아 acc_mean=0 → 스케일러 중립(1.0)으로 안전히 떨어진다.
        if (!use_imu_) return;

        // ⚠️ VESC 자이로는 deg/s로 발행한다(2026-07-19 실차 확인). 보정 안 하면 실측 요레이트가
        //    57.3배가 되어 카운터스티어가 즉시 반대로 포화한다.
        stability_controller_->update_yaw_rate(msg->angular_velocity.z * imu_angular_scale_);

        // 종가속 rolling buffer (조향 가감속 스케일러용).
        // ⚠️ VESC 가속도계는 m/s²가 아니라 g로 발행한다(imu_linear_scale로 환산).
        // ⚠️ 장착 회전은 180°다 — **전방 = −a_x**, 좌측 = −a_y, 위 = +z (2026-07-29 확정).
        //    독립적인 두 방법이 부호까지 일치: bag 회귀(−a_x↔dv/dt R²=0.787, −a_y↔v·ψ̇ R²=0.958),
        //    VESC Tool 정지 자세(수평 z=+1.04 / 앞코위 x=−1.00 / 좌측눕힘 y=+0.95).
        //    이전엔 −a_y(횡방향!)를 종방향으로 써서 스케일러가 **우선회에서만** 걸렸다.
        //    이 매핑은 젯슨 VESC의 `Imu Rotation Yaw`(현재 −90°)와 한 쌍 — 그 값이 바뀌면
        //    여기 부호도 같이 바뀌어야 한다(조용히 깨지는 결합).
        std::rotate(acc_now_.rbegin(), acc_now_.rbegin() + 1, acc_now_.rend());
        acc_now_[0] = -msg->linear_acceleration.x * imu_linear_scale_;
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

        RCLCPP_INFO(this->get_logger(), "🔄 글로벌 경로 수신! 웨이포인트 %zu개, 초기 인덱스 %zu",
                    waypoints_.size(), last_target_idx_);
    }

    // 로컬 경로: 상류 플래너의 전방 구간을 그대로 저장한다.
    // ⚠️ "짧은 열린 구간"이라고 가정하지 않는다 — 팀 플래너 구성에 따라 글로벌과 같은
    //    풀랩(닫힌 루프)이 실려 올 수 있고, 그걸 열린 경로로 취급하면 배열 끝에서 룩어헤드가
    //    끊긴다. 소스가 아니라 **기하로 판정**한다.
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
        // ⚠️ 램프 **상태**는 0이 아니라 실측으로 물린다(발행값은 아래에서 하드 0).
        //    ramp_speed는 증분을 실측 기준 오차로 정하는데, 오차가 0 이하면 "목표를 넘지 마라"
        //    클램프가 out을 target으로 **끌어올린다**. 그래서 램프 상태가 0인 채 차가 굴러가는
        //    상태에서 정상 제어로 복귀하면 명령이 한 사이클에 0 → target으로 계단 점프한다
        //    (07-27 engage 급발진과 같은 형태). engage 게이트가 쓰는 것과 동일한 bumpless
        //    처리를 안전정지에도 적용해, 복귀 명령이 실측 근처에서 이어지게 한다.
        last_target_speed_ = std::max(0.0, current_speed_);
        last_published_speed_ = 0.0;
        publish_drive(0.0, 0.0, 0.0);
    }

    // 명령 속도 램프(rate limit).
    // ⚠️ 증분은 **실측 속도** 기준 오차로 정하고 직전 **명령**에 더한다(원본 MAP 컨트롤러 규약).
    //    VESC 속도 PID는 ERPM 오차에 비례해 전류를 만들므로 명령이 실측보다 앞서 있어야
    //    가속이 나온다 — 이 선행을 좁히면 가속이 그대로 죽는다(07-22 lead-clamp 금지 사유).
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
        // ⚠️ dt는 **위아래 둘 다** 묶어야 한다. dt는 속도 램프 증분(base_max_*·dt), 런치 킥
        //    타이머, 발행 가속도(Δv/dt)에 전부 곱해지는데 wall_timer는 실시간 보장이 없다.
        //    젯슨에서 로깅/wifi/MCL 부하로 한 사이클이 0.2s 밀리면 램프가 한 스텝에
        //    8.0×0.2 = 1.6 m/s 튀고(= 계단 명령 = 07-27 급발진과 같은 형태), 반대로 dt가
        //    아주 작으면 Δv/dt가 폭발한다. 정상 20ms의 5배(0.1s)를 상한으로 자른다.
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
        if (!local_fresh && waypoints_.empty()) {
            publish_safe_stop();
            return;
        }

        const std::vector<Waypoint>& wps = local_fresh ? local_waypoints_ : waypoints_;

        // ⚠️ "경로 소스"와 "경로 기하"를 분리한다.
        //   following_local : 로컬 경로 추종 중인가 (인덱스 추적기 선택용)
        //   path_closed     : 그 경로가 실제로 닫힌 루프인가 (wrap 여부 — walk_forward/윈도우용)
        // 예전엔 `closed = !local_fresh` 하나로 겸했는데, /local_waypoints가 191점 풀랩이라
        // 매 랩 배열 끝에서 walk_forward가 끊겨(룩어헤드 truncation) 오프닝 헤어핀의 곡률
        // 사전감속 창이 붕괴했다.
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
            // 윈도우 크기는 고정 인덱스 개수가 아니라 물리 거리(후방 1m·전방 3m) 기준으로 잡는다.
            // 고정 개수면 촘촘한 소스에서 탐색 반경이 줄어, 트랙이 스스로에게 가까워지는 구간에서
            // 진짜 최근접점을 놓치고 엉뚱한 인덱스에 잠긴다(min_dist가 2.5m 밑이면 전역 재탐색도
            // 발동하지 않아 인덱스가 역행/진동하며 조향 포화로 이어짐).
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
            if (min_dist > 2.5) {
                std::tie(min_dist, closest_idx) = scan_closest(wps, current_x_, current_y_);
            }
        } else {
            // 열린 구간(짧은 회피경로): 전체 최근접 스캔(저렴, wrap 인덱스 미사용)
            std::tie(min_dist, closest_idx) = scan_closest(wps, current_x_, current_y_);
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

        // 프로파일 신뢰형 backward-pass: 오프라인 최적화된 vx_mps가 이미 각 지점의 최적 속도를
        // 담고 있다는 전제로, 전방 각 지점의 상한 v_cap[i]까지 prebrake_decel로 감속 가능한
        // 현재 최대 속도 v_reach = √(v_cap² + 2·a·d)의 최소값을 캡으로 쓴다(accum=0 항이
        // 순간 그립 클램프도 겸함). 직선은 κ≈0이라 안 눌리고, 코너는 제동거리만큼 앞에서
        // 정확히 선제동된다. (구 방식인 "창 내 최대 κ로 블랭킷 재캡"은 전 구간 과잉감속이라 폐기)
        double curvature_speed_limit = std::numeric_limits<double>::max();
        double steer_bound_k = 0.0, steer_bound_v = 0.0;   // 진단 로그용
        walk_forward(wps, closest_idx, curv_lookahead_dist, path_closed, [&](size_t i, double accum) {
            double v_cap_i = wps[i].speed;
            double k_i = std::abs(wps[i].smoothed_curvature);
            if (k_i > 0.01) {
                v_cap_i = std::min(v_cap_i, std::sqrt(max_lateral_accel_ / k_i));   // (a) 그립
                if (understeer_gradient_ > 1e-6) {                                  // (b) 조향 권한
                    // ⚠️ 좌우 중 **작은** 한계를 쓰고, 거기에 도달각 비율까지 곱한다 —
                    //    캡은 "바퀴가 실제로 꺾이는 각"으로 계산해야 의미가 있다(0.379를 다
                    //    낸다고 보면 코너 진입 속도를 그만큼 과대 허용한다).
                    double steer_budget = steer_avail() - wheelbase_ * k_i;
                    // 🔴 예전엔 budget ≤ 0(= 기구학적으로 불가능한 코너)일 때 이 캡을
                    //    **통째로 건너뛰어서**, 가장 급한 코너만 그립 캡만 받았다(실측:
                    //    ifac_track_v2 187점 중 7점). budget=0에서 v_steer→0이므로 0으로
                    //    이어 붙이는 것이 연속이고 문서(②-b)가 말하는 거동이다.
                    //    이후 backward-pass가 그 지점까지 정상 제동 프로파일을 만들고,
                    //    최종 max(min_speed, ...)가 정지를 막는다.
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

        // 3. sin(eta) — 차량 헤딩과 L1 목표점 사이의 횡방향 성분
        double L1_vector_x = L1_x - current_x_;
        double L1_vector_y = L1_y - current_y_;
        double L1_norm = std::hypot(L1_vector_x, L1_vector_y);
        double sin_eta = 0.0;
        if (L1_norm > 1e-5) {
            double lat = -std::sin(current_yaw_) * L1_vector_x + std::cos(current_yaw_) * L1_vector_y;
            sin_eta = std::clamp(lat / L1_norm, -1.0, 1.0);
        }

        // 4. 조향용 속도(speed_for_lu): 룩어헤드 예측 위치의 프로파일 속도
        // ❌ 2026-07-30: 여기 있던 `lat_err_scale`(횡오차 기반 속도 감쇠)을 **제거**했다.
        //    이유 3가지 — 되살리기 전에 읽을 것:
        //    ① **죽은 코드였다.** curv_factor = clamp(2·(mean|κ|/0.8) − 2, 0, 1)이라 랩 전체
        //       평균 |κ| ≥ 0.8 rad/m(평균 반경 1.25m)이어야 켜지는데, ifac_track_v2 실측
        //       평균은 0.273이다(트랙이 2.9배 더 꼬여야 함) → 항상 정확히 1.0. 조향용 속도와
        //       target_speed 두 곳 모두 무효였다. 즉 제거는 거동 변화 0.
        //    ② **모양이 레이싱에 못 쓴다.** 완전 발동 시 exp(−1) = 0.368, 횡오차 0.5m에서
        //       속도를 63% 깎는다. MCL 지터 수준의 오차로도 랩타임이 붕괴한다.
        //    ③ 라인 복귀 감속 전용 기구와 같은 신호에 모양이 다른 감속을 둘씩 걸면
        //       서로 싸운다(yaw_rate_gain ↔ 언더스티어 가드로 이미 겪은 패턴).
        double speed_for_lu =
            wps[find_lookahead_wp_idx(wps, path_closed, closest_idx, speed_lookahead_for_steering_)].speed;

        // 5. 목표 횡가속도 → LUT 조향각
        double lat_acc = 0.0;
        speed_for_lu = std::min(speed_for_lu, curvature_speed_limit);
        // ⚠️ 분모는 목표점까지의 **실제 직선거리**다(l1_use_actual_distance, 선언부 주석 참고).
        //    하한 l1_min_denom은 목표점이 차량에 붙은 경우(L1_norm→0) 발산 방지 — t_clip_min을
        //    재사용하던 것을 2026-07-30에 분리했다(룩어헤드 노브가 횡가속 상한을 조용히 흔들었다).
        double l1_denom = l1_use_actual_distance_ ? std::max(L1_norm, l1_min_denom_)
                                                  : std::max(L1_distance, l1_min_denom_);
        lat_acc = 2.0 * speed_for_lu * speed_for_lu / l1_denom * sin_eta;

        bool lut_saturated = false;
        double steering_angle = lookup_table_.lookup_steer_angle(lat_acc, speed_for_lu, &lut_saturated);

        // 5-b. LUT 속도축 상한(7.0 m/s) 초과 보정. 축을 넘으면 LUT가 끝 열로 클램프되는데,
        //      같은 lat_acc에 대해 느린 열은 **더 큰** 조향각을 준다(κ = a/v²) → 최고속에서
        //      과대 조향. 정상상태 자전거모델의 기구학 항 L·κ만큼 빼서 보정한다
        //      (타이어 슬립항 K_us·a_lat은 속도에 직접 의존하지 않아 그대로 유효).
        //      ⚠️ 크기는 작다(a_lat 3.0, 7→8 m/s에서 5 mrad). 근본 해결은 LUT CSV를
        //         9 m/s까지 재생성하는 것.
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

        // 5-c. LUT 그립 포화 진단(제어 개입 없음). 포화 중에는 lat_acc가 얼마나 커도 조향각이
        //      같아서 **조향 피드백이 개루프**가 된다 — 횡오차가 커지는 바로 그 순간에 복구
        //      권한이 없다는 뜻이다. 자주 뜨면 원인은 조향이 아니라 진입 속도(사전감속)다.
        //      ⚠️ 저속은 게이트로 제외한다 — v<2.5에서는 LUT 피크 조향각 자체가 ~0.39 rad(거의
        //         풀락)이라 헤어핀에서 포화가 **정상**이다(실측: v=0.5, a_lat=3.0 → 0.390 rad
        //         SAT). 위험한 건 고속 포화(피크각이 0.12 rad밖에 안 되는데 그마저 다 쓴 상태)다.
        if (lut_saturated && std::abs(sin_eta) > 0.05 && speed_for_lu > 2.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "LUT 그립 포화: 요구 a_lat %.2f m/s² @ %.2f m/s (조향 %.3f rad에서 saturate) — "
                "조향 피드백 개루프 상태. 코너 진입 속도/prebrake_decel 확인",
                std::abs(lat_acc), speed_for_lu, std::abs(steering_angle));
        }

        // 6. 조향각 보정 ─────────────────────────────────────────────────────────────
        // 6-1) 가감속 스케일링. ⚠️ 예전엔 acc_mean이 ±1.0을 **넘는 순간** 스케일러가 계단으로
        //      붙었다(조향 5% 점프). 실측 coast 감속이 −0.4 m/s²라 감속측은 급제동 스파이크에서만
        //      드물게 튀는 최악의 형태였고, 임계 근처를 오가면 50Hz 채터링이다.
        //      → 0 ~ steering_scaler_accel_ref 구간 선형 블렌딩(ref 이상은 구 거동과 동일).
        double acc_mean = 0.0;
        for (double a : acc_now_) acc_mean += a;
        acc_mean /= acc_now_.size();
        {
            const double w = std::clamp(std::abs(acc_mean) / steering_scaler_accel_ref_, 0.0, 1.0);
            const double target_scaler = (acc_mean >= 0.0) ? acceleration_scaler_for_steering_
                                                           : deceleration_scaler_for_steering_;
            steering_angle *= (1.0 - w) + w * target_scaler;
        }

        // 6-2) 속도 구간 다운스케일
        // ❌ 2026-07-30: 여기 있던 `*= clamp(1 + v/10, 1.0, 1.4)`를 제거하고 도달각 보상
        //    (6-6, steering_reach_ratio)으로 대체했다. 값 자체는 1/0.74 = 1.35 ≈ 1.4라
        //    사실상 같은 보상이었지만, ⓐ 기계적 손실인데 속도 램프 모양이라 4 m/s에서 천장에
        //    붙었고 ⓑ 바로 이 줄의 다운스케일(−10%)과 정면으로 싸웠고 ⓒ 이름·파라미터·문서가
        //    없었고 ⓓ 조향 권한 캡(②-b)의 δ_avail과 어긋나 있었다(캡은 0.379를 다 낸다고 가정).
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

        // 6-5) 요레이트 피드백 카운터스티어. 명령 조향각이 기하학적으로 의도하는 기대 요레이트
        //      (v·tanδ/L) 대비 IMU 실측의 오차에 비례해 보정한다(언더스티어 시 더 꺾음).
        //      rate limit·클리핑 **이전**에 더해 보정분까지 안전 한계 안으로 수렴시킨다.
        if (use_imu_) {
            steering_angle += stability_controller_->calculate_yaw_rate_correction(
                current_speed_, steering_angle, wheelbase_, yaw_rate_gain_);
        }

        // 6-6) 조향 도달각 보상 — 명령각 중 바퀴가 실제로 내는 비율이 74%(실차 3회 재현)라
        //      LUT/보정항이 의도한 각을 바퀴가 내도록 1/ratio를 곱한다.
        //      ⚠️ **모든 보정항 뒤, 클리핑 앞**이 유일하게 맞는 자리다 — 보정항(요레이트·
        //         heading·FF)도 같은 링키지를 통과하므로 함께 보상돼야 한다.
        if (steering_reach_ratio_ < 0.999) steering_angle /= steering_reach_ratio_;

        // 6-7) rate limit → 좌우 물리 한계 (δ>0 = 좌). 하드웨어가 못 내는 각을 명령해봐야
        //      vesc_driver의 servo_limit이 조용히 자를 뿐이고 컨트롤러는 그걸 모른다.
        //      ⚠️ rate limit은 dt에 비례해야 한다 — 예전엔 "사이클당 0.4 rad" 하드코딩이라
        //         루프가 밀리면 실효 제한이 느슨해졌고, 50Hz에서 20 rad/s = 사실상 무제한이었다.
        const double steer_step = max_steering_rate_ * dt;
        steering_angle = std::clamp(steering_angle,
                                    last_steering_angle_ - steer_step,
                                    last_steering_angle_ + steer_step);
        steering_angle = std::clamp(steering_angle, -max_steering_right_, max_steering_left_);
        last_steering_angle_ = steering_angle;

        // 7. 목표 속도 ───────────────────────────────────────────────────────────────
        double global_speed =
            wps[find_lookahead_wp_idx(wps, path_closed, closest_idx, speed_lookahead_)].speed;
        global_speed = std::min(global_speed, curvature_speed_limit);
        // 직선 최고속도 캡. 곡률 제한은 코너에서만 걸리므로(직선은 κ≈0) 이 줄이 컨트롤러 쪽
        // 유일한 상한이다 — 2026-07-19 이전엔 이 clamp가 빠져 max_speed:=X가 무효였다.
        global_speed = std::min(global_speed, max_speed_);
        // ❌ 2026-07-30: 여기 있던 `* lat_err_scale`을 제거했다(항상 1.0인 죽은 코드 + 모양이
        //    레이싱에 부적합 — 위 4 참고).
        double target_speed = global_speed;

        // 8. 명령 속도 램프
        double final_speed = ramp_speed(last_target_speed_, target_speed, dt,
                                        base_max_accel_, base_max_decel_);
        last_target_speed_ = final_speed;

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
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "자율 미체결 — 속도 램프를 실측(%.2f m/s)에 고정 중(engage 시 무충격 전환)",
                current_speed_);
        }

        // 8-c. 런치 킥 — 자율 정지출발 시 VESC 센서리스 데드존 관통.
        //   매뉴얼은 초반 스로틀 펀치로 데드존(~0.5 m/s)을 때려 관통하는데, 자율은 프로파일을
        //   살살 램프해 명령이 데드존에 걸터앉아 탈조한다. 정지 상태에서 짧게 높은 속도를 명령하면
        //   속도 PID가 ERPM 오차에 비례해 큰 전류를 뽑아 매뉴얼 펀치와 같은 효과가 난다.
        //   (오픈루프 전류 상향·HFI·Coupled HFI는 이 모터의 저돌극성 때문에 부하서 실패 확인)
        //   ⚠️ final_speed(램프 상태)는 건드리지 않고 발행값만 덮는다 → 킥 종료 후 램프가 이어짐.
        //   ⚠️ 킥 실패 시 포기한다(과열 방지 위해 차가 실제 움직일 때까지 재시도 안 함).
        //   ⚠️ 미체결 중에는 킥도 돌리지 않는다 — 킥은 발행값만 덮으므로 게이트가 램프를 눌러놔도
        //      킥이 켜져 있으면 정차 중 부스트 속도가 계속 발행된다.
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
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                            "런치 킥: 실측 %.2f → 발행 %.2f m/s (t=%.2fs)",
                            current_speed_, publish_speed, launch_time_);
                    }
                }
            } else {
                launch_active_ = false;   // 정지 명령 중엔 킥 안 함
            }
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Pose: (%.2f, %.2f, %.2f) | Target WP: (%.2f, %.2f), Idx: %zu -> %zu | Steer: %.4f | "
            "Speed: %.2f / %.2f | L1_dist: %.2f | acc_mean: %.2f",
            current_x_, current_y_, current_yaw_, L1_x, L1_y, closest_idx, idx_a, steering_angle,
            final_speed, current_speed_, L1_distance, acc_mean);

        // 9. 발행. ⚠️ acceleration 필드는 **명령 속도의 시간미분**이다 —
        //    예전엔 `(publish_speed − current_speed_)/dt`, 즉 "명령−실측 추종오차 ÷ dt"를
        //    가속도라고 발행했다. VESC 속도 PID는 명령이 실측보다 앞서야 전류가 나오는 구조라
        //    (60A를 뽑으려면 ~4.7 m/s 선행) 정상 가속 중에도 이 값이 200 m/s²급으로 나오고,
        //    odom 속도 노이즈가 ×50(=1/dt) 증폭돼 실렸다. 하류가 이 필드로 제동을 중재하면
        //    (젯슨 ackermann_to_vesc 서비스 브레이크 패치) 그대로 오작동한다.
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

    // 룩어헤드 투영점(현재 속도로 lookahead_time만큼 직진한 위치) 기준 최근접 웨이포인트.
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

    // 조향 스케일러 / 속도 룩어헤드
    double acceleration_scaler_for_steering_, deceleration_scaler_for_steering_;
    double steering_scaler_accel_ref_ = 1.0;  // 가감속 스케일러 완전 적용 기준 |a| [m/s²]
    double start_scale_speed_, end_scale_speed_, downscale_factor_;
    double speed_lookahead_, speed_lookahead_for_steering_;
    // 명령각 중 바퀴가 실제로 내는 비율. 조향 명령 보상(1/ratio)과 조향 권한 캡의 δ_avail을
    // 동시에 지배한다(steer_avail()). 1.0이면 둘 다 구 낙관 거동.
    double steering_reach_ratio_ = 0.74;
    double max_steering_rate_ = 20.0;         // 조향 rate limit [rad/s] (dt 비례)

    // 종방향
    double base_max_accel_;
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

    // IMU
    bool use_imu_;
    double imu_angular_scale_, imu_linear_scale_ = 1.0;
    double yaw_rate_gain_;
    std::vector<double> acc_now_;            // 종가속 rolling buffer

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
    // 자율 체결 게이트 (bumpless transfer)
    bool engage_gate_enable_ = true;
    std::string drive_mode_topic_ = "/drive_mode", engaged_mode_value_ = "autonomous";
    double drive_mode_timeout_ = 1.0;
    bool is_engaged_ = false, drive_mode_seen_ = false;
    rclcpp::Time drive_mode_last_recv_time_;

    std::unique_ptr<StabilityController> stability_controller_;

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
