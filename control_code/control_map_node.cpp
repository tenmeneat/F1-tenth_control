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
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "f1tenth_control/types.hpp"
#include "f1tenth_control/longitudinal_safety.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "f110_msgs/msg/state_machine.hpp"

// ⚠️ `ament_index_cpp`와 `using namespace f1tenth_control;`은 2026-08-17에 함께 제거됐다.
//    둘 다 LUT 로드 경로(share/cfg CSV 폴백 + steering_lookup_table.hpp의 네임스페이스)
//    전용이었고, LUT 삭제로 소비처가 사라졌다. `types.hpp`의 Waypoint는 전역 스코프다.

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
    std::vector<double> smoothed_signed(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        double sum = 0.0, sum_signed = 0.0;
        int cnt = 0;
        for (int off = -half_n; off <= half_n; ++off) {
            int idx = i + off;
            if (closed) {
                idx = ((idx % n) + n) % n;
            } else {
                if (idx < 0 || idx >= n) continue;  // 열린 경로: 창을 배열 안으로 자른다
            }
            sum += std::abs(wps[static_cast<size_t>(idx)].curvature);
            sum_signed += wps[static_cast<size_t>(idx)].curvature;
            ++cnt;
        }
        smoothed[static_cast<size_t>(i)] =
            (cnt > 0) ? (sum / cnt) : std::abs(wps[static_cast<size_t>(i)].curvature);
        // 🔑 부호 있는 평균은 **따로** 낸다. |κ|를 평균한 뒤 부호를 붙이면 S자 구간에서
        //    좌우가 상쇄되지 않아 크기가 부풀고, 부호를 어디서 가져올지도 애매해진다.
        smoothed_signed[static_cast<size_t>(i)] =
            (cnt > 0) ? (sum_signed / cnt) : wps[static_cast<size_t>(i)].curvature;
    }
    for (int i = 0; i < n; ++i) {
        wps[static_cast<size_t>(i)].smoothed_curvature = smoothed[static_cast<size_t>(i)];
        wps[static_cast<size_t>(i)].smoothed_curvature_signed =
            smoothed_signed[static_cast<size_t>(i)];
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

        // ── 조향 생성: 정상상태 자전거 역모델 + FF/FB 분리 (②-p) ───────────────
        // δ = a_lat·(L/v² + K_us). 곡률 사전감속(control_loop 1.5)의 조향 권한 캡이
        // 쓰는 것과 **같은 모델**이다 — 이 일원화가 ②-p의 핵심이다.
        // ⚠️ 2026-08-17 이전엔 여기가 LUT 역조회였고, 그래서 컨트롤러가 서로 다른 차량
        //    모델 둘을 갖고 있었다: 종방향은 "이 속도면 δ_avail 안에서 꺾인다"고 판단하는데
        //    LUT는 NaN 절단 때문에 훨씬 아래에서 막혔다(v=5.0에서 필요 조향의 절반,
        //    v=6.0에서 3분의 1). 종방향이 허용한 속도를 횡방향이 못 따라가는 구조였고,
        //    실제로 7.7 m/s 최고기록 주행이 조향 사이클의 37%를 그 천장에 붙어 달렸다.
        //    LUT 메커니즘은 삭제됐다 — 롤백이 필요하면 git 0d16173 참고.
        // FF/FB 분리 게인. L1이 만든 횡가속 명령 중 **경로 곡률로 설명되지 않는 몫**에만
        // 곱한다. 🔑 모델이 선형이라 1.0이면 분리 전(= a_ff + a_fb = lat_acc)과 수학적으로
        // 정확히 동일하다 → 1.0에서 시작해 내려가며 A/B 할 수 있는 연속 노브다.
        // 낮추면 "경로는 FF가 따라가고 L1은 오차만 고친다"에 가까워져, L1을 크게 유지한 채
        // (감쇠 여유 확보 = 횡진동 없음) 곡선 추종 정확도를 올릴 수 있다. ②-m에서
        // l1_offset 0.70→0.4가 횡진동으로 기각된 트레이드오프가 여기서 분리된다.
        steering_fb_gain_ = std::clamp(
            declare_parameter<double>("steering_fb_gain", 1.0), 0.0, 2.0);
        // FF가 참조할 곡률을 이만큼 앞에서 읽는다 [m]. 0이면 최근접점(정상상태 정의에 충실).
        // 조향→요레이트 지연이 실측 140 ms라, 지연 보상이 필요하면 v·0.14 정도가 출발점이다.
        curvature_ff_preview_ = std::max(0.0,
            declare_parameter<double>("curvature_ff_preview", 0.0));

        // ── K_us 온라인 적응 (②-p 3단계) ───────────────────────────────────────
        // 🔴 기본 0.0 = **관측 전용**(추정만 하고 로그만 찍는다, 적용 안 함).
        //    실측 K_us는 하중에 따라 0.0168(a_lat 3~4) → 0.0091(8~10)로 변하는데 정적
        //    상수는 그걸 못 따라간다. 다만 이 추정기는 조향 권한 캡까지 함께 지배하므로
        //    (모델 일원화의 대가) 관측 로그로 수렴을 먼저 확인한 뒤 켤 것.
        understeer_adapt_gain_ = std::max(0.0,
            declare_parameter<double>("understeer_gradient_adapt_gain", 0.0));
        understeer_min_ = std::max(1e-4, declare_parameter<double>("understeer_gradient_min", 0.008));
        understeer_max_ = std::max(understeer_min_,
                                   declare_parameter<double>("understeer_gradient_max", 0.025));
        // 관측성 게이트: K_us·a_lat 항이 지배해야 역산이 의미 있다. 코너에서만 배운다
        // — 직선에서 배우는 조향 트림 추정기(steering_trim_*, |a_lat| ≤ 2.0)와 작동
        // 영역이 겹치지 않게 갈라 두 추정기가 같은 신호를 두고 싸우지 않게 한다.
        understeer_adapt_min_lat_acc_ = std::max(0.5,
            declare_parameter<double>("understeer_adapt_min_lat_acc", 3.0));
        understeer_adapt_min_speed_ = std::max(0.5,
            declare_parameter<double>("understeer_adapt_min_speed", 2.0));

        // ── K_us(a_lat) 곡선 = LUT가 담으려던 비선형성의 1차원 대체 (②-q) ─────────
        // 🔴 기본 false = **관측 전용**. 켜도 곡선이 안 배워진 빈은 스칼라로 폴백한다.
        // 타이어가 포화하면 같은 조향으로 못 도는데, 그게 K_us가 하중과 함께 커지는
        // 것으로 나타난다. 상수 K_us는 이걸 못 담고 2-D LUT는 담으려다 실패했다.
        // 자이로가 이 곡선을 직접 준다 — 1-D라 몇 랩이면 동정된다.
        // 조향용 속도 하한 [m/s]. 0이면 구 거동(= vx_mps 0 구간에서 조향도 0).
        steering_speed_floor_ = std::max(0.0,
            declare_parameter<double>("steering_speed_floor", 0.5));

        understeer_curve_enable_ =
            declare_parameter<bool>("understeer_curve_enable", false);
        understeer_curve_min_samples_ = static_cast<int>(std::max<int64_t>(50,
            declare_parameter<int>("understeer_curve_min_samples", 300)));

        speed_lookahead_ = declare_parameter<double>("speed_lookahead", 0.15);
        speed_lookahead_for_steering_ =
            declare_parameter<double>("speed_lookahead_for_steering", 0.0);

        base_max_accel_ = declare_parameter<double>("base_max_accel", 4.0);
        base_max_decel_ = declare_parameter<double>("base_max_decel", 8.0);
        prebrake_decel_ = declare_parameter<double>("prebrake_decel", 1.5);

        ramp_lead_max_ = declare_parameter<double>("ramp_lead_max", 2.4);

        // HFI 정지출발 보호. 정지 상태에서 큰 속도 오차가 한 번에 걸리면 HFI가 로터각을
        // 포착하기 전에 속도 PID가 감긴다. 실측이 exit를 넘을 때까지만 저속 명령으로
        // 포착 시간을 주고, 내부 램프 상태도 같은 상한으로 되감아 해제 순간 계단을 막는다.
        hfi_launch_guard_enable_ =
            declare_parameter<bool>("hfi_launch_guard_enable", false);
        hfi_launch_speed_cap_ = std::max(
            0.0, declare_parameter<double>("hfi_launch_speed_cap", 0.7));
        hfi_launch_exit_speed_ = std::clamp(
            declare_parameter<double>("hfi_launch_exit_speed", 0.5),
            0.0, hfi_launch_speed_cap_);
        hfi_launch_standstill_speed_ = std::clamp(
            declare_parameter<double>("hfi_launch_standstill_speed", 0.1),
            0.0, hfi_launch_exit_speed_);
        hfi_launch_timeout_ = std::max(
            0.0, declare_parameter<double>("hfi_launch_timeout", 4.0));
        hfi_launch_exit_hold_ = std::max(
            0.0, declare_parameter<double>("hfi_launch_exit_hold", 0.1));
        hfi_launch_relatch_time_ = std::max(
            0.0, declare_parameter<double>("hfi_launch_relatch_time", 0.5));
        hfi_launch_retry_cooldown_ = std::max(
            0.0, declare_parameter<double>("hfi_launch_retry_cooldown", 0.5));
        hfi_launch_max_attempts_ = static_cast<unsigned int>(std::max<int64_t>(1,
            declare_parameter<int>("hfi_launch_max_attempts", 2)));
        hfi_launch_speed_topic_ =
            declare_parameter<std::string>("hfi_launch_speed_topic", "/odom");
        hfi_launch_speed_timeout_ = std::max(
            0.0, declare_parameter<double>("hfi_launch_speed_timeout", 0.2));
        if (hfi_launch_speed_cap_ <= 0.0 || hfi_launch_exit_speed_ <= 0.0) {
            hfi_launch_guard_enable_ = false;
        }

        // 런치 킥(자율 정지출발 시 센서리스 데드존 관통) — control_loop 8-c
        launch_boost_enable_ = declare_parameter<bool>("launch_boost_enable", true);
        launch_boost_speed_ = declare_parameter<double>("launch_boost_speed", 2.2);
        launch_boost_time_ = declare_parameter<double>("launch_boost_time", 0.6);
        launch_exit_speed_ = declare_parameter<double>("launch_exit_speed", 0.8);
        launch_standstill_speed_ = declare_parameter<double>("launch_standstill_speed", 0.3);
        launch_relatch_time_ = declare_parameter<double>("launch_relatch_time", 0.0);

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
        // ── 트림 웜업 단축 (2026-08-19) ─────────────────────────────────────────
        // 문제: 게이트 듀티가 25~29%뿐이라 실효 시상수가 τ/듀티 ≈ 14초 → 수렴에 3~4랩.
        //       그 사이 직선 횡오차가 +0.11 m에서 시작한다(0819 실측). 게다가 미체결마다
        //       0으로 리셋되므로 재체결하는 측정 하네스는 **매번** 웜업을 다시 산다.
        // ① steering_trim_init: 시작값(과 재체결 리셋값)을 알려진 값으로 준다.
        //    기계 중립은 주행 단위로 느리게 변하는 성질이라, 지난 주행의 수렴값
        //    (상태 로그 `trim: xx°`)을 그대로 실으면 웜업이 **0**이 된다.
        //    ⚠️ 서보암/타이로드/젯슨 offset을 만졌으면 반드시 0으로 되돌리고 다시 배울 것.
        // ② steering_trim_warmup_gain: 게이트가 열린 누적시간 t_g에 대해
        //    g_eff = clamp(1/t_g, gain, warmup_gain). 초기에는 사실상 표본평균이라
        //    빠르게 붙고, t_g > 1/gain 이후에는 **정확히 기존 LPF로 되돌아간다**
        //    → 정상상태 리플이 늘지 않는다(0819 리플레이: σ 0.056° → 0.055°).
        //    0819 리플레이 ±0.3° 정착: 20.9 s → 9.8 s. 0 = 비활성(구 거동).
        steering_trim_init_ = std::clamp(
            declare_parameter<double>("steering_trim_init", 0.0),
            -steering_trim_limit_, steering_trim_limit_);
        steering_trim_warmup_gain_ =
            std::max(0.0, declare_parameter<double>("steering_trim_warmup_gain", 2.0));
        // ③ 자동 저장/복원 — 사람이 값을 옮겨 적지 않아도 되게 한다.
        //    파일에 트림 + **그 값이 학습된 조건의 지문**을 같이 적고, 기동 시 지문이
        //    다르면 버린다. 지문이 없으면 "어제 서보암을 만졌는데 어제 트림을 싣는" 사고가 난다.
        steering_trim_persist_file_ =
            expand_user(declare_parameter<std::string>("steering_trim_persist_file",
                                                       "~/.f1tenth/steering_trim.yaml"));
        steering_trim_persist_max_age_ = std::max(
            0.0, declare_parameter<double>("steering_trim_persist_max_age", 43200.0));
        steering_trim_ = steering_trim_init_;

        max_speed_ = declare_parameter<double>("max_speed", 12.0);
        min_speed_ = declare_parameter<double>("min_speed", 2.0);

        // 곡률 룩어헤드 사전감속
        curvature_lookahead_count_ =
            static_cast<size_t>(declare_parameter<int>("curvature_lookahead_count", 60));
        max_lateral_accel_ = declare_parameter<double>("max_lateral_accel", 6.0);
        // ── 조향 권한 마진 (2026-08-19 신설) ──────────────────────────────────
        // 🔑 `mla`가 여태 **서로 다른 두 일을 겸했다**:
        //     ① 곡률 사전감속의 속도 캡 = "이 코너를 얼마로 돌 계획인가"(계획 예산)
        //     ② 조향 명령 클램프 `a_cmd` = "조향이 요구할 수 있는 최대 a_lat"(권한 천장)
        //    ②-p가 둘을 일부러 일원화했는데(사전감속 가정과 조향 근거를 맞추려고), 그
        //    부작용으로 **횡오차 보정분이 들어갈 자리가 구조적으로 0**이 됐다 — 라인이
        //    전 코너에서 a_lat = mla를 요구하면 ①이 곧 ②라서 클램프가 항상 포화한다.
        //    실측: 실제 K_us가 가정보다 25% 크면(타이어 마모·온도) 오차를 되돌릴 권한이
        //    없어 max 1.29 m로 **발산**한다. margin 1.10~1.17이면 0.34~0.41로 떨어진다.
        // 🔑 일원화의 취지는 유지된다 — 클램프는 여전히 **같은 mla에서 파생**되고(섹터
        //    스케일도 그대로 따라간다) 마진만 비례해서 얹는다. 즉 "계획보다 이만큼까지는
        //    더 써도 된다"는 보정 예산을 명시적으로 준 것이다.
        // 1.0 = 구 거동(보정 예산 0). 값은 CLAUDE.md ②-y 참고.
        steering_accel_margin_ =
            std::max(1.0, declare_parameter<double>("steering_accel_margin", 1.0));
        // ── U1 그립 권한 속도 클램프 (2026-08-21 재작업, run_220742·run_013203 충돌) ──
        // a_cmd 클램프는 조향 요구를 권한 안으로 자르지만 **속도는 아무도 안 줄였다**.
        // 요구 곡률(κ_L1 = 2|sinη|/L1, 추종오차 보정 포함)이 예산을 넘으면 목표 속도를
        // v ≤ √(예산/κ_L1) 로 캡한다. 검토 계약(2026-08-21):
        //  - 예산은 **마진 없는 MLA × grip_speed_clamp_margin(0.9)** — 조향 클램프의
        //    steering_accel_margin(1.15)은 얹지 않는다. "계획 속도는 보수적으로, 보정
        //    권한은 넉넉히"의 분리를 속도 쪽에서도 지키기 위함이다.
        //  - 필터는 **빠른 제한·느린 해제**: 요구가 튀면 즉시 물고(안전 기능의 진입을
        //    늦추지 않는다), 풀릴 때만 시상수를 둔다(경로 전환 스파이크 후 과감속 방지).
        //  - 하한 없음 — 안전 계산값이 항상 이긴다. 정지 권한은 플래너의 것이지만,
        //    이 캡은 감속 요구일 뿐 정지를 만들지 않는다(κ 가드로 0 나누기만 방지).
        //  - 적용 위치는 target_speed 단계 — 기존 종방향 램프(base_max_decel)가 감속을
        //    실현 가능하게 다듬은 뒤 나가고, 램프가 못 따라가면 deficit 경고를 남긴다.
        // 🔴 기본 false — 단독 셰이크다운(저속 2랩 → 정상 3랩)에서 검증 후 켠다.
        grip_speed_clamp_enable_ =
            declare_parameter<bool>("grip_speed_clamp_enable", false);
        // margin 기본 1.0 (2026-08-21 실측 스윕, run_220742 오픈루프): 0.9는 개입 43.5%
        // 랩 +2.15 s 로 과보수(현 라인이 v²κ=6.0 까지 쓰므로 계획속도까지 깎음), 1.0 은
        // 개입 34.7% 랩 +1.93 s(오픈루프 상한 — 폐루프에선 감속→오차 감소→요구 감소로
        // 자가완화). 두 충돌 창 모두 margin 무관하게 진입 4.1~4.2 → 3.0~3.8 로 예산 안.
        // 1.15(=조향 클램프 권한과 동일)는 비용 최소지만 보정 여유 0 — A/B 용으로만.
        grip_clamp_margin_ = std::clamp(
            declare_parameter<double>("grip_speed_clamp_margin", 1.0), 0.5, 1.3);
        grip_clamp_release_alpha_ = std::clamp(
            declare_parameter<double>("grip_speed_clamp_release_alpha", 0.05), 0.005, 1.0);
        understeer_gradient_ = declare_parameter<double>("understeer_gradient", 0.019);
        // ── 좌/우 분리 K_us (2026-08-18 실측, 2026-08-19 이 저장소로 이식) ──────
        // `rosbag2_2026_08_18-20_34_05` 정상상태 요레이트 전달률 역산: 좌 ≈0.008 /
        // 우 ≈0.024 — 공용 스칼라 0.014는 정확히 그 중간이라 우코너에서 FF가 만성
        // 부족했고, 매 랩 +1.05 m 와이드 → 복구 오버슈트 → 다음 좌커브 진입이 밀려
        // 벽 스침으로 이어졌다. 08-10 MCL bag에도 같은 비대칭(전달률 좌 0.66 / 우 0.18)이
        // 있어 **하드웨어 특성**으로 확정됐다 — 위치추정과 무관하다.
        // ⚠️ 젯슨 f1tenth_stack의 서보 좌/우 게인 분리(08-03)와는 **층위가 다르다**:
        //    그건 '명령각 → 서보', 이건 '바퀴각 → 요레이트'(차량 동역학)다.
        // ≤0 이면 공용값을 따른다(= 구 거동). 되돌리기: 둘 다 -1.0.
        understeer_gradient_left_ =
            declare_parameter<double>("understeer_gradient_left", -1.0);
        if (understeer_gradient_left_ <= 0.0) understeer_gradient_left_ = understeer_gradient_;
        understeer_gradient_right_ =
            declare_parameter<double>("understeer_gradient_right", -1.0);
        if (understeer_gradient_right_ <= 0.0) understeer_gradient_right_ = understeer_gradient_;
        // 적응 추정의 출발점은 런치가 준 정적값이다 — 학습 전/게이트가 안 열린 구간에서는
        // 정확히 구 거동으로 떨어진다. (범위 클램프는 파라미터 선언부에서 이미 읽었다.)
        understeer_gradient_adapted_ =
            std::clamp(understeer_gradient_, understeer_min_, understeer_max_);
        // 곡선 빈도 같은 출발점에서 시작 — 학습 전엔 어느 하중에서도 구 거동이다.
        for (int i = 0; i < kUsBins; ++i) kus_bin_[i] = understeer_gradient_adapted_;
        // δ_max 중 곡률 추종에 배정할 비율. 나머지는 횡오차 보정·요레이트 피드백 여유.
        steer_authority_ratio_ = declare_parameter<double>("steer_authority_ratio", 0.85);
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
        // ⚠️ 2026-08-16 게이트 수정: 원래 `local_fresh`(= `/local_waypoints`를 최근에 받았나)로
        //    켰는데, state_machine이 GLOBAL/CRUISE에서도 매 frenet odom마다 글로벌 라인을
        //    `/local_waypoints`로 발행하므로 local_fresh는 평상시 주행에서도 상시 true다.
        //    즉 "회피 중에만"이 아니라 "플래닝 스택이 살아있으면 항상" L1이 35% 커지고 있었다
        //    (실효 L1 offset 0.6→0.81, ②-m이 확정한 0.6 결론을 조용히 무효화). 진짜 회피 신호인
        //    `/state`(STATE_GLOBAL 아님)로 바꾼다 — 회피 판정은 아래 avoiding_now() 참고.
        avoidance_l1_damping_enable_ = declare_parameter<bool>("avoidance_l1_damping_enable", true);
        // 🔴 2026-08-20: 1.35 → 1.0. 1.35 에는 실측 근거가 없었다(런치에서 넘기지도 않아
        //    이 기본값이 그대로 쓰였다). 실측 근거와 되돌리기는 _control_common.py 의
        //    avoidance_l1_scale_max 인자 주석 참고.
        avoidance_l1_scale_max_ = declare_parameter<double>("avoidance_l1_scale_max", 1.0);

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
        // odom 워치독. /local_waypoints·/drive_mode는 전부 신선도 타임아웃이 있는데 **odom만
        // 없었다**(2026-08-04 a71890c에서 제거). 위치추정(MCL)이 죽으면 current_x_/y_/speed_가
        // stale 상태로 얼고, 컨트롤러는 그 얼어붙은 pose로 계산한 조향·속도를 50 Hz로 계속
        // 발행한다 — 노드는 완전히 정상으로 보인다.
        //   🔴 2026-08-18 `run_0818_134408`이 정확히 이 형태로 벽에 박았다: /pf/pose/odom이
        //      1125 ms 끊긴 동안 /drive_autonomous는 20.0 ms 간격을 유지한 채 값이 그대로
        //      얼었고(speed 3.572 / steer +0.1286이 0.8초간 동일), 그사이 차는 3.9 m를 더
        //      달려 횡오차가 0.44 → 1.09 m로 벌어졌다. MCL이 3.25 m 점프하며 복귀한 직후 접촉.
        //   같은 날 정상 런의 최대 공백은 51 ms(140819, 201초)이고, 기동 직후 과도구간에서만
        //   336~380 ms가 관측된다 — 0.5 s는 그 위에 충분한 여유를 두면서 1.1 s는 잡는 값이다.
        // 0이면 비활성. NaN pose(MCL 붕괴)는 odom_callback이 샘플을 버리므로 여기서 stale로 잡힌다.
        odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);

        drive_mode_topic_ = declare_parameter<std::string>("drive_mode_topic", "/drive_mode");
        engaged_mode_value_ = declare_parameter<std::string>("engaged_mode_value", "autonomous");
        drive_mode_timeout_ = declare_parameter<double>("drive_mode_timeout", 1.0);

        // 경로 소스 중재
        local_fresh_timeout_ = declare_parameter<double>("local_fresh_timeout", 0.3);

        // Cruise controller는 경로를 바꾸지 않고 종방향 속도 상한만 제공한다.
        cruise_limit_enable_ = declare_parameter<bool>("cruise_limit_enable", true);
        cruise_speed_limit_topic_ =
            declare_parameter<std::string>("cruise_speed_limit_topic", "/cruise_speed_limit");
        cruise_speed_limit_timeout_ =
            std::max(0.01, declare_parameter<double>("cruise_speed_limit_timeout", 0.15));
        cruise_stale_speed_ =
            std::max(0.0, declare_parameter<double>("cruise_stale_speed", 1.5));

        closest_idx_max_heading_err_ =
            declare_parameter<double>("closest_idx_max_heading_err", 1.40);

        acc_now_ = std::vector<double>(10, 0.0);

        // ── 3. 통신 채널 ──
        // 글로벌은 latched(transient_local), 로컬은 퍼블리셔에 맞춰 volatile.
        global_path_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/global_waypoints", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(&ControlMapNode::global_path_callback, this, std::placeholders::_1));
        local_path_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/local_waypoints", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            std::bind(&ControlMapNode::local_path_callback, this, std::placeholders::_1));
        local_last_recv_time_ = this->now();  // 노드 클럭 타입으로 초기화(clock mismatch 방지)

        if (cruise_limit_enable_) {
            cruise_speed_limit_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                cruise_speed_limit_topic_, 10,
                std::bind(&ControlMapNode::cruise_speed_limit_callback,
                          this, std::placeholders::_1));
        }

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&ControlMapNode::odom_callback, this, std::placeholders::_1));
        if (hfi_launch_guard_enable_) {
            // 경로 추종 pose/PF odom과 분리한다. HFI 성공·역회전·정지 판정은 모터에서
            // 직접 나온 VESC /odom만 사용해야 PF 정지값에 가려진 탈조를 볼 수 있다.
            hfi_speed_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                hfi_launch_speed_topic_, 10,
                std::bind(&ControlMapNode::hfi_speed_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),
                "HFI 정지출발 보호 속도원: %s (신선도 %.2fs), 최대 %u회 × %.1fs",
                hfi_launch_speed_topic_.c_str(), hfi_launch_speed_timeout_,
                hfi_launch_max_attempts_, hfi_launch_timeout_);
        }
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
        }
        // /state 구독 콜백은 sector_on_global_/sector_state_seen_을 채운다 — 섹터 스케일의
        // 회피 게이팅뿐 아니라 avoidance_l1_damping의 회피 판정도 이 값을 쓴다. 그래서 둘 중
        // 하나라도 필요하면 구독한다(섹터 스케일을 꺼도 L1 회피 감쇠는 살아 있어야 한다).
        const bool need_state_sub =
            (sector_scale_enable_ && sector_scale_global_only_) || avoidance_l1_damping_enable_;
        if (need_state_sub) {
            sector_state_sub_ = this->create_subscription<f110_msgs::msg::StateMachine>(
                sector_scale_state_topic_, 10,
                [this](const f110_msgs::msg::StateMachine::SharedPtr msg) {
                    const bool was = sector_on_global_ && sector_state_seen_;
                    sector_on_global_ = (msg->state == f110_msgs::msg::StateMachine::STATE_GLOBAL);
                    sector_state_last_recv_time_ = this->now();
                    sector_state_seen_ = true;
                    if (was != sector_on_global_ && sector_scale_enable_) {
                        // 상태가 바뀌면 이미 받아둔 경로의 mla를 즉시 다시 해소해야 한다 —
                        // 안 그러면 회피에 들어갔는데 다음 경로 메시지까지 옛 스케일이 남는다.
                        apply_sector_scales(waypoints_);
                        apply_sector_scales(local_waypoints_);
                    }
                });
        }
        if (sector_scale_enable_) {
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
        if (avoidance_l1_damping_enable_) {
            RCLCPP_INFO(this->get_logger(),
                        "회피 L1 감쇠 활성 — %s(STATE_GLOBAL 아님)에서 L1 ×%.2f "
                        "(%s미수신/끊김 시 자동 비활성)",
                        sector_scale_state_topic_.c_str(), avoidance_l1_scale_max_,
                        sector_scale_state_topic_.c_str());
        }

        // 조향 파라미터를 기동 시 1회 남긴다 — bag만 보고 "그 주행이 어떤 설정이었나"를
        // 되짚을 수 있어야 한다(0816 사후분석에서 파라미터 이력을 git으로 캐야 했던 교훈).
        RCLCPP_INFO(this->get_logger(),
                    "🟢 조향: 자전거 역모델 δ=a_lat·(L/v²+K_us) | K_us 좌 %.5f / 우 %.5f | "
                    "FF/FB 분리 게인 %.2f%s | FF 곡률 프리뷰 %.2f m | 조향 권한 마진 ×%.2f",
                    understeer_gradient_eff(+1.0), understeer_gradient_eff(-1.0),
                    steering_fb_gain_,
                    (std::abs(steering_fb_gain_ - 1.0) < 1e-9) ? "(=순수 L1과 동일)" : "",
                    curvature_ff_preview_, steering_accel_margin_);
        if (understeer_adapt_gain_ > 0.0) {
            RCLCPP_WARN(this->get_logger(),
                        "K_us 온라인 적응 **활성** — gain %.2f (τ=%.1fs), 범위 [%.4f, %.4f], "
                        "게이트: |a_lat|≥%.1f, v≥%.1f. ⚠️ 이 값은 조향 권한 캡(코너 진입속도)도 "
                        "함께 지배한다.",
                        understeer_adapt_gain_, 1.0 / understeer_adapt_gain_,
                        understeer_min_, understeer_max_,
                        understeer_adapt_min_lat_acc_, understeer_adapt_min_speed_);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "K_us 적응: 관측 전용(추정·로그만, 적용 안 함). 켜려면 "
                        "understeer_gradient_adapt_gain:=0.25");
        }
        if (understeer_curve_enable_) {
            RCLCPP_WARN(this->get_logger(),
                        "K_us(a_lat) 곡선 **활성** — 빈 중심 %.1f/%.1f/%.1f/%.1f m/s², "
                        "빈당 최소 %d샘플, 단조 비감소 강제. ⚠️ 조향이 고하중에서 커진다 — "
                        "저속 셰이크다운부터 할 것.",
                        kUsBinCenter[0], kUsBinCenter[1], kUsBinCenter[2], kUsBinCenter[3],
                        understeer_curve_min_samples_);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "K_us(a_lat) 곡선: 관측 전용(빈별 학습·로그만, 조향엔 스칼라 사용). "
                        "켜려면 understeer_curve_enable:=true");
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
            // ⚠️ 로그보다 **먼저** 싣는다 — 안 그러면 "시작값 +0.00°"를 찍어 놓고
            //    실제로는 저장본으로 출발해 사람이 로그를 오독한다.
            // (명시적으로 준 steering_trim_init 이 있으면 그쪽이 이미 반영돼 있고,
            //  저장본이 지문·나이 검사를 통과하면 저장본이 최신이므로 이긴다.)
            load_persisted_trim();
            RCLCPP_INFO(this->get_logger(),
                "  트림 웜업: 시작값 %+.2f° (재체결 시에도 이 값으로 복귀) | 웜업 상한 게인 %.1f 1/s%s",
                steering_trim_init_ * 180.0 / M_PI, steering_trim_warmup_gain_,
                (steering_trim_warmup_gain_ > steering_trim_gain_)
                    ? " (게이트 누적 1/t_g 스케줄 → 정상 게인으로 자동 복귀)"
                    : " (비활성 = 구 거동)");
            if (!steering_trim_persist_file_.empty()) {
                RCLCPP_INFO(this->get_logger(),
                    "  트림 자동 저장: %s (5초마다, 지문·나이 %.0f시간 검사)",
                    steering_trim_persist_file_.c_str(),
                    steering_trim_persist_max_age_ / 3600.0);
                trim_persist_timer_ = this->create_wall_timer(
                    std::chrono::seconds(5), [this]() { save_persisted_trim(); });
            }
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
        // 노드 클럭으로 초기화해 둔다. (odom_seen_ 게이트가 먼저 걸리므로 논리적으로는
        //  미수신 상태에서 이 값이 쓰이지 않지만, 기본 생성된 rclcpp::Time은 클럭 타입이
        //  달라 use_sim_time에서 뺄셈이 예외를 던진다.)
        odom_last_recv_time_ = this->now();
        hfi_speed_last_recv_time_ = this->now();
        RCLCPP_INFO(this->get_logger(),
                    "RoboRacer L1 Guidance + 자전거 역모델 조향 제어 노드가 시작되었습니다.");
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

        // ⚠️ 여기에 포즈 저역통과 필터(EMA)를 걸지 말 것. 2026-08-15에 `pose_lpf_alpha`(0.30)로
        //    한 번 들어왔다가 제거됐다. MCL 포즈는 이미 EKF+EMA로 평활돼 있어서 여기서 한 번 더
        //    거는 건 잡음 제거가 아니라 **순수 지연**이다(40 Hz·α=0.3 → τ≈58 ms). L1은 적분기가
        //    없는 기하 추종기라 피드백 지연이 곧 감쇠 손실이고, ω_n·τ가 건전 대역(0.29~0.44,
        //    ②-f)을 넘어가면 0.9 Hz 리밋사이클로 나타난다 — 0815 실측에서 요레이트 진동 대역
        //    에너지가 0.33 → 0.65로 배가됐다. 포즈가 튀면 여기가 아니라 MCL에서 고칠 것.
        current_x_ = x;
        current_y_ = y;
        current_yaw_ = yaw;
        current_speed_ = v;
        odom_last_recv_time_ = this->now();
        odom_seen_ = true;
    }

    void hfi_speed_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const double v = msg->twist.twist.linear.x;
        if (!std::isfinite(v)) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "HFI 속도원(%s)에 NaN/Inf 수신 — 샘플 폐기",
                hfi_launch_speed_topic_.c_str());
            return;
        }
        hfi_speed_ = v;
        hfi_speed_last_recv_time_ = this->now();
        hfi_speed_seen_ = true;
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        // use_imu=false는 "IMU를 신뢰하지 않는다"는 뜻이므로 파생값을 전부 쓰지 않는다.
        // acc_now_는 0 초기화 상태로 남아 acc_mean=0 → 스케일러 중립(1.0)으로 안전히 떨어진다.
        if (!use_imu_) return;

        std::rotate(acc_now_.rbegin(), acc_now_.rbegin() + 1, acc_now_.rend());
        acc_now_[0] = -msg->linear_acceleration.x * imu_linear_scale_;

        // 좌측 = −a_y (CLAUDE.md "VESC 가속도계 축 확정"). 슬립 잔차 진단 전용이다.
        lat_acc_now_ = -msg->linear_acceleration.y * imu_linear_scale_;

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

    // ── 조향 트림 영속화 (2026-08-19) ───────────────────────────────────────────
    // 왜: 트림 게이트 듀티가 25~29%라 매 기동마다 웜업을 다시 산다. 게다가 미체결마다
    //     리셋되므로 랩마다 재체결하는 하네스는 영영 수렴값을 못 본다(②-n).
    // 🔑 **지문(fingerprint) 검사가 이 기능의 안전장치다.** 트림은 "이 조향 파이프라인
    //    에서의 잔차"라, 파이프라인이 바뀌면 그 값은 의미가 없다. 지문이 다르면 버린다.
    // ⚠️ 지문으로 못 잡는 변화가 하나 있다: **젯슨 vesc.yaml 의
    //    steering_angle_to_servo_offset**. 그건 다른 패키지라 여기서 안 보인다.
    //    그래서 ① 나이 제한(steering_trim_persist_max_age)과 ② 웜업 스케줄을 같이 둔다 —
    //    웜업이 켜져 있으면 틀린 값을 실어도 게이트 열린 뒤 ~10초면 실측으로 덮인다.
    //    즉 **최악이 "0에서 시작한 것과 같음"**이지 그보다 나빠지지 않는다.
    static std::string expand_user(const std::string& p) {
        if (p.empty() || p[0] != '~') return p;
        const char* home = std::getenv("HOME");
        if (!home) return p;
        return std::string(home) + p.substr(1);
    }

    // 트림이 유효한 조건. 하나라도 다르면 저장값을 버린다.
    std::string trim_fingerprint() const {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "kl=%.5f,kr=%.5f,kg=%.5f,reach=%.4f,lag=%.3f,sl=%.4f,sr=%.4f",
                      understeer_gradient_left_, understeer_gradient_right_,
                      understeer_gradient_, steering_reach_ratio_, steering_trim_lag_,
                      max_steering_left_, max_steering_right_);
        return std::string(buf);
    }

    // 기동 시 1회. 실패는 전부 "안 싣는다"로 수렴한다(치명적이지 않다).
    void load_persisted_trim() {
        if (steering_trim_persist_file_.empty() || steering_trim_gain_ <= 0.0) return;
        std::ifstream f(steering_trim_persist_file_);
        if (!f) {
            RCLCPP_INFO(this->get_logger(),
                "  트림 저장본 없음(%s) — 0에서 학습 시작. 이번 주행 끝에 자동 저장된다.",
                steering_trim_persist_file_.c_str());
            return;
        }
        std::string line, fp; double val = 0.0, stamp = 0.0; bool has_val = false;
        while (std::getline(f, line)) {
            auto c = line.find(':');
            if (c == std::string::npos) continue;
            std::string k = line.substr(0, c), v = line.substr(c + 1);
            auto trim_ws = [](std::string& x) {
                const char* ws = " \t\r\n\"";
                auto b = x.find_first_not_of(ws); auto e = x.find_last_not_of(ws);
                x = (b == std::string::npos) ? "" : x.substr(b, e - b + 1);
            };
            trim_ws(k); trim_ws(v);
            if (k == "steering_trim_rad") { val = std::atof(v.c_str()); has_val = true; }
            else if (k == "fingerprint")  { fp = v; }
            else if (k == "stamp")        { stamp = std::atof(v.c_str()); }
        }
        if (!has_val) {
            RCLCPP_WARN(this->get_logger(), "  트림 저장본을 못 읽었다(형식 이상) — 0에서 시작");
            return;
        }
        const std::string cur = trim_fingerprint();
        if (fp != cur) {
            RCLCPP_WARN(this->get_logger(),
                "  트림 저장본 폐기 — 조향 파이프라인이 바뀌었다.\n"
                "      저장: %s\n      현재: %s", fp.c_str(), cur.c_str());
            return;
        }
        const double age = this->now().seconds() - stamp;
        if (steering_trim_persist_max_age_ > 0.0 &&
            (age < 0.0 || age > steering_trim_persist_max_age_)) {
            RCLCPP_WARN(this->get_logger(),
                "  트림 저장본 폐기 — 나이 %.1f시간 > 한계 %.1f시간. 기계 중립은 정비/주행마다 "
                "움직이므로(0810 실측 −2.2/−1.9/+1.6°) 오래된 값은 싣지 않는다.",
                age / 3600.0, steering_trim_persist_max_age_ / 3600.0);
            return;
        }
        steering_trim_init_ = std::clamp(val, -steering_trim_limit_, steering_trim_limit_);
        steering_trim_ = steering_trim_init_;
        RCLCPP_INFO(this->get_logger(),
            "  🟢 트림 저장본 적용: %+.2f° (나이 %.0f분) — 웜업 없이 시작한다",
            steering_trim_init_ * 180.0 / M_PI, age / 60.0);
    }

    // 저속 타이머(0.2 Hz)에서만 호출한다. 50 Hz 제어 루프에서 파일을 쓰면 젯슨이 bag을
    // 디스크에 쓰는 동안(PSI io) 블로킹될 수 있다.
    void save_persisted_trim() {
        if (steering_trim_persist_file_.empty() || steering_trim_gain_ <= 0.0) return;
        // 의미 있는 학습이 실제로 일어난 뒤에만 쓴다. 안 그러면 시작값을 그대로 다시 써서
        // 틀린 값이 영원히 자기 자신을 갱신한다(나이 검사가 무력화된다).
        if (steering_trim_samples_ < 200) return;
        if (std::abs(steering_trim_ - steering_trim_saved_) < 1e-4) return;

        const std::string dir = steering_trim_persist_file_.substr(
            0, steering_trim_persist_file_.find_last_of('/'));
        if (!dir.empty() && dir != steering_trim_persist_file_) {
            std::string cmd = "mkdir -p '" + dir + "'";
            if (std::system(cmd.c_str()) != 0) { /* 실패해도 아래에서 조용히 포기 */ }
        }
        // 원자적 교체 — 쓰는 도중 전원이 끊겨도 반쪽 파일이 남지 않는다.
        const std::string tmp = steering_trim_persist_file_ + ".tmp";
        {
            std::ofstream f(tmp, std::ios::trunc);
            if (!f) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 60000,
                    "트림 저장 실패(쓰기 불가): %s", steering_trim_persist_file_.c_str());
                return;
            }
            f << "# control_map_node 조향 트림 자동 저장 — 손으로 고치지 말 것\n"
              << "# fingerprint 가 다르거나 stamp 가 오래되면 기동 시 자동 폐기된다.\n"
              << "steering_trim_rad: " << steering_trim_ << "\n"
              << "steering_trim_deg: " << steering_trim_ * 180.0 / M_PI << "\n"
              << "samples: " << steering_trim_samples_ << "\n"
              << "stamp: " << std::fixed << this->now().seconds() << "\n"
              << "fingerprint: " << trim_fingerprint() << "\n";
        }
        if (std::rename(tmp.c_str(), steering_trim_persist_file_.c_str()) == 0) {
            steering_trim_saved_ = steering_trim_;
        } else {
            std::remove(tmp.c_str());
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 60000,
                "트림 저장 실패(rename): %s", steering_trim_persist_file_.c_str());
        }
    }

    double steer_avail() const {
        return steer_authority_ratio_ * steer_limit_min_ * steering_reach_ratio_;
    }

    // 지금 유효한 K_us(하중 무관 스칼라). 적응이 꺼져 있으면(기본) 런치 파라미터 그대로다.
    // ⚠️ 조향 권한 캡·트림 추정이 **이 하나를 쓴다**(모델 일원화).
    // turn_sign: 좌회전(κ>0, a_lat>0, ψ̇>0) 양수 / 우회전 음수 / 0 = 방향 미상(공용값).
    // ⚠️ 적응(adapt_gain>0)이 켜지면 적응 스칼라가 좌우 공용으로 **우선**한다 — 추정기가
    //    방향을 분리하지 않으므로 좌/우 분리값과 동시에 쓸 수 없다.
    double understeer_gradient_eff(double turn_sign = 0.0) const {
        if (understeer_adapt_gain_ > 0.0) return understeer_gradient_adapted_;
        if (turn_sign > 0.0) return understeer_gradient_left_;
        if (turn_sign < 0.0) return understeer_gradient_right_;
        return understeer_gradient_;
    }

    // ── K_us(a_lat) 곡선 (②-q) ────────────────────────────────────────────────
    // 빈 중심 [m/s²]. 자이로 실측(0813/0814 bag)에서 K_us는 a_lat 5부터 확실히 올라간다:
    //   a_lat  4~5   5~6   6~7   7~8   (r=1.0 가정, 절대값은 r에 비례하지만 **모양은 불변**)
    //   K_us  .0127 .0128 .0168 .0197
    // 저하중(2~4)의 큰 값은 a_lat으로 나누는 데서 오는 노이즈·트림 잔차라 학습 대상이 아니다
    // — 그 영역은 조향 트림 추정기(②-n)가 이미 담당한다.
    static constexpr int kUsBins = 4;
    static constexpr double kUsBinCenter[kUsBins] = {3.5, 5.0, 6.5, 8.5};

    // |a_lat|에서 곡선을 읽는다. 빈 사이는 선형보간, 양 끝은 그 값으로 평평하게 유지
    // (외삽 금지 — 관측 못 한 하중대에서 조향이 튀는 것이 LUT의 실패 방식이었다).
    // ⚠️ 인자는 **부호 있는** a_lat이다. 부호는 좌/우 base 선택에만 쓰고, 곡선 조회는
    //    |a_lat|로 한다 — 학습 빈은 방향 혼합이라 빈이 찬 하중대에서는 곡선값이 좌/우
    //    분리를 대체하고, 폴백 빈에서만 분리값이 남는다.
    double understeer_from_curve(double a_lat_signed) const {
        const double base = understeer_gradient_eff(a_lat_signed);
        if (!understeer_curve_enable_) return base;

        // 학습된 빈만 쓴다. 부족하면 그 빈은 스칼라값으로 대체 = 구 거동으로 폴백.
        double k[kUsBins];
        bool any = false;
        for (int i = 0; i < kUsBins; ++i) {
            const bool ready = kus_bin_n_[i] >= understeer_curve_min_samples_;
            k[i] = ready ? kus_bin_[i] : base;
            any = any || ready;
        }
        if (!any) return base;

        // 🔑 단조 비감소 강제. 타이어 포화는 한 방향으로만 간다 — 이 제약이 노이즈로
        //    곡선이 출렁여 조향이 하중에 대해 비단조가 되는 것을 구조적으로 막는다.
        //    (LUT는 이 제약이 없어서 그립피크 이후 접혀 NaN이 됐다.)
        for (int i = 1; i < kUsBins; ++i) k[i] = std::max(k[i], k[i - 1]);

        const double a = std::abs(a_lat_signed);
        if (a <= kUsBinCenter[0]) return k[0];
        if (a >= kUsBinCenter[kUsBins - 1]) return k[kUsBins - 1];
        for (int i = 1; i < kUsBins; ++i) {
            if (a < kUsBinCenter[i]) {
                const double w = (a - kUsBinCenter[i - 1]) /
                                 (kUsBinCenter[i] - kUsBinCenter[i - 1]);
                return k[i - 1] + w * (k[i] - k[i - 1]);
            }
        }
        return k[kUsBins - 1];
    }

    int understeer_bin_of(double a_lat_abs) const {
        int best = 0;
        double bd = 1e9;
        for (int i = 0; i < kUsBins; ++i) {
            const double d = std::abs(a_lat_abs - kUsBinCenter[i]);
            if (d < bd) { bd = d; best = i; }
        }
        return best;
    }

    // 정상상태 자전거 역모델: 요구 횡가속도 → 조향각.
    //   δ = L·κ + K_us·a_lat,  a_lat = κv²   ⟹   δ = a_lat·(L/v² + K_us)
    // 🔑 L1 유도법칙 a_lat = 2v²·sinη/L1 을 대입하면 v²이 첫 항에서 소거되어
    //      δ = 2L·sinη/L1 + 2K_us·v²·sinη/L1
    //    = (순수 pure pursuit) + (속도제곱 비례 언더스티어 보정)
    //    즉 v→0에서 게인이 폭발하지 않고 pure pursuit 기하로 수렴한다 — ②-f의
    //    "정지 상태에서 조향이 풀락에 붙던" 구조가 원천적으로 생기지 않는다.
    // 🔑 K_us는 **요구 횡가속도**에서 읽는다(달성값이 아니라) — 그래서 역해에 반복이
    //    필요 없다. 곡선이 꺼져 있으면 정확히 상수 K_us로 떨어진다.
    double bicycle_steer_from_lat_acc(double a_lat, double v) const {
        // v=0에서 a_lat도 0이므로(∝v²) 이 하한은 0/0 방어일 뿐 거동을 바꾸지 않는다.
        const double v2 = std::max(v * v, 1e-4);
        return a_lat * (wheelbase_ / v2 + understeer_from_curve(a_lat));
    }

    // FF가 참조할 경로 곡률. 노이즈가 그대로 조향에 실리지 않도록 **평활 곡률**을 쓴다.
    // 🔴 반드시 **부호 있는** 쪽을 쓸 것. smoothed_curvature는 |κ|(사전감속 전용)이라
    //    그걸 쓰면 FF가 항상 한쪽으로만 나가고, steering_fb_gain을 1.0 미만으로 내리는
    //    순간(= FF/FB 분리의 목적) 우코너에서 조향이 상쇄돼 사라진다.
    double curvature_for_ff(const std::vector<Waypoint>& wps, size_t closest_idx,
                            bool path_closed) const {
        if (curvature_ff_preview_ <= 1e-6) return wps[closest_idx].smoothed_curvature_signed;
        const size_t idx = walk_forward(wps, closest_idx, curvature_ff_preview_, path_closed,
                                        [](size_t, double) { return true; });
        return wps[idx].smoothed_curvature_signed;
    }

    // steer_hist_에서 lag만큼 과거의 발행 조향을 선형보간으로 꺼낸다.
    // 트림 추정기와 K_us 추정기가 **같은 이력·같은 지연 정의**를 쓰도록 공유한다
    // (②-k 지뢰 1번: 지연을 안 넣으면 자전거모델 회귀가 L=0.16으로 거짓말한다).
    bool steer_at_lag(double tnow, double* out) const {
        const double t_target = tnow - steering_trim_lag_;
        if (steer_hist_.size() < 2 || steer_hist_.front().first > t_target) return false;
        double past = steer_hist_.back().second;
        for (size_t i = 1; i < steer_hist_.size(); ++i) {
            if (steer_hist_[i].first >= t_target) {
                const auto &a = steer_hist_[i - 1], &b = steer_hist_[i];
                const double w = (b.first > a.first) ? (t_target - a.first) / (b.first - a.first) : 0.0;
                past = a.second + w * (b.second - a.second);
                break;
            }
        }
        *out = past;
        return true;
    }

    // K_us 온라인 추정. 정상상태 자전거모델을 K_us에 대해 풀면
    //   K_us = (δ_wheel − L·κ) / a_lat,   κ = ψ̇/v,  a_lat = v·ψ̇
    // 🔑 자이로만 쓴다 — MCL(우리가 줄이려는 오차원)에 의존하지 않는다.
    // 🔴 적응이 꺼져 있어도 **추정과 로그는 항상 돈다**(관측 모드). 값을 실제로 쓰는 건
    //    understeer_gradient_eff()이고 그건 gain>0일 때만 적응값을 돌려준다.
    void update_understeer_gradient(double dt, double published) {
        if (!use_imu_ || !yaw_rate_seen_) return;
        if (!is_engaged_ && engage_gate_active()) return;
        if (launch_active_) return;
        if ((this->now() - yaw_rate_last_recv_).seconds() > 0.2) return;

        const double v = std::abs(current_speed_);
        if (v < understeer_adapt_min_speed_) return;
        const double a_lat = v * yaw_rate_now_;
        // 관측성: K_us·a_lat 항이 충분히 커야 역산이 노이즈에 안 묻힌다(코너 전용 게이트).
        if (std::abs(a_lat) < understeer_adapt_min_lat_acc_) return;
        // 포화 구간은 정상상태 모델이 성립하지 않는다(명령과 실제 바퀴각이 갈라진다).
        if (std::abs(published) > 0.9 * steer_limit_min_) return;

        double past = 0.0;
        if (!steer_at_lag(this->now().seconds(), &past)) return;

        // 명령 공간 → 실제 바퀴각. 트림이 수렴한 뒤엔 발행값이 곧 의도한 바퀴각이므로
        // 도달각 비율만 곱하면 된다.
        const double delta_wheel = past * steering_reach_ratio_;
        const double kappa = yaw_rate_now_ / v;
        const double kus_meas = (delta_wheel - wheelbase_ * kappa) / a_lat;
        if (!std::isfinite(kus_meas)) return;

        // 1차 LPF. 되먹임이 아니라 순수 측정 평활이라 구조적으로 발산하지 않는다.
        const double g = (understeer_adapt_gain_ > 0.0) ? understeer_adapt_gain_ : kUsObserveGain;
        understeer_gradient_adapted_ +=
            g * (std::clamp(kus_meas, understeer_min_, understeer_max_)
                 - understeer_gradient_adapted_) * dt;
        understeer_gradient_adapted_ =
            std::clamp(understeer_gradient_adapted_, understeer_min_, understeer_max_);
        understeer_adapt_samples_++;

        // ── K_us(a_lat) 곡선 학습 (②-q) ───────────────────────────────────────
        // 스칼라와 **같은 측정치**를 하중 빈에만 나눠 담는다. 게이트가 하나라 두 추정이
        // 갈라지지 않고, 곡선을 꺼도 스칼라는 그대로 배운다(관측 로그가 계속 유효).
        // ⚠️ 준정상상태에서만 배운다: a_lat = v·ψ̇ 는 β̇=0일 때만 참이고, 요레이트가
        //    빠르게 쌓이는 과도구간에선 실제 횡가속을 과대평가한다(오프라인 분석에서
        //    이 게이트를 넣자 자이로↔가속도계 상관이 0.86 → 0.97~0.99로 올랐다).
        const double yaw_acc = (dt > 1e-4) ? (yaw_rate_now_ - yaw_rate_prev_) / dt : 0.0;
        yaw_rate_prev_ = yaw_rate_now_;
        if (std::abs(yaw_acc) > kUsCurveMaxYawAcc) return;

        const int b = understeer_bin_of(std::abs(a_lat));
        const double kc = std::clamp(kus_meas, understeer_min_, understeer_max_);
        kus_bin_n_[b]++;
        // 빈마다 러닝 평균(샘플 수 기반) — 초기 수렴이 빠르고 늦게는 안정된다.
        kus_bin_[b] += (kc - kus_bin_[b]) / std::min<long>(kus_bin_n_[b], 400);
    }

    // 자이로↔가속도계 잔차 v̇_y = a_y − v·ψ̇. 슬립이 없으면 0 주변이다.
    // 🔴 **진단 전용 — 주행에 개입하지 않는다.** 오프라인 실측 σ: 준정상 0.4~1.5 /
    //    스핀 1.1~4.3 (배율 1.2~3.7배)이라 그립 이탈의 조기 신호로 쓸 수 있다.
    void update_slip_residual(double dt) {
        if (!use_imu_ || !yaw_rate_seen_) return;
        const double v = std::abs(current_speed_);
        if (v < 1.5) { slip_residual_ = 0.0; return; }
        const double res = lat_acc_now_ - v * yaw_rate_now_;
        if (!std::isfinite(res)) return;
        const double a = std::clamp(dt / 0.25, 0.0, 1.0);   // τ=0.25s LPF
        slip_residual_ += a * (std::abs(res) - slip_residual_);
        if (slip_residual_ > slip_residual_peak_) slip_residual_peak_ = slip_residual_;
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
        double past = 0.0;
        if (!steer_at_lag(tnow, &past)) return;

        // 실측 요레이트가 함의하는 바퀴각 → 명령 공간으로 환산.
        const double delta_wheel =
            yaw_rate_now_ * (wheelbase_ / v + understeer_gradient_eff(yaw_rate_now_) * v);
        const double e = past - delta_wheel / std::max(0.3, steering_reach_ratio_);
        // 게이트가 실제로 열려 있던 누적시간. 웜업 스케줄의 유일한 입력이다.
        steering_trim_gated_time_ += dt;
        double g = steering_trim_gain_;
        if (steering_trim_warmup_gain_ > steering_trim_gain_) {
            // 1/t_g = 표본평균과 등가. 아래로는 정상 게인, 위로는 warmup 상한으로 자른다.
            // ⚠️ 상한이 필요하다: 자르지 않으면 첫 샘플에서 α = g·dt가 1을 넘어 발산한다.
            g = std::clamp(1.0 / std::max(steering_trim_gated_time_, 1e-3),
                           steering_trim_gain_, steering_trim_warmup_gain_);
        }
        steering_trim_ += g * (e - steering_trim_) * dt;
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

    void cruise_speed_limit_callback(const std_msgs::msg::Float64::ConstSharedPtr msg) {
        if (!std::isfinite(msg->data) || msg->data < 0.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "비정상 cruise speed limit %.3f 무시", msg->data);
            return;
        }
        cruise_speed_limit_ = msg->data;
        cruise_speed_limit_last_recv_time_ = this->now();
        cruise_speed_limit_seen_ = true;
    }

    // 경로를 모를 때의 안전 정지. 발행을 멈추지 않고 명시적 0을 보내는 이유는, 침묵하면
    // 하류(ackermann_mux→VESC)가 **직전 명령을 그대로 유지**해 타력주행이 되기 때문이다.
    //
    // hold_steering=true면 조향을 0으로 펴지 않고 **직전 각을 유지한 채** 속도만 0으로 준다.
    // ⚠️ odom 워치독 전용 옵션이다. 굴러가는 중에 조향을 0으로 만드는 것은 ②-r에서
    //    이미 확인된 실패 모드다 — 코너 한복판 비상정지가 바깥 벽으로의 직진이 된다.
    //    경로를 아예 모르는 경우(기존 호출부)는 직전 각도 근거가 없으므로 종전대로 0을 쓴다.
    void publish_safe_stop(bool hold_steering = false) {
        const double steer = hold_steering ? last_steering_angle_ : 0.0;
        last_steering_angle_ = steer;
        // 램프 상태를 실측에 붙여 둔다 — 복귀 시 계단 명령이 안 나가게 하는 bumpless 처리.
        last_target_speed_ = std::max(0.0, current_speed_);
        last_published_speed_ = 0.0;
        publish_drive(steer, 0.0, 0.0);
    }

    double ramp_speed(double last_cmd, double target, double dt,
                      double max_accel, double max_decel) const {
        return f1tenth_control::rate_limit_speed_command(
            last_cmd, target, dt, max_accel, max_decel);
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

        // 0-a2. odom 워치독 — 위치추정이 도중에 끊기면 제어가 성립하지 않는다.
        //   ⚠️ 경로 중재보다 **먼저** 와야 한다. 아래 단계는 전부 current_x_/y_/yaw_를 쓰는데,
        //      끊긴 동안 그 값은 얼어 있고 차는 계속 달린다(0818 134408: 0.8초에 3.9 m).
        //   조향은 직전 각을 유지한 채 속도만 0으로 준다 — ②-r 참고.
        const double odom_age = (current_time - odom_last_recv_time_).seconds();
        if (odom_timeout_ > 0.0 && odom_age > odom_timeout_) {
            if (!odom_stale_) {
                odom_stale_ = true;
                ++odom_stale_count_;
                odom_stale_since_ = odom_last_recv_time_;   // 마지막 정상 수신 시각
            }
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "odom(%s) %.2fs 끊김(> %.2fs) — 안전 정지(조향 %.3f rad 유지). "
                "위치추정/네트워크 확인 (누적 %u회)",
                odom_topic_.c_str(), odom_age, odom_timeout_,
                last_steering_angle_, odom_stale_count_);
            publish_safe_stop(/*hold_steering=*/true);
            return;
        }
        if (odom_stale_) {
            odom_stale_ = false;
            // 복귀 시점의 pose는 크게 점프해 있을 수 있다(134408: 3.25 m). 램프는 위
            // publish_safe_stop이 매 사이클 실측에 붙여 뒀으므로 계단 명령은 안 나가지만,
            // 최근접 인덱스는 다시 찾아야 하므로 로그로 남긴다.
            // ⚠️ 여기서 odom_age를 찍으면 **방금 받은 샘플의 나이**(≈0)가 나와 두절 길이를
            //    오해하게 된다. 마지막 정상 수신 시각부터 재개까지의 실제 공백을 찍는다.
            const double outage = (odom_last_recv_time_ - odom_stale_since_).seconds();
            RCLCPP_WARN(this->get_logger(),
                "odom(%s) 복구 — 실제 두절 %.2fs (누적 %u회). 복귀 직후 pose 점프 주의",
                odom_topic_.c_str(), outage, odom_stale_count_);
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
                // ⚠️ 조향 생성과 **같은** K_us를 쓴다(understeer_gradient_eff). 이 일원화가
                //    ②-p의 핵심이다 — 종방향이 "이 속도면 꺾인다"고 판단하는 근거와 실제
                //    조향을 만드는 근거가 다르면, 그 차이만큼 코너에서 조향이 모자란다.
                //    좌/우 분리도 같은 이유로 **부호 있는 곡률**로 그 코너의 K_us를 고른다.
                const double kus = understeer_gradient_eff(wps[i].smoothed_curvature_signed);
                if (kus > 1e-6) {                                                   // (b) 조향 권한
                    // ⚠️ 좌우 중 **작은** 한계를 쓰고, 거기에 도달각 비율까지 곱한다 —
                    //    캡은 "바퀴가 실제로 꺾이는 각"으로 계산해야 의미가 있다(0.379를 다
                    //    낸다고 보면 코너 진입 속도를 그만큼 과대 허용한다).
                    double steer_budget = steer_avail() - wheelbase_ * k_i;
                    double v_steer = (steer_budget > 0.0)
                        ? std::sqrt(steer_budget / (kus * k_i))
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
        // ⚠️ 게이트는 local_fresh가 아니라 avoiding_now()(= /state != STATE_GLOBAL)다.
        //    local_fresh는 "/local_waypoints를 최근에 받았나"일 뿐이고, state_machine은
        //    GLOBAL/CRUISE에서도 매 frenet odom마다 글로벌 라인을 그 토픽으로 발행하므로
        //    local_fresh는 평상시 주행에서도 상시 true다(과거 이 조건은 사실상 no-op 게이트였다).
        if (avoidance_l1_damping_enable_ && avoiding_now()) {
            L1_distance *= avoidance_l1_scale_max_;
        }
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

        // 5. 목표 횡가속도 → 조향각 (자전거 역모델)
        double lat_acc = 0.0;
        speed_for_lu = std::min(speed_for_lu, curvature_speed_limit);
        // 🔴 실측 속도 상한 (선언부 주석 참고). L1_distance가 실측 속도로 계산되므로 게인도
        //    같은 속도를 써야 한다 — 안 그러면 정지/저속에서 게인이 (v_prof/v_meas)²배로 뛴다.
        //    ⚠️ 반드시 lat_acc 계산과 역모델 변환 **앞**에 둘 것(둘 다 speed_for_lu를 쓴다).
        //    (자전거 역모델에선 v²이 소거돼 이 캡 없이도 발산하지 않지만, 언더스티어
        //     항 K_us·v²의 크기는 여전히 여기서 정해지므로 실측 속도를 쓰는 게 맞다.)
        if (steering_speed_cap_measured_) {
            speed_for_lu = std::min(speed_for_lu, current_speed_);
        }
        // 🔴 **조향용 속도에는 바닥이 필요하다.** local_planning은 safe stop과 제동경로
        //    말단에서 vx_mps=0 웨이포인트를 발행한다. 그대로 두면 a_lat=0 → a_cmd=0 →
        //    조향 0이 되어, **차가 아직 4 m/s로 굴러가는 중에 바퀴가 곧게 펴진다**
        //    (코너 한복판 비상정지 = 바깥 벽으로 직진). 종방향은 그 0을 그대로 따라야
        //    맞지만(정지는 planning의 권한), 조향까지 버릴 이유는 없다.
        // 🔑 자전거 역모델에서 v²이 소거되므로(δ = 2L·sinη/L1) 이 바닥값이 얼마든
        //    pure pursuit 기하는 그대로 나온다 — K_us·v² 항만 미미하게 줄 뿐이다.
        //    ②-f의 "정지 상태 풀락"과 반대 방향으로 안전하다: 거기선 v²이 안 나뉘어
        //    게인이 폭발했고, 여기선 소거되어 기하로 수렴한다.
        speed_for_lu = std::max(speed_for_lu, steering_speed_floor_);
        double l1_denom = l1_use_actual_distance_ ? std::max(L1_norm, l1_min_denom_)
                                                  : std::max(L1_distance, l1_min_denom_);
        lat_acc = 2.0 * speed_for_lu * speed_for_lu / l1_denom * sin_eta;

        // ── FF/FB 분리 + 정상상태 자전거 역모델 (②-p) ─────────────────────────
        // FF: 경로 곡률이 요구하는 정상상태 몫. **오차가 생기기 전에** 이미 작동하므로
        //     일정 곡률 구간에서 L1이 오차를 만들어 가며 따라갈 필요가 없다.
        // FB: L1 명령 중 그 몫으로 설명되지 않는 나머지 = 순수 보정분.
        // 🔑 steering_fb_gain=1.0이면 a_ff + 1.0·(lat_acc − a_ff) = lat_acc 로
        //    분리 전(= 순수 L1)과 **수학적으로 동일**하다(모델이 선형이라 성립).
        const double ff_kappa = curvature_for_ff(wps, closest_idx, path_closed);
        const double a_ff = ff_kappa * speed_for_lu * speed_for_lu;
        double a_cmd = a_ff + steering_fb_gain_ * (lat_acc - a_ff);
        // 그립 한계를 **명시적으로** 건다. 구 LUT는 이걸 "표가 NaN으로 끊긴 지점"이라는
        // 암묵적·속도의존적 경계로 갖고 있었고(속도별 0.12~0.36 rad로 제각각), 그게
        // 사전감속이 가정한 δ_avail과 어긋나 코너에서 조향이 모자랐다. 여기서는
        // 사전감속이 쓰는 것과 **같은 기준**(섹터 스케일이 적용된 mla)을 쓴다.
        // ⚠️ 여기만 마진을 얹는다. 속도 캡(:1162 부근)은 마진 없는 mla를 그대로 쓴다 —
        //    그래야 "계획 속도는 보수적으로, 보정 권한은 그보다 넉넉히"가 성립한다.
        const double a_max = ((wps[closest_idx].mla > 0.0) ? wps[closest_idx].mla
                                                           : max_lateral_accel_)
                             * steering_accel_margin_;
        a_cmd = std::clamp(a_cmd, -a_max, a_max);
        double steering_angle = bicycle_steer_from_lat_acc(a_cmd, speed_for_lu);

        if (std::abs(lat_acc) > a_max && std::abs(sin_eta) > 0.05 && speed_for_lu > 2.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "그립 권한 포화: 요구 a_lat %.2f > 권한 %.2f m/s² @ %.2f m/s — "
                "코너 진입 속도/prebrake_decel 확인",
                std::abs(lat_acc), a_max, speed_for_lu);
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

        // ⚠️ 여기 있던 `curvature_ff_blend`(곡률 FF 가중평균)는 2026-08-17에 제거됐다.
        //    그 항은 `atan(L·κ)` = Ackermann 몫뿐이라 언더스티어 항(K_us·κ·v²)이 통째로
        //    빠져 있었고(정상상태 필요 조향의 85%@2 m/s ~ 44%@5.5 m/s), 게다가 가중평균이라
        //    켤수록 조향이 **줄어들었다** — 런치 기본값이 0.0에 방치돼 온 이유로 보인다.
        //    지금은 위 6-1)에서 완전한 정상상태 FF(a_ff)가 들어가므로 역할이 대체됐다.

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
        // 6-9) K_us 추정기. 트림과 **같은 이력·같은 지연**을 쓰되 게이트가 서로 배타적이다
        //      (트림 = 직선 |a_lat| ≤ 2.0 / K_us = 코너 |a_lat| ≥ 3.0) — 두 추정기가 같은
        //      신호를 두고 싸우지 않게 작동 영역을 갈라 둔 것이다.
        update_understeer_gradient(dt, steering_angle);
        // 6-10) 슬립 잔차 — 진단 전용. 자이로와 가속도계라는 **서로 독립인** 두 경로가
        //       갈라지는 정도가 곧 타이어가 미끄러진 정도다(둘 다 MCL과 무관하다).
        update_slip_residual(dt);

        // 7. 목표 속도 ───────────────────────────────────────────────────────────────
        double global_speed =
            wps[find_lookahead_wp_idx(wps, path_closed, closest_idx, speed_lookahead_)].speed;
        global_speed = std::min(global_speed, curvature_speed_limit);
        global_speed = std::min(global_speed, max_speed_);
        double target_speed = global_speed;

        // Cruise는 기존 경로 기하를 유지하고 종방향 목표 속도에 상한만 적용한다.
        if (cruise_limit_enable_) {
            // Fail closed from process start. Waiting until the first cruise message made a
            // missing/failed cruise node indistinguishable from "no constraint" and allowed the
            // full waypoint speed indefinitely. Once a message has been seen, the same timeout
            // watchdog covers a later publisher failure.
            const bool cruise_fresh = cruise_speed_limit_seen_ &&
                (current_time - cruise_speed_limit_last_recv_time_).seconds() <=
                cruise_speed_limit_timeout_;
            const double cruise_cap = cruise_fresh ? cruise_speed_limit_ : cruise_stale_speed_;
            target_speed = std::min(target_speed, cruise_cap);
            if (!cruise_fresh) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "cruise speed limit %s — %.2f m/s fail-safe cap 적용",
                    cruise_speed_limit_seen_ ? "stale" : "not received yet",
                    cruise_stale_speed_);
            }
        }

        // 7-a. U1 그립 권한 속도 클램프 (계약은 선언부 주석 참고, 기본 비활성).
        //      target_speed 단계 적용이라 아래 8번 램프가 감속률을 실현 가능하게 다듬고,
        //      램프가 못 따라가는 판(진입이 이미 과속)은 deficit 카운트로 드러난다.
        {
            const bool engaged_now = !(engage_gate_active() && !is_engaged_);
            if (following_local != grip_prev_following_local_ || !engaged_now) {
                // 경로 세대 전환(로컬↔글로벌)·미체결 구간: 이전 경로의 요구 곡률을
                // 새 경로에 물려주지 않는다 (검토 계약: 전환 시 필터 리셋).
                grip_demand_kappa_filt_ = 0.0;
            }
            grip_prev_following_local_ = following_local;
            const double demand_kappa = 2.0 * std::abs(sin_eta) / l1_denom;
            if (demand_kappa > grip_demand_kappa_filt_) {
                grip_demand_kappa_filt_ = demand_kappa;   // 빠른 제한 — 진입을 늦추지 않는다
            } else {
                grip_demand_kappa_filt_ +=
                    grip_clamp_release_alpha_ * (demand_kappa - grip_demand_kappa_filt_);
            }
            if (grip_speed_clamp_enable_ && engaged_now &&
                grip_demand_kappa_filt_ > 1e-4)
            {
                const double budget =
                    ((wps[closest_idx].mla > 0.0) ? wps[closest_idx].mla
                                                  : max_lateral_accel_) * grip_clamp_margin_;
                const double v_grip = std::sqrt(budget / grip_demand_kappa_filt_);
                if (v_grip < target_speed) {
                    if (target_speed - v_grip > 0.3) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "그립 클램프: 목표 %.2f → %.2f m/s (κ_L1 %.3f, 예산 %.2f m/s², "
                            "누적 %lu)",
                            target_speed, v_grip, grip_demand_kappa_filt_, budget,
                            static_cast<unsigned long>(grip_clamp_count_));
                    }
                    target_speed = v_grip;
                    ++grip_clamp_count_;
                }
                const double v_meas = std::max(0.0, current_speed_);
                if (v_meas * v_meas * grip_demand_kappa_filt_ > budget * 1.15) {
                    // 실측이 이미 예산 밖 = 진입 과속을 램프가 못 따라간 것. 진단 전용.
                    ++grip_decel_deficit_count_;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                        "그립 클램프 감속 부족: 실측 %.2f m/s 요구 %.2f > 예산 %.2f m/s² "
                        "(누적 %lu)",
                        v_meas, v_meas * v_meas * grip_demand_kappa_filt_, budget,
                        static_cast<unsigned long>(grip_decel_deficit_count_));
                }
            }
        }

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
            launch_relatch_timer_ = 0.0;
            // 조향 트림 추정도 리셋한다 — 미체결 중엔 발행이 하류로 안 나가므로 그 구간의
            // "명령 vs 요레이트"는 물리적 의미가 없고, 재체결 시 남은 값이 계단으로 나간다.
            // ⚠️ 0이 아니라 steering_trim_init_ 로 되돌린다. 미체결 구간의 관측이
            //    무의미한 것이지, 차의 **기계 중립이 0이 되는 게 아니다**. 0으로 되돌리면
            //    재체결마다 웜업을 다시 사고(0819 실측 직선오차 +0.11 m에서 재시작),
            //    랩마다 재체결하는 측정 하네스는 영영 수렴값을 못 본다.
            steering_trim_ = steering_trim_init_;
            steering_trim_samples_ = 0;
            steering_trim_gated_time_ = 0.0;
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
                // 🟢 2026-08-20: 포기 래치 재무장(`launch_relatch_time`, 0 = 구 거동).
                //    예전엔 래치가 한 번 서면 **차가 실제로 launch_exit_speed를 넘을 때까지**
                //    영구히 안 풀렸다 = 못 나가고 있을 때 킥이 사라진다. 재출발이 한 번
                //    실패하면 그 뒤로는 램프(=플래너 목표)만 남는데, 회피 서행 목표가 킥 바닥
                //    (launch_boost_speed)보다 낮은 재출발에서 정확히 이게 손해다.
                //    0819 `run_0819_214041` 실측: 5.51 s 무장 → 7.01 s에 정확히 1.50 s로 만료
                //    → 명령이 2.00에서 플래너 목표 1.49로 떨어짐 → 7.43 s 인계 시도는 킥 없이
                //    1.49로 하다 붕괴 → 7.71 s에야 돌파(총 1.82 s).
                // 🔑 **성공하는 출발에는 비용이 정확히 0이다** — 래치가 서지 않으면 이 타이머는
                //    돌지도 않는다. 즉 "이미 실패 중일 때만" 작동하는 변경이다.
                // ⚠️ 조건을 "정지 지속"으로 잡은 것이 핵심이다. 시간만 세면 데드존을 걸터앉아
                //    기어가는 중(0.3~0.9 m/s)에도 재무장돼 굴러가는 차에 킥이 꽂힌다.
                if (launch_relatch_time_ > 0.0 && launch_latched_off_ && standstill) {
                    launch_relatch_timer_ += dt;
                    if (launch_relatch_timer_ >= launch_relatch_time_) {
                        launch_latched_off_ = false;
                        launch_relatch_timer_ = 0.0;
                        ++launch_relatch_count_;
                        RCLCPP_WARN(this->get_logger(),
                            "런치 킥 래치 재무장: 정지 %.1fs 지속 + 목표 %.2f m/s → 다시 시도한다 "
                            "(누적 %lu회). 계속 반복되면 데드존이 아니라 기계적 구속을 의심할 것",
                            launch_relatch_time_, target_speed, launch_relatch_count_);
                    }
                } else {
                    launch_relatch_timer_ = 0.0;
                }
                if (!launch_active_ && standstill && !launch_latched_off_) {
                    launch_active_ = true; launch_time_ = 0.0;
                }
                if (launch_active_) {
                    launch_time_ += dt;
                    if (moving) {
                        launch_active_ = false;                        // 관통 성공
                        // 킥이 끝나는 이 사이클에만, 램프가 방금 발행하던 킥 값 근처에서
                        // 이어지도록 1회 맞춘다(해제 직후 속도 절벽 방지). ⚠️ ramp_lead_max로
                        // 상한을 걸어서 이번 사이클 안티와인드업 클램프(위 8절)를 벗어나지
                        // 않게 한다 — 벗어나면 세이프스톱 등으로 target이 낮게 바뀌어도
                        // last_target_speed_가 킥 값에 고착돼 요구보다 과속 발행할 수 있다
                        // (B-4: ramp_speed 정지 래치와 결합하면 더 나쁘다).
                        const double catchup_cap = ramp_lead_max_ > 0.0
                            ? current_speed_ + ramp_lead_max_ : launch_boost_speed_;
                        last_target_speed_ = std::max(last_target_speed_,
                            std::min({publish_speed, launch_boost_speed_, catchup_cap}));
                    } else if (launch_time_ > launch_boost_time_) {
                        launch_active_ = false; launch_latched_off_ = true;
                        RCLCPP_WARN(this->get_logger(),
                            "런치 킥 %.2fs 관통 실패 → 포기. 데드존 심함 — %s",
                            launch_time_,
                            launch_relatch_time_ > 0.0
                                ? "정지가 이어지면 재무장한다(launch_relatch_time)"
                                : "푸시스타트 필요");
                    } else {
                        publish_speed = std::max(publish_speed, launch_boost_speed_);
                        // ⚠️ 여기서 last_target_speed_를 건드리지 않는다 — 킥은 "발행값만
                        //    덮는다"는 불변식(②-i)을 지킨다. 매 사이클 밀면 플래너가 낮은
                        //    target(예: 회피 서행 0.5)을 요구하는 중에도 램프가 킥 값에
                        //    고착돼 요구 속도를 초과 발행하게 된다.
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                            "런치 킥: 실측 %.2f → 발행 %.2f m/s (t=%.2fs)",
                            current_speed_, publish_speed, launch_time_);
                    }
                }
            } else {
                launch_active_ = false;   // 정지 명령 중엔 킥 안 함
                launch_relatch_timer_ = 0.0;   // 갈 의도가 없으면 재무장 카운트도 멈춘다
            }
        }

        // 8-d. HFI 정지출발 포착 보호. 수동 명령은 control_map_node를 거치지 않으므로
        //      이 로직의 영향을 받지 않는다. 경로제어용 PF odom(current_speed_)이 아니라
        //      별도 VESC /odom(hfi_speed_)으로 전진 포착·역회전·완전정지를 판정한다.
        //      launch boost보다 뒤에서 적용해, 실수로 두 기능을 함께 켜도 HFI 상한이 이긴다.
        if (hfi_launch_guard_enable_) {
            f1tenth_control::HfiLaunchGuardConfig config;
            config.enabled = true;
            config.speed_cap = hfi_launch_speed_cap_;
            config.exit_speed = hfi_launch_exit_speed_;
            config.standstill_speed = hfi_launch_standstill_speed_;
            config.timeout = hfi_launch_timeout_;
            config.exit_hold = hfi_launch_exit_hold_;
            config.relatch_time = hfi_launch_relatch_time_;
            config.retry_cooldown = hfi_launch_retry_cooldown_;
            config.max_attempts = hfi_launch_max_attempts_;

            const double hfi_speed_age = hfi_speed_seen_
                ? (current_time - hfi_speed_last_recv_time_).seconds()
                : std::numeric_limits<double>::infinity();
            const bool hfi_speed_fresh = hfi_speed_seen_ &&
                (hfi_launch_speed_timeout_ <= 0.0 ||
                 hfi_speed_age <= hfi_launch_speed_timeout_);

            f1tenth_control::HfiLaunchDecision decision;
            if (hfi_speed_fresh) {
                decision = f1tenth_control::update_hfi_launch_guard(
                    hfi_launch_state_, config, disengaged, target_speed, hfi_speed_, dt);
            } else {
                // 속도가 끊긴 동안 정지 요청이 들어왔다는 사실은 기억하되, 가짜 0속도로
                // dwell을 채워 재무장하지 않는다. standstill 문턱과 같은 값을 넣으면
                // strict '<' 판정상 정지가 아니며 dt=0이라 타이머도 진행하지 않는다.
                if (disengaged || target_speed <= 0.1) {
                    (void)f1tenth_control::update_hfi_launch_guard(
                        hfi_launch_state_, config, disengaged, target_speed,
                        hfi_launch_standstill_speed_, 0.0);
                }
                const bool speed_required = hfi_launch_state_.active ||
                    hfi_launch_state_.retry_waiting || hfi_launch_state_.relatch_pending ||
                    hfi_launch_state_.failure_latched ||
                    (hfi_launch_state_.armed && !disengaged && target_speed > 0.1);
                if (speed_required) {
                    decision.force_stop = true;
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "HFI 속도원(%s) 미수신/%.3fs stale — 출발 판정 불가, 속도 0 유지",
                        hfi_launch_speed_topic_.c_str(), hfi_speed_age);
                }
            }

            if (decision.event == f1tenth_control::HfiLaunchEvent::kStarted) {
                RCLCPP_INFO(this->get_logger(),
                    "HFI 정지출발 보호 시작(%u/%u): 발행 %.2f m/s 상한, VESC +%.2f m/s를 "
                    "%.2fs 유지하면 해제, 제한 %.1fs",
                    hfi_launch_state_.attempt, hfi_launch_max_attempts_,
                    hfi_launch_speed_cap_, hfi_launch_exit_speed_, hfi_launch_exit_hold_,
                    hfi_launch_timeout_);
            } else if (decision.event == f1tenth_control::HfiLaunchEvent::kReleased) {
                RCLCPP_INFO(this->get_logger(),
                    "HFI 정지출발 보호 해제: VESC 전진속도 %.2f m/s / %.2fs / 시도 %u "
                    "(누적 %lu회)",
                    hfi_speed_, hfi_launch_state_.elapsed, hfi_launch_state_.attempt,
                    hfi_launch_state_.release_count);
            } else if (decision.event == f1tenth_control::HfiLaunchEvent::kRetryScheduled) {
                RCLCPP_WARN(this->get_logger(),
                    "HFI 정지출발 시도 %u/%u가 %.1fs에 실패 — 속도 0 및 VESC 완전정지 "
                    "%.1fs 후 제한 재시도",
                    hfi_launch_state_.attempt, hfi_launch_max_attempts_,
                    hfi_launch_timeout_, hfi_launch_retry_cooldown_);
            } else if (decision.event == f1tenth_control::HfiLaunchEvent::kRetryStarted) {
                RCLCPP_WARN(this->get_logger(),
                    "HFI 정지출발 제한 재시도 시작(%u/%u): VESC %.2f m/s",
                    hfi_launch_state_.attempt, hfi_launch_max_attempts_, hfi_speed_);
            } else if (decision.event == f1tenth_control::HfiLaunchEvent::kTimedOut) {
                RCLCPP_ERROR(this->get_logger(),
                    "HFI 정지출발 %u회 모두 관통 실패 → 속도 0 실패 래치 (누적 %lu회). "
                    "정지 목표+VESC 완전정지 %.1fs 후에만 재무장",
                    hfi_launch_max_attempts_, hfi_launch_state_.failure_count,
                    hfi_launch_relatch_time_);
            } else if (decision.event == f1tenth_control::HfiLaunchEvent::kFailureReset) {
                RCLCPP_WARN(this->get_logger(),
                    "HFI 정지출발 실패 래치 해제 — 정지 목표+VESC 완전정지 %.1fs 확인",
                    hfi_launch_relatch_time_);
            }

            if (decision.constrain_to_cap) {
                publish_speed = std::min(publish_speed, hfi_launch_speed_cap_);
                final_speed = std::min(final_speed, hfi_launch_speed_cap_);
                last_target_speed_ = std::min(last_target_speed_, hfi_launch_speed_cap_);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "HFI 정지출발 보호 중(%u/%u): VESC %.2f / PF %.2f / 발행 %.2f / "
                    "목표 %.2f m/s (%.2f/%.1fs)",
                    hfi_launch_state_.attempt, hfi_launch_max_attempts_,
                    hfi_speed_, current_speed_, publish_speed, target_speed,
                    hfi_launch_state_.elapsed, hfi_launch_timeout_);
            } else if (decision.force_stop) {
                publish_speed = 0.0;
                final_speed = 0.0;
                last_target_speed_ = 0.0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "HFI 속도 0 유지: VESC %.2f m/s | 재시도대기=%s 재무장대기=%s "
                    "최종래치=%s",
                    hfi_speed_, hfi_launch_state_.retry_waiting ? "Y" : "N",
                    hfi_launch_state_.relatch_pending ? "Y" : "N",
                    hfi_launch_state_.failure_latched ? "Y" : "N");
            }
        }

        if (status_log_period_ms_ > 0) {
            // ⚠️ Idx 앞의 L/G는 **어느 배열의 인덱스인지**다. 로컬(L)과 글로벌(G)은 배열이
            //    달라 소스가 바뀌면 번호가 크게 튀는데(실측 L53→G134) 실제 목표점 좌표는
            //    연속이다. 이 표기가 없으면 "룩어헤드가 트랙 반대쪽으로 튀었다"로 오독된다.
            // FF/FB 내역과 K_us 추정 — 이 셋(a_ff / a_cmd / K_us)이 있어야 로그만으로
            // "FF가 얼마나 일했나 / 적응이 수렴했나"를 가를 수 있다.
            char model_buf[256] = "";
            std::snprintf(model_buf, sizeof(model_buf),
                " | FF κ %+.3f a_ff %+.2f / a_cmd %+.2f | K_us %.5f%s(n=%ld)"
                " | 곡선%s[%.4f/%.4f/%.4f/%.4f n=%ld/%ld/%ld/%ld] | 슬립 %.2f(peak %.2f)",
                ff_kappa, a_ff, a_cmd, understeer_gradient_eff(a_cmd),
                (understeer_adapt_gain_ > 0.0) ? "" : "(관측)", understeer_adapt_samples_,
                understeer_curve_enable_ ? "" : "(관측)",
                kus_bin_[0], kus_bin_[1], kus_bin_[2], kus_bin_[3],
                kus_bin_n_[0], kus_bin_n_[1], kus_bin_n_[2], kus_bin_n_[3],
                slip_residual_, slip_residual_peak_);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), status_log_period_ms_,
                "Pose: (%.2f, %.2f, %.2f) | Target WP: (%.2f, %.2f), Idx: %c%zu -> %c%zu | Steer: %.4f | "
                "Speed: %.2f / %.2f | L1_dist: %.2f | acc_mean: %.2f | 점프 %lu/뒤쪽 %lu/경로반전 %u"
                " | trim: %+.2f° (n=%ld)%s%s",
                current_x_, current_y_, current_yaw_, L1_x, L1_y,
                following_local ? 'L' : 'G', closest_idx, following_local ? 'L' : 'G', idx_a,
                steering_angle, final_speed, current_speed_, L1_distance, acc_mean,
                l1_jump_count_, l1_behind_count_, local_heading_reject_count_,
                steering_trim_ * 180.0 / M_PI, steering_trim_samples_,
                // 섹터가 켜져 있으면 **지금 이 지점에 실제로 적용된 MLA**를 같이 찍는다.
                // 파라미터가 아니라 적용값을 찍어야 "켜졌는데 왜 안 빨라지나"를 로그만으로 가른다.
                sector_status_suffix(wps[closest_idx].mla).c_str(), model_buf);
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

    // 회피/추월 중인가(= /state가 STATE_GLOBAL이 아님). 섹터 스케일과 같은 /state 구독을
    // 공유하고, 같은 "모르면 끈다" 규약을 쓴다 — 미수신/끊김이면 회피 여부를 모르므로 false
    // (avoidance_l1_damping은 그 경우 평상 L1을 유지한다. sector_active()와 달리 이 함수는
    // sector_scale_enable_과 무관하게 동작해야 한다 — 섹터 스케일을 꺼도 L1 감쇠는 살아있다).
    bool avoiding_now() const {
        if (!sector_state_seen_) return false;
        if ((this->now() - sector_state_last_recv_time_).seconds() > sector_scale_state_timeout_)
            return false;
        return !sector_on_global_;
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
    bool avoidance_l1_damping_enable_ = true;
    double avoidance_l1_scale_max_ = 1.0;
    bool steering_speed_cap_measured_ = true;  // 조향용 속도를 실측 속도로 상한
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

    // 종방향
    double base_max_accel_;
    double ramp_lead_max_ = 2.4;   // 램프 안티와인드업 선행 상한 [m/s], 0이면 비활성
    bool hfi_launch_guard_enable_ = false;
    double hfi_launch_speed_cap_ = 0.7;
    double hfi_launch_exit_speed_ = 0.5;
    double hfi_launch_standstill_speed_ = 0.1;
    double hfi_launch_timeout_ = 4.0;
    double hfi_launch_exit_hold_ = 0.1;
    double hfi_launch_relatch_time_ = 0.5;
    double hfi_launch_retry_cooldown_ = 0.5;
    unsigned int hfi_launch_max_attempts_ = 2;
    std::string hfi_launch_speed_topic_ = "/odom";
    double hfi_launch_speed_timeout_ = 0.2;
    double hfi_speed_ = 0.0;
    bool hfi_speed_seen_ = false;
    rclcpp::Time hfi_speed_last_recv_time_;
    f1tenth_control::HfiLaunchGuardState hfi_launch_state_;
    double base_max_decel_;                  // 명령 속도 하강 rate limit [m/s²]
    double prebrake_decel_ = 1.5;            // 곡률 사전감속용 실측 감속 권한 [m/s²]
    double max_speed_, min_speed_;
    bool cruise_limit_enable_ = true;
    std::string cruise_speed_limit_topic_ = "/cruise_speed_limit";
    double cruise_speed_limit_timeout_ = 0.15;
    double cruise_stale_speed_ = 1.5;
    double cruise_speed_limit_ = 0.0;
    bool cruise_speed_limit_seen_ = false;
    rclcpp::Time cruise_speed_limit_last_recv_time_{0, 0, RCL_ROS_TIME};

    // 런치 킥
    bool launch_boost_enable_ = true;
    double launch_boost_speed_ = 2.2, launch_boost_time_ = 0.6;
    double launch_exit_speed_ = 0.8, launch_standstill_speed_ = 0.3;
    bool launch_active_ = false;
    double launch_time_ = 0.0;
    bool launch_latched_off_ = false;        // 관통 실패로 포기(재무장 조건은 control_loop 8-c 참고)
    double launch_relatch_time_ = 0.0;       // 포기 후 재시도까지 필요한 정지 지속 시간 [s], 0 = 끔
    double launch_relatch_timer_ = 0.0;      // 위 조건 누적 시간
    unsigned long launch_relatch_count_ = 0; // 재무장 횟수(진단용)

    bool use_imu_;
    double imu_linear_scale_ = 1.0;
    double imu_angular_scale_ = 1.0;         // deg/s → rad/s (real=pi/180, sim=1.0)
    std::vector<double> acc_now_;            // 종가속 rolling buffer
    double yaw_rate_now_ = 0.0;              // 실측 요레이트 [rad/s] (트림·K_us 추정)
    double yaw_rate_prev_ = 0.0;             // 준정상상태 게이트용(요각가속 산출)
    double lat_acc_now_ = 0.0;               // 실측 횡가속 [m/s²] (슬립 잔차 진단 전용)
    rclcpp::Time yaw_rate_last_recv_{0, 0, RCL_ROS_TIME};
    bool yaw_rate_seen_ = false;

    double steering_trim_ = 0.0;             // 추정된 트림 [rad], 발행 명령에 더해진다
    double steering_trim_init_ = 0.0;        // 시작값 = 재체결 리셋값 [rad]
    double steering_trim_warmup_gain_ = 0.0; // 웜업 상한 게인 [1/s], 0 = 비활성
    double steering_trim_gated_time_ = 0.0;  // 게이트가 열려 있던 누적시간 [s]
    std::string steering_trim_persist_file_;  // 트림 저장 경로, 빈 문자열 = 비활성
    double steering_trim_persist_max_age_ = 0.0;  // 저장본 유효 나이 [s]
    double steering_trim_saved_ = 1e9;        // 마지막으로 파일에 쓴 값 [rad]
    rclcpp::TimerBase::SharedPtr trim_persist_timer_;
    double steering_trim_gain_ = 0.0;        // 1/τ [1/s], 0 = 비활성
    double steering_trim_limit_ = 0.06;      // |trim| 상한 [rad] (≈3.4°)
    double steering_trim_max_steer_ = 0.10;  // 이 각을 넘는 조향 중엔 학습 정지 [rad]
    double steering_trim_min_speed_ = 2.0;   // 이 속도 미만에선 학습 정지 [m/s]
    double steering_trim_max_lat_acc_ = 2.0; // 이 횡가속을 넘으면 학습 정지 [m/s²]
    double steering_trim_lag_ = 0.14;        // 조향→요레이트 지연 [s] (0810 bag 실측 140 ms)
    long   steering_trim_samples_ = 0;       // 학습 샘플 수 (로그용)
    std::deque<std::pair<double, double>> steer_hist_;   // (t, 발행 조향) — lag 조회용

    // 조향 생성 모델 (②-p)
    double steering_fb_gain_ = 1.0;          // FF/FB 분리 게인. 1.0 = 분리 전과 수학적 동일
    double curvature_ff_preview_ = 0.0;      // FF가 곡률을 읽을 전방 거리 [m], 0 = 최근접점

    // K_us 온라인 적응 (관측 전용이 기본 — understeer_adapt_gain_ = 0)
    double understeer_adapt_gain_ = 0.0;     // 1/τ [1/s], 0 = 추정만 하고 적용 안 함
    double understeer_gradient_adapted_ = 0.019;
    double understeer_min_ = 0.008, understeer_max_ = 0.025;
    double understeer_adapt_min_lat_acc_ = 3.0;  // 관측성 게이트(코너 전용)
    double understeer_adapt_min_speed_ = 2.0;
    long   understeer_adapt_samples_ = 0;
    // 관측 모드에서 쓰는 LPF 게인(τ≈4 s). 적용은 안 하고 로그로만 수렴을 보여준다.
    static constexpr double kUsObserveGain = 0.25;

    // 조향용 속도 하한 [m/s]. planning의 vx_mps=0(safe stop)에서 조향까지 0이 되는 것 방지.
    double steering_speed_floor_ = 0.5;

    // K_us(a_lat) 곡선 (②-q). 기본 비활성 = 스칼라 그대로.
    bool understeer_curve_enable_ = false;
    int  understeer_curve_min_samples_ = 300;   // 빈당 이만큼 쌓여야 그 빈을 쓴다
    double kus_bin_[kUsBins] = {0.019, 0.019, 0.019, 0.019};
    long   kus_bin_n_[kUsBins] = {0, 0, 0, 0};
    // 준정상상태 게이트 [rad/s²]. a_lat = v·ψ̇ 는 β̇=0에서만 참이다.
    static constexpr double kUsCurveMaxYawAcc = 3.0;

    // 슬립 잔차 |a_y − v·ψ̇| (진단 전용, 주행 개입 없음)
    double slip_residual_ = 0.0;
    double slip_residual_peak_ = 0.0;

    // 곡률 사전감속
    size_t curvature_lookahead_count_;
    double max_lateral_accel_;
    // 조향 클램프 = mla × 이 값. 1.0이면 구 거동(보정 예산 0). ②-y
    double steering_accel_margin_ = 1.0;
    // U1 그립 권한 속도 클램프 상태 (선언부 주석 참고)
    bool grip_speed_clamp_enable_ = false;
    double grip_clamp_margin_ = 0.9;
    double grip_clamp_release_alpha_ = 0.05;
    double grip_demand_kappa_filt_ = 0.0;
    bool grip_prev_following_local_ = false;
    uint64_t grip_clamp_count_ = 0;
    uint64_t grip_decel_deficit_count_ = 0;
    double understeer_gradient_ = 0.019;     // K_us [rad/(m/s²)] — 조향 권한 캡, 0이면 비활성
    double understeer_gradient_left_ = 0.019;   // 좌회전 K_us (생성자에서 해소, ≤0 = 공용값)
    double understeer_gradient_right_ = 0.019;  // 우회전 K_us (실측상 좌보다 크다 — 0818)
    double steer_authority_ratio_ = 0.85;

    // 좌우 조향 한계 [rad]. 둘 다 같으면 기존 대칭 거동과 동일.
    double max_steering_left_ = MAX_STEERING_ANGLE;
    double max_steering_right_ = MAX_STEERING_ANGLE;
    double steer_limit_min_ = MAX_STEERING_ANGLE;   // 속도 캡용 보수값


    // 차량 상태 / 출력 이력
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0, current_speed_ = 0.0;
    double last_target_speed_ = 0.0, last_steering_angle_ = 0.0;
    double last_published_speed_ = 0.0;      // 발행 acceleration(명령 속도 미분)의 기준
    rclcpp::Time last_time_;

    // odom 워치독 — 위치추정 없이/끊긴 채로 주행하지 않는다
    bool odom_seen_ = false;
    double odom_timeout_ = 0.5;              // [s] 0이면 비활성
    rclcpp::Time odom_last_recv_time_;
    bool odom_stale_ = false;                // 워치독 발동 중인가(에지 로그·복귀 처리용)
    rclcpp::Time odom_stale_since_;          // 두절 직전 마지막 정상 수신 시각
    uint32_t odom_stale_count_ = 0;          // 발동 누적(진단용)

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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr hfi_speed_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_mode_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_path_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr local_path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cruise_speed_limit_sub_;
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
