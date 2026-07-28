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
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"

#include "f1tenth_control/types.hpp"
#include "f1tenth_control/gap_follower.hpp"
#include "f1tenth_control/imu_stability_controller.hpp"
#include "f1tenth_control/steering_lookup_table.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "f110_msgs/msg/obstacle_array.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace f1tenth_control;

namespace {

// 조향각 물리 한계 [rad] — 차량 스펙 기준값(±23.5°). 좌우 한계 파라미터의 기본값이며,
// 하드웨어가 대칭이면 이 값 하나로 충분하다.
//
// ⚠️ 2026-07-28: 좌우 한계를 파라미터로 분리했다. 실차에서 한동안 우조향이 잘리고 있었다.
//   젯슨 vesc.yaml의 servo_min 0.2703 / servo_max 0.6363 은 servo **0.4533 중심 ±0.1830**
//   (= ±0.41 rad)으로 계산된 값인데, 이후 기계적 센터를 맞추며 트림 offset이 **0.4672로
//   의도적으로 이동**됐고 servo_min/max는 그대로 남았다. 그 결과 실제 가동각이
//     좌(servo_min) → +0.441 rad       우(servo_max) → **-0.379 rad (잘림)**
//   로 갈렸다. 07-27 bag(run_0727_203040)에서 servo 0.6502가 발행돼 vesc_driver의
//   servo_limit에 잘린 것이 실제로 관측된다. 즉 트림이 아니라 **범위가 낡은 것**이 원인.
//   → 해결: vesc.yaml의 servo_min/max를 현 트림(0.4672) 기준으로 재계산
//     (±0.42 rad → [0.2798, 0.6546]). 컨트롤러 쪽은 아래 두 파라미터로 맞춘다.
//   ⚠️ 두 변경은 **반드시 같이** 가야 한다. 컨트롤러만 0.42로 올리면 vesc_driver가 조용히
//     자르고 컨트롤러는 "꺾었다"고 착각한다(요레이트 피드백·조향 권한 캡이 전부 틀어짐).
constexpr double MAX_STEERING_ANGLE = 0.41;

// 전 구간 최근접 웨이포인트 스캔. 반환 {최단거리, 인덱스}.
// (경로 최초 수신 초기화 / 윈도우 이탈 fail-safe 재탐색 / 로컬 짧은 경로 — 3곳 공용)
std::pair<double, size_t> scan_closest(const std::vector<Waypoint>& wps, double x, double y) {
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    for (size_t i = 0; i < wps.size(); ++i) {
        double dist = std::hypot(wps[i].x - x, wps[i].y - y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    return {min_dist, closest_idx};
}

// 각도를 [-pi, pi]로 정규화.
inline double wrap_pi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// 헤딩 정합 최근접 스캔 (2026-07-28 신설).
//
// scan_closest는 순수 거리 최소화라, 트랙이 스스로에게 가까워지는 구간이나 MCL pose가 깨진
// 직후에 **차량이 향한 방향과 정반대인 웨이포인트**를 최근접으로 고를 수 있다. 그 인덱스로
// 만든 L1 목표점은 차 뒤쪽에 찍히고, sin_eta 부호가 뒤집혀 조향이 역전된다.
// 07-27 실차 bag(run_0727_195937)에서 실제로 관측: closest_idx 86→27→31→89로 튀며
// 경로 접선과 차량 헤딩의 오차가 +146.7°/+151.8°/-173.0°까지 벌어졌고(주행 샘플의 9.5%가
// |오차|>90°), 그 구간에서 조향 명령이 0.2초마다 부호를 뒤집었다.
//
// → 경로 접선(wp.yaw)이 차량 헤딩과 max_heading_err 이내인 후보만 고려한다.
//   조건을 만족하는 후보가 하나도 없으면(차를 반대로 놓은 초기 획득 등) 게이트를 포기하고
//   기존 무제한 스캔 결과를 그대로 쓴다 — 이 함수가 "아무것도 못 찾는" 경우를 만들지 않는다.
// 반환 {최단거리, 인덱스, 게이트가 실제로 적용됐는지}.
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
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
            found = true;
        }
    }
    if (found) return {min_dist, closest_idx, true};
    auto [d, i] = scan_closest(wps, x, y);   // 후보 전무 → 폴백
    return {d, i, false};
}

// start_idx에서 경로를 따라 호 길이 max_dist만큼 전진하며 각 웨이포인트를 방문한다.
// visit(idx, accum_dist)가 false를 반환하면 중단. 닫힌 경로는 한 바퀴에서, 열린 경로는
// 끝점에서 멈춘다. 반환값은 마지막으로 도달한 인덱스.
// (곡률 룩어헤드 사전감속 / L1 목표점 탐색 — 동일한 wrap·종료 가드를 공용화)
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
        if (closed && idx == start_idx) break;  // 한바퀴 방지
    }
    return idx;
}

// 곡률 사전감속(1.5절)용 물리거리 창 평활 곡률 계산.
//
// wp.curvature(kappa_radpm)는 인접점 헤딩차분으로 산출되어, 웨이포인트가 촘촘할수록
// 짧은 구간의 헤딩 노이즈가 증폭돼 개별 포인트 kappa가 실제 지속 곡률보다 훨씬 크게
// 튈 수 있다. 사전감속이 "윈도우 내 최대 단일점 kappa"를 그대로 쓰면 노이즈 스파이크
// 하나로 오프라인 최적화된 프로파일 속도보다 훨씬 낮게 순간 과잉감속된다. 물리거리
// ±window_half_m 창으로 |kappa| 평균을 내면 순간 노이즈는 눌리되 실제 지속 곡률(헤어핀
// 등)은 거의 그대로 반영된다. 원본 wp.curvature 필드는 FF 조향(curvature_ff_blend_,
// 기본 비활성) 등 다른 용도를 위해 그대로 둔다.
//
// ⚠️ 글로벌·로컬 **양쪽 모두**에 적용해야 한다(2026-07-21). 예전엔 글로벌에만 걸고
// 로컬은 "짧은 회피경로라 평활 불필요"라며 원본 kappa를 그대로 썼는데, 팀 플래너의
// /local_waypoints가 실제로는 191점 풀랩이라 그 가정이 깨졌다 — 2026-07-13에 고쳤던
// 단일점 kappa 과잉감속 버그가 로컬 추종 경로로 고스란히 재유입되고 있었다.
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
        // base_max_decel: **명령 속도의 하강 rate limit 전용** [m/s²] (control_loop 8의 램프).
        //   차가 실제로 낼 수 있는 감속도가 아니라 "명령을 얼마나 빨리 떨어뜨릴 수 있나"이므로
        //   낮추면 오히려 감속 명령이 늦게 도달한다 → 높게 유지할 것.
        this->declare_parameter<double>("base_max_decel", 8.0);
        // prebrake_decel: **곡률 사전감속 계산 전용** [m/s²] (control_loop 1.5의 룩어헤드 거리
        //   v²/2a 와 backward-pass v_reach). 2026-07-25 실차 bag에서 base_max_decel과 분리.
        //   ⚠️ 여기엔 차의 **실측 감속 권한**을 넣어야 한다. 07-25 bag 기준 주행 중 실측은
        //   약 -0.4 m/s²(명령 4.00→3.11로 내렸는데 실속 4.03→3.80). VESC 속도모드는 회생제동이
        //   거의 없어 사실상 coast다. 8.0을 쓰면 4 m/s에서 제동거리를 1.0m로 착각해 사전감속
        //   개시가 ~16배 늦어진다(→ 시케인 언더스티어 크래시). 실측 스텝 테스트 전 잠정 1.5.
        this->declare_parameter<double>("prebrake_decel", 1.5);

        // 기동 실패(VESC 센서리스 탈조) 가드 — 아래 control_loop 8-b 참고.
        // ⚠️ "명령이 실측보다 앞서지 못하게" 일반 clamp를 거는 방식은 쓰면 안 된다. VESC 속도
        //    PID가 ERPM 오차에 비례해 전류를 만들어(kp=0.003) 60A를 뽑으려면 20000 ERPM
        //    ≈ 4.7 m/s의 명령 선행이 물리적으로 필요하다 — 선행을 좁히면 가속이 그대로 죽는다.
        //    그래서 "차가 실제로 안 움직이는 동안"에만 발동하는 표적형 가드로 둔다.
        this->declare_parameter<bool>("stall_guard_enable", false);
        this->declare_parameter<double>("stall_speed_threshold", 0.7);
        this->declare_parameter<double>("stall_hold_speed", 1.5);
        this->declare_parameter<double>("stall_hold_delay", 1.0);
        // 런치 킥(자율 정지출발 시 VESC 센서리스 데드존 관통) — 아래 8-c 참고
        this->declare_parameter<bool>("launch_boost_enable", true);
        this->declare_parameter<double>("launch_boost_speed", 3.0);       // 데드존 관통용 펀치 속도 명령 [m/s]
        this->declare_parameter<double>("launch_boost_time", 0.6);        // 관통 실패 시 포기까지 최대 펀치 시간 [s] (< stall_hold_delay)
        this->declare_parameter<double>("launch_exit_speed", 0.8);        // 실측이 이 속도 넘으면 관통 성공 → 킥 종료 [m/s]
        this->declare_parameter<double>("launch_standstill_speed", 0.3);  // 실측이 이 속도 미만이면 정지 판정 → 킥 시작 [m/s]

        // IMU 센서 안전 토글 및 횡슬립 방지
        this->declare_parameter<bool>("use_imu", true);
        // IMU 각속도 단위 보정. 실제 값은 런치가 넘긴다(_control_common.py IMU_ANGULAR_SCALE).
        this->declare_parameter<double>("imu_angular_scale", 1.0);
        // IMU 선형가속도 단위 보정. 실제 값은 런치가 넘긴다(_control_common.py IMU_LINEAR_SCALE).
        this->declare_parameter<double>("imu_linear_scale", 1.0);
        this->declare_parameter<double>("yaw_rate_gain", 0.1);
        this->declare_parameter<double>("max_speed", 12.0);
        this->declare_parameter<double>("min_speed", 2.0);

        // 곡률 룩어헤드 감속 파라미터
        this->declare_parameter<int>("curvature_lookahead_count", 60);
        this->declare_parameter<double>("max_lateral_accel", 6.0);
        // 조향 권한 캡 (2026-07-26 추가) — 곡률 캡이 **그립만** 보던 구멍을 메운다.
        //   정상상태 자전거 모델: δ = L·κ + K_us·a_lat = L·κ + K_us·κ·v²
        //   δ ≤ δ_avail 로 풀면  v ≤ √( (δ_avail − L·κ) / (K_us·κ) )
        //   ⚠️ 이건 그립 캡과 **다른 물리**다. 그립은 "타이어가 그 횡가속을 낼 수 있나",
        //      조향은 "바퀴가 그만큼 꺾일 수 있나"다. 07-26 실차 bag의 κ=1.190(R=0.84m)
        //      헤어핀에서 그립 한계는 2.11 m/s인데 조향 한계는 0.87 m/s — 조향이 먼저 걸린다.
        //      그립만 보면 컨트롤러가 2배 빠르게 진입해 풀락(0.410)에도 안 돌아가고
        //      크로스트랙이 0.11 → 2.07m로 발산했다(실제 이탈).
        //   understeer_gradient=0 이면 이 항 전체 비활성(구 거동).
        this->declare_parameter<double>("understeer_gradient", 0.019); // K_us [rad/(m/s²)] — 07-25 bag 회귀 실측
        // δ_max 중 곡률 추종에 배정할 비율. 나머지는 횡오차 보정·요레이트 피드백·노면 외란용
        // 여유로 남긴다. 1.0으로 두면 정상 곡률에서 이미 풀락이라 보정 여력이 0이 된다.
        this->declare_parameter<double>("steer_authority_ratio", 0.85);
        this->declare_parameter<double>("curvature_ff_blend", 0.0); // 곡률 FF 비활성: 검증된 순수 L1 격리 (원본 MAP 컨트롤러 미보유 항목)
        this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");

        // 안전라인 시프트: 플래너 최적라인이 벽에 과도하게 붙은(클리어런스 부족) 구간에서
        // 차체(길이 0.58m)가 벽을 스치는 충돌을 방지하기 위해, 메시지의 d_left/d_right(트랙 경계까지
        // 거리)를 이용해 해당 웨이포인트를 트랙 중심 쪽으로 밀어 최소 벽 클리어런스 C를 확보한다.
        // C는 차량 반폭+자세/추종 마진. 0이면 원본 라인 그대로(비활성).
        this->declare_parameter<double>("wall_safety_margin", 0.6);

        // 경로 이탈 복구 가드 (2026-07-21). 횡오차가 이 값을 넘으면 L1 목표점을 차량 기준
        // 직선거리로 재선정하고 속도를 recovery_speed로 낮춰 라인 복귀를 우선한다.
        // 0으로 두면 비활성(기존 거동). 기본 1.0m는 트랙 반폭(0.55~0.8m)보다 살짝 크게 잡아
        // 정상 추종 중에는 절대 안 걸리도록 한 값.
        this->declare_parameter<double>("recovery_lat_error", 1.0);
        this->declare_parameter<double>("recovery_speed", 2.0);

        // ── L1 목표점까지의 "실제" 거리로 횡가속을 산출할지 (2026-07-28) ──
        // pure pursuit 법칙은 a_lat = 2·v²·sin(eta)/L_실제 인데, 목표점은 호 길이 기준으로
        // 고르므로 |목표점-차량| != L1_distance다. 07-27 실차 bag 실측 비율(중앙 1.06~1.31,
        // p95 최대 1.72) — 명목 L1_distance를 분모로 쓰면 횡가속 명령이 최대 70% 과대해지고,
        // 경로에서 벗어날수록(= 복귀가 필요한 바로 그 순간) 더 심해져 오버슈트·발진을 만든다.
        // false로 두면 구 거동(명목 L1_distance 분모) — 즉시 롤백용.
        this->declare_parameter<bool>("l1_use_actual_distance", true);

        // ── 좌우 조향 한계 (2026-07-28) ──
        // 서보 트림이 기계 중심에서 밀려 좌우 가동각이 다르다(위 MAX_STEERING_ANGLE 주석 참고).
        // 둘 다 0.41이면 기존 대칭 거동과 100% 동일하다.
        // 실차 현재값 기준 권장: left 0.41 / right 0.379.
        // ⚠️ 곡률 사전감속의 **조향 권한 속도 캡은 둘 중 작은 쪽**을 쓴다 — 큰 쪽을 쓰면
        //    우선회 코너에서 실제보다 8% 더 꺾을 수 있다고 보고 속도를 덜 줄인다(낙관 오류).
        this->declare_parameter<double>("max_steering_left", MAX_STEERING_ANGLE);
        this->declare_parameter<double>("max_steering_right", MAX_STEERING_ANGLE);

        // ── 최근접 인덱스 견고화 (2026-07-28, MCL pose 붕괴 대응) ──
        // closest_idx_max_heading_err: 전역 재탐색에서 경로 접선과 차량 헤딩의 허용 오차 [rad].
        //   0이면 게이트 비활성(구 거동). 기본 1.75rad(100°) — 정상 추종에선 절대 안 걸리고
        //   역주행 웨이포인트(오차 >90°)만 배제하는 값.
        // idx_jump_confirm_dist / idx_jump_confirm_cycles: 한 사이클(20ms)에 물리적으로 가능한
        //   인덱스 이동은 몇 점뿐이다. 그보다 먼 점프는 즉시 받아들이지 않고, 연속으로
        //   confirm_cycles 사이클 동안 같은 점프를 가리킬 때만 채택한다(= pose 1회성 튐 무시).
        //   보류 중에는 pose를 신뢰할 수 없으므로 조향을 직전값으로 홀드하고 감속한다.
        this->declare_parameter<double>("closest_idx_max_heading_err", 1.75);
        this->declare_parameter<double>("idx_jump_confirm_dist", 2.0);
        this->declare_parameter<int>("idx_jump_confirm_cycles", 5);
        this->declare_parameter<double>("pose_suspect_speed", 1.5);

        // ── 자율 미체결 중 속도 명령 와인드업 차단 (bumpless transfer, 2026-07-28) ──
        // 이 노드는 /drive_mode를 모른 채 상시 돌기 때문에, MANUAL/E-stop으로 서 있는 동안에도
        // 속도 램프(last_target_speed_)가 계속 감겨 올라간다. A를 눌러 ackermann_mux가 열리는
        // 순간 그 값이 **계단으로** VESC에 꽂힌다. 07-27 실차 bag 8개 전부에서 확인:
        // 정차 중 /drive_autonomous가 1.50(최대 3.98)까지 감겨 있었고, engage 순간
        // commands/motor/speed가 0 → 6348 ERPM 한 스텝, 모터전류가 매번 60~62A(l_current_max)
        // 포화 → 급발진. (s_pid_ramp_erpms_s 21160으로 VESC 쪽 완충도 사실상 없음)
        // → autonomous가 아닐 때 램프를 실측 속도로 눌러두면 engage가 무충격이 된다.
        // ⚠️ 주행 중(autonomous)에는 아무 일도 하지 않는다 — 07-22에 금지한 일반 lead-clamp
        //    (명령이 실측보다 앞서지 못하게 막는 것)와 다르다. VESC 속도 PID가 전류를 뽑는 데
        //    필요한 명령 선행 여유는 그대로 보존된다.
        // ⚠️ /drive_mode를 한 번도 못 받았거나 끊긴 지 timeout이 지나면 게이트는 **비활성**이 된다.
        //    시뮬(joy_teleop_monitor는 /drive_mode를 발행하지 않음)과 drive_mode_manager 미기동
        //    상황에서 기존 거동이 그대로 유지되도록 하기 위함이다.
        this->declare_parameter<bool>("engage_gate_enable", true);
        this->declare_parameter<std::string>("drive_mode_topic", "/drive_mode");
        this->declare_parameter<std::string>("engaged_mode_value", "autonomous");
        this->declare_parameter<double>("drive_mode_timeout", 1.0);

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
        this->get_parameter("stall_guard_enable", stall_guard_enable_);
        this->get_parameter("stall_speed_threshold", stall_speed_threshold_);
        this->get_parameter("stall_hold_speed", stall_hold_speed_);
        this->get_parameter("stall_hold_delay", stall_hold_delay_);
        this->get_parameter("launch_boost_enable", launch_boost_enable_);
        this->get_parameter("launch_boost_speed", launch_boost_speed_);
        this->get_parameter("launch_boost_time", launch_boost_time_);
        this->get_parameter("launch_exit_speed", launch_exit_speed_);
        this->get_parameter("launch_standstill_speed", launch_standstill_speed_);
        this->get_parameter("base_max_decel", base_max_decel_);
        this->get_parameter("prebrake_decel", prebrake_decel_);
        this->get_parameter("use_imu", use_imu_);
        this->get_parameter("imu_angular_scale", imu_angular_scale_);
        this->get_parameter("imu_linear_scale", imu_linear_scale_);
        this->get_parameter("yaw_rate_gain", yaw_rate_gain_);
        this->get_parameter("max_speed", max_speed_);
        this->get_parameter("min_speed", min_speed_);
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("wall_safety_margin", wall_safety_margin_);
        this->get_parameter("recovery_lat_error", recovery_lat_error_);
        this->get_parameter("recovery_speed", recovery_speed_);

        this->get_parameter("l1_use_actual_distance", l1_use_actual_distance_);
        this->get_parameter("max_steering_left", max_steering_left_);
        this->get_parameter("max_steering_right", max_steering_right_);
        max_steering_left_ = std::max(0.05, std::abs(max_steering_left_));
        max_steering_right_ = std::max(0.05, std::abs(max_steering_right_));
        steer_limit_min_ = std::min(max_steering_left_, max_steering_right_);
        if (std::abs(max_steering_left_ - max_steering_right_) > 1e-6) {
            RCLCPP_WARN(this->get_logger(),
                "조향 한계 좌우 비대칭: 좌 %.3f / 우 %.3f rad — 속도 캡은 작은 쪽(%.3f)을 사용. "
                "근본 해결은 서보 트림 재정렬(링키지)임",
                max_steering_left_, max_steering_right_, steer_limit_min_);
        }
        this->get_parameter("closest_idx_max_heading_err", closest_idx_max_heading_err_);
        this->get_parameter("idx_jump_confirm_dist", idx_jump_confirm_dist_);
        int jump_cycles;
        this->get_parameter("idx_jump_confirm_cycles", jump_cycles);
        idx_jump_confirm_cycles_ = std::max(0, jump_cycles);
        this->get_parameter("pose_suspect_speed", pose_suspect_speed_);

        this->get_parameter("engage_gate_enable", engage_gate_enable_);
        this->get_parameter("drive_mode_topic", drive_mode_topic_);
        this->get_parameter("engaged_mode_value", engaged_mode_value_);
        this->get_parameter("drive_mode_timeout", drive_mode_timeout_);

        int cl_count;
        this->get_parameter("curvature_lookahead_count", cl_count);
        curvature_lookahead_count_ = static_cast<size_t>(cl_count);
        this->get_parameter("max_lateral_accel", max_lateral_accel_);
        this->get_parameter("understeer_gradient", understeer_gradient_);
        this->get_parameter("steer_authority_ratio", steer_authority_ratio_);
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
        // 경로 소실 failsafe: 글로벌·로컬 웨이포인트가 **둘 다 없을 때** GapFollower로 자율주행할지.
        // 기본 false = 안전 정지 명령 발행(control_mppi_node와 동일 거동).
        // ⚠️ true면 플래닝 스택이 안 떠 있거나 죽었을 때 컨트롤러가 라이다 갭만 보고 **차를 스스로
        //    몰기 시작한다**(1.2~3.5 m/s). 2026-07-22 실차에서 플래닝 없이 자율 버튼을 누르자
        //    바퀴가 즉시 우측 풀조향된 것이 이 경로였다. 비상정지 판단은 planning 파트 소관이므로
        //    제어 파트가 경로를 모르는 채 독자 주행할 이유가 없다. 시뮬 갭팔로워 시험용으로만 켤 것.
        gap_follower_failsafe_ = this->declare_parameter<bool>("gap_follower_failsafe", false);
        obstacle_cone_halfangle_ = this->declare_parameter<double>("obstacle_cone_halfangle", 0.14);
        obstacle_trigger_dist_ = this->declare_parameter<double>("obstacle_trigger_dist", 1.5);
        obstacle_margin_ = this->declare_parameter<double>("obstacle_margin", 0.3);
        obstacle_avoid_hold_cycles_ = this->declare_parameter<int>("obstacle_avoid_hold_cycles", 15);

        // 장애물 종방향 감속 (2026-07-23) — opponent_detector의 raw 클러스터(추적 확정 전, 벽
        // 필터 끝난 Frenet 장애물)를 받아, 내 통로 전방에 물체가 있으면 그 앞에서 멈출 수 있는
        // 속도로 target_speed를 캡한다. 첫 바퀴 직선 강가속 중 장애물을 늦게 인지해 회피경로를
        // 못 따라가고 박던 문제 대응 — 감속으로 회피 기동을 실행 가능한 속도까지 낮춰준다.
        // ⚠️ 이건 "비상정지"가 아니라 종방향 soft 감속이다(조향 미개입). 최종 e-stop은 여전히
        //    planning 파트 소관.
        obstacle_brake_enable_ = this->declare_parameter<bool>("obstacle_brake_enable", true);
        obstacle_raw_topic_ = this->declare_parameter<std::string>(
            "obstacle_raw_topic", "/perception/detection/raw_obstacles");
        // v_cap 산출용 감속도. 실제 감속은 램프의 max_decel이 담당하므로 이 값이 낮으면 더
        // 일찍/완만히 제동한다.
        // ⚠️ 2026-07-25 미해결: 이 값도 prebrake_decel과 같은 성격(실측 감속 권한)인데 6.0은
        //    실측(~0.4 m/s²)보다 훨씬 낙관적이다. 즉 장애물 앞 정지거리를 실제의 1/15로 보고
        //    있어 늦게 제동한다. 장애물 회피/추월 거동에 직접 영향이 있어 곡률 사전감속과
        //    분리해 별도 실차 검증 후 조정할 것(무턱대고 낮추면 상대차만 봐도 기어간다).
        obstacle_brake_decel_ = this->declare_parameter<double>("obstacle_brake_decel", 6.0);
        obstacle_stop_gap_ = this->declare_parameter<double>("obstacle_stop_gap", 1.0);       // 장애물 앞 정지 여유[m]
        obstacle_corridor_halfwidth_ = this->declare_parameter<double>("obstacle_corridor_halfwidth", 0.35); // 통로 반폭(차폭/2+여유)[m]
        obstacle_max_range_ = this->declare_parameter<double>("obstacle_max_range", 9.0);      // 이 전방거리[m] 밖 장애물 무시
        obstacle_brake_hold_cycles_ = this->declare_parameter<int>("obstacle_brake_hold_cycles", 10); // 소실 후 캡 유지(채터링 방지)
        obstacle_brake_timeout_ = this->declare_parameter<double>("obstacle_brake_timeout", 0.3); // raw 토픽 신선도[s]
        // 로컬 회피경로 추종 중엔 캡을 0(완전정지)이 아니라 이 하한에서 바닥 처리 — planner가 커밋한
        // 회피 라인을 신뢰해 정지 대신 관통. 글로벌 대기(회피경로 없음)에선 하한 없이 정지까지 허용.
        obstacle_avoid_min_speed_ = this->declare_parameter<double>("obstacle_avoid_min_speed", 1.5);

        auto qos_local = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // 로컬 퍼블리셔에 맞춰 volatile
        local_path_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/local_waypoints", qos_local,
            std::bind(&ControlMapNode::local_path_callback, this, std::placeholders::_1));
        local_last_recv_time_ = this->now(); // 노드 클럭 타입으로 초기화(비교 시 clock mismatch 방지)
        RCLCPP_INFO(this->get_logger(), "로컬 경로 토픽(/local_waypoints) 구독 설정 완료.");

        // 3. 알고리즘 인스턴스 초기화 및 통신 채널 설정
        // 갭팔로워는 좌우 대칭 한계 하나만 받으므로 작은 쪽(= 확실히 낼 수 있는 각)을 준다.
        gap_follower_ = std::make_unique<GapFollower>(180.0, 0.38, 3.0, steer_limit_min_);
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

        if (obstacle_brake_enable_) {
            obstacle_sub_ = this->create_subscription<f110_msgs::msg::ObstacleArray>(
                obstacle_raw_topic_, 10,
                std::bind(&ControlMapNode::obstacle_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),
                        "장애물 감속 활성 — raw 장애물 토픽(%s) 구독", obstacle_raw_topic_.c_str());
        }
        obstacle_last_recv_time_ = this->now(); // 노드 클럭 타입으로 초기화(clock mismatch 방지)

        // 자율 체결 상태 구독 (실차 f1tenth_stack drive_mode_manager가 "estop"/"manual"/
        // "autonomous"를 발행). 시뮬엔 발행자가 없어 아래 게이트가 자동으로 비활성이 된다.
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

        // L1 look-ahead 목표점 디버그 시각화 (RViz MarkerArray) — 제어 경로와 무관한 표시 전용
        l1_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/debug/l1_lookahead", 10);

        // 실시간 50Hz (20ms) 주기 타이머 가동
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControlMapNode::control_loop, this));

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "RoboRacer L1 Guidance & Steer LUT 제어 노드가 시작되었습니다.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        // 웨이포인트와 pose가 같은 프레임(보통 map)에서 직접 뺄셈되므로, L1 마커도 이 프레임에 그린다.
        if (!msg->header.frame_id.empty()) odom_frame_ = msg->header.frame_id;
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        current_speed_ = msg->twist.twist.linear.x;
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        // use_imu=false는 "IMU를 신뢰하지 않는다"는 뜻이므로 IMU에서 파생되는 값은 전부
        // 쓰지 않는다. 예전엔 아래 가속도 버퍼만 이 게이트 밖에 있어서, IMU가 이상해
        // use_imu를 꺼도 종가속(조향 스케일러) 경로는 계속 그 IMU를 쓰는 모순이 있었다.
        // use_imu_는 생성자에서 1회만 읽히므로, false면 acc_now_는 0으로 초기화된 상태를
        // 그대로 유지 → acc_mean=0 → 스케일러 중립(1.0)으로 안전하게 떨어진다.
        if (!use_imu_) return;

        // VESC 자이로가 deg/s로 발행하는 것이 실차에서 확인되어(2026-07-19) 여기서 rad/s로
        // 환산한다. 보정 안 하면 실측 요레이트가 57.3배 → 카운터스티어가 즉시 반대로 포화.
        // 값의 근거·재확인 절차는 launch/_control_common.py의 IMU_ANGULAR_SCALE 주석 참고.
        stability_controller_->update_imu(msg->orientation,
                                          msg->angular_velocity.z * imu_angular_scale_);

        // 롤 인지 ESC가 실제로 걸리는 구간이 있는지 계측(3(a)). 1/10 스케일 차량은 서스펜션이
        // 단단해 max_roll_limit(0.15rad≈8.6도)까지 기울지 않을 가능성이 커, 그러면 ESC가
        // 사실상 상시 비활성이다. 주행 후 아래 로그의 최댓값으로 임계치 타당성을 판단한다.
        max_abs_roll_seen_ = std::max(max_abs_roll_seen_,
                                      std::abs(stability_controller_->filtered_roll()));

        // 가속도 rolling buffer 업데이트 (longitudinal acceleration)
        // VESC의 장착 방향 회전(90도)에 맞춰 -linear_acceleration.y 값을 적용.
        // ⚠️ VESC 가속도계는 m/s²가 아니라 g로 발행한다(2026-07-19 소스 확인) — 자이로의
        // deg/s와 같은 계열의 비-SI 발행이라 여기서 환산한다. 보정 전에는 acc_mean이 실제의
        // 1/9.8이라 아래 ±1.0 임계값에 도달하지 못해 조향 스케일러가 계속 중립이었다.
        std::rotate(acc_now_.rbegin(), acc_now_.rbegin() + 1, acc_now_.rend());
        acc_now_[0] = -msg->linear_acceleration.y * imu_linear_scale_;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        latest_scan_ = msg;
    }

    void drive_mode_callback(const std_msgs::msg::String::ConstSharedPtr msg) {
        bool engaged = (msg->data == engaged_mode_value_);
        if (engaged != is_engaged_) {
            RCLCPP_INFO(this->get_logger(),
                        "자율 체결 상태 변경: %s → %s (램프 %s)",
                        is_engaged_ ? "체결" : "미체결", engaged ? "체결" : "미체결",
                        engaged ? "진행" : "실측 속도로 고정");
        }
        is_engaged_ = engaged;
        drive_mode_last_recv_time_ = this->now();
        drive_mode_seen_ = true;
    }

    // engage 게이트가 지금 실제로 작동 중인가.
    // /drive_mode를 한 번도 못 받았거나 timeout 넘게 끊겼으면 false(= 구 거동 유지).
    bool engage_gate_active() const {
        if (!engage_gate_enable_ || !drive_mode_seen_) return false;
        return (this->now() - drive_mode_last_recv_time_).seconds() < drive_mode_timeout_;
    }

    // opponent_detector의 raw 장애물(추적 확정 전, 벽 제거+Frenet 투영 완료)을 그대로 보관.
    void obstacle_callback(const f110_msgs::msg::ObstacleArray::ConstSharedPtr msg) {
        latest_obstacles_ = msg;
        obstacle_last_recv_time_ = this->now();
    }

    // 에고를 글로벌 raceline에 투영해 Frenet (s, d)를 얻는다. 장애물이 글로벌 raceline 프레임의
    // (s,d)라 감속 판정도 반드시 같은 프레임에서 해야 한다 — 로컬 회피경로를 추종 중이어도
    // 에고 기준은 글로벌로 고정한다. 직전 인덱스 주변 윈도우 스캔(+이탈 시 전역 재탐색).
    // 반환 true = 유효 투영. ego_s/ego_d에 결과 기록.
    bool project_ego_to_global(double& ego_s, double& ego_d) {
        const size_t n = waypoints_.size();
        if (n == 0) return false;
        if (last_global_proj_idx_ >= n) last_global_proj_idx_ = 0;

        const double spacing = std::max(0.01, avg_waypoint_spacing_);
        int win = std::max(8, static_cast<int>(std::ceil(3.0 / spacing)));
        win = std::min(win, static_cast<int>(n / 2));

        double min_dist = std::numeric_limits<double>::max();
        size_t g = last_global_proj_idx_;
        for (int i = -win; i <= win; ++i) {
            size_t idx = (last_global_proj_idx_ + i + n) % n;
            double dist = std::hypot(waypoints_[idx].x - current_x_, waypoints_[idx].y - current_y_);
            if (dist < min_dist) { min_dist = dist; g = idx; }
        }
        if (min_dist > 2.5) {
            std::tie(min_dist, g) = scan_closest(waypoints_, current_x_, current_y_);
        }
        last_global_proj_idx_ = g;

        ego_s = waypoints_[g].s;
        // 부호 있는 횡거리 d: 진행방향 좌측 법선(-sin ψ, cos ψ)에 (에고−wp)를 투영.
        const double nx = -std::sin(waypoints_[g].yaw);
        const double ny =  std::cos(waypoints_[g].yaw);
        ego_d = (current_x_ - waypoints_[g].x) * nx + (current_y_ - waypoints_[g].y) * ny;
        return true;
    }

    // 통로 전방 장애물에 대한 속도 상한을 계산한다. 반환값은 target_speed에 min으로 씌울 캡
    // (장애물 없음/신선도 만료/비활성이면 +inf). latch-on 즉시 / release는 hold로 느리게(채터링 방지).
    // following_local=true(로컬 회피경로 추종 중)면 캡을 obstacle_avoid_min_speed에서 바닥 처리해
    // 정지 대신 회피 관통(planner 라인 신뢰). 글로벌 대기 중이면 하한 없이 정지까지 허용.
    double compute_obstacle_speed_limit(bool following_local) {
        const double kInf = std::numeric_limits<double>::max();
        if (!obstacle_brake_enable_) return kInf;

        // 신선도: raw 토픽이 끊기면 마지막 홀드만 소진하고 해제.
        bool fresh = latest_obstacles_ &&
                     (this->now() - obstacle_last_recv_time_).seconds() < obstacle_brake_timeout_;

        double cap = kInf;
        if (fresh && track_length_s_ > 1e-3) {
            double ego_s = 0.0, ego_d = 0.0;
            if (project_ego_to_global(ego_s, ego_d)) {
                const double half_len = 0.5 * track_length_s_;
                const double lane_lo = ego_d - obstacle_corridor_halfwidth_;
                const double lane_hi = ego_d + obstacle_corridor_halfwidth_;
                for (const auto& ob : latest_obstacles_->obstacles) {
                    if (!ob.is_visible) continue;
                    // 횡방향 통로 겹침: 장애물 [d_right, d_left] 구간이 내 통로 밴드와 겹치나.
                    double od_lo = std::min(ob.d_right, ob.d_left);
                    double od_hi = std::max(ob.d_right, ob.d_left);
                    if (od_hi < lane_lo || od_lo > lane_hi) continue;
                    // 전방거리(s-wrap): 장애물 근단(s_start)까지.
                    double ds = ob.s_start - ego_s;
                    while (ds > half_len)  ds -= track_length_s_;
                    while (ds < -half_len) ds += track_length_s_;
                    if (ds <= 0.0 || ds > obstacle_max_range_) continue; // 뒤/너무 먼 것 제외
                    // 이 앞에서 stop_gap 두고 멈출 수 있는 속도. ds<=gap이면 0(정지).
                    double free = ds - obstacle_stop_gap_;
                    double v_cap = (free <= 0.0) ? 0.0
                                                 : std::sqrt(2.0 * obstacle_brake_decel_ * free);
                    if (v_cap < cap) cap = v_cap;
                }
            }
        }

        // latch-on 즉시 / release 느리게: 이번 사이클에 유효 캡이 잡히면 홀드 재충전,
        // 아니면 홀드가 남아있는 동안 직전 캡을 유지한다.
        if (cap < kInf) {
            obstacle_held_cap_ = cap;
            obstacle_brake_hold_counter_ = obstacle_brake_hold_cycles_;
        } else if (obstacle_brake_hold_counter_ > 0) {
            obstacle_brake_hold_counter_--;
            cap = obstacle_held_cap_;
        }

        // 회피 관통 속도 하한: 로컬 회피경로를 따라가는 중이면 캡을 완전정지(0)까지 내리지 않고
        // obstacle_avoid_min_speed에서 바닥 처리한다. 차가 옆으로 스윙아웃하는 동안 서버리지 않고
        // 최소 회피속도로 관통 → 부드러운 회피. 실제 벽처럼 정말 막혔으면 planner가 라인을
        // 안 주거나 상류 e-stop이 판단한다. (장애물 없음(cap=inf)엔 영향 없음)
        if (following_local && cap < kInf) {
            cap = std::max(cap, obstacle_avoid_min_speed_);
        }
        return cap;
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
            interp_wp.s = wp.s_m; // Frenet 호길이(장애물 감속의 에고 s 기준)

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
            last_target_idx_ = scan_closest(waypoints_, current_x_, current_y_).second;
            waypoints_initialized_ = true;
            // 추적기를 새로 잡았으므로 점프 게이트의 비교 기준도 무효화한다 — 안 하면 다음
            // 사이클의 정상적인 재획득이 "점프"로 오판돼 불필요한 홀드가 걸린다.
            global_idx_valid_ = false;
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

        // 장애물(s,d) 감속의 s-wrap 기준 트랙 총길이. s_m은 호길이라 Euclidean 총둘레와
        // 사실상 동일 — 이미 계산한 값을 재사용한다(닫힌 루프 전제).
        track_length_s_ = total_path_length;

        // 곡률 창 평활 (근거·주의사항은 smooth_curvature 정의부 주석 참고). 글로벌은 닫힌 루프.
        smooth_curvature(waypoints_, /*closed=*/true);

        RCLCPP_INFO(this->get_logger(), "🔄 플래닝 팀의 글로벌 경로 수신 완료! 웨이포인트 개수: %zu, 초기 인덱스: %zu", waypoints_.size(), last_target_idx_);
    }

    // 로컬 경로 콜백: 상류 플래너의 전방 구간을 그대로 저장.
    // wall_safety_margin 시프트는 적용하지 않는다(회피/추월 경로의 원본 기하 유지, 참조 컨트롤러와 동일).
    //
    // ⚠️ 로컬 경로가 "짧은 열린 구간"이라고 가정하지 않는다(2026-07-21). 팀 플래너 구성에 따라
    // 글로벌과 같은 풀랩(닫힌 루프)이 그대로 실려 올 수 있고, 그걸 열린 경로로 취급하면
    // 배열 끝에서 룩어헤드가 끊긴다. 소스가 아니라 **기하로 판정**한다.
    void local_path_callback(const f110_msgs::msg::WpntArray::ConstSharedPtr msg) {
        if (msg->wpnts.empty()) {
            local_waypoints_.clear(); // 빈 로컬 → 다음 사이클에 글로벌로 폴백
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
            w.smoothed_curvature = wp.kappa_radpm; // 아래 smooth_curvature가 덮어씀(임시값)
            w.yaw = wp.psi_rad;
            w.s = wp.s_m; // Frenet 호길이(로컬도 글로벌 raceline s로 발행됨)
            local_waypoints_.push_back(w);
        }

        // 닫힘 판정: 끝점→시작점 간격이 평균 웨이포인트 간격의 2배 이내면 닫힌 루프로 본다.
        // (한 바퀴를 다 담은 경로는 끝점이 시작점 바로 뒤에 오고, 짧은 회피 세그먼트는
        //  양 끝이 경로 길이만큼 떨어져 있어 확실히 구분된다)
        const size_t n = local_waypoints_.size();
        local_is_closed_ = false;
        if (n >= 8) {
            double total_len = 0.0;
            for (size_t i = 1; i < n; ++i) {
                total_len += std::hypot(local_waypoints_[i].x - local_waypoints_[i - 1].x,
                                        local_waypoints_[i].y - local_waypoints_[i - 1].y);
            }
            const double avg_spacing = total_len / static_cast<double>(n - 1);
            const double closing_gap = std::hypot(local_waypoints_[n - 1].x - local_waypoints_[0].x,
                                                  local_waypoints_[n - 1].y - local_waypoints_[0].y);
            local_is_closed_ = (avg_spacing > 1e-6) && (closing_gap <= 2.0 * avg_spacing);
        }

        // 곡률 창 평활 — 글로벌과 동일하게 적용한다(2026-07-21).
        // 여기를 빼두면 단일점 kappa 노이즈로 v_cap=sqrt(a_lat/kappa)가 튀어 순간 과잉감속한다.
        smooth_curvature(local_waypoints_, local_is_closed_);

        // 배열이 교체되면 로컬 인덱스 추적기를 초기화(다음 사이클에서 전역 재탐색으로 복구)
        // ⚠️ 추적기를 0으로 되돌릴 땐 점프 게이트의 비교 기준도 같이 무효화해야 한다
        // (2026-07-28). 안 하면 다음 사이클에 wps[0] → 진짜 최근접점(팀 플래너의 191점 풀랩에선
        // 최대 17m)이 "pose 튐"으로 오판돼 매 재발행마다 조향 홀드+감속이 걸린다.
        if (n != prev_size) {
            last_local_idx_ = 0;
            local_idx_valid_ = false;
        }

        if (local_is_closed_ != last_logged_local_closed_) {
            RCLCPP_INFO(this->get_logger(), "로컬 경로 기하: %s (웨이포인트 %zu개)",
                        local_is_closed_ ? "닫힌 루프(wrap 적용)" : "열린 구간", n);
            last_logged_local_closed_ = local_is_closed_;
        }

        local_last_recv_time_ = this->now();
    }

    // 경로를 모를 때의 안전 정지 명령. 발행을 아예 멈추지 않고 명시적 0을 보내는 이유는,
    // 침묵하면 하류(ackermann_mux→VESC)가 **직전 명령을 그대로 유지**해 타력주행이 되기 때문이다.
    // (control_mppi_node의 publish_stop과 동일 규약)
    void publish_safe_stop() {
        last_steering_angle_ = 0.0;
        last_target_speed_ = 0.0;
        stall_time_ = 0.0;  // 정지 명령 중엔 탈조 판정을 누적하지 않는다(명령이 0이라 애초에 무의미)
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = 0.0;
        drive_msg.drive.speed = 0.0;
        drive_msg.drive.acceleration = 0.0;
        drive_pub_->publish(drive_msg);
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

        double steer_ratio = std::abs(final_steering_angle) / steer_limit_min_;
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
            // 글로벌·로컬 둘 다 없음(초기/플래닝 미기동/전체 소실).
            // 기본은 **안전 정지** — 경로를 모르는 상태에서 제어 파트가 독자 주행하지 않는다.
            // gap_follower_failsafe:=true일 때만 순수 Lidar Gap Follower 주행(위 선언부 경고 참고).
            if (gap_follower_failsafe_) {
                publish_gap_follower(dt);
            } else {
                publish_safe_stop();
            }
            return;
        }

        // 활성 경로 선택: 신선한 로컬 우선, 없으면 글로벌(닫힌 루프)
        const std::vector<Waypoint>& wps = local_fresh ? local_waypoints_ : waypoints_;

        // ⚠️ "경로 소스"와 "경로 기하"를 분리한다(2026-07-21).
        //   following_local : 로컬 경로를 추종 중인가 (상류 회피 신뢰 여부 — 장애물 폴백 게이트용)
        //   path_closed     : 그 경로가 실제로 닫힌 루프인가 (wrap 여부 — walk_forward/윈도우 탐색용)
        // 예전엔 이 둘을 `closed = !local_fresh` 하나로 겸했는데, 팀 플래너의 /local_waypoints가
        // 짧은 회피 세그먼트가 아니라 **글로벌과 같은 191점 풀랩**이라 매 랩 배열 끝에서
        // walk_forward가 끊겼다(룩어헤드 truncation). 시뮬 로그로 확인된 증상:
        //   로컬 추종 시 `Idx: 185→190, 187→190, 188→190`(끝점 고정)
        //   글로벌 추종 시 `Idx: 185→2, 189→5`(정상 wrap)
        // 스타트/피니시 직후가 마진 0인 오프닝 헤어핀이라 곡률 사전감속 창이 거기서 붕괴했다.
        // → 소스가 아니라 기하로 판정한다(local_path_callback의 local_is_closed_ 참고).
        const bool following_local = local_fresh;
        const bool path_closed = local_fresh ? local_is_closed_ : true;

        // 1. 차량 위치 기준 최단 거리 인덱스 (closest_idx) 스캔
        size_t n = wps.size();
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;

        // 인덱스 추적기는 경로 소스별로 따로 둔다 — 로컬/글로벌은 배열 길이·인덱싱이 다를 수
        // 있어 하나를 공유하면 소스가 바뀔 때 엉뚱한 인덱스에서 탐색을 시작한다.
        size_t& idx_tracker = following_local ? last_local_idx_ : last_target_idx_;
        // 점프 게이트의 "비교 기준이 유효한가"도 소스별로 따로 둔다 — 소스가 바뀐 첫 사이클에
        // 남의 추적기와 비교하면 정상 전환이 매번 점프로 오판된다.
        bool& idx_valid = following_local ? local_idx_valid_ : global_idx_valid_;
        if (idx_tracker >= n) { idx_tracker = 0; idx_valid = false; }   // 배열이 교체되어 범위를 벗어난 경우

        if (path_closed) {
            // 닫힌 루프: 직전 인덱스 주변 윈도우 스캔 + 이탈 시 전역 재탐색
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

            closest_idx = idx_tracker;
            for (int i = -back_count; i <= fwd_count; ++i) {
                size_t idx = (idx_tracker + i + n) % n;
                double dx = wps[idx].x - current_x_;
                double dy = wps[idx].y - current_y_;
                double dist = std::hypot(dx, dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_idx = idx;
                }
            }
            // Fail-safe recovery: 경로와 2.5m 초과하여 멀어지면 전체 탐색 (U턴 옆차선 점프 방지)
            // ⚠️ 이 전역 재탐색이 pose 붕괴 시 인덱스 텔레포트의 통로다 — 헤딩 게이트를 건다.
            if (min_dist > 2.5) {
                auto [d, i, gated] = scan_closest_heading_gated(
                    wps, current_x_, current_y_, current_yaw_, closest_idx_max_heading_err_);
                min_dist = d; closest_idx = i;
                if (!gated && closest_idx_max_heading_err_ > 0.0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "전역 재탐색: 헤딩 정합(±%.0f°) 후보가 없어 게이트 없이 선택 — "
                        "차량이 경로 반대 방향이거나 pose가 깨졌을 수 있음",
                        closest_idx_max_heading_err_ * 180.0 / PI);
                }
            }
        } else {
            // 열린 구간(짧은 회피경로): 전체 최근접 스캔(~50점이라 저렴, wrap 인덱스 미사용)
            std::tie(min_dist, closest_idx, std::ignore) = scan_closest_heading_gated(
                wps, current_x_, current_y_, current_yaw_, closest_idx_max_heading_err_);
        }

        // 1-b. 인덱스 점프 확인 게이트 (2026-07-28, MCL pose 붕괴 대응)
        // 한 사이클(20ms)에 물리적으로 가능한 경로 이동은 최대 v*dt(= 8m/s에서 16cm)다.
        // 그보다 훨씬 먼 점프는 pose가 튄 것이므로 즉시 채택하지 않고, 연속 confirm_cycles
        // 동안 같은 점프가 유지될 때만 받아들인다. 보류 중에는 직전 인덱스를 그대로 쓰고
        // pose_suspect_ 플래그를 세워 조향 홀드 + 감속으로 넘어간다.
        // ⚠️ 최초 획득(경로 수신 직후)과 배열 교체 때는 비교 대상이 없으므로 게이트를 건너뛴다.
        pose_suspect_ = false;
        if (idx_jump_confirm_cycles_ > 0 && idx_valid && closest_idx != idx_tracker &&
            idx_tracker < n) {
            double jump = std::hypot(wps[closest_idx].x - wps[idx_tracker].x,
                                     wps[closest_idx].y - wps[idx_tracker].y);
            if (jump > idx_jump_confirm_dist_) {
                if (++idx_jump_count_ < idx_jump_confirm_cycles_) {
                    // 보류: 직전 인덱스 유지 (min_dist도 그 기준으로 다시 잰다)
                    closest_idx = idx_tracker;
                    min_dist = std::hypot(wps[closest_idx].x - current_x_,
                                          wps[closest_idx].y - current_y_);
                    pose_suspect_ = true;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "인덱스 점프 %.2fm 보류(%d/%d) — pose 튐 의심, 조향 홀드+감속",
                        jump, idx_jump_count_, idx_jump_confirm_cycles_);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "인덱스 점프 %.2fm 확정 채택(%d 사이클 연속) — 경로 재획득",
                        jump, idx_jump_count_);
                    idx_jump_count_ = 0;
                }
            } else {
                idx_jump_count_ = 0;
            }
        } else {
            idx_jump_count_ = 0;
        }

        // ⚠️ 추적기 갱신은 두 분기 공통이어야 한다(2026-07-21). 예전엔 닫힌 분기에서만
        // 되썼기 때문에, 로컬 추종 중에는 last_target_idx_가 0에 고정된 채 얼어붙었다
        // (로그의 "초기 인덱스: 0"이 매 재발행마다 0으로 찍히던 정체). 그 상태로 로컬→글로벌
        // 폴백이 일어나면 stale 인덱스 주변에서 윈도우 탐색을 시작해 2.5m failsafe에만
        // 의존해 복구했다 — 시작/피니시처럼 트랙이 스스로에게 가까운 구간에선 failsafe가
        // 안 걸려 엉뚱한 인덱스에 잠길 수 있다.
        idx_tracker = closest_idx;
        idx_valid = true;
        double lateral_error = min_dist;

        // 1.5 곡률 룩어헤드 사전 감속 (Curvature Lookahead Pre-deceleration)
        // 1.5 곡률 룩어헤드 사전 감속 (Curvature Lookahead Pre-deceleration)
        // 속도비례 룩어헤드: 현재속도 제동거리(v^2/2a)만큼 전방 곡률을 미리 스캔.
        // 추가: 글로벌 경로 형상 Adaptive 룩어헤드 (전방 12m 스캔하여 시케인/헤어핀 등 고곡률 코너 감지 시 룩어헤드 확장)
        double brake_dist = (current_speed_ * current_speed_) / (2.0 * std::max(0.1, prebrake_decel_));
        double min_lookahead_dist = static_cast<double>(curvature_lookahead_count_) * 0.1; // 기존 고정값을 하한으로 유지

        // 경로 형상 Adaptive 룩어헤드: 전방 12m 내 고곡률 피크 지점 감지
        double adaptive_lookahead_dist = min_lookahead_dist;
        const double path_scan_horizon = 12.0;
        walk_forward(wps, closest_idx, path_scan_horizon, path_closed,
                     [&](size_t i, double accum) {
            double k_i = std::abs(wps[i].smoothed_curvature);
            if (k_i > 0.4) { // 고곡률 코너 감지 시 코너 피크 지점까지 룩어헤드 확장
                adaptive_lookahead_dist = std::max(adaptive_lookahead_dist, accum + 1.0);
            }
            return true;
        });

        double curv_lookahead_dist = std::max({min_lookahead_dist, brake_dist, adaptive_lookahead_dist});

        // 프로파일 신뢰형 사전감속 (backward-pass): 오프라인 최적화된 프로파일 vx_mps는 이미
        // 각 지점의 최적 속도(코너 감속 램프 포함)를 담고 있다는 전제로, 전방 각 지점의 그립
        // 제한 목표속도 v_cap[i] = min(vx_profile[i], √(a_lat/κ_smoothed[i]))까지
        // prebrake_decel로 감속 가능한 현재 최대 속도 v_reach = √(v_cap[i]² + 2·a_decel·d_i)의
        // 최소값을 사전감속 캡으로 쓴다(accum=0인 현재 위치 항이 순간 그립 클램프 역할도 겸함).
        // 직선·완만구간은 κ≈0 → v_cap=프로파일이라 안 눌리고, 코너는 제동거리만큼 앞에서부터
        // 정확히 그립속도로 선제동된다. (구 방식인 "창 내 최대 κ로 √(a_lat/κ) 블랭킷 재캡"은
        // 프로파일보다 낮은 속도로 전 구간을 과잉감속시켜 폐기 — 상세 비교는 CLAUDE.md 참고.)
        double curvature_speed_limit = std::numeric_limits<double>::max();
        // 진단용: 조향 권한이 그립보다 먼저 걸린 최악 지점(튜닝 로그에만 쓰임)
        double steer_bound_k = 0.0, steer_bound_v = 0.0;
        walk_forward(wps, closest_idx, curv_lookahead_dist, path_closed,
                     [&](size_t i, double accum) {
            double v_cap_i = wps[i].speed;
            double k_i = std::abs(wps[i].smoothed_curvature);
            if (k_i > 0.01) {
                // (a) 그립 한계: a_lat = κ·v² ≤ a_lat_max
                double v_grip = std::sqrt(max_lateral_accel_ / k_i);
                v_cap_i = std::min(v_cap_i, v_grip);

                // (b) 조향 권한 한계 (2026-07-26) — δ = L·κ + K_us·κ·v² ≤ δ_avail
                if (understeer_gradient_ > 1e-6) {
                    // ⚠️ 좌우 중 **작은** 한계를 쓴다 — 이 지점의 곡률이 좌회전인지 우회전인지는
                    // 알 수 있지만(kappa 부호), 캡은 코너 진입 전에 미리 걸어야 하고 경로가
                    // S자면 양쪽이 다 온다. 큰 쪽을 쓰면 우선회에서 8% 낙관이 된다.
                    double steer_budget = steer_authority_ratio_ * steer_limit_min_
                                          - wheelbase_ * k_i;
                    // budget ≤ 0 → 기구학적으로도 못 도는 곡률(R < L/tanδ_avail).
                    // 여기서 0으로 두면 아래 backward-pass가 "가능한 한 늦게까지 감속"으로
                    // 자연히 처리하고, 최종 min_speed_ 하한이 정지는 막는다.
                    double v_steer = (steer_budget > 0.0)
                        ? std::sqrt(steer_budget / (understeer_gradient_ * k_i))
                        : 0.0;
                    if (v_steer < v_cap_i) {
                        v_cap_i = v_steer;
                        if (k_i > steer_bound_k) {   // 창 안에서 가장 조인 곡률만 기록
                            steer_bound_k = k_i;
                            steer_bound_v = v_steer;
                        }
                    }
                }
            }
            double v_reach = std::sqrt(v_cap_i * v_cap_i + 2.0 * prebrake_decel_ * accum);
            if (v_reach < curvature_speed_limit) {
                curvature_speed_limit = v_reach;
            }
            return true;
        });
        // ⚠️ min_speed_ 하한이 조향 캡보다 높으면 캡이 무력화된다. 07-26 실차의 κ=1.190은
        //    조향 한계가 0.87 m/s라 min_speed_=2.5로는 여전히 못 돈다 — 고곡률 트랙에선
        //    min_speed를 함께 낮춰야 이 캡이 실제로 일을 한다(런치 인자).
        if (steer_bound_k > 0.0 && steer_bound_v < min_speed_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "조향 권한 한계 %.2f m/s (κ=%.3f, R=%.2fm) < min_speed %.2f — 하한이 캡을 무력화 중",
                steer_bound_v, steer_bound_k, 1.0 / steer_bound_k, min_speed_);
        }
        curvature_speed_limit = std::max(min_speed_, curvature_speed_limit);

        // 2. L1 Guidance Distance 계산 및 L1 Point 스캔
        // 코너(시케인/헤어핀) 구간에서는 L1 룩어헤드를 적절히 축소하여 조향 반응성을 높이고 외벽 침범 억제
        double curv_closest = std::abs(wps[closest_idx].smoothed_curvature);
        double L1_distance = l1_gain_ + current_speed_ * l1_distance_;
        if (curv_closest > 0.3) {
            double curv_factor = std::min(1.0, (curv_closest - 0.3) / 1.0);
            L1_distance *= (1.0 - 0.25 * curv_factor); // 고곡률 진입 시 L1 최대 25% 축소
        }
        double lower_bound = std::max(t_clip_min_, std::sqrt(2.0) * lateral_error);
        L1_distance = std::max(lower_bound, std::min(L1_distance, t_clip_max_));

        // closest_idx로부터 물리적으로 L1_distance만큼 전방에 위치한 목표 인덱스 스캔
        size_t idx_a = walk_forward(wps, closest_idx, L1_distance, path_closed,
                                    [](size_t, double) { return true; });

        // 2.5 경로 이탈 복구 가드 (2026-07-21 추가)
        // walk_forward는 closest_idx로부터의 **호 길이**로 목표점을 고르므로, 차량이 경로에서
        // 크게 벗어나 있으면 그 목표점의 **차량 기준 직선거리**가 L1_distance보다 훨씬 짧아진다.
        // 그러면 pure-pursuit 특성상 요구 회전반경이 차량 최소 선회반경보다 작아져 목표점을
        // 따라잡지 못하고 그 주위를 계속 도는 limit cycle에 빠진다(시뮬에서 헤딩이 360° 연속
        // 회전하며 복귀 실패하는 것으로 재현됨 — 접촉/위치추정 점프/회피 기동 직후 실차에서도
        // 동일 조건이 만들어진다).
        // → 목표점을 "차량으로부터 직선거리 L1_distance 이상"이 될 때까지 전진시켜 기하를
        //   복원하고, 동시에 속도를 낮춰 선회반경을 줄인다. 임계값 미만(정상 추종)에서는
        //   아무것도 하지 않으므로 검증된 기존 거동은 그대로 유지된다.
        bool recovery_active = false;
        if (recovery_lat_error_ > 0.0 && lateral_error > recovery_lat_error_) {
            recovery_active = true;
            size_t idx = idx_a;
            for (size_t k = 0; k < n; ++k) {
                if (std::hypot(wps[idx].x - current_x_, wps[idx].y - current_y_) >= L1_distance) break;
                size_t next = idx + 1;
                if (next >= n) {
                    if (!path_closed) break;   // 열린 경로: 끝점에서 종료
                    next = 0;
                }
                if (next == closest_idx) break; // 닫힌 경로 한바퀴 방지
                idx = next;
            }
            idx_a = idx;
        }

        double L1_x = wps[idx_a].x;
        double L1_y = wps[idx_a].y;

        // 디버그: L1 look-ahead 목표점을 RViz에 시각화 (표시 전용, 제어에 영향 없음)
        publish_l1_marker(L1_x, L1_y);

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
        // 로컬 추종 중이면 상류 회피를 신뢰하고 이 폴백을 끈다.
        // ⚠️ 여기는 경로 "기하"(path_closed)가 아니라 "소스"(following_local)로 판정해야 한다 —
        // 로컬 경로가 닫힌 루프여도 상류 회피를 신뢰하는 건 동일하다(2026-07-21 분리).
        if (obstacle_avoid_enable_ && !following_local) {
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
        double speed_la_for_lu = wps[find_lookahead_wp_idx(wps, path_closed, closest_idx, speed_lookahead_for_steering_)].speed;

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

            // ⚠️ 분모는 **목표점까지의 실제 직선거리**여야 한다 (2026-07-28 수정).
            // pure pursuit 법칙은 a_lat = 2·v²·sin(eta)/L_실제 인데, 목표점은 walk_forward가
            // 경로 **호 길이** L1_distance만큼 전진해 고르므로 |목표점-차량| != L1_distance다.
            // 차량이 경로 뒤/옆에 있으면 실제 거리가 더 길어진다 — 07-27 실차 bag 실측 비율
            // (|목표점-차량| / L1_distance)은 중앙 1.06~1.31, p95 최대 1.72였다.
            // 명목값을 분모로 쓰면 그만큼 횡가속 명령이 과대해지고(최대 +70%), 경로에서
            // 벗어날수록 = 복귀가 필요한 바로 그 순간에 더 심해져 오버슈트·조향 발진을 만든다.
            // 하한은 t_clip_min_ — 목표점이 차량에 붙어버린 경우(L1_norm→0) 발산 방지.
            double l1_denom = L1_distance;
            if (l1_use_actual_distance_) {
                l1_denom = std::max(L1_norm, t_clip_min_);
            }
            lat_acc = 2.0 * std::pow(speed_for_lu, 2) / l1_denom * sin_eta;
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

        // 3.9) pose 튐 보류 중 조향 홀드 (2026-07-28, 1-b 게이트와 한 쌍)
        // 인덱스 점프를 보류한 사이클은 closest_idx가 직전값으로 고정돼 있어 이번 사이클의
        // L1 기하 자체를 신뢰할 수 없다. 새 값을 만들어내지 않고 직전 조향을 그대로 유지한다
        // (아래 rate limit·클리핑은 그대로 통과 — 값이 안 변하므로 no-op).
        if (pose_suspect_) {
            steering_angle = last_steering_angle_;
        }

        // 4) Rate limit
        double threshold = 0.4;
        steering_angle = std::max(last_steering_angle_ - threshold, std::min(steering_angle, last_steering_angle_ + threshold));

        // 5) 물리 한계 적용
        // 좌우 한계를 따로 적용 (δ>0 = 좌). 하드웨어가 못 내는 각을 명령해봐야 vesc_driver의
        // servo_limit이 조용히 자를 뿐이고, 컨트롤러는 그 사실을 모른 채 "꺾었다"고 가정하게 된다.
        steering_angle = std::max(-max_steering_right_, std::min(steering_angle, max_steering_left_));
        last_steering_angle_ = steering_angle;

        // 7. 종방향 제어 명령 (Target Speed) 산출
        // 속도용 룩어헤드 예측
        double global_speed = wps[find_lookahead_wp_idx(wps, path_closed, closest_idx, speed_lookahead_)].speed;
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

        // 경로 이탈 복구 중에는 속도를 낮춰 선회반경을 줄인다(위 2.5 가드와 한 쌍).
        // 최소 선회반경은 R = L/tan(δ_max) = 0.33/tan(0.41) ≈ 0.75m로 속도와 무관하지만,
        // 실제로는 속도가 높을수록 타이어 그립·요레이트 응답 한계로 그 반경에 못 미친다.
        // min_speed_ 하한은 두지 않는다 — 이탈 상태에서 최저순항속도를 지키는 것보다
        // 라인 복귀가 우선이고, 정지가 필요하면 상류 비상제동이 별도로 판단한다.
        if (recovery_active) {
            target_speed = std::min(target_speed, recovery_speed_);
        }

        // pose 튐 보류 중에는 감속한다 — 기하를 못 믿는 상태로 고속 주행하지 않는다.
        // 보류가 confirm_cycles(기본 5 = 100ms) 안에 풀리면 원래 속도로 자연 복귀한다.
        if (pose_suspect_) {
            target_speed = std::min(target_speed, pose_suspect_speed_);
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

        // 8-a. 장애물 종방향 감속 캡. 모든 스케일링 뒤 최종 상한으로 적용 — min_speed 하한을
        // 무시하고 0(정지)까지 눌러야 하므로 여기(램프 직전)가 마지막 게이트다. 실제 감속률은
        // 아래 램프의 max_decel이 제한하므로 급브레이크는 나지 않는다.
        double obstacle_speed_limit = compute_obstacle_speed_limit(following_local);
        if (obstacle_speed_limit < target_speed) {
            target_speed = obstacle_speed_limit;
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

        // 8-a2. 자율 미체결 중 램프 고정 — bumpless transfer (2026-07-28)
        //   이 노드는 drive_mode와 무관하게 상시 50Hz로 돌기 때문에, MANUAL/E-stop으로 서 있는
        //   동안에도 위 램프가 계속 감겨 올라간다. A를 눌러 ackermann_mux가 navigation 채널을
        //   여는 순간 그 값이 계단으로 VESC에 꽂힌다. 07-27 실차 bag 8개 전부에서 확인:
        //     정차 중 /drive_autonomous 1.50(최대 3.98) → engage 순간 commands/motor/speed가
        //     0 → 6348 ERPM 한 스텝 → 모터전류 60~62A(l_current_max 포화) → 급발진
        //   램프 상태를 실측 속도로 눌러두면 engage 시점의 명령이 실측과 같아져 계단이 사라지고,
        //   그 뒤엔 base_max_accel 램프가 정상적으로 가속을 만든다.
        //
        //   ⚠️ 체결(autonomous) 중에는 아무것도 하지 않는다. 07-22에 금지한 일반 lead-clamp
        //      ("명령이 실측보다 앞서지 못하게")와 다르다 — VESC 속도 PID가 전류를 뽑는 데
        //      필요한 명령 선행 여유(60A에 ~4.7 m/s)는 주행 중 그대로 보존된다.
        //   ⚠️ /drive_mode 미수신·끊김 시 게이트는 자동 비활성(engage_gate_active() 참고) →
        //      시뮬과 drive_mode_manager 미기동 상황에서 기존 거동이 그대로 유지된다.
        const bool disengaged = engage_gate_active() && !is_engaged_;
        if (disengaged) {
            final_speed = std::max(0.0, current_speed_);
            last_target_speed_ = final_speed;
            // 미체결 중엔 탈조·런치 상태도 누적하지 않는다(둘 다 "출발하려는데 안 나간다"를
            // 판정하는 로직인데, 애초에 출발 명령이 하류로 나가지 않는 구간이다).
            // ⚠️ launch_latched_off_는 여기서 건드리지 않는다 — 매 사이클 false로 되돌리면
            //    아래 8-c의 킥이 무한 재무장돼 미체결 중에도 publish_speed가 계속 부스트
            //    값으로 덮인다(램프를 고정해도 발행값이 안 따라오는 원인이었음).
            stall_time_ = 0.0;
            launch_active_ = false;
            launch_time_ = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "자율 미체결 — 속도 램프를 실측(%.2f m/s)에 고정 중(engage 시 무충격 전환)",
                current_speed_);
        }

        // 8-b. 기동 실패(VESC 센서리스 탈조) 안티와인드업 가드 (2026-07-22 실차 증상)
        //   위 램프의 증분은 실측 속도와 무관하게 매 사이클 max_accel*dt씩 쌓인다. 실차 VESC는
        //   센서리스 FOC라 정지→출발 시 오픈루프 구간(~0.59 m/s)을 못 넘기고 수 초간 탈조할 수
        //   있는데, 그동안 명령만 프로파일 속도까지 감겨 올라가 모터가 물리는 순간 풀 명령이
        //   걸린 채 차가 튀어나간다. 
        if (stall_guard_enable_) {
            if (std::abs(current_speed_) < stall_speed_threshold_ && final_speed > stall_hold_speed_) {
                stall_time_ += dt;
            } else {
                stall_time_ = 0.0;
            }
            if (stall_time_ > stall_hold_delay_) {
                final_speed = stall_hold_speed_;
                last_target_speed_ = final_speed;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "기동 실패 의심(%.1fs): 실측 %.2f m/s인데 명령이 감겨 올라감 → %.2f m/s로 제한. "
                    "VESC 센서리스 오픈루프 확인 필요",
                    stall_time_, current_speed_, stall_hold_speed_);
            }
        }

        // 8-c. 런치 킥(Launch Kick) — 자율 정지출발 시 VESC 센서리스 데드존 관통 (2026-07-25)
        //   매뉴얼은 초반 스로틀을 세게 넣어 큰 전류 펀치로 데드존(~0.5 m/s)을 때려 관통하는데,
        //   자율은 프로파일을 살살 램프해 명령이 데드존에 걸터앉아 토칭·탈조한다. VESC 속도 PID는
        //   ERPM 오차에 비례해 전류를 뽑으므로(s_pid_kp), 정지 상태에서 짧게 높은 속도를 명령하면
        //   매뉴얼 펀치와 동일하게 큰 전류가 흘러 데드존을 관통한다. 오픈루프 전류 상향·기본 HFI·
        //   Coupled HFI 모두 이 모터의 저돌극성(Lq-Ld 0.39µH) 때문에 부하서 실패 확인(2026-07-25 실차)
        //   → 컨트롤 측 관통이 유일한 자율 해법.
        //   ⚠️ final_speed(램프 상태)는 건드리지 않고 발행값(publish_speed)만 오버라이드 → 킥 종료 후
        //      램프가 부드럽게 이어짐. 히스테리시스: 정지(<standstill) 진입, 관통(>exit) 이탈.
        //   ⚠️ launch_boost_time(0.6s) < stall_hold_delay(1.0s)라 stall_guard와 안 싸움:
        //      킥 성공 시 차가 나가 stall_guard 미발동, 킥 실패(0.6s 초과) 시 포기하고 stall_guard가
        //      급발진 안전망으로 인계(과열 방지 위해 차가 실제 움직일 때까지 재시도 안 함).
        //   ⚠️ 미체결 중에는 킥도 돌리지 않는다(2026-07-28). 킥은 final_speed가 아니라
        //      publish_speed만 덮으므로, 게이트가 램프를 눌러놔도 킥이 켜져 있으면 발행값이
        //      계속 launch_boost_speed로 나간다 — 정차 중 3.0 m/s가 상시 발행되던 원인.
        double publish_speed = final_speed;
        if (launch_boost_enable_ && !disengaged) {
            bool moving = std::abs(current_speed_) > launch_exit_speed_;
            bool standstill = std::abs(current_speed_) < launch_standstill_speed_;
            if (moving) launch_latched_off_ = false;                    // 움직였으면 다음 정지서 다시 킥
            if (target_speed > 0.1) {                                   // 갈 의도가 있을 때만
                if (!launch_active_ && standstill && !launch_latched_off_) {
                    launch_active_ = true; launch_time_ = 0.0;          // 킥 시작
                }
                if (launch_active_) {
                    launch_time_ += dt;
                    if (moving) {                                       // 관통 성공
                        launch_active_ = false;
                    } else if (launch_time_ > launch_boost_time_) {     // 관통 실패 → 포기(stall_guard 인계)
                        launch_active_ = false; launch_latched_off_ = true;
                        RCLCPP_WARN(this->get_logger(),
                            "런치 킥 %.2fs 관통 실패 → 포기(stall_guard 인계). 데드존 심함 — 푸시스타트 필요",
                            launch_time_);
                    } else {
                        if (publish_speed < launch_boost_speed_) publish_speed = launch_boost_speed_;
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                            "런치 킥: 실측 %.2f → 발행 %.2f m/s (t=%.2fs)",
                            current_speed_, publish_speed, launch_time_);
                    }
                }
            } else {
                launch_active_ = false;                                // 정지 명령 중엔 킥 안 함
            }
        }

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";

        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = publish_speed;
        drive_msg.drive.acceleration = (publish_speed - current_speed_) / dt;

        // roll_max/limit%: 롤 ESC 실효성 계측(3(a)). 한 랩 돌고 이 %가 계속 낮게(예: 30% 미만)
        // 머무르면 max_roll_limit이 1/10 차량에 비해 과대하다는 뜻 — 롤각 대신 횡가속도
        // (a_lat = v*yaw_rate) 기반으로 ESC 신호를 바꾸는 것을 검토할 것.
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Pose: (%.2f, %.2f, %.2f) | Target WP: (%.2f, %.2f), Idx: %zu -> %zu | Steer: %.4f | Speed: %.2f / %.2f | L1_dist: %.2f | acc_mean: %.2f | roll_max: %.2f deg (%.0f%% of limit)",
            current_x_, current_y_, current_yaw_, L1_x, L1_y, closest_idx, idx_a, steering_angle, final_speed, current_speed_, L1_distance,
            acc_mean, max_abs_roll_seen_ * 180.0 / PI,
            max_roll_limit_ > 1e-6 ? (max_abs_roll_seen_ / max_roll_limit_ * 100.0) : 0.0);

        drive_pub_->publish(drive_msg);
    }

    // 8.4 디버그: L1 look-ahead 목표점 시각화
    // - 초록 구: 현재 겨냥 중인 L1 목표점
    // - 노란 선: 차량 위치 → L1 목표점 (룩어헤드 벡터)
    // 웨이포인트/pose와 동일 프레임(odom_frame_, 보통 map)에 그린다. 제어 경로와 완전 분리된 표시 전용.
    void publish_l1_marker(double L1_x, double L1_y) {
        visualization_msgs::msg::MarkerArray arr;
        auto stamp = this->now();

        // 목표점 구
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = odom_frame_;
        sphere.header.stamp = stamp;
        sphere.ns = "l1_lookahead";
        sphere.id = 0;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = L1_x;
        sphere.pose.position.y = L1_y;
        sphere.pose.position.z = 0.0;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.30;
        sphere.color.r = 0.0f; sphere.color.g = 1.0f; sphere.color.b = 0.0f; sphere.color.a = 1.0f;
        arr.markers.push_back(sphere);

        // 차량 → 목표점 룩어헤드 선
        visualization_msgs::msg::Marker line;
        line.header.frame_id = odom_frame_;
        line.header.stamp = stamp;
        line.ns = "l1_lookahead";
        line.id = 1;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.05; // 선 두께
        line.color.r = 1.0f; line.color.g = 1.0f; line.color.b = 0.0f; line.color.a = 0.8f;
        geometry_msgs::msg::Point p0, p1;
        p0.x = current_x_; p0.y = current_y_; p0.z = 0.0;
        p1.x = L1_x;       p1.y = L1_y;       p1.z = 0.0;
        line.points.push_back(p0);
        line.points.push_back(p1);
        arr.markers.push_back(line);

        l1_marker_pub_->publish(arr);
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
    bool stall_guard_enable_ = true;
    double stall_speed_threshold_ = 0.7;
    double stall_hold_speed_ = 1.5;
    double stall_hold_delay_ = 1.0;
    double stall_time_ = 0.0;   // 실측이 멈춰 있는데 명령만 커진 상태의 누적 시간 [s]
    // 런치 킥 상태(8-c)
    bool launch_boost_enable_ = true;
    double launch_boost_speed_ = 3.0;
    double launch_boost_time_ = 0.6;
    double launch_exit_speed_ = 0.8;
    double launch_standstill_speed_ = 0.3;
    bool launch_active_ = false;       // 현재 펀치 중
    double launch_time_ = 0.0;         // 현재 펀치 누적 시간 [s]
    bool launch_latched_off_ = false;  // 관통 실패로 포기 상태(차가 실제로 움직일 때까지 재시도 안 함)
    double base_max_decel_;                    // 명령 속도 하강 rate limit [m/s²]
    double prebrake_decel_ = 1.5;              // 곡률 사전감속 계산용 실측 감속 권한 [m/s²]
    bool use_imu_;
    double imu_angular_scale_;
    double imu_linear_scale_ = 1.0;
    double max_abs_roll_seen_ = 0.0;  // 롤 ESC 실효성 계측용(3(a)) — 주행 중 관측된 최대 |롤각|
    double yaw_rate_gain_;
    double max_speed_;
    double min_speed_;
    std::string odom_topic_;
    double wall_safety_margin_;

    // 곡률 룩어헤드 감속
    size_t curvature_lookahead_count_;
    double max_lateral_accel_;
    double understeer_gradient_ = 0.019;    // K_us [rad/(m/s²)] — 조향 권한 캡용, 0이면 비활성
    double steer_authority_ratio_ = 0.85;   // δ_max 중 곡률 추종 배정 비율
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

    size_t last_target_idx_ = 0;   // 글로벌 경로 인덱스 추적기
    size_t last_local_idx_ = 0;    // 로컬 경로 인덱스 추적기 (배열/인덱싱이 달라 분리)
    bool local_is_closed_ = false;         // 로컬 경로가 닫힌 루프인지(기하 판정)
    bool last_logged_local_closed_ = false; // 기하 판정 변화 시에만 로그
    double recovery_lat_error_ = 1.0;
    double recovery_speed_ = 2.0;

    // L1 횡가속 분모를 목표점까지의 실제 직선거리로 쓸지 (false면 구 거동: 명목 L1_distance)
    bool l1_use_actual_distance_ = true;

    // 좌우 조향 한계 [rad] (2026-07-28). 둘 다 MAX_STEERING_ANGLE이면 기존 대칭 거동과 동일.
    double max_steering_left_ = MAX_STEERING_ANGLE;
    double max_steering_right_ = MAX_STEERING_ANGLE;
    double steer_limit_min_ = MAX_STEERING_ANGLE;   // 속도 캡·갭팔로워용 보수값

    // 최근접 인덱스 견고화 (2026-07-28, MCL pose 붕괴 대응)
    double closest_idx_max_heading_err_ = 1.75;  // 전역 재탐색 헤딩 게이트 [rad], 0이면 비활성
    double idx_jump_confirm_dist_ = 2.0;         // 이 거리[m] 초과 점프는 확인 대기
    int idx_jump_confirm_cycles_ = 5;            // 연속 이 사이클 유지되면 채택 (0이면 비활성)
    int idx_jump_count_ = 0;                     // 현재 점프가 연속 유지된 사이클 수
    bool pose_suspect_ = false;                  // 이번 사이클 pose를 못 믿음(조향 홀드+감속)
    double pose_suspect_speed_ = 1.5;            // 보류 중 속도 상한 [m/s]
    bool global_idx_valid_ = false;              // 점프 게이트 비교 기준 유효성(소스별)
    bool local_idx_valid_ = false;

    // 자율 체결 게이트 (bumpless transfer, 2026-07-28)
    bool engage_gate_enable_ = true;
    std::string drive_mode_topic_ = "/drive_mode";
    std::string engaged_mode_value_ = "autonomous";
    double drive_mode_timeout_ = 1.0;
    bool is_engaged_ = false;        // 마지막 수신 기준 자율 체결 여부
    bool drive_mode_seen_ = false;   // /drive_mode를 한 번이라도 받았는지(미수신 시 게이트 비활성)
    rclcpp::Time drive_mode_last_recv_time_;
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
    bool gap_follower_failsafe_ = false;
    double obstacle_cone_halfangle_ = 0.14;    // L1 방향 콘 반각 [rad] (~8도)
    double obstacle_trigger_dist_ = 1.5;       // 이 거리[m] 이내 근접 시 차단 판정
    double obstacle_margin_ = 0.3;             // 목표점 거리 대비 최소 여유[m]
    int obstacle_avoid_hold_cycles_ = 15;      // 회피 유지 사이클(50Hz→0.3s), 채터링 방지
    int avoid_hold_counter_ = 0;

    // 장애물 종방향 감속 (opponent_detector raw 장애물 → target_speed 캡)
    bool obstacle_brake_enable_ = true;
    std::string obstacle_raw_topic_ = "/perception/detection/raw_obstacles";
    double obstacle_brake_decel_ = 6.0;        // v_cap 산출용 감속도 [m/s²]
    double obstacle_stop_gap_ = 1.0;           // 장애물 앞 정지 여유 [m]
    double obstacle_corridor_halfwidth_ = 0.35;// 통로 반폭(차폭/2+여유) [m]
    double obstacle_max_range_ = 9.0;          // 이 전방거리[m] 밖 장애물 무시
    int obstacle_brake_hold_cycles_ = 10;      // 소실 후 캡 유지 사이클(채터링 방지)
    double obstacle_brake_timeout_ = 0.3;      // raw 토픽 신선도 [s]
    double obstacle_avoid_min_speed_ = 1.5;    // 로컬 회피경로 추종 중 캡 하한(정지 대신 관통) [m/s]
    int obstacle_brake_hold_counter_ = 0;
    double obstacle_held_cap_ = std::numeric_limits<double>::max();
    double track_length_s_ = 0.0;              // s-wrap 기준 트랙 총길이 [m]
    size_t last_global_proj_idx_ = 0;          // 에고 글로벌 투영 인덱스 추적기
    f110_msgs::msg::ObstacleArray::ConstSharedPtr latest_obstacles_ = nullptr;
    rclcpp::Time obstacle_last_recv_time_;

    std::unique_ptr<GapFollower> gap_follower_;
    std::unique_ptr<StabilityController> stability_controller_;

    // ROS 2 통신
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_mode_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_path_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr local_path_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr l1_marker_pub_;
    std::string odom_frame_ = "map"; // odom 메시지 header.frame_id (L1 마커 프레임)
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
