// ============================================================================
// control_mppi_solver_cpu.cpp — 샘플링 기반 MPPI(Model Predictive Path Integral) 컨트롤러
// ----------------------------------------------------------------------------
// 정보이론 MPPI(Williams et al. 2018, IEEE T-RO):
//   1) 명목 제어열 U={u_0..u_{N-1}} 주위로 K개 잡음 롤아웃 v_{t,k}=u_t+ε_{t,k} 샘플
//   2) 각 롤아웃을 차량 동역학으로 전진시켜 비용 J_k 누적
//   3) 가중 w_k = exp(-(J_k-β)/λ)/Σ  (β=min_k J_k, 수치안정용 감산)
//   4) 갱신 u_t += Σ_k w_k ε_{t,k},  첫 제어 u_0를 출력, U를 한 칸 warm-start 시프트
//
// 제어 u = [조향 δ, 종가속 a]  (조향+종방향 동시 최적화)
// 동역학: 동역학 자전거(single-track) + Pacejka 마법공식 횡력.
//   저속(vx→0)에서 슬립각이 발산하므로 v_switch 미만은 기구학 자전거로 블렌드.
//
// ⚠️ 기존 NUC6 Pacejka LUT는 (횡가속도,속도)→조향각의 *역방향* 맵(MAP L1 전용)이라
//    전방 롤아웃에는 못 쓴다. 여기서는 전방 Pacejka 모델을 자체 파라미터로 구현하고,
//    기본값은 f1tenth_gym/forzaETH 차량값에서 가져온다(추후 실차 보정 대상).
//
// 단일 파일 원칙(control_map_node.cpp와 동일): 별도 헤더 없이 알고리즘 클래스를 이 파일에
//   인라인 정의한다. 다음 세션에 ControlMppiNode + main()을 이 파일에 덧붙이고
//   CMake를 add_library → add_executable(control_mppi_node)로 전환한다.
//   파일 하단 #ifdef MPPI_SMOKE_TEST 블록은 ROS 없이 알고리즘을 폐루프 검증하는 하네스.
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace f1tenth_control {

// ---------------------------------------------------------------------------
// 자료구조
// ---------------------------------------------------------------------------

// 차량 상태 (전역 위치 + 차체프레임 속도)
struct MppiState {
    double x = 0.0, y = 0.0, yaw = 0.0;  // 전역 X,Y,헤딩 ψ
    double vx = 0.0, vy = 0.0;           // 차체프레임 종/횡 속도
    double yaw_rate = 0.0;               // 요레이트 ω
};

// 예측 수평 한 스테이지 기준점 (호출부가 채움: 플래너 웨이포인트에서 유도)
struct MppiRef {
    double x = 0.0, y = 0.0, yaw = 0.0;  // 기준 위치·헤딩
    double v = 0.0;                      // 목표 속도 (플래너 vx_mps)
    double half_width = 1.0;             // 트랙 반폭 ≈ min(d_left,d_right) (경계비용용)
};

// 제어 입력
struct MppiControl {
    double steer = 0.0;  // 조향 δ [rad]
    double accel = 0.0;  // 종가속 a [m/s²]
};

// MPPI + 차량/타이어 + 비용 파라미터 (기본값: f1tenth_gym F1TENTH 차량)
struct MppiParams {
    // --- MPPI 코어 ---
    int    N  = 25;      // 예측 수평(스텝)
    int    K  = 512;     // 롤아웃(샘플) 수
    double dt = 0.05;    // 스텝 [s] (50Hz)
    double lambda = 1.0; // 역온도(temperature): 작을수록 저비용 롤아웃에 집중
    double sigma_steer = 0.15;  // 조향 노이즈 σ [rad]
    double sigma_accel = 1.5;   // 가속 노이즈 σ [m/s²]
    unsigned int seed = 0xC0FFEEu;

    // --- 차량/타이어 ---
    double wheelbase = 0.33, lf = 0.15875, lr = 0.17145;  // L = lf+lr
    double mass = 3.74, Iz = 0.04712, g = 9.81, mu = 1.0489;
    // Pacejka 마법공식 계수(전/후) — TUNE. D는 μ·Fz로 코드에서 계산.
    double Bf = 10.0, Cf = 1.9, Ef = 0.97;
    double Br = 10.0, Cr = 1.9, Er = 0.97;
    double v_switch = 2.0;   // 이 속도[m/s] 미만은 기구학 블렌드(발산 방지)
    int    substeps = 2;     // 동역학 오일러 적분 서브스텝(수치 안정)

    // --- 비용 가중 ---
    double w_pos = 8.0, w_yaw = 2.0, w_v = 1.0;
    double w_boundary = 500.0;  // 트랙 경계 이탈 소프트 페널티
    double margin = 0.15;       // 경계 여유[m] (차 반폭+마진)
    double w_terminal = 20.0;   // 종단(마지막 스텝) 위치·헤딩 가중 배수

    // --- 한계 ---
    double steer_max = 0.41, accel_max = 4.0, accel_min = -8.0;
    double v_min = 0.0, v_max = 8.0;

    // --- 출력 평활화(LP-MPPI 근사): 0=off, 0<α<1이면 이전 출력과 저역통과 블렌드 ---
    double u_smooth = 0.3;
};

// ---------------------------------------------------------------------------
// 유틸
// ---------------------------------------------------------------------------
namespace {
inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
// 각도를 [-π, π]로 정규화
inline double wrap_pi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
// Pacejka 마법공식 정규화 횡력계수: μ_y(α) = sin(C·atan(B·α − E·(B·α − atan(B·α))))
inline double pacejka_mu(double alpha, double B, double C, double E) {
    const double Ba = B * alpha;
    return std::sin(C * std::atan(Ba - E * (Ba - std::atan(Ba))));
}
}  // namespace

// ---------------------------------------------------------------------------
// MPPI 컨트롤러
// ---------------------------------------------------------------------------
class MPPIController {
 public:
    explicit MPPIController(const MppiParams& p)
        : p_(p), rng_(p.seed) {
        U_.assign(static_cast<size_t>(p_.N), MppiControl{});
        eps_.assign(static_cast<size_t>(p_.K),
                    std::vector<MppiControl>(static_cast<size_t>(p_.N)));
        costs_.assign(static_cast<size_t>(p_.K), 0.0);
    }

    // nominal 제어열·이전 출력 리셋 (정지/모드 재진입 시)
    void reset() {
        std::fill(U_.begin(), U_.end(), MppiControl{});
        last_out_ = MppiControl{};
    }

    double last_solve_ms() const { return last_solve_ms_; }
    const MppiParams& params() const { return p_; }

    // 전방 동역학 1스텝 노출(폐루프 시뮬/테스트에서 동일 모델 되먹임에 사용).
    MppiState propagate(const MppiState& s, const MppiControl& u) const {
        return step_dynamics(s, u);
    }

    // 현재 상태 + 기준궤적(길이 N+1) → 최적 첫 제어 out.
    // 내부 nominal U_는 호출 간 warm-start로 유지된다.
    bool solve(const MppiState& cur, const std::vector<MppiRef>& ref, MppiControl& out) {
        if (static_cast<int>(ref.size()) < p_.N + 1) {
            return false;  // 기준궤적 부족
        }
        const auto t0 = std::chrono::steady_clock::now();

        std::normal_distribution<double> nd_steer(0.0, p_.sigma_steer);
        std::normal_distribution<double> nd_accel(0.0, p_.sigma_accel);

        // (1) 잡음 샘플 + (2) 롤아웃·비용
        for (int k = 0; k < p_.K; ++k) {
            MppiState s = cur;
            double Jk = 0.0;
            for (int t = 0; t < p_.N; ++t) {
                // 잡음 있는 제어 v = u_t + ε, 한계 클램프
                MppiControl eps{nd_steer(rng_), nd_accel(rng_)};
                MppiControl v{
                    clampd(U_[t].steer + eps.steer, -p_.steer_max, p_.steer_max),
                    clampd(U_[t].accel + eps.accel,  p_.accel_min, p_.accel_max)};
                // 클램프로 실제 적용된 제어와 명목의 차이를 잡음으로 재정의(경계에서 일관성)
                eps.steer = v.steer - U_[t].steer;
                eps.accel = v.accel - U_[t].accel;
                eps_[k][t] = eps;

                s = step_dynamics(s, v);
                Jk += stage_cost(s, ref[t + 1], /*terminal=*/false);
            }
            // 종단 비용(마지막 상태)
            Jk += stage_cost(s, ref[p_.N], /*terminal=*/true);
            // NaN/Inf 방어 → 해당 샘플 사실상 배제
            costs_[k] = std::isfinite(Jk) ? Jk : std::numeric_limits<double>::infinity();
        }

        // (3) 가중: β 감산 후 exp, 정규화
        double beta = std::numeric_limits<double>::infinity();
        for (int k = 0; k < p_.K; ++k) beta = std::min(beta, costs_[k]);
        if (!std::isfinite(beta)) {
            // 전 샘플 발산 → 갱신 없이 안전 홀드(이전 출력 유지)
            finalize_time(t0);
            out = last_out_;
            return false;
        }
        double eta = 0.0;
        for (int k = 0; k < p_.K; ++k) {
            double w = std::exp(-(costs_[k] - beta) / p_.lambda);
            costs_[k] = w;  // 이제 costs_는 정규화 전 가중치
            eta += w;
        }
        if (eta < 1e-12) {
            finalize_time(t0);
            out = last_out_;
            return false;
        }
        const double inv_eta = 1.0 / eta;

        // (4) 갱신: u_t += Σ_k w_k ε_{t,k}
        for (int t = 0; t < p_.N; ++t) {
            double ds = 0.0, da = 0.0;
            for (int k = 0; k < p_.K; ++k) {
                const double w = costs_[k] * inv_eta;
                ds += w * eps_[k][t].steer;
                da += w * eps_[k][t].accel;
            }
            U_[t].steer = clampd(U_[t].steer + ds, -p_.steer_max, p_.steer_max);
            U_[t].accel = clampd(U_[t].accel + da,  p_.accel_min, p_.accel_max);
        }

        // (5) 출력 = 첫 제어, 저역통과 평활화
        MppiControl u0 = U_[0];
        if (p_.u_smooth > 0.0) {
            u0.steer = p_.u_smooth * last_out_.steer + (1.0 - p_.u_smooth) * u0.steer;
            u0.accel = p_.u_smooth * last_out_.accel + (1.0 - p_.u_smooth) * u0.accel;
        }
        last_out_ = u0;
        out = u0;

        // (6) warm-start 시프트 (한 칸 당기고 마지막은 유지)
        for (int t = 0; t < p_.N - 1; ++t) U_[t] = U_[t + 1];
        // U_[N-1]은 그대로 유지 — 다음 사이클 탐색 시드

        finalize_time(t0);
        return true;
    }

 private:
    void finalize_time(const std::chrono::steady_clock::time_point& t0) {
        const auto t1 = std::chrono::steady_clock::now();
        last_solve_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    // 전방 동역학 1스텝(내부에서 substeps회 적분). 동역학 자전거+Pacejka,
    // 저속에서는 기구학 자전거로 블렌드하여 슬립각 발산을 방지한다.
    MppiState step_dynamics(const MppiState& s0, const MppiControl& u) const {
        const double L  = p_.wheelbase;
        const int    sub = std::max(1, p_.substeps);
        const double h  = p_.dt / sub;
        // 정하중 → Pacejka 최대 횡력 D = μ·Fz
        const double Fzf = p_.mass * p_.g * p_.lr / L;
        const double Fzr = p_.mass * p_.g * p_.lf / L;
        const double Df  = p_.mu * Fzf;
        const double Dr  = p_.mu * Fzr;
        const double delta = clampd(u.steer, -p_.steer_max, p_.steer_max);
        const double accel = clampd(u.accel,  p_.accel_min, p_.accel_max);

        MppiState s = s0;
        for (int i = 0; i < sub; ++i) {
            // 동역학↔기구학 블렌드 계수 (vx ≤ v_switch/2: 순수 기구학, ≥ v_switch: 순수 동역학)
            const double half = 0.5 * p_.v_switch;
            const double blend = clampd((s.vx - half) / std::max(1e-3, p_.v_switch - half), 0.0, 1.0);

            // --- 동역학 자전거(Pacejka) 미분 ---
            double dvy_dyn = 0.0, dw_dyn = 0.0;
            {
                const double vx_safe = std::max(s.vx, 1e-3);  // 슬립각 분모 보호
                const double af = delta - std::atan2(s.vy + p_.lf * s.yaw_rate, vx_safe);
                const double ar =         -std::atan2(s.vy - p_.lr * s.yaw_rate, vx_safe);
                const double Fyf = Df * pacejka_mu(af, p_.Bf, p_.Cf, p_.Ef);
                const double Fyr = Dr * pacejka_mu(ar, p_.Br, p_.Cr, p_.Er);
                dvy_dyn = (Fyf * std::cos(delta) + Fyr) / p_.mass - s.vx * s.yaw_rate;
                dw_dyn  = (p_.lf * Fyf * std::cos(delta) - p_.lr * Fyr) / p_.Iz;
            }

            // --- 기구학 자전거 대수식(저속용) ---
            const double w_kin  = s.vx * std::tan(delta) / L;
            const double vy_kin = p_.lr * w_kin;  // 후축 기준 슬립 근사

            // --- 종속도는 항상 종가속으로 적분(양 모델 공통) ---
            const double dvx = accel;

            // --- 횡/요 상태: 블렌드 ---
            if (blend >= 1.0) {
                s.vy       += h * dvy_dyn;
                s.yaw_rate += h * dw_dyn;
            } else if (blend <= 0.0) {
                // 순수 기구학: vy·ω를 대수식으로 강제
                s.vy = vy_kin;
                s.yaw_rate = w_kin;
            } else {
                // 중간대: 동역학 적분값과 기구학 대수식을 블렌드
                const double vy_dyn_next = s.vy + h * dvy_dyn;
                const double w_dyn_next  = s.yaw_rate + h * dw_dyn;
                s.vy       = blend * vy_dyn_next + (1.0 - blend) * vy_kin;
                s.yaw_rate = blend * w_dyn_next  + (1.0 - blend) * w_kin;
            }

            // --- 전역 포즈 적분 ---
            const double cyaw = std::cos(s.yaw), syaw = std::sin(s.yaw);
            s.x   += h * (s.vx * cyaw - s.vy * syaw);
            s.y   += h * (s.vx * syaw + s.vy * cyaw);
            s.yaw  = wrap_pi(s.yaw + h * s.yaw_rate);
            s.vx   = clampd(s.vx + h * dvx, p_.v_min, p_.v_max);
        }
        return s;
    }

    // 스테이지 비용: 위치·헤딩·속도 추종 + 트랙 경계 소프트 페널티.
    double stage_cost(const MppiState& s, const MppiRef& r, bool terminal) const {
        const double tw = terminal ? p_.w_terminal : 1.0;
        const double dx = s.x - r.x;
        const double dy = s.y - r.y;

        double c = 0.0;
        c += tw * p_.w_pos * (dx * dx + dy * dy);              // 위치
        const double eyaw = wrap_pi(s.yaw - r.yaw);
        c += tw * p_.w_yaw * (eyaw * eyaw);                    // 헤딩
        const double ev = s.vx - r.v;
        c += p_.w_v * (ev * ev);                               // 속도 추종

        // 경계(충돌) 소프트 페널티: 기준점 법선방향 횡편차가 (반폭−마진)을 넘으면 강한 벌점.
        // 법선 n=(-sin ryaw, cos ryaw), e_lat = n·(pos−ref)
        const double e_lat = -std::sin(r.yaw) * dx + std::cos(r.yaw) * dy;
        const double limit = std::max(0.0, r.half_width - p_.margin);
        const double over  = std::abs(e_lat) - limit;
        if (over > 0.0) c += p_.w_boundary * (over * over);

        return c;
    }

    MppiParams p_;
    std::vector<MppiControl> U_;                     // nominal 제어열(길이 N, warm-start)
    std::vector<std::vector<MppiControl>> eps_;      // 잡음버퍼 [K][N] (재사용)
    std::vector<double> costs_;                      // [K] 비용→가중치 재사용
    MppiControl last_out_{};
    std::mt19937 rng_;
    double last_solve_ms_ = 0.0;
};

}  // namespace f1tenth_control


// ============================================================================
// 스탠드얼론 스모크 테스트 (ROS 무관, 알고리즘 폐루프 검증)
//   빌드: g++ -O2 -std=c++17 -DMPPI_SMOKE_TEST control_code/control_mppi_solver_cpu.cpp -o /tmp/mppi_smoke
//   static lib 빌드(colcon) 시엔 매크로 미정의라 main 없음.
// ============================================================================
#ifdef MPPI_SMOKE_TEST
#include <cstdio>

int main() {
    using namespace f1tenth_control;

    MppiParams p;                 // 기본값 사용
    MPPIController mppi(p);

    // 합성 기준경로: 반지름 R 원호(반시계). 목표속도 일정.
    const double R = 6.0;         // [m]
    const double v_ref = 3.0;     // [m/s]
    const double track_halfwidth = 1.2;

    // 현재 진행각 θ0에서 시작해 호 길이 v_ref·dt 간격으로 N+1개 기준점 생성
    auto build_ref = [&](double theta0) {
        std::vector<MppiRef> ref;
        ref.reserve(p.N + 1);
        const double dtheta = (v_ref * p.dt) / R;  // 스텝당 각 증가
        for (int t = 0; t <= p.N; ++t) {
            const double th = theta0 + dtheta * t;
            MppiRef r;
            r.x = R * std::cos(th);
            r.y = R * std::sin(th);
            r.yaw = wrap_pi(th + M_PI / 2.0);  // 반시계 접선방향
            r.v = v_ref;
            r.half_width = track_halfwidth;
            ref.push_back(r);
        }
        return ref;
    };

    // 초기 상태: 원 위 θ=0 지점에서 안쪽으로 0.4m 벗어난 채 정지(저속 출발 + 횡오차 수렴 검증)
    MppiState s;
    s.x = (R - 0.4);
    s.y = 0.0;
    s.yaw = M_PI / 2.0;

    auto theta_of = [&](const MppiState& st) { return std::atan2(st.y, st.x); };

    const int STEPS = 400;        // 20s @ 50Hz
    double max_lat = 0.0, worst_ms = 0.0;
    double lat_first = 0.0, lat_last = 0.0;
    bool control_ok = true, finite_ok = true;

    for (int i = 0; i < STEPS; ++i) {
        auto ref = build_ref(theta_of(s));
        MppiControl u;
        bool ok = mppi.solve(s, ref, u);
        (void)ok;

        if (!(std::isfinite(u.steer) && std::isfinite(u.accel))) finite_ok = false;
        if (std::abs(u.steer) > p.steer_max + 1e-6 ||
            u.accel > p.accel_max + 1e-6 || u.accel < p.accel_min - 1e-6) control_ok = false;

        // 컨트롤러와 동일한 동역학으로 되먹임(폐루프)
        s = mppi.propagate(s, u);

        const double lat = std::abs(std::hypot(s.x, s.y) - R);  // 원에서의 반경오차
        max_lat = std::max(max_lat, lat);
        worst_ms = std::max(worst_ms, mppi.last_solve_ms());
        if (i == 0) lat_first = lat;
        lat_last = lat;
    }

    std::printf("== MPPI smoke test ==\n");
    std::printf("lateral error: first=%.3f m  last=%.3f m  (max=%.3f)\n", lat_first, lat_last, max_lat);
    std::printf("final speed  : %.3f m/s (target %.1f)\n", s.vx, v_ref);
    std::printf("worst solve  : %.2f ms  (budget 20.0 ms @50Hz)\n", worst_ms);
    std::printf("controls in-limit: %s | finite: %s\n",
                control_ok ? "OK" : "FAIL", finite_ok ? "OK" : "FAIL");

    const bool converged = (lat_last < lat_first) && (lat_last < 0.5);
    const bool rt_ok = worst_ms < 20.0;
    std::printf("converged: %s | realtime: %s\n",
                converged ? "OK" : "FAIL", rt_ok ? "OK" : "FAIL");
    std::printf("RESULT: %s\n",
                (converged && rt_ok && control_ok && finite_ok) ? "PASS" : "CHECK");
    return (converged && rt_ok && control_ok && finite_ok) ? 0 : 1;
}
#endif  // MPPI_SMOKE_TEST
