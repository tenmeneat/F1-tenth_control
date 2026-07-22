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
//   인라인 정의한다(노드 본체는 control_mppi_node.cpp가 별도로 #include). 파일 하단
//   #ifdef MPPI_SMOKE_TEST 블록은 ROS 없이 알고리즘을 폐루프 검증하는 하네스.
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
    // 경계비용용 좌/우 벽까지 거리 — **비대칭**으로 따로 받는다(2026-07-22).
    // 이전에는 half_width = min(d_left,d_right) 하나로 대칭 튜브를 만들었는데, 최적
    // 레이싱라인은 정의상 한쪽 벽에 붙으므로 좁은 쪽 값이 넓은 쪽 여유까지 깎아버렸다.
    double d_left = 1.0, d_right = 1.0;
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
    double lambda = 1.0; // 고정 역온도(lambda_rel<=0일 때만 사용)
    // 적응 역온도(2026-07-22): λ_eff = lambda_rel·(J_mean − J_min). >0이면 이쪽이 우선.
    //   λ를 비용 스케일에 **불변**으로 만든다. 고정 λ는 w_*를 하나만 바꿔도 유효샘플수(ESS)가
    //   1까지 무너지거나 K까지 뭉개져, 07-22 시뮬에서 λ 1↔60↔200이 전부 다른 실패를 냈다.
    //   0.02면 대체로 ESS가 K의 10% 부근에 앉는다(07-22 원호 스윕: 0.5→ESS 338로 뭉개져
    //   가속 자체가 안 나왔고, 0.02→ESS 52에서 속도·횡오차가 동시에 최선이었다).
    double lambda_rel = 0.02;
    double sigma_steer = 0.15;  // 조향 노이즈 σ [rad]
    double sigma_accel = 1.5;   // 가속 노이즈 σ [m/s²]
    // 잡음 시간상관 AR(1) 계수 [0,1): z_t = β·z_{t-1} + √(1−β²)·n_t (정상분포 N(0,1) 유지).
    // 0이면 기존 백색잡음 — 스텝마다 독립이라 **고주파로 진동하는 제어열**이 샘플 풀을 채우고,
    // 그런 해가 한 번 뽑히면 그대로 출력돼 조향 채터링이 된다. 0.6~0.8이 매끈한 기동을 만든다.
    double noise_beta = 0.7;
    unsigned int seed = 0xC0FFEEu;

    // --- 차량/타이어 ---
    double wheelbase = 0.33, lf = 0.15875, lr = 0.17145;  // L = lf+lr
    double mass = 3.74, Iz = 0.04712, g = 9.81, mu = 1.0489;
    // Pacejka 마법공식 계수(전/후) — TUNE. D는 μ·Fz로 코드에서 계산.
    double Bf = 10.0, Cf = 1.9, Ef = 0.97;
    double Br = 10.0, Cr = 1.9, Er = 0.97;
    double v_switch = 2.0;   // 이 속도[m/s] 미만은 기구학 블렌드(발산 방지)
    int    substeps = 2;     // 동역학 오일러 적분 서브스텝(수치 안정)

    // --- 비용 가중 (2026-07-22 컨투어링 재정식화) ---
    // 이전엔 w_pos·|pos−ref|²(점추종)이었다. 이러면 차가 기준점보다 뒤처졌을 때 "앞의 점으로
    // 최단거리로 가라" = 코너를 가로지르라는 명령이 되어 헤어핀 탈출에서 벽으로 몰았다.
    // 오차를 경로 접선 기준으로 분해해, **횡(컨투어링)오차는 강하게** 잡고 **진행방향(lag)
    // 오차는 약하게**만 잡는다(시간정합용). 진행 속도는 w_v가 따로 담당.
    double w_lat = 150.0;        // 경로 횡오차 가중 — 주력. w_v와의 **비율**이 안정성을 정한다(07-22 실측: w_v/w_lat이
    // 0.004 이하면 안정, 0.0067이면 헤어핀에서 마찰포화로 크래시)
    double w_lon = 1.0;         // 경로 진행방향 오차 가중 — 작게
    double w_yaw = 5.0, w_v = 0.5;
    double w_boundary = 500.0;  // 트랙 경계 이탈 소프트 페널티
    double margin = 0.15;       // 경계 여유[m] (차 반폭+마진)
    double w_terminal = 20.0;   // 종단(마지막 스텝) 위치·헤딩 가중 배수
    // 제어 변화율(Δu) 비용 — 채터링 억제의 본체. t=0의 기준은 **직전에 실제로 출력한 제어**라,
    // 사이클 간 불연속(부호 반전)에도 벌점이 걸린다. 잡음 시간상관(noise_beta)과 한 쌍.
    double w_dsteer = 100.0;
    double w_daccel = 0.5;

    // --- 한계 ---
    double steer_max = 0.41, accel_max = 4.0, accel_min = -8.0;
    double v_min = 0.0, v_max = 8.0;

    // --- 출력 평활화(LP-MPPI 근사): 0=off, 0<α<1이면 이전 출력과 저역통과 블렌드 ---
    double u_smooth = 0.3;

    // 진단 계측 on/off. 켜면 롤아웃 1회+리덕션 몇 개가 추가된다(CPU는 K=512 대비 0.2%,
    // GPU는 커널 런치 몇 개). 젯슨 실시간 예산이 빡빡하면 끌 것 — 제어 결과는 동일하다.
    bool diag_enable = true;
};

// 솔버 진단값 (2026-07-22 추가) — 튜닝을 숫자로 하기 위한 계측. 제어 로직에는 쓰이지 않는다.
struct MppiDiag {
    double ess = 0.0;       // 유효 샘플수 (Σw)²/Σw². K의 10~40%가 건강한 범위
    double lambda_eff = 0.0;// 이번 사이클에 실제 쓴 λ (적응 λ면 매 사이클 달라짐)
    double j_min = 0.0;     // 최저 롤아웃 비용(=β)
    double j_mean = 0.0;    // 평균 롤아웃 비용
    double j_max = 0.0;     // 최고 롤아웃 비용
    // 갱신된 U_(잡음 없는 명목 계획) 기준 항목별 비용 — 어느 항이 지배하는지 보는 용도
    double c_lat = 0.0, c_lon = 0.0, c_yaw = 0.0, c_v = 0.0, c_bnd = 0.0, c_du = 0.0;
    double nominal_max_lat = 0.0;  // 명목 계획의 최대 **횡**오차 [m] (점거리가 아니라 횡오차)
    int    n_finite = 0;    // 비용이 유한한(=발산하지 않은) 롤아웃 수. K보다 훨씬 작으면 모델 적분이 터진 것
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
    const MppiDiag& last_diag() const { return diag_; }

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

        std::normal_distribution<double> nd(0.0, 1.0);
        // AR(1) 시간상관 잡음 계수: z_t = beta·z_{t-1} + alpha·n_t, alpha=√(1−β²)면
        // z는 정상 N(0,1)을 유지한다(σ가 β에 따라 변하지 않음 → σ 튜닝과 β 튜닝이 독립).
        const double nb = clampd(p_.noise_beta, 0.0, 0.99);
        const double na = std::sqrt(std::max(0.0, 1.0 - nb * nb));

        // (1) 잡음 샘플 + (2) 롤아웃·비용
        for (int k = 0; k < p_.K; ++k) {
            MppiState s = cur;
            double Jk = 0.0;
            double zs = 0.0, za = 0.0;          // AR(1) 잡음 상태(조향/가속)
            MppiControl prev = last_out_;        // Δu 기준: t=0은 직전에 실제로 출력한 제어
            for (int t = 0; t < p_.N; ++t) {
                // 시간상관 잡음 → 제어 v = u_t + ε, 한계 클램프
                zs = nb * zs + na * nd(rng_);
                za = nb * za + na * nd(rng_);
                MppiControl eps{zs * p_.sigma_steer, za * p_.sigma_accel};
                MppiControl v{
                    clampd(U_[t].steer + eps.steer, -p_.steer_max, p_.steer_max),
                    clampd(U_[t].accel + eps.accel,  p_.accel_min, p_.accel_max)};
                // 클램프로 실제 적용된 제어와 명목의 차이를 잡음으로 재정의(경계에서 일관성)
                eps.steer = v.steer - U_[t].steer;
                eps.accel = v.accel - U_[t].accel;
                eps_[k][t] = eps;

                // 제어 변화율 비용(채터링 억제)
                const double dsteer = v.steer - prev.steer;
                const double daccel = v.accel - prev.accel;
                Jk += p_.w_dsteer * dsteer * dsteer + p_.w_daccel * daccel * daccel;
                prev = v;

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
        double j_max = 0.0, j_sum = 0.0;
        int j_finite = 0;
        for (int k = 0; k < p_.K; ++k) {
            beta = std::min(beta, costs_[k]);
            if (std::isfinite(costs_[k])) { j_max = std::max(j_max, costs_[k]); j_sum += costs_[k]; ++j_finite; }
        }
        diag_.j_max = j_max;
        diag_.j_mean = (j_finite > 0) ? j_sum / j_finite : 0.0;
        diag_.n_finite = j_finite;  // 발산(inf)해서 버려진 롤아웃이 몇 개인지 — ESS 해석에 필수
        if (!std::isfinite(beta)) {
            // 전 샘플 발산 → 갱신 없이 안전 홀드(이전 출력 유지)
            finalize_time(t0);
            out = last_out_;
            return false;
        }
        // 적응 역온도: 비용 스프레드에 비례시켜 λ를 비용 스케일에 불변으로 만든다.
        // (lambda_rel<=0이면 고정 λ 사용 — 예전 거동 재현용)
        const double spread = std::max(0.0, diag_.j_mean - beta);
        const double lambda_eff = (p_.lambda_rel > 0.0)
                                      ? std::max(1e-6, p_.lambda_rel * spread)
                                      : std::max(1e-6, p_.lambda);
        diag_.lambda_eff = lambda_eff;

        double eta = 0.0;
        for (int k = 0; k < p_.K; ++k) {
            double w = std::exp(-(costs_[k] - beta) / lambda_eff);
            costs_[k] = w;  // 이제 costs_는 정규화 전 가중치
            eta += w;
        }
        if (eta < 1e-12) {
            finalize_time(t0);
            out = last_out_;
            return false;
        }
        const double inv_eta = 1.0 / eta;

        // (3-b) 진단 계측 (2026-07-22 추가) — 튜닝을 눈대중이 아니라 숫자로 하기 위한 것.
        // ESS(유효 샘플수) = (Σw)²/Σw². λ가 비용 스프레드에 비해 너무 작으면 ESS→1이 되어
        // "가장 운 좋은 난수 시퀀스 복사" = 랜덤서치가 되고, 조향이 매 사이클 튄다(채터링).
        // 너무 크면 ESS→K로 전 샘플 평균 = 명령이 뭉개진다. 경험적 목표는 K의 5~20%.
        {
            double sum_w2 = 0.0;
            for (int k = 0; k < p_.K; ++k) sum_w2 += costs_[k] * costs_[k];
            diag_.ess = (sum_w2 > 0.0) ? (eta * eta / sum_w2) : 0.0;
            diag_.j_min = beta;
        }

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

        // (4-b) 진단: 갱신된 U_를 잡음 없이 굴려 비용 항목별 구성비를 본다(롤아웃 1회 추가,
        //       K=512 대비 0.2%라 실시간 예산에 영향 없음). 어느 항이 비용을 지배하는지
        //       모르면 w_* 튜닝이 추측이 된다.
        if (p_.diag_enable) {
            MppiState s = cur;
            CostTerms ct;
            double max_lat = 0.0;
            MppiControl prev = last_out_;
            for (int t = 0; t < p_.N; ++t) {
                const double dsteer = U_[t].steer - prev.steer;
                const double daccel = U_[t].accel - prev.accel;
                ct.du += p_.w_dsteer * dsteer * dsteer + p_.w_daccel * daccel * daccel;
                prev = U_[t];
                s = step_dynamics(s, U_[t]);
                accumulate_cost_terms(s, ref[t + 1], false, ct);
                const double dx = s.x - ref[t + 1].x, dy = s.y - ref[t + 1].y;
                max_lat = std::max(max_lat,
                                   std::abs(-std::sin(ref[t + 1].yaw) * dx + std::cos(ref[t + 1].yaw) * dy));
            }
            accumulate_cost_terms(s, ref[p_.N], true, ct);
            diag_.c_lat = ct.lat; diag_.c_lon = ct.lon; diag_.c_yaw = ct.yaw;
            diag_.c_v = ct.v; diag_.c_bnd = ct.bnd; diag_.c_du = ct.du;
            diag_.nominal_max_lat = max_lat;
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

    // 진단용 항목별 비용 누산기 (stage_cost와 **같은 수식**을 항목별로 쪼갠 것).
    struct CostTerms { double lat = 0.0, lon = 0.0, yaw = 0.0, v = 0.0, bnd = 0.0, du = 0.0; };

    void accumulate_cost_terms(const MppiState& s, const MppiRef& r, bool terminal,
                               CostTerms& ct) const {
        const double tw = terminal ? p_.w_terminal : 1.0;
        const double dx = s.x - r.x, dy = s.y - r.y;
        const double sy = std::sin(r.yaw), cy = std::cos(r.yaw);
        const double e_lat = -sy * dx + cy * dy;
        const double e_lon =  cy * dx + sy * dy;
        ct.lat += tw * p_.w_lat * (e_lat * e_lat);
        ct.lon += tw * p_.w_lon * (e_lon * e_lon);
        const double eyaw = wrap_pi(s.yaw - r.yaw);
        ct.yaw += tw * p_.w_yaw * (eyaw * eyaw);
        const double ev = s.vx - r.v;
        ct.v += p_.w_v * (ev * ev);
        ct.bnd += boundary_cost(e_lat, r);
    }

    // 비대칭 트랙 경계 소프트 페널티. e_lat>0 = 경로 기준 **좌측**(법선 n=(-sin ψ, cos ψ)).
    double boundary_cost(double e_lat, const MppiRef& r) const {
        const double lim_l = std::max(0.0, r.d_left  - p_.margin);
        const double lim_r = std::max(0.0, r.d_right - p_.margin);
        const double over = (e_lat > lim_l) ? (e_lat - lim_l)
                          : ((-e_lat > lim_r) ? (-e_lat - lim_r) : 0.0);
        return (over > 0.0) ? p_.w_boundary * over * over : 0.0;
    }

    // 스테이지 비용: 컨투어링(횡/종 분해) 추종 + 헤딩 + 속도 + 비대칭 경계 페널티.
    double stage_cost(const MppiState& s, const MppiRef& r, bool terminal) const {
        const double tw = terminal ? p_.w_terminal : 1.0;
        const double dx = s.x - r.x;
        const double dy = s.y - r.y;

        // 경로 접선 기준 분해: 법선 n=(-sin ψ, cos ψ), 접선 t=(cos ψ, sin ψ)
        const double sy = std::sin(r.yaw), cy = std::cos(r.yaw);
        const double e_lat = -sy * dx + cy * dy;   // 횡(컨투어링) 오차
        const double e_lon =  cy * dx + sy * dy;   // 진행방향(lag) 오차

        double c = 0.0;
        c += tw * p_.w_lat * (e_lat * e_lat);                  // 횡오차 — 주력
        c += tw * p_.w_lon * (e_lon * e_lon);                  // 진행방향 — 약하게
        const double eyaw = wrap_pi(s.yaw - r.yaw);
        c += tw * p_.w_yaw * (eyaw * eyaw);                    // 헤딩
        const double ev = s.vx - r.v;
        c += p_.w_v * (ev * ev);                               // 속도 추종
        c += boundary_cost(e_lat, r);                          // 트랙 경계

        return c;
    }

    MppiParams p_;
    std::vector<MppiControl> U_;                     // nominal 제어열(길이 N, warm-start)
    std::vector<std::vector<MppiControl>> eps_;      // 잡음버퍼 [K][N] (재사용)
    std::vector<double> costs_;                      // [K] 비용→가중치 재사용
    MppiControl last_out_{};
    std::mt19937 rng_;
    double last_solve_ms_ = 0.0;
    MppiDiag diag_;
};

}  // namespace f1tenth_control


// ============================================================================
// 스탠드얼론 스모크 테스트 (ROS 무관, 알고리즘 폐루프 검증)
//   빌드: g++ -O2 -std=c++17 -DMPPI_SMOKE_TEST control_code/control_mppi_solver_cpu.cpp -o /tmp/mppi_smoke
//   static lib 빌드(colcon) 시엔 매크로 미정의라 main 없음.
// ============================================================================
#ifdef MPPI_SMOKE_TEST
#include <cstdio>
#include <cstdlib>

int main() {
    using namespace f1tenth_control;

    MppiParams p;                 // 기본값 (환경변수로 개별 오버라이드 가능 — 아래 참고)

    // 파라미터 스윕용 환경변수 오버라이드. gym/ROS를 띄우지 않고 비용 정식화·가중치를
    // 초 단위로 반복 시험하기 위한 것(시뮬 폐루프 1회가 2분, 여기는 1초).
    //   예) W_LAT=80 W_DSTEER=20 LAMBDA_REL=0.3 /tmp/mppi_smoke
    auto envd = [](const char* k, double d) { const char* v = std::getenv(k); return v ? std::atof(v) : d; };
    p.lambda_rel  = envd("LAMBDA_REL",  p.lambda_rel);
    p.lambda      = envd("LAMBDA",      p.lambda);
    p.noise_beta  = envd("NOISE_BETA",  p.noise_beta);
    p.sigma_steer = envd("SIGMA_STEER", p.sigma_steer);
    p.sigma_accel = envd("SIGMA_ACCEL", p.sigma_accel);
    p.w_lat       = envd("W_LAT",       p.w_lat);
    p.w_lon       = envd("W_LON",       p.w_lon);
    p.w_yaw       = envd("W_YAW",       p.w_yaw);
    p.w_v         = envd("W_V",         p.w_v);
    p.w_dsteer    = envd("W_DSTEER",    p.w_dsteer);
    p.w_daccel    = envd("W_DACCEL",    p.w_daccel);
    p.w_terminal  = envd("W_TERMINAL",  p.w_terminal);
    p.u_smooth    = envd("U_SMOOTH",    p.u_smooth);
    p.accel_max   = envd("ACCEL_MAX",   p.accel_max);
    p.N           = static_cast<int>(envd("N", p.N));
    p.K           = static_cast<int>(envd("K", p.K));

    MPPIController mppi(p);

    // 합성 기준경로: 반지름 R 원호(반시계). 목표속도 일정.
    const double R = 6.0;         // [m]
    const double v_ref = 3.0;     // [m/s]
    const double track_halfwidth = 1.2;

    // 현재 진행각 θ0에서 시작해 호 길이 v_ref·dt 간격으로 N+1개 기준점 생성
    // ⚠️ 수평 간격은 **차량 현재 속도**로 잡는다 — control_mppi_node의 build_reference와
    //    동일한 규약(2026-07-22). 프로파일 속도로 잡으면 차가 못 따라갈 때 기준점이 달아나
    //    lag 오차가 쌓이고, 이 하네스가 노드와 다른 것을 시험하게 된다.
    auto build_ref = [&](double theta0, double vx0) {
        std::vector<MppiRef> ref;
        ref.reserve(p.N + 1);
        double vh = std::max(1.0, vx0), th = theta0;
        for (int t = 0; t <= p.N; ++t) {
            MppiRef r;
            r.x = R * std::cos(th);
            r.y = R * std::sin(th);
            r.yaw = wrap_pi(th + M_PI / 2.0);  // 반시계 접선방향
            r.v = v_ref;
            r.d_left = track_halfwidth;
            r.d_right = track_halfwidth;
            ref.push_back(r);
            // 다음 스테이지: 도달가능 가속으로 프로파일 속도까지 램프시킨 간격
            vh = std::min(v_ref, vh + p.accel_max * p.dt);
            th += (vh * p.dt) / R;
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
    int flips = 0;               // 조향 부호전환 횟수 — 채터링 정량 지표
    double prev_steer = 0.0;

    for (int i = 0; i < STEPS; ++i) {
        auto ref = build_ref(theta_of(s), s.vx);
        MppiControl u;
        bool ok = mppi.solve(s, ref, u);
        (void)ok;

        if (!(std::isfinite(u.steer) && std::isfinite(u.accel))) finite_ok = false;
        if (std::abs(u.steer) > p.steer_max + 1e-6 ||
            u.accel > p.accel_max + 1e-6 || u.accel < p.accel_min - 1e-6) control_ok = false;

        if (i > 0 && u.steer * prev_steer < 0.0 &&
            std::abs(u.steer - prev_steer) > 0.02) ++flips;
        prev_steer = u.steer;

        // 컨트롤러와 동일한 동역학으로 되먹임(폐루프)
        s = mppi.propagate(s, u);

        const double lat = std::abs(std::hypot(s.x, s.y) - R);  // 원에서의 반경오차
        max_lat = std::max(max_lat, lat);
        worst_ms = std::max(worst_ms, mppi.last_solve_ms());
        if (i == 0) lat_first = lat;
        lat_last = lat;
        if (std::getenv("TRACE") && (i % 25) == 0)
            std::printf("  t=%4.1fs vx=%5.2f lat=%5.3f steer=%6.3f accel=%6.2f\n",
                        i * p.dt, s.vx, lat, u.steer, u.accel);
    }

    std::printf("== MPPI smoke test ==\n");
    std::printf("lateral error: first=%.3f m  last=%.3f m  (max=%.3f)\n", lat_first, lat_last, max_lat);
    std::printf("final speed  : %.3f m/s (target %.1f)\n", s.vx, v_ref);
    std::printf("worst solve  : %.2f ms  (budget 20.0 ms @50Hz)\n", worst_ms);
    std::printf("controls in-limit: %s | finite: %s\n",
                control_ok ? "OK" : "FAIL", finite_ok ? "OK" : "FAIL");
    {   // 마지막 사이클의 솔버 진단 — 가중치가 어디로 쏠렸는지 숫자로 본다
        const MppiDiag& d = mppi.last_diag();
        std::printf("diag: ESS=%.1f/%d fin=%d lam_eff=%.2f | C[lat=%.1f lon=%.1f yaw=%.1f v=%.1f bnd=%.1f du=%.1f] max_lat=%.2f\n",
                    d.ess, p.K, d.n_finite, d.lambda_eff,
                    d.c_lat, d.c_lon, d.c_yaw, d.c_v, d.c_bnd, d.c_du, d.nominal_max_lat);
        std::printf("steer 부호전환/100스텝: %.1f (채터링 지표)\n", flips * 100.0 / STEPS);
    }

    const bool converged = (lat_last < lat_first) && (lat_last < 0.5);
    const bool rt_ok = worst_ms < 20.0;
    std::printf("converged: %s | realtime: %s\n",
                converged ? "OK" : "FAIL", rt_ok ? "OK" : "FAIL");
    std::printf("RESULT: %s\n",
                (converged && rt_ok && control_ok && finite_ok) ? "PASS" : "CHECK");
    return (converged && rt_ok && control_ok && finite_ok) ? 0 : 1;
}
#endif  // MPPI_SMOKE_TEST
