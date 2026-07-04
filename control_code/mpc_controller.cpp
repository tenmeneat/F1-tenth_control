// ============================================================================
// mpc_controller.cpp — 전역좌표 Kinematic Bicycle LTV-MPC (조향 전용) 구현
// ----------------------------------------------------------------------------
// 결정변수  z = [x_0..x_N (각 [X,Y,ψ]),  u_0..u_{N-1} (각 δ)]   (총 4N+3)
// 제약행    [초기조건 3] + [동역학 3N] + [입력한계 N] + [조향율 N]  (총 5N+3)
//
// 비용(½zᵀPz + qᵀz):  Σ_k (x_k-x_ref,k)ᵀ Q (x_k-x_ref,k)
//                    + Σ_k [ r·δ_k² + r_d·(δ_k-δ_{k-1})² ] , δ_{-1}=last_delta
//
// 기준궤적 주위 1차 선형화(δ_ref=0):
//   X_{k+1} = X_k + Ts·v·cosψ ,  Y_{k+1} = Y_k + Ts·v·sinψ ,  ψ_{k+1} = ψ_k + (Ts·v/L)·δ_k
//   A_k = [[1,0,-Ts v sinψ],[0,1,Ts v cosψ],[0,0,1]] ,  B_k = [0,0,Ts v/L]ᵀ
//   C_k = Ts v [ cosψ + ψ·sinψ ,  sinψ - ψ·cosψ ,  0 ]ᵀ   (affine 보정)
//
// OSQP C API 직접 사용. P는 상수라 setup 시 1회 구성, A/q/l/u만 매 사이클 갱신.
// ============================================================================

#include "f1tenth_control/mpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <osqp.h>  // osqp-vendor: 헤더가 include/osqp 에 위치 (타겟이 경로 전파)

namespace f1tenth_control {

namespace {

// 희소행렬 조립용 삼중항 (행, 열, 값)
struct Trip {
    c_int   r;
    c_int   c;
    c_float v;
};

// 삼중항 → CSC(열우선) 변환. 중복 (행,열) 없음을 가정.
// 정렬 순서가 결정적이므로 매 사이클 동일 패턴에 대해 값 배열 정렬 순서가 일치한다.
void tripletsToCSC(c_int n,
                   std::vector<Trip>& T,
                   std::vector<c_int>& p,
                   std::vector<c_int>& i,
                   std::vector<c_float>& x) {
    std::sort(T.begin(), T.end(), [](const Trip& a, const Trip& b) {
        return (a.c < b.c) || (a.c == b.c && a.r < b.r);
    });
    p.assign(static_cast<size_t>(n) + 1, 0);
    i.resize(T.size());
    x.resize(T.size());
    for (size_t k = 0; k < T.size(); ++k) {
        i[k] = T[k].r;
        x[k] = T[k].v;
        p[static_cast<size_t>(T[k].c) + 1]++;
    }
    for (c_int c = 0; c < n; ++c) {
        p[static_cast<size_t>(c) + 1] += p[static_cast<size_t>(c)];
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// PIMPL: OSQP/CSC 상태 은닉
// ---------------------------------------------------------------------------
struct MPCController::Impl {
    const MpcParams p;

    // 차원
    const int nx = 3;
    const int nu = 1;
    int nz = 0;   // 결정변수 수
    int nc = 0;   // 제약 행 수

    // OSQP 자료구조
    OSQPWorkspace* work = nullptr;
    csc P_csc{};
    csc A_csc{};
    std::vector<c_int>   P_p, P_i, A_p, A_i;
    std::vector<c_float> P_x, A_x;
    std::vector<c_float> q, l, u;
    bool setup_done = false;

    double last_solve_ms = 0.0;

    explicit Impl(const MpcParams& params) : p(params) {
        nz = nx * (p.N + 1) + nu * p.N;
        nc = 3 + 3 * p.N + p.N + p.N;  // = 5N+3
    }

    ~Impl() {
        if (work) {
            osqp_cleanup(work);
            work = nullptr;
        }
    }

    // 결정변수 인덱스
    inline c_int xidx(int k, int d) const { return static_cast<c_int>(nx * k + d); }
    inline c_int uidx(int k) const { return static_cast<c_int>(nx * (p.N + 1) + k); }

    // 상수 비용행렬 P(상삼각) 1회 구성
    void buildP() {
        std::vector<Trip> T;
        const double qd[3] = {p.q_x, p.q_y, p.q_yaw};
        // 상태 가중 (대각): P = 2·q_d
        for (int k = 0; k <= p.N; ++k) {
            for (int d = 0; d < nx; ++d) {
                T.push_back({xidx(k, d), xidx(k, d), 2.0 * qd[d]});
            }
        }
        // 입력 가중 (조향율 페널티 → 입력 블록 삼중대각)
        for (int k = 0; k < p.N; ++k) {
            // δ_k² 계수: r(크기) + r_d(rate항 k) + r_d(rate항 k+1, 존재 시)
            double diag = 2.0 * p.r + 2.0 * p.r_delta;        // r + rate항 k
            if (k <= p.N - 2) diag += 2.0 * p.r_delta;        // + rate항 k+1
            T.push_back({uidx(k), uidx(k), diag});
            // 교차항 δ_k·δ_{k-1} 계수 = -2·r_d (상삼각: 행 uidx(k-1) < 열 uidx(k))
            if (k >= 1) {
                T.push_back({uidx(k - 1), uidx(k), -2.0 * p.r_delta});
            }
        }
        tripletsToCSC(static_cast<c_int>(nz), T, P_p, P_i, P_x);
        P_csc.m = nz;
        P_csc.n = nz;
        P_csc.nz = -1;
        P_csc.nzmax = static_cast<c_int>(P_x.size());
        P_csc.p = P_p.data();
        P_csc.i = P_i.data();
        P_csc.x = P_x.data();
    }

    // 매 사이클: 선형화 동역학으로 A 삼중항 + l/u + q 생성
    void buildAqlu(const MpcState& cur, const std::vector<MpcRef>& ref, double last_delta,
                   std::vector<Trip>& T) {
        const double Ts = p.Ts;
        const double L  = p.wheelbase;
        const double qd[3] = {p.q_x, p.q_y, p.q_yaw};

        l.assign(static_cast<size_t>(nc), 0.0);
        u.assign(static_cast<size_t>(nc), 0.0);
        q.assign(static_cast<size_t>(nz), 0.0);
        T.clear();
        T.reserve(static_cast<size_t>(3 + 9 * p.N + p.N + 2 * p.N));

        // --- 비용 선형항 q ---
        for (int k = 0; k <= p.N; ++k) {
            q[static_cast<size_t>(xidx(k, 0))] = -2.0 * qd[0] * ref[k].x;
            q[static_cast<size_t>(xidx(k, 1))] = -2.0 * qd[1] * ref[k].y;
            q[static_cast<size_t>(xidx(k, 2))] = -2.0 * qd[2] * ref[k].yaw;
        }
        // 조향율 페널티의 last_delta 항: -2·r_d·last_delta (δ_0에만)
        q[static_cast<size_t>(uidx(0))] = -2.0 * p.r_delta * last_delta;

        // --- (1) 초기조건: x_0 = cur ---
        const double cur3[3] = {cur.x, cur.y, cur.yaw};
        for (int d = 0; d < nx; ++d) {
            T.push_back({static_cast<c_int>(d), xidx(0, d), 1.0});
            l[static_cast<size_t>(d)] = cur3[d];
            u[static_cast<size_t>(d)] = cur3[d];
        }

        // --- (2) 동역학 등식: x_{k+1} - A_k x_k - B_k u_k = C_k ---
        const c_int dyn_base = 3;
        for (int k = 0; k < p.N; ++k) {
            const double psi = ref[k].yaw;
            const double v   = ref[k].v;
            const double s = std::sin(psi);
            const double c = std::cos(psi);
            const double Tv = Ts * v;

            const c_int r0 = dyn_base + 3 * k + 0;
            const c_int r1 = dyn_base + 3 * k + 1;
            const c_int r2 = dyn_base + 3 * k + 2;

            // row0 (X): x_{k+1,0} - X_k - (-Tv·s)ψ_k = C0
            T.push_back({r0, xidx(k + 1, 0), 1.0});
            T.push_back({r0, xidx(k, 0), -1.0});
            T.push_back({r0, xidx(k, 2), Tv * s});            // -A[0,2] = -(-Tv s)
            // row1 (Y): x_{k+1,1} - Y_k - (Tv·c)ψ_k = C1
            T.push_back({r1, xidx(k + 1, 1), 1.0});
            T.push_back({r1, xidx(k, 1), -1.0});
            T.push_back({r1, xidx(k, 2), -Tv * c});           // -A[1,2] = -(Tv c)
            // row2 (ψ): x_{k+1,2} - ψ_k - (Tv/L)δ_k = C2
            T.push_back({r2, xidx(k + 1, 2), 1.0});
            T.push_back({r2, xidx(k, 2), -1.0});
            T.push_back({r2, uidx(k), -Tv / L});              // -B[2]

            // C_k (등식이므로 l=u=C_k)
            const double C0 = Tv * (c + psi * s);
            const double C1 = Tv * (s - psi * c);
            const double C2 = 0.0;
            l[static_cast<size_t>(r0)] = u[static_cast<size_t>(r0)] = C0;
            l[static_cast<size_t>(r1)] = u[static_cast<size_t>(r1)] = C1;
            l[static_cast<size_t>(r2)] = u[static_cast<size_t>(r2)] = C2;
        }

        // --- (3) 입력 크기 한계: -δ_max ≤ δ_k ≤ δ_max ---
        const c_int ub_base = 3 + 3 * p.N;
        for (int k = 0; k < p.N; ++k) {
            const c_int row = ub_base + k;
            T.push_back({row, uidx(k), 1.0});
            l[static_cast<size_t>(row)] = -p.delta_max;
            u[static_cast<size_t>(row)] = p.delta_max;
        }

        // --- (4) 조향율 한계: -Δδ_max ≤ δ_k - δ_{k-1} ≤ Δδ_max (δ_{-1}=last_delta) ---
        const c_int rt_base = 3 + 3 * p.N + p.N;
        for (int k = 0; k < p.N; ++k) {
            const c_int row = rt_base + k;
            T.push_back({row, uidx(k), 1.0});
            if (k == 0) {
                l[static_cast<size_t>(row)] = last_delta - p.ddelta_max;
                u[static_cast<size_t>(row)] = last_delta + p.ddelta_max;
            } else {
                T.push_back({row, uidx(k - 1), -1.0});
                l[static_cast<size_t>(row)] = -p.ddelta_max;
                u[static_cast<size_t>(row)] = p.ddelta_max;
            }
        }
    }

    bool setup() {
        buildP();

        std::vector<Trip> T;
        // 초기 setup 시 A 패턴/값 확보를 위해 더미 호출은 불필요 —
        // solve()에서 이미 T를 채운 뒤 setup을 부른다.
        (void)T;

        OSQPSettings settings;
        osqp_set_default_settings(&settings);
        settings.warm_start = 1;
        settings.verbose = 0;
        settings.polish = 0;
        settings.max_iter = p.osqp_max_iter;
        settings.eps_abs = p.osqp_eps_abs;
        settings.eps_rel = p.osqp_eps_rel;

        OSQPData data;
        data.n = nz;
        data.m = nc;
        data.P = &P_csc;
        data.A = &A_csc;
        data.q = q.data();
        data.l = l.data();
        data.u = u.data();

        const c_int ret = osqp_setup(&work, &data, &settings);
        return (ret == 0 && work != nullptr);
    }

    bool solve(const MpcState& cur, const std::vector<MpcRef>& ref, double last_delta,
               double& out_delta) {
        if (static_cast<int>(ref.size()) < p.N + 1) {
            return false;
        }

        const auto t0 = std::chrono::steady_clock::now();

        std::vector<Trip> T;
        buildAqlu(cur, ref, last_delta, T);
        tripletsToCSC(static_cast<c_int>(nz), T, A_p, A_i, A_x);
        A_csc.m = nc;
        A_csc.n = nz;
        A_csc.nz = -1;
        A_csc.nzmax = static_cast<c_int>(A_x.size());
        A_csc.p = A_p.data();
        A_csc.i = A_i.data();
        A_csc.x = A_x.data();

        if (!setup_done) {
            if (!setup()) {
                return false;
            }
            setup_done = true;
        } else {
            // 패턴 동일, 값만 갱신
            osqp_update_A(work, A_x.data(), OSQP_NULL, static_cast<c_int>(A_x.size()));
            osqp_update_lin_cost(work, q.data());
            osqp_update_bounds(work, l.data(), u.data());
        }

        const c_int ret = osqp_solve(work);
        const auto t1 = std::chrono::steady_clock::now();
        last_solve_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (ret != 0 || work->info == nullptr) {
            return false;
        }
        const c_int st = work->info->status_val;
        if (st != OSQP_SOLVED && st != OSQP_SOLVED_INACCURATE) {
            return false;
        }
        const double d0 = work->solution->x[static_cast<size_t>(uidx(0))];
        if (!std::isfinite(d0)) {
            return false;
        }
        out_delta = std::max(-p.delta_max, std::min(d0, p.delta_max));
        return true;
    }
};

// ---------------------------------------------------------------------------
// 공개 인터페이스
// ---------------------------------------------------------------------------
MPCController::MPCController(const MpcParams& params)
    : params_(params), impl_(std::make_unique<Impl>(params)) {}

MPCController::~MPCController() = default;

bool MPCController::solve(const MpcState& cur, const std::vector<MpcRef>& ref,
                         double last_delta, double& out_delta) {
    return impl_->solve(cur, ref, last_delta, out_delta);
}

double MPCController::last_solve_ms() const { return impl_->last_solve_ms; }

}  // namespace f1tenth_control
