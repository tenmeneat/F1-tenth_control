// ============================================================================
// control_mppi_solver_gpu.cu — MPPI 롤아웃 코어의 CUDA GPU 구현
// ----------------------------------------------------------------------------
// control_mppi_solver_cpu.cpp(CPU, double, 순차)의 알고리즘을 GPU로 포팅한 것.
//   수학/구조(동역학 자전거+Pacejka, 저속 기구학 블렌드, 정보이론 MPPI 가중 갱신,
//   warm-start)는 CPU 레퍼런스와 동일하게 유지하되, K개 롤아웃을 GPU에서 병렬 실행한다.
//
// 병렬화 구조:
//   1) rollout_kernel      — 롤아웃당 스레드 1개(k∈[0,K)). N-스텝은 상태 의존성이 있어
//                            스레드 내부에서 순차 처리. 잡음은 스레드별 영속 curand
//                            Philox 스트림(생성자에서 1회 init, 매 cycle 재사용).
//   2) β=min, w=exp, η=sum — thrust::reduce/transform (K=512라 레이턴시 바운드, 커스텀
//                            리덕션 불필요).
//   3) weighted_update_kernel — 타임스텝당 블록 1개(t∈[0,N)), 블록 내 K 스레드가 공유
//                            메모리 트리 리덕션으로 U_t += Σ_k w_k·ε_{k,t}.
//
// eps 버퍼는 [t*K + k] 레이아웃 → weighted_update_kernel(리덕션)의 접근이 coalesced.
// (rollout_kernel의 eps 쓰기는 stride-K가 되지만, 그 커널은 동역학 연산이 지배적이라 무방.)
//
// GPU 쪽은 float32(정밀도 결정: 소비자/임베디드 GPU FP64 처리량 열세 회피). CPU
// 레퍼런스(control_mppi_solver_cpu.cpp)의 double 구현은 그대로 두고 이 파일과 독립.
//
// 단일 파일 관용구(control_map_node.cpp/control_mppi_solver_cpu.cpp와 동일): 파일 하단
// #ifdef MPPI_GPU_SMOKE_TEST 블록으로 ROS 없이 폐루프 검증 가능.
//   빌드: nvcc -O3 -std=c++17 -arch=sm_89 -Iinclude -DMPPI_GPU_SMOKE_TEST \
//         control_code/control_mppi_solver_gpu.cu -o /tmp/mppi_gpu_smoke
// ============================================================================

#include "f1tenth_control/mppi_gpu.hpp"
#include "f1tenth_control/mppi_types_gpu.hpp"

#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include <curand_kernel.h>
#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/fill.h>
#include <thrust/functional.h>
#include <thrust/reduce.h>
#include <thrust/transform.h>
#include <thrust/transform_reduce.h>

namespace f1tenth_control {
namespace {

// CUDA 호출 에러를 stderr에 보고(치명 오류 조기 발견용). 실시간 루프 성능엔 영향 없음.
inline void cuda_check(cudaError_t e, const char* what) {
    if (e != cudaSuccess) {
        std::fprintf(stderr, "[mppi_gpu] CUDA 오류 (%s): %s\n", what, cudaGetErrorString(e));
    }
}

template <typename T>
inline T* raw(thrust::device_vector<T>& v) { return thrust::raw_pointer_cast(v.data()); }

// ---------------------------------------------------------------------------
// 디바이스 동역학/비용 (control_mppi_solver_cpu.cpp의 step_dynamics/stage_cost float 포팅)
// ---------------------------------------------------------------------------

// 전방 동역학 1스텝(내부 substeps회 적분). 동역학 자전거+Pacejka, 저속은 기구학 블렌드.
__device__ inline MppiStateF step_dynamics_f(const MppiStateF& s0, const MppiControlF& u,
                                             const MppiParamsF& p) {
    const float L   = p.wheelbase;
    const int   sub = p.substeps < 1 ? 1 : p.substeps;
    const float h   = p.dt / sub;
    const float Fzf = p.mass * p.g * p.lr / L;
    const float Fzr = p.mass * p.g * p.lf / L;
    const float Df  = p.mu * Fzf;
    const float Dr  = p.mu * Fzr;
    const float delta = clampf(u.steer, -p.steer_max, p.steer_max);
    const float accel = clampf(u.accel,  p.accel_min, p.accel_max);

    MppiStateF s = s0;
    for (int i = 0; i < sub; ++i) {
        const float half  = 0.5f * p.v_switch;
        const float blend = clampf((s.vx - half) / fmaxf(1e-3f, p.v_switch - half), 0.f, 1.f);

        float dvy_dyn = 0.f, dw_dyn = 0.f;
        {
            const float vx_safe = fmaxf(s.vx, 1e-3f);
            const float af = delta - atan2f(s.vy + p.lf * s.yaw_rate, vx_safe);
            const float ar =        -atan2f(s.vy - p.lr * s.yaw_rate, vx_safe);
            const float Fyf = Df * pacejka_mu_f(af, p.Bf, p.Cf, p.Ef);
            const float Fyr = Dr * pacejka_mu_f(ar, p.Br, p.Cr, p.Er);
            dvy_dyn = (Fyf * cosf(delta) + Fyr) / p.mass - s.vx * s.yaw_rate;
            dw_dyn  = (p.lf * Fyf * cosf(delta) - p.lr * Fyr) / p.Iz;
        }

        const float w_kin  = s.vx * tanf(delta) / L;
        const float vy_kin = p.lr * w_kin;
        const float dvx    = accel;

        if (blend >= 1.f) {
            s.vy       += h * dvy_dyn;
            s.yaw_rate += h * dw_dyn;
        } else if (blend <= 0.f) {
            s.vy = vy_kin;
            s.yaw_rate = w_kin;
        } else {
            const float vy_dyn_next = s.vy + h * dvy_dyn;
            const float w_dyn_next  = s.yaw_rate + h * dw_dyn;
            s.vy       = blend * vy_dyn_next + (1.f - blend) * vy_kin;
            s.yaw_rate = blend * w_dyn_next  + (1.f - blend) * w_kin;
        }

        const float cyaw = cosf(s.yaw), syaw = sinf(s.yaw);
        s.x   += h * (s.vx * cyaw - s.vy * syaw);
        s.y   += h * (s.vx * syaw + s.vy * cyaw);
        s.yaw  = wrap_pi_f(s.yaw + h * s.yaw_rate);
        s.vx   = clampf(s.vx + h * dvx, p.v_min, p.v_max);
    }
    return s;
}

// 비대칭 트랙 경계 소프트 페널티. e_lat>0 = 경로 기준 좌측(법선 n=(-sin ψ, cos ψ)).
__device__ inline float boundary_cost_f(float e_lat, const MppiRefF& r, const MppiParamsF& p) {
    const float lim_l = fmaxf(0.f, r.d_left  - p.margin);
    const float lim_r = fmaxf(0.f, r.d_right - p.margin);
    const float over = (e_lat > lim_l) ? (e_lat - lim_l)
                     : ((-e_lat > lim_r) ? (-e_lat - lim_r) : 0.f);
    return (over > 0.f) ? p.w_boundary * over * over : 0.f;
}

// 스테이지 비용: 컨투어링(횡/종 분해) 추종 + 헤딩 + 속도 + 비대칭 경계.
// (CPU 레퍼런스 control_mppi_solver_cpu.cpp의 stage_cost와 동일 수식)
__device__ inline float stage_cost_f(const MppiStateF& s, const MppiRefF& r, bool terminal,
                                     const MppiParamsF& p) {
    const float tw = terminal ? p.w_terminal : 1.f;
    const float dx = s.x - r.x;
    const float dy = s.y - r.y;

    const float sy = sinf(r.yaw), cy = cosf(r.yaw);
    const float e_lat = -sy * dx + cy * dy;   // 횡(컨투어링) 오차
    const float e_lon =  cy * dx + sy * dy;   // 진행방향(lag) 오차

    float c = 0.f;
    c += tw * p.w_lat * (e_lat * e_lat);
    c += tw * p.w_lon * (e_lon * e_lon);
    const float eyaw = wrap_pi_f(s.yaw - r.yaw);
    c += tw * p.w_yaw * (eyaw * eyaw);
    const float ev = s.vx - r.v;
    c += p.w_v * (ev * ev);
    c += boundary_cost_f(e_lat, r, p);

    return c;
}

// ---------------------------------------------------------------------------
// 커널
// ---------------------------------------------------------------------------

// 스레드별 curand Philox 스트림 초기화(생성자/reset 아님, 최초 1회). subsequence=k로
// 스레드마다 통계적으로 독립인 스트림 확보(단일 시드에서).
__global__ void init_rng_kernel(curandStatePhilox4_32_10_t* st, int K, unsigned long long seed) {
    const int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k < K) curand_init(seed, k, 0, &st[k]);
}

// (1) 롤아웃당 스레드 1개: 잡음 샘플 → N-스텝 전진 → 비용 누적. eps[t*K+k], costs[k] 기록.
__global__ void rollout_kernel(MppiStateF cur,
                               const MppiRefF* __restrict__ ref,      // [N+1]
                               const MppiControlF* __restrict__ U,    // [N] nominal
                               MppiControlF* __restrict__ eps,        // [N*K], [t*K+k]
                               float* __restrict__ costs,             // [K]
                               curandStatePhilox4_32_10_t* rng,       // [K] 영속
                               MppiControlF last_out,                 // Δu 비용의 t=0 기준
                               MppiParamsF p) {
    const int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= p.K) return;

    curandStatePhilox4_32_10_t st = rng[k];  // 영속 상태를 로컬로 복사
    MppiStateF s = cur;
    float Jk = 0.f;

    // AR(1) 시간상관 잡음: z_t = β·z_{t-1} + √(1−β²)·n_t (정상 N(0,1) 유지).
    // 백색잡음이면 고주파로 진동하는 제어열이 샘플 풀을 채우고, 그게 뽑히면 채터링이 된다.
    const float nb = clampf(p.noise_beta, 0.f, 0.99f);
    const float na = sqrtf(fmaxf(0.f, 1.f - nb * nb));
    float zs = 0.f, za = 0.f;
    MppiControlF prev = last_out;   // Δu 기준: t=0은 직전에 실제로 출력한 제어

    for (int t = 0; t < p.N; ++t) {
        // 시간상관 잡음 → 제어 v = u_t + ε, 한계 클램프
        const float2 n = curand_normal2(&st);  // (steer, accel) 정규난수 한 쌍
        zs = nb * zs + na * n.x;
        za = nb * za + na * n.y;
        MppiControlF v;
        v.steer = clampf(U[t].steer + zs * p.sigma_steer, -p.steer_max, p.steer_max);
        v.accel = clampf(U[t].accel + za * p.sigma_accel,  p.accel_min, p.accel_max);
        // 클램프로 실제 적용된 제어와 명목의 차이를 잡음으로 재정의(경계에서 일관성)
        MppiControlF e;
        e.steer = v.steer - U[t].steer;
        e.accel = v.accel - U[t].accel;
        eps[t * p.K + k] = e;

        // 제어 변화율 비용(채터링 억제)
        const float dsteer = v.steer - prev.steer;
        const float daccel = v.accel - prev.accel;
        Jk += p.w_dsteer * dsteer * dsteer + p.w_daccel * daccel * daccel;
        prev = v;

        s = step_dynamics_f(s, v, p);
        Jk += stage_cost_f(s, ref[t + 1], false, p);
    }
    Jk += stage_cost_f(s, ref[p.N], true, p);  // 종단 비용

    // NaN/Inf 방어 → 해당 샘플 사실상 배제(FLT_MAX; 이후 β=min 감산에서 inf-inf 회피)
    costs[k] = isfinite(Jk) ? Jk : FLT_MAX;
    rng[k] = st;  // 전진된 상태를 다음 cycle 위해 되돌려 저장
}

// 진단 전용: 갱신된 U를 잡음 없이 1회 롤아웃해 항목별 비용/최대 횡오차를 뽑는다(스레드 1개).
// out[0..5]=lat,lon,yaw,v,bnd,du, out[6]=max|e_lat|
__global__ void diag_kernel(MppiStateF cur,
                            const MppiRefF* __restrict__ ref,
                            const MppiControlF* __restrict__ U,
                            MppiControlF last_out,
                            MppiParamsF p,
                            float* __restrict__ out) {
    float lat = 0.f, lon = 0.f, yw = 0.f, cv = 0.f, bnd = 0.f, du = 0.f, maxlat = 0.f;
    MppiStateF s = cur;
    MppiControlF prev = last_out;
    for (int t = 0; t <= p.N; ++t) {
        if (t < p.N) {
            const float ds_ = U[t].steer - prev.steer;
            const float da_ = U[t].accel - prev.accel;
            du += p.w_dsteer * ds_ * ds_ + p.w_daccel * da_ * da_;
            prev = U[t];
            s = step_dynamics_f(s, U[t], p);
        }
        const MppiRefF r = ref[t < p.N ? t + 1 : p.N];
        const float tw = (t == p.N) ? p.w_terminal : 1.f;
        const float dx = s.x - r.x, dy = s.y - r.y;
        const float sy = sinf(r.yaw), cy = cosf(r.yaw);
        const float e_lat = -sy * dx + cy * dy;
        const float e_lon =  cy * dx + sy * dy;
        lat += tw * p.w_lat * e_lat * e_lat;
        lon += tw * p.w_lon * e_lon * e_lon;
        const float eyaw = wrap_pi_f(s.yaw - r.yaw);
        yw  += tw * p.w_yaw * eyaw * eyaw;
        const float ev = s.vx - r.v;
        cv  += p.w_v * ev * ev;
        bnd += boundary_cost_f(e_lat, r, p);
        maxlat = fmaxf(maxlat, fabsf(e_lat));
    }
    out[0] = lat; out[1] = lon; out[2] = yw; out[3] = cv; out[4] = bnd; out[5] = du; out[6] = maxlat;
}

// 유한 비용만 골라 합/개수를 내기 위한 펑터 (FLT_MAX 사멸 샘플 제외).
struct FiniteCost {
    __host__ __device__ float operator()(float c) const { return (c < 0.5f * FLT_MAX) ? c : 0.f; }
};
struct FiniteFlag {
    __host__ __device__ float operator()(float c) const { return (c < 0.5f * FLT_MAX) ? 1.f : 0.f; }
};
struct SquareF {
    __host__ __device__ float operator()(float w) const { return w * w; }
};

// β 감산 후 정규화 전 가중치 w_k = exp(-(J_k-β)/λ) 로 변환하는 thrust 펑터.
struct ExpWeight {
    float beta, inv_lambda;
    __host__ __device__ float operator()(float c) const {
        return expf(-(c - beta) * inv_lambda);
    }
};

// (2) 타임스텝당 블록 1개: U_t += Σ_k w_k·ε_{k,t}. 블록 내 공유메모리 트리 리덕션.
//     blockDim.x = min(nextpow2(K), 1024). ⚠️ CUDA 블록당 스레드 상한이 1024라 K가 그보다
//     크면 스레드당 여러 k를 그리드-스트라이드로 먼저 누산한 뒤 트리 리덕션한다.
//     (2026-07-22: 이 상한을 안 지켜 K=2048에서 커널 런치가 조용히 실패했고, U_가 갱신되지
//      않은 채 쓰레기 제어가 나가 출발 직후 벽으로 갔다. 런치 에러 체크도 그때 함께 추가.)
__global__ void weighted_update_kernel(MppiControlF* __restrict__ U,        // [N] in/out
                                       const MppiControlF* __restrict__ eps, // [N*K], [t*K+k]
                                       const float* __restrict__ weights,    // [K] 정규화 전 w_k
                                       float inv_eta,
                                       MppiParamsF p) {
    extern __shared__ float sh[];
    float* sh_ds = sh;
    float* sh_da = sh + blockDim.x;

    const int t = blockIdx.x;    // 0..N-1
    const int k = threadIdx.x;   // 0..blockDim.x-1

    float ds = 0.f, da = 0.f;
    for (int kk = k; kk < p.K; kk += blockDim.x) {   // K > blockDim.x면 스레드당 여러 개
        const float w = weights[kk] * inv_eta;
        const MppiControlF e = eps[t * p.K + kk];
        ds += w * e.steer;
        da += w * e.accel;
    }
    sh_ds[k] = ds;
    sh_da[k] = da;
    __syncthreads();

    for (int stride = blockDim.x / 2; stride > 0; stride >>= 1) {
        if (k < stride) {
            sh_ds[k] += sh_ds[k + stride];
            sh_da[k] += sh_da[k + stride];
        }
        __syncthreads();
    }

    if (k == 0) {
        U[t].steer = clampf(U[t].steer + sh_ds[0], -p.steer_max, p.steer_max);
        U[t].accel = clampf(U[t].accel + sh_da[0],  p.accel_min, p.accel_max);
    }
}

// 단위검증/폐루프용 1스텝 전진(스레드 1개). propagate()가 호출.
__global__ void propagate_kernel(MppiStateF s, MppiControlF u, MppiParamsF p, MppiStateF* out) {
    *out = step_dynamics_f(s, u, p);
}

}  // namespace

// ---------------------------------------------------------------------------
// MPPIControllerGPU::Impl (thrust/CUDA 상태를 PImpl 뒤에 은닉)
// ---------------------------------------------------------------------------
struct MPPIControllerGPU::Impl {
    MppiParamsF p;
    thrust::device_vector<MppiControlF> d_U;        // [N] nominal (warm-start, device 상주)
    thrust::device_vector<MppiControlF> d_U_shift;  // [N] warm-start 시프트 임시버퍼
    thrust::device_vector<MppiControlF> d_eps;      // [N*K] 잡음버퍼
    thrust::device_vector<float>        d_costs;    // [K] 비용→가중치 재사용
    thrust::device_vector<MppiRefF>     d_ref;      // [N+1] 기준궤적
    thrust::device_vector<curandStatePhilox4_32_10_t> d_rng;  // [K] 영속 RNG 상태
    thrust::device_vector<MppiStateF>   d_prop;     // [1] propagate 출력
    thrust::device_vector<float>        d_diag;     // [7] 진단 커널 출력
    MppiControlF last_out{};
    MppiDiagF diag;
    double last_solve_ms = 0.0;
    int block_upd = 1;  // weighted_update_kernel의 blockDim = nextpow2(K)
};

MPPIControllerGPU::MPPIControllerGPU(const MppiParamsF& p) : impl_(new Impl) {
    Impl& im = *impl_;
    im.p = p;
    im.d_U.assign(static_cast<size_t>(p.N), MppiControlF{});
    im.d_U_shift.assign(static_cast<size_t>(p.N), MppiControlF{});
    im.d_eps.assign(static_cast<size_t>(p.N) * p.K, MppiControlF{});
    im.d_costs.assign(static_cast<size_t>(p.K), 0.f);
    im.d_ref.assign(static_cast<size_t>(p.N) + 1, MppiRefF{});
    im.d_rng.resize(static_cast<size_t>(p.K));
    im.d_prop.resize(1);
    im.d_diag.assign(7, 0.f);

    int b = 1;
    while (b < p.K && b < 1024) b <<= 1;   // CUDA 블록당 스레드 상한 1024
    im.block_upd = b;

    const int bs = 128, gs = (p.K + bs - 1) / bs;
    init_rng_kernel<<<gs, bs>>>(raw(im.d_rng), p.K, static_cast<unsigned long long>(p.seed));
    cuda_check(cudaDeviceSynchronize(), "init_rng");
}

MPPIControllerGPU::~MPPIControllerGPU() = default;

void MPPIControllerGPU::reset() {
    thrust::fill(impl_->d_U.begin(), impl_->d_U.end(), MppiControlF{});
    impl_->last_out = MppiControlF{};
}

double MPPIControllerGPU::last_solve_ms() const { return impl_->last_solve_ms; }
const MppiParamsF& MPPIControllerGPU::params() const { return impl_->p; }
const MppiDiagF& MPPIControllerGPU::last_diag() const { return impl_->diag; }

MppiStateF MPPIControllerGPU::propagate(const MppiStateF& s, const MppiControlF& u) const {
    Impl& im = *impl_;
    propagate_kernel<<<1, 1>>>(s, u, im.p, raw(im.d_prop));
    cuda_check(cudaDeviceSynchronize(), "propagate");
    MppiStateF out;
    cuda_check(cudaMemcpy(&out, raw(im.d_prop), sizeof(MppiStateF), cudaMemcpyDeviceToHost),
               "propagate copy");
    return out;
}

bool MPPIControllerGPU::solve(const MppiStateF& cur, const std::vector<MppiRefF>& ref,
                              MppiControlF& out) {
    Impl& im = *impl_;
    if (static_cast<int>(ref.size()) < im.p.N + 1) return false;

    const auto t0 = std::chrono::steady_clock::now();
    auto finalize = [&]() {
        // 런치 실패(블록 크기 초과·공유메모리 초과 등)는 동기화 전에는 안 보인다.
        // 잡지 않으면 U_가 갱신되지 않은 채 제어가 계속 나가므로 반드시 확인한다.
        cuda_check(cudaGetLastError(), "kernel launch");
        cuda_check(cudaDeviceSynchronize(), "solve sync");
        const auto t1 = std::chrono::steady_clock::now();
        im.last_solve_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    };

    // 기준궤적 H2D 업로드 (N+1개, 소량)
    cuda_check(cudaMemcpy(raw(im.d_ref), ref.data(),
                          (static_cast<size_t>(im.p.N) + 1) * sizeof(MppiRefF),
                          cudaMemcpyHostToDevice), "ref upload");

    // (1) 롤아웃 + 비용
    const int bs = 128, gs = (im.p.K + bs - 1) / bs;
    rollout_kernel<<<gs, bs>>>(cur, raw(im.d_ref), raw(im.d_U), raw(im.d_eps),
                               raw(im.d_costs), raw(im.d_rng), im.last_out, im.p);

    // (2) β = min(costs)
    const float beta = thrust::reduce(im.d_costs.begin(), im.d_costs.end(),
                                      FLT_MAX, thrust::minimum<float>());
    if (!std::isfinite(beta)) {  // 전 샘플 발산 → 갱신 없이 안전 홀드
        out = im.last_out;
        finalize();
        return false;
    }

    // (2-b) 유한 비용의 합/개수 → 평균. 적응 λ가 이걸 쓰므로 diag_enable과 무관하게 계산.
    const float sum_fin = thrust::transform_reduce(im.d_costs.begin(), im.d_costs.end(),
                                                   FiniteCost{}, 0.f, thrust::plus<float>());
    const float n_fin   = thrust::transform_reduce(im.d_costs.begin(), im.d_costs.end(),
                                                   FiniteFlag{}, 0.f, thrust::plus<float>());
    const float j_mean = (n_fin > 0.f) ? (sum_fin / n_fin) : beta;
    im.diag.j_min = beta;
    im.diag.j_mean = j_mean;
    im.diag.n_finite = static_cast<int>(n_fin);

    // 적응 역온도 λ_eff = lambda_rel·(J_mean − β). λ를 비용 스케일에 불변으로 만든다
    // (고정 λ는 w_*를 하나만 바꿔도 ESS가 1까지 무너지거나 K까지 뭉개진다).
    const float spread = fmaxf(0.f, j_mean - beta);
    const float lambda_eff = (im.p.lambda_rel > 0.f)
                                 ? fmaxf(1e-6f, im.p.lambda_rel * spread)
                                 : fmaxf(1e-6f, im.p.lambda);
    im.diag.lambda_eff = lambda_eff;

    // (3) 가중치 w_k = exp(-(J_k-β)/λ_eff) (in-place), η = Σ w_k
    thrust::transform(im.d_costs.begin(), im.d_costs.end(), im.d_costs.begin(),
                      ExpWeight{beta, 1.f / lambda_eff});
    const float eta = thrust::reduce(im.d_costs.begin(), im.d_costs.end(), 0.f,
                                     thrust::plus<float>());
    if (eta < 1e-12f) {
        out = im.last_out;
        finalize();
        return false;
    }
    const float inv_eta = 1.f / eta;

    // (4) 갱신: U_t += Σ_k w_k·ε_{k,t}
    const size_t shbytes = 2 * static_cast<size_t>(im.block_upd) * sizeof(float);
    weighted_update_kernel<<<im.p.N, im.block_upd, shbytes>>>(
        raw(im.d_U), raw(im.d_eps), raw(im.d_costs), inv_eta, im.p);

    // (4-b) 진단: ESS(유효 샘플수)와 명목 계획의 비용 구성. ESS가 K의 1~2%면 가중치 퇴화
    //       (랜덤서치=채터링), 80%↑면 뭉개짐(제어 권한 상실). 10~40%가 건강한 범위.
    if (im.p.diag_enable) {
        const float sum_w2 = thrust::transform_reduce(im.d_costs.begin(), im.d_costs.end(),
                                                      SquareF{}, 0.f, thrust::plus<float>());
        im.diag.ess = (sum_w2 > 0.f) ? static_cast<double>(eta) * eta / sum_w2 : 0.0;
        diag_kernel<<<1, 1>>>(cur, raw(im.d_ref), raw(im.d_U), im.last_out, im.p, raw(im.d_diag));
        float h[7] = {0};
        cuda_check(cudaMemcpy(h, raw(im.d_diag), sizeof(h), cudaMemcpyDeviceToHost), "diag copy");
        im.diag.c_lat = h[0]; im.diag.c_lon = h[1]; im.diag.c_yaw = h[2];
        im.diag.c_v = h[3]; im.diag.c_bnd = h[4]; im.diag.c_du = h[5];
        im.diag.nominal_max_lat = h[6];
    }

    // (5) 출력 = 첫 제어(비평활 U_0), 호스트에서 저역통과 블렌드 (CPU와 동일: U_는 불변)
    MppiControlF u0;
    cuda_check(cudaMemcpy(&u0, raw(im.d_U), sizeof(MppiControlF), cudaMemcpyDeviceToHost),
               "U0 download");
    if (im.p.u_smooth > 0.f) {
        u0.steer = im.p.u_smooth * im.last_out.steer + (1.f - im.p.u_smooth) * u0.steer;
        u0.accel = im.p.u_smooth * im.last_out.accel + (1.f - im.p.u_smooth) * u0.accel;
    }
    im.last_out = u0;
    out = u0;

    // (6) warm-start 시프트: d_U[0..N-2] = d_U[1..N-1], d_U[N-1] 유지.
    //     겹치는 in-place copy는 병렬 레이스라, 임시버퍼 경유로 안전하게 처리(비겹침 2회).
    thrust::copy(im.d_U.begin() + 1, im.d_U.end(), im.d_U_shift.begin());
    thrust::copy(im.d_U_shift.begin(), im.d_U_shift.begin() + (im.p.N - 1), im.d_U.begin());

    finalize();
    return true;
}

}  // namespace f1tenth_control


// ============================================================================
// 스탠드얼론 GPU 스모크 테스트 (ROS 무관, 알고리즘 폐루프 검증)
//   빌드: nvcc -O3 -std=c++17 -arch=sm_89 -Iinclude -DMPPI_GPU_SMOKE_TEST \
//         control_code/control_mppi_solver_gpu.cu -o /tmp/mppi_gpu_smoke
//   CPU 버전(control_mppi_solver_cpu.cpp)의 스모크 테스트와 동일 시나리오(원호 추종).
// ============================================================================
#ifdef MPPI_GPU_SMOKE_TEST

int main() {
    using namespace f1tenth_control;

    MppiParamsF p;                 // 기본값 사용
    MPPIControllerGPU mppi(p);

    // 합성 기준경로: 반지름 R 원호(반시계). 목표속도 일정.
    const float R = 6.0f;
    const float v_ref = 3.0f;
    const float track_halfwidth = 1.2f;

    // ⚠️ 수평 간격은 control_mppi_node의 build_reference와 **동일 규약**으로 만든다
    //    (현재 속도에서 시작해 도달가능 가속으로 프로파일 속도까지 램프). CPU 스모크
    //    테스트와도 같은 하네스라 두 솔버 결과를 직접 비교할 수 있다.
    auto build_ref = [&](float theta0, float vx0) {
        std::vector<MppiRefF> ref;
        ref.reserve(p.N + 1);
        float vh = std::max(1.0f, vx0), th = theta0;
        for (int t = 0; t <= p.N; ++t) {
            MppiRefF r;
            r.x = R * std::cos(th);
            r.y = R * std::sin(th);
            r.yaw = wrap_pi_f(th + static_cast<float>(M_PI) / 2.0f);
            r.v = v_ref;
            r.d_left = track_halfwidth;
            r.d_right = track_halfwidth;
            ref.push_back(r);
            vh = std::min(v_ref, vh + p.accel_max * p.dt);
            th += (vh * p.dt) / R;
        }
        return ref;
    };

    MppiStateF s;
    s.x = (R - 0.4f);
    s.y = 0.0f;
    s.yaw = static_cast<float>(M_PI) / 2.0f;

    auto theta_of = [&](const MppiStateF& st) { return std::atan2(st.y, st.x); };

    const int STEPS = 400;         // 20s @ 50Hz
    float max_lat = 0.f, worst_ms = 0.f;
    float lat_first = 0.f, lat_last = 0.f;
    bool control_ok = true, finite_ok = true;

    for (int i = 0; i < STEPS; ++i) {
        auto ref = build_ref(theta_of(s), s.vx);
        MppiControlF u;
        bool ok = mppi.solve(s, ref, u);
        (void)ok;

        if (!(std::isfinite(u.steer) && std::isfinite(u.accel))) finite_ok = false;
        if (std::abs(u.steer) > p.steer_max + 1e-6f ||
            u.accel > p.accel_max + 1e-6f || u.accel < p.accel_min - 1e-6f) control_ok = false;

        s = mppi.propagate(s, u);  // 동일 동역학으로 폐루프 되먹임

        const float lat = std::abs(std::hypot(s.x, s.y) - R);
        max_lat = std::max(max_lat, lat);
        worst_ms = std::max(worst_ms, static_cast<float>(mppi.last_solve_ms()));
        if (i == 0) lat_first = lat;
        lat_last = lat;
    }

    std::printf("== MPPI GPU smoke test ==\n");
    std::printf("lateral error: first=%.3f m  last=%.3f m  (max=%.3f)\n", lat_first, lat_last, max_lat);
    std::printf("final speed  : %.3f m/s (target %.1f)\n", s.vx, v_ref);
    std::printf("worst solve  : %.2f ms  [RTX 4060 기준 — Jetson 성능 예측 아님]\n", worst_ms);
    std::printf("controls in-limit: %s | finite: %s\n",
                control_ok ? "OK" : "FAIL", finite_ok ? "OK" : "FAIL");
    {   // CPU 스모크 테스트와 같은 형식의 진단 — 두 솔버 정합성 비교용
        const MppiDiagF& d = mppi.last_diag();
        std::printf("diag: ESS=%.1f/%d fin=%d lam_eff=%.2f | C[lat=%.1f lon=%.1f yaw=%.1f v=%.1f bnd=%.1f du=%.1f] max_lat=%.2f\n",
                    d.ess, p.K, d.n_finite, d.lambda_eff,
                    d.c_lat, d.c_lon, d.c_yaw, d.c_v, d.c_bnd, d.c_du, d.nominal_max_lat);
    }

    const bool converged = (lat_last < lat_first) && (lat_last < 0.5f);
    std::printf("converged: %s\n", converged ? "OK" : "FAIL");
    std::printf("RESULT: %s\n",
                (converged && control_ok && finite_ok) ? "PASS" : "CHECK");
    return (converged && control_ok && finite_ok) ? 0 : 1;
}
#endif  // MPPI_GPU_SMOKE_TEST
