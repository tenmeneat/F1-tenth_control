#ifndef F1TENTH_CONTROL_MPPI_TYPES_GPU_HPP_
#define F1TENTH_CONTROL_MPPI_TYPES_GPU_HPP_

// ============================================================================
// mppi_types_gpu.hpp — MPPI GPU 솔버용 float32 자료구조 + host/device 공용 유틸
// ----------------------------------------------------------------------------
// control_mppi_solver_cpu.cpp(CPU, double)의 구조체·유틸을 float32로 미러링한 것.
//   - 소비자/임베디드 GPU(Ampere/Ada)의 FP64 처리량은 FP32의 ~1/32~1/64라,
//     롤아웃 수학 전체를 float으로 돌려야 실질 처리량 이득이 난다(MPPI-on-GPU 표준).
//   - CPU 레퍼런스(control_mppi_solver_cpu.cpp)는 double 그대로 두고 절대 건드리지 않는다.
//     두 정밀도 버전을 별도 파일로 독립 유지하기 위한 공용 헤더.
//   - 유틸 함수는 __host__ __device__ 로 선언해 호스트 검증코드와 디바이스 커널이
//     동일 정의를 공유한다(NVCC가 아닌 순수 C++ 컴파일 시엔 매크로가 비워짐).
// ============================================================================

#include <cmath>

// NVCC로 컴파일될 때만 __host__/__device__ 수식어를 붙인다(순수 C++ TU에선 제거).
#if defined(__CUDACC__)
#define MPPI_HD __host__ __device__
#else
#define MPPI_HD
#endif

namespace f1tenth_control {

// 차량 상태 (전역 위치 + 차체프레임 속도) — MppiState의 float 버전
struct MppiStateF {
    float x = 0.f, y = 0.f, yaw = 0.f;  // 전역 X,Y,헤딩 ψ
    float vx = 0.f, vy = 0.f;           // 차체프레임 종/횡 속도
    float yaw_rate = 0.f;               // 요레이트 ω
};

// 예측 수평 한 스테이지 기준점 — MppiRef의 float 버전
struct MppiRefF {
    float x = 0.f, y = 0.f, yaw = 0.f;  // 기준 위치·헤딩
    float v = 0.f;                      // 목표 속도
    // 좌/우 벽까지 거리 — 비대칭(2026-07-22). CPU MppiRef와 동일 규약.
    float d_left = 1.f, d_right = 1.f;
};

// 제어 입력 — MppiControl의 float 버전
struct MppiControlF {
    float steer = 0.f;  // 조향 δ [rad]
    float accel = 0.f;  // 종가속 a [m/s²]
};

// MPPI + 차량/타이어 + 비용 파라미터 (control_mppi_solver_cpu.cpp의 MppiParams와 동일 기본값)
struct MppiParamsF {
    // --- MPPI 코어 ---
    int   N  = 25;
    int   K  = 512;
    float dt = 0.05f;
    float lambda = 1.0f;      // 고정 역온도(lambda_rel<=0일 때만)
    float lambda_rel = 0.02f; // 적응 역온도: λ_eff = lambda_rel·(J_mean − J_min)
    float sigma_steer = 0.15f;
    float sigma_accel = 1.5f;
    float noise_beta = 0.7f;  // 잡음 시간상관 AR(1) 계수 [0,1)
    unsigned int seed = 0xC0FFEEu;

    // --- 차량/타이어 ---
    float wheelbase = 0.33f, lf = 0.15875f, lr = 0.17145f;
    float mass = 3.74f, Iz = 0.04712f, g = 9.81f, mu = 1.0489f;
    float Bf = 10.0f, Cf = 1.9f, Ef = 0.97f;
    float Br = 10.0f, Cr = 1.9f, Er = 0.97f;
    float v_switch = 2.0f;
    int   substeps = 2;

    // --- 비용 가중 (컨투어링 재정식화 — CPU 레퍼런스와 동일) ---
    float w_lat = 150.0f;      // 경로 횡오차 — 주력. w_v와의 **비율**이 안정성을 정한다(07-22 실측: w_v/w_lat이
    // 0.004 이하면 안정, 0.0067이면 헤어핀에서 마찰포화로 크래시)
    float w_lon = 1.0f;       // 경로 진행방향(lag) 오차 — 작게
    float w_yaw = 5.0f, w_v = 0.5f;
    float w_boundary = 500.0f;
    float margin = 0.15f;
    float w_terminal = 20.0f;
    float w_dsteer = 100.0f;  // 제어 변화율 비용(조향) — 채터링 억제
    float w_daccel = 0.5f;

    // --- 한계 ---
    float steer_max = 0.41f, accel_max = 4.0f, accel_min = -8.0f;
    float v_min = 0.0f, v_max = 8.0f;

    // --- 출력 평활화 ---
    float u_smooth = 0.3f;

    // 진단 계측 on/off (리덕션·커널 몇 개 추가). 제어 결과에는 영향 없음.
    bool diag_enable = true;
};

// 솔버 진단값 — CPU MppiDiag의 float 버전(필드명 동일 → 노드가 같은 코드로 출력).
struct MppiDiagF {
    double ess = 0.0;
    double lambda_eff = 0.0;
    double j_min = 0.0, j_mean = 0.0, j_max = 0.0;
    double c_lat = 0.0, c_lon = 0.0, c_yaw = 0.0, c_v = 0.0, c_bnd = 0.0, c_du = 0.0;
    double nominal_max_lat = 0.0;
    int    n_finite = 0;
};

// ---------------------------------------------------------------------------
// host/device 공용 유틸 (control_mppi_solver_cpu.cpp 익명namespace 함수의 float 버전)
// ---------------------------------------------------------------------------
MPPI_HD inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// 각도를 [-π, π]로 정규화
MPPI_HD inline float wrap_pi_f(float a) {
    const float twopi = 6.28318530717958647692f;
    while (a >  3.14159265358979323846f) a -= twopi;
    while (a < -3.14159265358979323846f) a += twopi;
    return a;
}

// Pacejka 마법공식 정규화 횡력계수: μ_y(α) = sin(C·atan(B·α − E·(B·α − atan(B·α))))
MPPI_HD inline float pacejka_mu_f(float alpha, float B, float C, float E) {
    const float Ba = B * alpha;
    return std::sin(C * std::atan(Ba - E * (Ba - std::atan(Ba))));
}

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_MPPI_TYPES_GPU_HPP_
