#ifndef F1TENTH_CONTROL_MPPI_GPU_HPP_
#define F1TENTH_CONTROL_MPPI_GPU_HPP_

// ============================================================================
// mppi_gpu.hpp — MPPI GPU 솔버(MPPIControllerGPU)의 순수 C++ 인터페이스
// ----------------------------------------------------------------------------
// control_mppi_solver_gpu.cu의 thrust/CUDA 구현을 PImpl로 감춰, 이 헤더를 include하는
// 쪽(교차검증 .cpp, 향후 ControlMppiNode 등)은 CUDA 없이 컴파일된다.
// 공개 인터페이스는 CPU 레퍼런스 MPPIController(control_mppi_solver_cpu.cpp)와 동일한 형태
// (reset/last_solve_ms/params/propagate/solve)로 맞춰, 나중에 노드가 두 컨트롤러를
// 상호 교체 가능하게 쓸 수 있게 한다. 단 자료형은 float32 버전(...F)을 쓴다.
// ============================================================================

#include <memory>
#include <vector>

#include "f1tenth_control/mppi_types_gpu.hpp"

namespace f1tenth_control {

class MPPIControllerGPU {
 public:
    explicit MPPIControllerGPU(const MppiParamsF& p);
    ~MPPIControllerGPU();

    MPPIControllerGPU(const MPPIControllerGPU&) = delete;
    MPPIControllerGPU& operator=(const MPPIControllerGPU&) = delete;

    // nominal 제어열·이전 출력 리셋 (curand 상태는 유지 — CPU reset()과 동일 의미)
    void reset();

    double last_solve_ms() const;
    const MppiParamsF& params() const;

    // 전방 동역학 1스텝 노출 (폐루프 시뮬/단위검증에서 동일 모델 되먹임에 사용)
    MppiStateF propagate(const MppiStateF& s, const MppiControlF& u) const;

    // 현재 상태 + 기준궤적(길이 N+1) → 최적 첫 제어 out.
    // 내부 nominal 제어열은 호출 간 warm-start로 유지된다.
    bool solve(const MppiStateF& cur, const std::vector<MppiRefF>& ref, MppiControlF& out);

 private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_MPPI_GPU_HPP_
