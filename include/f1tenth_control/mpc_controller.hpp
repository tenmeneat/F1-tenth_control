#ifndef F1TENTH_CONTROL_MPC_CONTROLLER_HPP_
#define F1TENTH_CONTROL_MPC_CONTROLLER_HPP_

// ============================================================================
// 전역좌표 Kinematic Bicycle LTV-MPC (조향 전용)
// ----------------------------------------------------------------------------
// 상태 x = [X, Y, ψ] (전역 위치·헤딩), 결정변수 u = δ (조향각).
// 속도 v[k]는 결정변수가 아니라 플래너 속도 프로파일에서 받아오는 *알려진 파라미터*.
// 기준궤적(웨이포인트) 주위에서 1차 선형화한 LTV 모델을 매 사이클 갱신하여
// sparse QP(OSQP)로 N스텝 앞을 내다보고 첫 조향각 δ₀를 산출한다.
//
// 설계 청사진: aaurandt/MMPRV_F1Tenth (무라이선스 → 구조/정식화만 참고, 코드는 독자 작성).
// OSQP는 C API를 직접 호출(osqp-eigen 미사용). 내부 구현은 PIMPL로 은닉하여
// 공개 헤더에 OSQP/C 타입이 새어나오지 않게 한다.
// ============================================================================

#include <memory>
#include <vector>

namespace f1tenth_control {

// 차량 현재 상태 (전역좌표)
struct MpcState {
    double x;    // 전역 X [m]
    double y;    // 전역 Y [m]
    double yaw;  // 헤딩 ψ [rad]
};

// 예측 수평의 한 스테이지 기준점 (웨이포인트 + 그 지점의 목표 속도)
struct MpcRef {
    double x;    // 기준 X [m]
    double y;    // 기준 Y [m]
    double yaw;  // 기준 헤딩 ψ [rad] (연속이 되도록 호출부에서 unwrap 권장)
    double v;    // 그 스테이지에서 가정하는 속도 [m/s] (플래너 프로파일)
};

// MPC 튜닝 파라미터
struct MpcParams {
    int    N         = 12;     // 예측 수평 (스텝 수)
    double Ts        = 0.05;   // MPC 이산 스텝 [s]
    double wheelbase = 0.33;   // 휠베이스 L = lf + lr [m]
    double lr        = 0.165;  // 무게중심~후축 거리 (슬립각 계산용, 기본 L/2)

    // 상태 가중 Q = diag(q_x, q_y, q_yaw)
    double q_x   = 5.0;
    double q_y   = 5.0;
    double q_yaw = 0.5;

    // 입력 가중
    double r       = 0.1;   // 조향각 크기 페널티 (r·δ²)
    double r_delta = 5.0;   // 조향율 페널티 (r_d·Δδ²) — 부드러운 조향

    // 제약
    double delta_max  = 0.41;  // 물리 조향 한계 [rad]
    double ddelta_max = 0.20;  // MPC 스텝당 조향 변화 한계 [rad/step]

    // OSQP 솔버 옵션
    int    osqp_max_iter = 4000;
    double osqp_eps_abs  = 1e-3;
    double osqp_eps_rel  = 1e-3;
};

class MPCController {
public:
    explicit MPCController(const MpcParams& params);
    ~MPCController();

    MPCController(const MPCController&) = delete;
    MPCController& operator=(const MPCController&) = delete;

    // N스텝 MPC를 풀어 첫 최적 조향각을 out_delta에 채운다.
    //  cur       : 차량 현재 전역 상태
    //  ref       : 길이 N+1 의 기준 궤적/속도 (ref[0]=현재 근방, ref[N]=가장 먼 미래)
    //  last_delta: 직전 사이클 조향각 (조향율 제약/페널티 기준)
    // 반환: 성공 시 true. 실패(미해결 등) 시 false → 호출부에서 L1 폴백.
    bool solve(const MpcState& cur,
               const std::vector<MpcRef>& ref,
               double last_delta,
               double& out_delta);

    // 직전 solve 소요시간 [ms] (실시간성 로깅용)
    double last_solve_ms() const;

    const MpcParams& params() const { return params_; }

private:
    MpcParams params_;
    struct Impl;                 // OSQP/CSC 상태를 은닉 (PIMPL)
    std::unique_ptr<Impl> impl_;
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_MPC_CONTROLLER_HPP_
