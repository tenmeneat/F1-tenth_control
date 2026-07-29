#pragma once

#include <cmath>

namespace f1tenth_control {

// IMU 요레이트 피드백 (control_map_node 전용).
// 실측 요레이트를 LPF로 걸러 두고 카운터스티어 보정에 쓴다.
//
// ⚠️ 2026-07-29: 롤 인지 ESC(롤각으로 가감속 한계를 줄이던 것)는 제거했다. 1/10 차량은
//    서스펜션이 단단해 max_roll_limit(0.15rad≈8.6°)까지 기울지 않아 사실상 상시 비활성
//    이었고, 레이싱에서 롤 기반 감속은 얻는 것 없이 코너 속도만 깎는다. 롤각이 필요해지면
//    quaternion→roll 변환과 함께 되살릴 것.
class StabilityController {
public:
    // alpha_yaw_rate: LPF 계수 (클수록 실측을 빨리 따라감)
    explicit StabilityController(double alpha_yaw_rate = 0.2)
        : alpha_yaw_rate_(alpha_yaw_rate) {}

    // ⚠️ raw_yaw_rate는 rad/s여야 한다. VESC는 deg/s로 발행하므로 호출부에서 환산해
    //    넘긴다(launch/_control_common.py의 IMU_ANGULAR_SCALE).
    void update_yaw_rate(double raw_yaw_rate) {
        filtered_yaw_rate_ += alpha_yaw_rate_ * (raw_yaw_rate - filtered_yaw_rate_);
    }

    // 요레이트 피드백 카운터스티어 보정각. 기하학적 기대 요레이트(v·tanδ/L) 대비 실측
    // 오차에 비례해 조향을 더한다(언더스티어 시 더 꺾음). 저속은 v·tanδ/L이 특이해져
    // 보정이 발산하므로 0으로 게이트한다.
    double calculate_yaw_rate_correction(double current_speed, double current_steering_angle,
                                         double wheelbase, double yaw_rate_gain) const {
        if (std::abs(current_speed) < 0.5) return 0.0;
        double expected = current_speed * std::tan(current_steering_angle) / wheelbase;
        return yaw_rate_gain * (expected - filtered_yaw_rate_);
    }

    double filtered_yaw_rate() const { return filtered_yaw_rate_; }

private:
    double filtered_yaw_rate_ = 0.0;
    double alpha_yaw_rate_;
};

} // namespace f1tenth_control
