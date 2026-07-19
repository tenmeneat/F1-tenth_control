#pragma once

#include "geometry_msgs/msg/quaternion.hpp"

namespace f1tenth_control {

// IMU 피드백 기반 차량 동적 안정성 보정 (control_map_node 전용).
// 롤각과 요레이트를 LPF로 걸러 두고, 요레이트 카운터스티어와 롤 인지 ESC에 쓴다.
// ⚠️ 롤"레이트"는 구현돼 있지 않다(예전 주석이 있다고 적어 두었으나 실제 상태는 없었음).
//    예측형 전복 억제를 넣으려면 angular_velocity.x를 새로 받아야 하는데, 그 축이 정말
//    롤레이트인지(VESC 90도 장착 회전 가설) 실차 확인이 선행돼야 한다.
class StabilityController {
public:
    // alpha_*: 각 상태의 LPF 계수 (클수록 실측을 빨리 따라감)
    StabilityController(double alpha_roll = 0.15, double alpha_yaw_rate = 0.2);

    // ⚠️ raw_yaw_rate는 rad/s여야 한다. VESC는 deg/s로 발행하므로 호출부에서 환산해
    //    넘긴다(launch/_control_common.py의 IMU_ANGULAR_SCALE).
    void update_imu(const geometry_msgs::msg::Quaternion& orientation,
                    double raw_yaw_rate);

    // 요레이트 피드백 카운터스티어 보정각. 기하학적 기대 요레이트(v·tanδ/L) 대비
    // 실측 오차에 비례해 조향을 더한다(언더스티어 시 더 꺾음).
    double calculate_yaw_rate_correction(double current_speed,
                                         double current_steering_angle,
                                         double wheelbase,
                                         double yaw_rate_gain) const;

    // 롤 각도 / 전복 임계각 비율 (0.0 ~ 1.0). 가감속 한계 축소에 쓴다.
    double calculate_roll_ratio(double max_roll_limit) const;

    // LPF 통과 롤각 [rad]. 롤 ESC가 실제로 걸리는 구간이 있는지 계측하기 위한 관찰용
    // (max_roll_limit 임계치가 1/10 차량에 타당한지 판단 — 제어에는 쓰지 않는다).
    double filtered_roll() const { return filtered_roll_; }

private:
    double filtered_roll_ = 0.0;
    double filtered_yaw_rate_ = 0.0;

    double alpha_roll_;
    double alpha_yaw_rate_;
};

} // namespace f1tenth_control
