#include "f1tenth_control/stability_controller.hpp"
#include <cmath>
#include <algorithm>

namespace f1tenth_control {

StabilityController::StabilityController(double alpha_roll, double alpha_roll_rate, double alpha_yaw_rate)
    : alpha_roll_(alpha_roll), alpha_roll_rate_(alpha_roll_rate), alpha_yaw_rate_(alpha_yaw_rate) {}

void StabilityController::update_imu(const geometry_msgs::msg::Quaternion& orientation, 
                                     double raw_roll_rate, 
                                     double raw_yaw_rate) {
    // 1. Quaternion -> Roll Angle 산출 (오일러 롤 각 산출)
    double sinr_cosp = 2.0 * (orientation.w * orientation.x + orientation.y * orientation.z);
    double cosr_cosp = 1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // Roll 각도 LPF 필터 적용
    filtered_roll_ = alpha_roll_ * roll + (1.0 - alpha_roll_) * filtered_roll_;

    // Roll Rate (X축 각속도) LPF 적용
    filtered_roll_rate_ = alpha_roll_rate_ * raw_roll_rate + (1.0 - alpha_roll_rate_) * filtered_roll_rate_;

    // Yaw Rate (Z축 각속도) LPF 적용
    filtered_yaw_rate_ = alpha_yaw_rate_ * raw_yaw_rate + (1.0 - alpha_yaw_rate_) * filtered_yaw_rate_;
}

double StabilityController::calculate_steering_attenuation(double roll_rate_gain, double max_gain_attenuation) const {
    double attenuation = roll_rate_gain * std::abs(filtered_roll_rate_);
    return std::min(attenuation, max_gain_attenuation);
}

double StabilityController::calculate_yaw_rate_correction(double current_speed, 
                                                           double current_steering_angle, 
                                                           double wheelbase, 
                                                           double yaw_rate_gain) const {
    // 저속 구동 시 슬립 보정 불필요 (연산 특이점 방지)
    if (std::abs(current_speed) < 0.5) {
        return 0.0;
    }
    
    // 기하학적 2-륜 모델 기준 정상 선회 요 레이트: Yaw Rate = V * tan(Steering) / Wheelbase
    double expected_yaw_rate = current_speed * std::tan(current_steering_angle) / wheelbase;
    double yaw_rate_error = expected_yaw_rate - filtered_yaw_rate_;
    
    // 오차에 비례 게인을 적용한 카운터 조향 각 보정치 리턴
    return yaw_rate_gain * yaw_rate_error;
}

double StabilityController::calculate_roll_ratio(double max_roll_limit) const {
    if (max_roll_limit <= 0.001) {
        return 0.0;
    }
    double roll_ratio = std::abs(filtered_roll_) / max_roll_limit;
    return std::min(roll_ratio, 1.0);
}

} // namespace f1tenth_control
