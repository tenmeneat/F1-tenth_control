#include "f1tenth_control/imu_stability_controller.hpp"
#include <cmath>
#include <algorithm>

namespace f1tenth_control {

StabilityController::StabilityController(double alpha_roll, double alpha_yaw_rate)
    : alpha_roll_(alpha_roll), alpha_yaw_rate_(alpha_yaw_rate) {}

void StabilityController::update_imu(const geometry_msgs::msg::Quaternion& orientation,
                                     double raw_yaw_rate) {
    // Quaternion -> 오일러 롤 각
    double sinr_cosp = 2.0 * (orientation.w * orientation.x + orientation.y * orientation.z);
    double cosr_cosp = 1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    filtered_roll_ = alpha_roll_ * roll + (1.0 - alpha_roll_) * filtered_roll_;
    filtered_yaw_rate_ = alpha_yaw_rate_ * raw_yaw_rate + (1.0 - alpha_yaw_rate_) * filtered_yaw_rate_;
}

double StabilityController::calculate_yaw_rate_correction(double current_speed,
                                                          double current_steering_angle,
                                                          double wheelbase,
                                                          double yaw_rate_gain) const {
    // 저속에서는 v·tanδ/L이 특이해져 보정이 발산하므로 건너뛴다
    if (std::abs(current_speed) < 0.5) {
        return 0.0;
    }

    // 기하학적 2륜 모델의 정상 선회 요레이트 대비 실측 오차에 비례 게인
    double expected_yaw_rate = current_speed * std::tan(current_steering_angle) / wheelbase;
    return yaw_rate_gain * (expected_yaw_rate - filtered_yaw_rate_);
}

double StabilityController::calculate_roll_ratio(double max_roll_limit) const {
    if (max_roll_limit <= 0.001) {
        return 0.0;
    }
    return std::min(std::abs(filtered_roll_) / max_roll_limit, 1.0);
}

} // namespace f1tenth_control
