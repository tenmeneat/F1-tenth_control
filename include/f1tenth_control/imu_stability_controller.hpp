#pragma once

#include "geometry_msgs/msg/quaternion.hpp"

namespace f1tenth_control {

/**
 * @brief 차량 동적 안정성 보정을 담당하는 제어 모듈 (IMU 피드백 기반)
 */
class StabilityController {
public:
    /**
     * @brief 생성자
     * @param alpha_roll 롤 필터 계수 (LPF)
     * @param alpha_roll_rate 롤 레이트 필터 계수 (LPF)
     * @param alpha_yaw_rate 요 레이트 필터 계수 (LPF)
     */
    StabilityController(double alpha_roll = 0.15, double alpha_roll_rate = 0.2, double alpha_yaw_rate = 0.2);

    /**
     * @brief IMU 데이터를 받아 상태값(Roll, Roll Rate, Yaw Rate)을 LPF 필터링하여 업데이트
     */
    void update_imu(const geometry_msgs::msg::Quaternion& orientation, 
                    double raw_roll_rate, 
                    double raw_yaw_rate);

    /**
     * @brief 롤 레이트 기반의 조향 이득 감쇠율 계산
     */
    double calculate_steering_attenuation(double roll_rate_gain, double max_gain_attenuation) const;

    /**
     * @brief 횡방향 슬립 방지를 위한 요 레이트(Yaw Rate) 피드백 조향 보정각 계산
     */
    double calculate_yaw_rate_correction(double current_speed, 
                                         double current_steering_angle, 
                                         double wheelbase, 
                                         double yaw_rate_gain) const;

    /**
     * @brief 롤 각도 기반 차량 전복 임계 비율 계산 (0.0 ~ 1.0)
     */
    double calculate_roll_ratio(double max_roll_limit) const;

    // 상태조회 게터 함수
    double get_filtered_roll() const { return filtered_roll_; }
    double get_filtered_roll_rate() const { return filtered_roll_rate_; }
    double get_filtered_yaw_rate() const { return filtered_yaw_rate_; }

    // 필터 리셋 (IMU 미장착 우회 시 호출)
    void reset() {
        filtered_roll_ = 0.0;
        filtered_roll_rate_ = 0.0;
        filtered_yaw_rate_ = 0.0;
    }

private:
    double filtered_roll_ = 0.0;
    double filtered_roll_rate_ = 0.0;
    double filtered_yaw_rate_ = 0.0;

    double alpha_roll_;
    double alpha_roll_rate_;
    double alpha_yaw_rate_;
};

} // namespace f1tenth_control
