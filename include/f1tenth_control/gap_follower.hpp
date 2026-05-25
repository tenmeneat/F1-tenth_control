#ifndef F1TENTH_CONTROL_GAP_FOLLOWER_HPP_
#define F1TENTH_CONTROL_GAP_FOLLOWER_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "f1tenth_control/types.hpp"
#include <vector>

namespace f1tenth_control {

class GapFollower {
public:
    GapFollower(double max_fov_deg = 110.0, double safety_bubble_dist = 0.38, double trigger_dist = 1.8, double steering_limit = 0.41);

    /**
     * LiDAR 스캔 데이터를 바탕으로 장애물 감지 여부와 회피 조향각을 계산합니다.
     * @param msg 입력 scan 메시지
     * @param avoid_steering_angle 반환할 회피 조향각 (rad)
     * @param min_obstacle_dist FOV 내 최소 장애물 거리 (m)
     * @return 장애물 감지 여부
     */
    bool process_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg,
                      double& avoid_steering_angle,
                      double& min_obstacle_dist);

private:
    double max_fov_deg_;
    double safety_bubble_dist_;
    double trigger_dist_;
    double steering_limit_;
};

} // namespace f1tenth_control

#endif // F1TENTH_CONTROL_GAP_FOLLOWER_HPP_
