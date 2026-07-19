#ifndef F1TENTH_CONTROL_GAP_FOLLOWER_HPP_
#define F1TENTH_CONTROL_GAP_FOLLOWER_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "f1tenth_control/types.hpp"
#include <vector>

namespace f1tenth_control {

// 순수 LiDAR 갭 추종. 글로벌 경로를 못 받았을 때의 failsafe 주행과,
// 글로벌 추종 중 전방이 막혔을 때의 회피 폴백 두 곳에서 쓴다.
class GapFollower {
public:
    GapFollower(double max_fov_deg = 110.0, double safety_bubble_dist = 0.38, double trigger_dist = 1.8, double steering_limit = 0.41);

    // 뒤 두 인자는 출력. 반환값은 장애물 감지 여부.
    bool process_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg,
                      double& avoid_steering_angle,   // [out] 회피 조향각 [rad]
                      double& min_obstacle_dist);     // [out] FOV 내 최소 장애물 거리 [m]

private:
    double max_fov_deg_;
    double safety_bubble_dist_;
    double trigger_dist_;
    double steering_limit_;
};

} // namespace f1tenth_control

#endif // F1TENTH_CONTROL_GAP_FOLLOWER_HPP_
