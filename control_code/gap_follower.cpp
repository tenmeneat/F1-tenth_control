#include "f1tenth_control/gap_follower.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace f1tenth_control {

GapFollower::GapFollower(double max_fov_deg, double safety_bubble_dist, double trigger_dist, double steering_limit)
    : max_fov_deg_(max_fov_deg),
      safety_bubble_dist_(safety_bubble_dist),
      trigger_dist_(trigger_dist),
      steering_limit_(steering_limit) {}

bool GapFollower::process_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg, double& avoid_steering_angle) {
    if (msg == nullptr) {
        avoid_steering_angle = 0.0;
        return false;
    }

    double max_fov_rad = (max_fov_deg_ * PI) / 180.0;
    bool obstacle_detected = false;

    std::vector<double> processed_ranges(msg->ranges.size());
    
    // 1) NaN/Inf 예외 처리 및 스캔 필터링
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double r = msg->ranges[i];
        if (std::isnan(r) || std::isinf(r)) {
            processed_ranges[i] = msg->range_max;
        } else {
            processed_ranges[i] = r;
        }
    }

    // 2) 장애물 주변 세이프티 버블(가려짐) 적용
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double angle = msg->angle_min + i * msg->angle_increment;
        if (std::abs(angle) > (max_fov_rad / 2.0)) {
            processed_ranges[i] = 0.0; // 관심 영역 밖은 갈 수 없음 처리
            continue;
        }

        double r = processed_ranges[i];
        if (r < trigger_dist_ && r > msg->range_min) {
            obstacle_detected = true;
            // 장애물 감지 시 주변 빔들을 0으로 마스킹 (세이프티 버블)
            double bubble_angle = std::atan2(safety_bubble_dist_, r);
            for (size_t j = 0; j < msg->ranges.size(); ++j) {
                double other_angle = msg->angle_min + j * msg->angle_increment;
                if (std::abs(other_angle - angle) < bubble_angle) {
                    processed_ranges[j] = 0.0; // 갈 수 없는 공간으로 마스킹
                }
            }
        }
    }

    // 3) 가장 넓고 깊은 Gap(안전 공간) 탐색
    if (obstacle_detected) {
        size_t max_gap_start = 0;
        size_t max_gap_len = 0;
        size_t current_gap_start = 0;
        size_t current_gap_len = 0;

        for (size_t i = 0; i < processed_ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            if (std::abs(angle) > (max_fov_rad / 2.0)) continue;

            if (processed_ranges[i] > 0.5) { // 갈 수 있는 공간
                if (current_gap_len == 0) {
                    current_gap_start = i;
                }
                current_gap_len++;
            } else { // 마스킹된 공간
                if (current_gap_len > max_gap_len) {
                    max_gap_len = current_gap_len;
                    max_gap_start = current_gap_start;
                }
                current_gap_len = 0;
            }
        }
        if (current_gap_len > max_gap_len) {
            max_gap_len = current_gap_len;
            max_gap_start = current_gap_start;
        }

        // Gap이 존재하면 그 중 가장 깊은(먼) 거리 지점 또는 Gap의 중심으로 조향 설정
        if (max_gap_len > 0) {
            size_t gap_center_idx = max_gap_start + max_gap_len / 2;
            
            // Gap 내에서 가장 멀리 뚫린 인덱스 찾기
            double max_val = 0.0;
            for (size_t idx = max_gap_start; idx < max_gap_start + max_gap_len; ++idx) {
                if (processed_ranges[idx] > max_val) {
                    max_val = processed_ranges[idx];
                    gap_center_idx = idx;
                }
            }

            double best_angle = msg->angle_min + gap_center_idx * msg->angle_increment;
            avoid_steering_angle = best_angle;
            
            // 조향각을 물리적 조향 한계 내로 클리핑
            avoid_steering_angle = std::max(-steering_limit_, std::min(avoid_steering_angle, steering_limit_));
        } else {
            avoid_steering_angle = 0.0; // 다 막혔을 시 중립
        }
    } else {
        avoid_steering_angle = 0.0;
    }

    return obstacle_detected;
}

} // namespace f1tenth_control
