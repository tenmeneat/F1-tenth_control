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

bool GapFollower::process_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg,
                               double& avoid_steering_angle,
                               double& min_obstacle_dist) {
    if (msg == nullptr) {
        avoid_steering_angle = 0.0;
        min_obstacle_dist = 999.0;
        return false;
    }

    double max_fov_rad = (max_fov_deg_ * PI) / 180.0;
    bool obstacle_detected = false;
    min_obstacle_dist = msg->range_max;

    std::vector<double> processed_ranges(msg->ranges.size());
    
    // 1) NaN/Inf 예외 처리 및 스캔 필터링 + 최소 거리 계산
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double r = msg->ranges[i];
        double angle = msg->angle_min + i * msg->angle_increment;
        if (std::isnan(r) || std::isinf(r)) {
            processed_ranges[i] = msg->range_max;
        } else {
            processed_ranges[i] = r;
            // FOV 내 최소 거리 추적
            if (std::abs(angle) <= (max_fov_rad / 2.0) && r > msg->range_min) {
                min_obstacle_dist = std::min(min_obstacle_dist, r);
            }
        }
    }

    // 2) 장애물 주변 세이프티 버블(가려짐) 적용
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double angle = msg->angle_min + i * msg->angle_increment;
        if (std::abs(angle) > (max_fov_rad / 2.0)) {
            processed_ranges[i] = 0.0; // 관심 영역 밖은 갈 수 없음 처리
            continue;
        }

        double r = msg->ranges[i];
        if (std::isnan(r) || std::isinf(r)) {
            continue;
        }

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

    // 3) 항상 가장 넓고 깊은 Gap(안전 공간) 탐색 (장애물 유무 무관)
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

    // Gap이 존재하면 가장 깊게 뚫린 넓은 영역의 평균 방향으로 조향 설정
    if (max_gap_len > 0) {
        // 1. Gap 내부의 최대 거리값 찾기
        double max_val = 0.0;
        for (size_t idx = max_gap_start; idx < max_gap_start + max_gap_len; ++idx) {
            if (processed_ranges[idx] > max_val) {
                max_val = processed_ranges[idx];
            }
        }

        // 2. 최대 거리에 가까운 빔들(max_val - 0.5m 이상)의 평균 인덱스 구하기
        double threshold = std::max(0.5, max_val - 0.5);
        double sum_idx = 0.0;
        int count = 0;
        for (size_t idx = max_gap_start; idx < max_gap_start + max_gap_len; ++idx) {
            if (processed_ranges[idx] >= threshold) {
                sum_idx += idx;
                count++;
            }
        }

        size_t gap_center_idx = (count > 0) ? static_cast<size_t>(sum_idx / count) : (max_gap_start + max_gap_len / 2);

        double best_angle = msg->angle_min + gap_center_idx * msg->angle_increment;
        avoid_steering_angle = best_angle;
        
        // 조향각을 물리적 조향 한계 내로 클리핑
        avoid_steering_angle = std::max(-steering_limit_, std::min(avoid_steering_angle, steering_limit_));
    } else {
        // [Fail-safe] 모든 공간이 마스킹되어 갈 수 있는 Gap이 아예 없는 경우:
        // 직진(0.0)으로 풀지 않고, 세이프티 버블을 치기 전 원본 스캔(msg->ranges)에서 
        // FOV 영역 내의 가장 멀리 뚫려 있던(최대 거리) 인덱스 방향으로 조향을 유지하여 벽을 회피함.
        size_t best_fallback_idx = 0;
        double max_fallback_val = 0.0;
        for (size_t idx = 0; idx < msg->ranges.size(); ++idx) {
            double angle = msg->angle_min + idx * msg->angle_increment;
            if (std::abs(angle) > (max_fov_rad / 2.0)) continue;
            
            double r = msg->ranges[idx];
            if (!std::isnan(r) && !std::isinf(r) && r > max_fallback_val) {
                max_fallback_val = r;
                best_fallback_idx = idx;
            }
        }
        
        if (max_fallback_val > 0.5) {
            double best_angle = msg->angle_min + best_fallback_idx * msg->angle_increment;
            avoid_steering_angle = std::max(-steering_limit_, std::min(best_angle, steering_limit_));
        } else {
            avoid_steering_angle = 0.0; 
        }
    }

    return obstacle_detected;
}

} // namespace f1tenth_control
