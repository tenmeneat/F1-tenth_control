#include "f1tenth_control/velocity_profiler.hpp"
#include "f1tenth_control/geometry_utils.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace f1tenth_control {

std::vector<Waypoint> VelocityProfiler::generate_figure_eight_path(double scale_x, double scale_y, double max_speed) {
    int num_points = 300;
    std::vector<Waypoint> waypoints;
    for (int i = 0; i < num_points; ++i) {
        double t = (2.0 * PI * i) / num_points;
        Waypoint wp;
        // Lissajous Figure-8 공식
        wp.x = scale_x * std::sin(t);
        wp.y = scale_y * std::sin(t) * std::cos(t);
        wp.speed = max_speed; // 초기화
        wp.curvature = 0.0;
        wp.raw_speed_limit = max_speed;
        
        waypoints.push_back(wp);
    }
    return waypoints;
}

void VelocityProfiler::generate_velocity_profile(std::vector<Waypoint>& waypoints,
                                                double max_lat_accel,
                                                double max_speed,
                                                double min_speed,
                                                double base_max_accel,
                                                double base_max_decel) {
    size_t n = waypoints.size();
    if (n < 3) return;

    // 1단계: 각 점에서의 기하학적 곡률 계산 및 횡가속도 한계 속도 도출
    for (size_t i = 0; i < n; ++i) {
        size_t prev_idx = (i - 1 + n) % n;
        size_t next_idx = (i + 1) % n;

        // 3점 원형 피팅 적용 (geometry 네임스페이스 함수 사용)
        double kappa = geometry::calculate_curvature(waypoints[prev_idx], waypoints[i], waypoints[next_idx]);
        waypoints[i].curvature = kappa;

        if (kappa > 0.001) {
            // v = sqrt(a_lat_max / kappa)
            double v_limit = std::sqrt(max_lat_accel / kappa);
            waypoints[i].raw_speed_limit = std::max(min_speed, std::min(v_limit, max_speed));
        } else {
            waypoints[i].raw_speed_limit = max_speed;
        }
        // 임시 속도 대입
        waypoints[i].speed = waypoints[i].raw_speed_limit;
    }

    // 2단계: Forward-Backward Dynamic Integration Pass 적용 (3회 반복)
    for (int pass = 0; pass < 3; ++pass) {
        // (A) Backward Pass: 코너 진입 제동 한계 반영
        for (size_t i = n; i > 0; --i) {
            size_t curr = (i - 1) % n;
            size_t next = (curr + 1) % n;

            double dx = waypoints[next].x - waypoints[curr].x;
            double dy = waypoints[next].y - waypoints[curr].y;
            double ds = std::hypot(dx, dy);

            double max_safe_entry_speed = std::sqrt(std::pow(waypoints[next].speed, 2) + 2.0 * base_max_decel * ds);
            waypoints[curr].speed = std::min(waypoints[curr].speed, max_safe_entry_speed);
        }

        // (B) Forward Pass: 코너 탈출 가속 한계 반영
        for (size_t i = 0; i < n; ++i) {
            size_t curr = i;
            size_t prev = (curr - 1 + n) % n;

            double dx = waypoints[curr].x - waypoints[prev].x;
            double dy = waypoints[curr].y - waypoints[prev].y;
            double ds = std::hypot(dx, dy);

            double max_safe_exit_speed = std::sqrt(std::pow(waypoints[prev].speed, 2) + 2.0 * base_max_accel * ds);
            waypoints[curr].speed = std::min(waypoints[curr].speed, max_safe_exit_speed);
        }
    }
}

} // namespace f1tenth_control
