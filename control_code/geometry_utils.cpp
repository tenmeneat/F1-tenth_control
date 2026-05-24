#include "f1tenth_control/geometry_utils.hpp"
#include <cmath>
#include <algorithm>

namespace f1tenth_control {
namespace geometry {

double calculate_curvature(const Waypoint& p1, const Waypoint& p2, const Waypoint& p3) {
    double a = std::hypot(p2.x - p1.x, p2.y - p1.y);
    double b = std::hypot(p3.x - p2.x, p3.y - p2.y);
    double c = std::hypot(p1.x - p3.x, p1.y - p3.y);
    
    // 세 점이 이루는 삼각형의 면적 (사선 공식)
    double area = 0.5 * std::abs(p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
    
    // 만약 세 점이 거의 직선 위에 있는 경우 곡률은 0
    if (area < 1e-6) {
        return 0.0;
    }
    
    // 외접원 반지름 R = (a * b * c) / (4 * Area)
    double radius = (a * b * c) / (4.0 * area);
    
    // 곡률 kappa = 1 / R
    return 1.0 / radius;
}

} // namespace geometry
} // namespace f1tenth_control
