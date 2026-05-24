#ifndef F1TENTH_CONTROL_GEOMETRY_UTILS_HPP_
#define F1TENTH_CONTROL_GEOMETRY_UTILS_HPP_

#include "f1tenth_control/types.hpp"

namespace f1tenth_control {
namespace geometry {

/**
 * 3점 피팅 원형 외접원 반지름을 활용한 로컬 곡률 연산 함수
 * A(p1), B(p2), C(p3) 세 점이 이루는 곡률을 계산하여 반환
 */
double calculate_curvature(const Waypoint& p1, const Waypoint& p2, const Waypoint& p3);

} // namespace geometry
} // namespace f1tenth_control

#endif // F1TENTH_CONTROL_GEOMETRY_UTILS_HPP_
