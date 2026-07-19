#ifndef F1TENTH_CONTROL_TYPES_HPP_
#define F1TENTH_CONTROL_TYPES_HPP_

// 수학 상수 정의
const double PI = 3.14159265358979323846;

// 웨이포인트 구조체 정의 (곡률 및 계산된 동적 한계 속도 포함)
struct Waypoint {
    double x;
    double y;
    double speed;      // 최종 적용되는 곡률 기반 최적 속도
    double curvature;  // 계산된 기하학적 곡률 (1/R)
    double yaw;        // 맵 웨이포인트의 진행방향 각도 (heading)
    double smoothed_curvature = 0.0; // 물리거리 창 평활 곡률 (곡률 사전감속용, 단일점 노이즈 억제)
};

#endif // F1TENTH_CONTROL_TYPES_HPP_
