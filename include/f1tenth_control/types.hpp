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
    double raw_speed_limit; // 가감속 한계 적용 전의 순수 횡방향 마찰 한계 속도
};

#endif // F1TENTH_CONTROL_TYPES_HPP_
