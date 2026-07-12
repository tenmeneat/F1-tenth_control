#ifndef F1TENTH_CONTROL_TYPES_HPP_
#define F1TENTH_CONTROL_TYPES_HPP_

// 수학 상수 정의
const double PI = 3.14159265358979323846;

// 웨이포인트 구조체 정의 (곡률 및 계산된 동적 한계 속도 포함)
struct Waypoint {
    double x;
    double y;
    double speed;      // 최종 적용되는 곡률 기반 최적 속도
    double curvature;  // 계산된 기하학적 곡률 (1/R) — 원본(비평활) 값, FF 등 다른 용도 유지
    double raw_speed_limit; // 가감속 한계 적용 전의 순수 횡방향 마찰 한계 속도
    double yaw;        // 맵 웨이포인트의 진행방향 각도 (heading)
    double smoothed_curvature = 0.0; // 곡률 사전감속 전용 물리거리 창 평활값(아래 참고). 기본값=0이면
                                      // 호출부에서 curvature로 폴백 가능하도록 미설정 시 구분되게 초기화.

};

#endif // F1TENTH_CONTROL_TYPES_HPP_
