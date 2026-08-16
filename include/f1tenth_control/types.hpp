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
    // ⚠️ smoothed_curvature는 |κ|다(사전감속은 크기만 쓴다). 조향 FF는 **좌우를 구분해야**
    //    하므로 부호를 살린 별도 필드를 쓴다 — 이걸 혼동하면 우코너에서 FF가 반대로 나간다.
    double smoothed_curvature_signed = 0.0;
    double s = 0.0;    // Frenet 호길이 좌표(s_m) — raceline 프레임. 장애물(s,d) 감속 판정용
    // 이 지점에 적용할 횡가속 권한 [m/s²]. 섹터 스케일을 **웨이포인트 수신 시점에 한 번**
    // 해소해 여기 박아둔다 — 50 Hz 제어 루프에 검색·분기가 들어가지 않고, 실제로 적용된
    // 값이 그대로 로그로 나온다. 섹터 기능이 꺼져 있으면 전역 max_lateral_accel과 같다.
    double mla = 0.0;
};

#endif // F1TENTH_CONTROL_TYPES_HPP_
