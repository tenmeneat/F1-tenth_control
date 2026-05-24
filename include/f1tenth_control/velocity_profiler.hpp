#ifndef F1TENTH_CONTROL_VELOCITY_PROFILER_HPP_
#define F1TENTH_CONTROL_VELOCITY_PROFILER_HPP_

#include "f1tenth_control/types.hpp"
#include <vector>

namespace f1tenth_control {

class VelocityProfiler {
public:
    VelocityProfiler() = default;

    /**
     * Lissajous Figure-8 공식을 이용해 테스트용 글로벌 8자 궤적 생성
     */
    std::vector<Waypoint> generate_figure_eight_path(double scale_x, double scale_y, double max_speed);

    /**
     * 종방향/횡방향 물리 동역학 한계를 결합하여 최적 속도 프로파일 생성 (Forward-Backward Sweep)
     */
    void generate_velocity_profile(std::vector<Waypoint>& waypoints,
                                   double max_lat_accel,
                                   double max_speed,
                                   double min_speed,
                                   double base_max_accel,
                                   double base_max_decel);
};

} // namespace f1tenth_control

#endif // F1TENTH_CONTROL_VELOCITY_PROFILER_HPP_
