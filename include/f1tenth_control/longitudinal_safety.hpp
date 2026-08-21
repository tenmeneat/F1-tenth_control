#pragma once

#include <algorithm>
#include <cmath>

namespace f1tenth_control {

// 명령 자체를 목표 쪽으로만 움직이는 비대칭 rate limiter.
// 실측 속도는 아래 ramp_lead_max 안티와인드업에서 별도로 사용한다. 분기 방향을
// target-current_speed로 정하면 실측이 목표보다 높은 동안 목표가 상승할 때
// last_cmd를 건너뛰고 target으로 clamp되는 역방향 계단이 생긴다(0821 23시 bag).
inline double rate_limit_speed_command(double last_cmd, double target, double dt,
                                       double max_accel, double max_decel) {
    const double safe_dt = std::max(0.0, dt);
    const double accel_step = std::max(0.0, max_accel) * safe_dt;
    const double decel_step = std::max(0.0, max_decel) * safe_dt;
    if (target > last_cmd) return std::min(target, last_cmd + accel_step);
    if (target < last_cmd) return std::max(target, last_cmd - decel_step);
    return target;
}

enum class HfiLaunchEvent {
    kNone,
    kStarted,
    kReleased,
    kRetryScheduled,
    kRetryStarted,
    kTimedOut,
    kFailureReset,
};

struct HfiLaunchGuardConfig {
    bool enabled = false;
    double speed_cap = 0.7;
    double exit_speed = 0.5;
    double standstill_speed = 0.1;
    double timeout = 4.0;
    double exit_hold = 0.1;
    double relatch_time = 0.5;
    double retry_cooldown = 0.5;
    unsigned int max_attempts = 2;
};

struct HfiLaunchGuardState {
    bool armed = true;
    bool active = false;
    bool retry_waiting = false;
    bool relatch_pending = false;
    bool failure_latched = false;
    double elapsed = 0.0;
    double exit_hold_elapsed = 0.0;
    double relatch_elapsed = 0.0;
    double retry_cooldown_elapsed = 0.0;
    unsigned int attempt = 0;
    unsigned long release_count = 0;
    unsigned long retry_count = 0;
    unsigned long attempt_timeout_count = 0;
    unsigned long failure_count = 0;
};

struct HfiLaunchDecision {
    bool constrain_to_cap = false;
    bool force_stop = false;
    HfiLaunchEvent event = HfiLaunchEvent::kNone;
};

inline HfiLaunchDecision update_hfi_launch_guard(
    HfiLaunchGuardState& state, const HfiLaunchGuardConfig& config,
    bool disengaged, double target_speed, double current_speed, double dt) {
    HfiLaunchDecision decision;
    if (!config.enabled) {
        state.active = false;
        state.retry_waiting = false;
        state.relatch_pending = false;
        state.failure_latched = false;
        state.elapsed = 0.0;
        state.exit_hold_elapsed = 0.0;
        state.relatch_elapsed = 0.0;
        state.retry_cooldown_elapsed = 0.0;
        state.attempt = 0;
        state.armed = true;
        return decision;
    }

    const double safe_dt = std::max(0.0, dt);
    const double abs_speed = std::abs(current_speed);
    const bool standstill = abs_speed < config.standstill_speed;
    const bool reset_requested = disengaged || target_speed <= 0.1;
    const double relatch_time = std::max(0.0, config.relatch_time);
    const unsigned int max_attempts = std::max(1U, config.max_attempts);

    // 순간적인 target=0이나 모드 채터로 즉시 재무장하지 않는다. 정지 명령(또는 자율
    // 미체결)과 실제 정지가 relatch_time 동안 함께 유지되어야 다음 출발을 허용한다.
    if (reset_requested) {
        // 출발 시도/완료 뒤의 정지 요청은 relatch dwell이 끝날 때까지 다음 양의
        // 목표를 차단한다. target 채터가 0을 한 번 찍은 직후 보호 없이 재출발하는
        // 우회 경로를 없앤다. 최초 기동의 armed=true 상태에는 이 대기를 추가하지 않는다.
        if (!state.armed) state.relatch_pending = true;
        state.active = false;
        state.retry_waiting = false;
        state.elapsed = 0.0;
        state.exit_hold_elapsed = 0.0;
        state.retry_cooldown_elapsed = 0.0;
        state.attempt = 0;

        if (standstill) {
            state.relatch_elapsed += safe_dt;
            if (state.relatch_elapsed >= relatch_time) {
                const bool reset_failure = state.failure_latched;
                state.armed = true;
                state.relatch_pending = false;
                state.failure_latched = false;
                if (reset_failure) decision.event = HfiLaunchEvent::kFailureReset;
            }
        } else {
            state.relatch_elapsed = 0.0;
            state.armed = false;
        }
        return decision;
    }
    state.relatch_elapsed = 0.0;

    if (state.relatch_pending) {
        decision.force_stop = true;
        return decision;
    }

    if (state.failure_latched) {
        decision.force_stop = true;
        return decision;
    }

    // 한 시도가 timeout되면 바로 다시 토크를 걸지 않고, 실제 정지가 cooldown 동안
    // 유지된 뒤에만 같은 저속 포착 시도를 한 번 더 한다.
    if (state.retry_waiting) {
        decision.force_stop = true;
        if (standstill) {
            state.retry_cooldown_elapsed += safe_dt;
        } else {
            state.retry_cooldown_elapsed = 0.0;
        }

        if (standstill &&
            state.retry_cooldown_elapsed >= std::max(0.0, config.retry_cooldown)) {
            state.retry_waiting = false;
            state.active = true;
            state.elapsed = 0.0;
            state.exit_hold_elapsed = 0.0;
            state.retry_cooldown_elapsed = 0.0;
            ++state.attempt;
            ++state.retry_count;
            decision.force_stop = false;
            decision.constrain_to_cap = true;
            decision.event = HfiLaunchEvent::kRetryStarted;
        }
        return decision;
    }

    if (state.active) {
        state.elapsed += safe_dt;

        // 전진 명령이므로 양의 속도만 성공이다. 역방향 -exit_speed는 절대 성공으로
        // 취급하지 않으며, 한 샘플 스파이크가 아니라 exit_hold 연속 유지를 요구한다.
        if (current_speed >= config.exit_speed) {
            state.exit_hold_elapsed += safe_dt;
        } else {
            state.exit_hold_elapsed = 0.0;
        }
        if (current_speed >= config.exit_speed &&
            state.exit_hold_elapsed >= std::max(0.0, config.exit_hold)) {
            state.active = false;
            state.armed = false;
            ++state.release_count;
            decision.event = HfiLaunchEvent::kReleased;
            return decision;
        }

        if (config.timeout > 0.0 && state.elapsed >= config.timeout) {
            state.active = false;
            state.armed = false;
            state.exit_hold_elapsed = 0.0;
            ++state.attempt_timeout_count;
            decision.force_stop = true;
            if (state.attempt < max_attempts) {
                state.retry_waiting = true;
                state.retry_cooldown_elapsed = 0.0;
                decision.event = HfiLaunchEvent::kRetryScheduled;
            } else {
                state.failure_latched = true;
                ++state.failure_count;
                decision.event = HfiLaunchEvent::kTimedOut;
            }
            return decision;
        }
        decision.constrain_to_cap = true;
        return decision;
    }

    if (state.armed && standstill) {
        state.active = true;
        state.armed = false;
        state.elapsed = 0.0;
        state.exit_hold_elapsed = 0.0;
        state.retry_cooldown_elapsed = 0.0;
        state.attempt = 1;
        decision.constrain_to_cap = true;
        decision.event = HfiLaunchEvent::kStarted;
    }
    return decision;
}

}  // namespace f1tenth_control
