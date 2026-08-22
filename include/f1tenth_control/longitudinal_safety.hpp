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
    kMovingBypass,
    kRetryScheduled,
    kRetryStarted,
    kTimedOut,
    kFailureReset,
};

enum class HfiLaunchFailureReason {
    kNone,
    kTimeout,
    kNoForwardProgress,
    kSustainedReverse,
    kPostReleaseDropout,
};

struct HfiLaunchGuardConfig {
    bool enabled = false;
    // 4420 ERPM/(m/s) 기준 0.7 m/s는 3094 ERPM으로 VESC의 HFI→sensorless 전환
    // 문턱(3000 ERPM)에 3%밖에 여유가 없었다. 060605/060751 실차에서 이 상한에
    // 머문 시도가 반복적으로 제자리 덜걱임을 보였으므로 0.9(3978 ERPM)로 둔다.
    double speed_cap = 0.9;
    double exit_speed = 0.5;
    // 정지 진입/해제 Schmitt trigger. 0822 VESC /odom의 0.13~0.15 m/s 정지 노이즈가
    // 0.5초 재무장 dwell을 계속 초기화하지 않되, 실제 0.20 m/s 이상 움직임은 즉시 깬다.
    double standstill_speed = 0.12;
    double standstill_exit_speed = 0.20;
    double standstill_filter_tau = 0.10;
    double timeout = 4.0;
    double exit_hold = 0.3;
    // exit_hold를 통과한 직후 sensorless 동기를 다시 잃는 0822 실차 회귀를 잡는다.
    // 이 시간 동안 속도가 post_release_min_speed 아래로 떨어지면 성공을 취소하고
    // 동일한 bounded retry/failure-latch 경로로 보낸다. 0 이하면 감시 비활성.
    double post_release_monitor_time = 0.5;
    double post_release_min_speed = 0.20;
    double relatch_time = 0.5;
    // 이미 전진 중인 수동→자율/일시 정지명령 복귀는 정지출발이 아니다. 이 속도를
    // 연속 유지하면 relatch_pending을 우회한다. 0 이하면 우회 비활성.
    double moving_bypass_speed = 0.5;
    double moving_bypass_hold = 0.1;
    double retry_cooldown = 0.5;
    unsigned int max_attempts = 2;
    // 060751에서 정상 포착도 1.58~1.62초가 걸렸고 1.2초 watchdog이 아직 살아날
    // 시도를 강제로 0으로 잘랐다. 2초까지 5 cm도 전진하지 못한 시도만 조기 중단한다.
    double no_progress_timeout = 2.0;
    double no_progress_min_distance = 0.05;
    double reverse_abort_speed = 0.10;
    double reverse_abort_hold = 0.15;
};

struct HfiLaunchGuardState {
    bool armed = true;
    bool active = false;
    bool retry_waiting = false;
    bool relatch_pending = false;
    bool failure_latched = false;
    bool post_release_monitoring = false;
    bool standstill_filter_initialized = false;
    bool standstill_latched = true;
    double elapsed = 0.0;
    double exit_hold_elapsed = 0.0;
    double post_release_elapsed = 0.0;
    double relatch_elapsed = 0.0;
    double moving_bypass_elapsed = 0.0;
    double retry_cooldown_elapsed = 0.0;
    double standstill_filtered_speed = 0.0;
    double launch_signed_distance = 0.0;
    double reverse_elapsed = 0.0;
    HfiLaunchFailureReason last_failure_reason = HfiLaunchFailureReason::kNone;
    unsigned int attempt = 0;
    unsigned long release_count = 0;
    unsigned long moving_bypass_count = 0;
    unsigned long retry_count = 0;
    unsigned long attempt_timeout_count = 0;
    unsigned long failure_count = 0;
    unsigned long no_progress_abort_count = 0;
    unsigned long reverse_abort_count = 0;
    unsigned long post_release_dropout_count = 0;
};

struct HfiLaunchDecision {
    bool constrain_to_cap = false;
    bool force_stop = false;
    HfiLaunchEvent event = HfiLaunchEvent::kNone;
};

inline HfiLaunchDecision update_hfi_launch_guard(
    HfiLaunchGuardState& state, const HfiLaunchGuardConfig& config,
    bool disengaged, double target_speed, double current_speed, double dt,
    bool launch_allowed = true) {
    HfiLaunchDecision decision;
    if (!config.enabled) {
        state.active = false;
        state.retry_waiting = false;
        state.relatch_pending = false;
        state.failure_latched = false;
        state.post_release_monitoring = false;
        state.elapsed = 0.0;
        state.exit_hold_elapsed = 0.0;
        state.post_release_elapsed = 0.0;
        state.relatch_elapsed = 0.0;
        state.moving_bypass_elapsed = 0.0;
        state.retry_cooldown_elapsed = 0.0;
        state.standstill_filter_initialized = false;
        state.standstill_latched = true;
        state.standstill_filtered_speed = 0.0;
        state.launch_signed_distance = 0.0;
        state.reverse_elapsed = 0.0;
        state.last_failure_reason = HfiLaunchFailureReason::kNone;
        state.attempt = 0;
        state.armed = true;
        return decision;
    }

    const double safe_dt = std::max(0.0, dt);
    const double abs_speed = std::abs(current_speed);
    const double standstill_enter = std::max(0.0, config.standstill_speed);
    const double standstill_exit = std::max(
        standstill_enter, config.standstill_exit_speed);
    const double standstill_filter_tau = std::max(0.0, config.standstill_filter_tau);
    if (!state.standstill_filter_initialized) {
        state.standstill_filtered_speed = abs_speed;
        state.standstill_latched = abs_speed < standstill_enter;
        state.standstill_filter_initialized = true;
    } else if (safe_dt > 0.0) {
        const double alpha = standstill_filter_tau > 0.0
            ? 1.0 - std::exp(-safe_dt / standstill_filter_tau)
            : 1.0;
        state.standstill_filtered_speed +=
            alpha * (abs_speed - state.standstill_filtered_speed);
        if (state.standstill_latched) {
            // 실제 exit 이상 움직임은 필터 지연 없이 즉시 정지 판정을 해제한다.
            if (abs_speed >= standstill_exit ||
                state.standstill_filtered_speed >= standstill_exit) {
                state.standstill_latched = false;
            }
        } else if (abs_speed < standstill_enter ||
                   state.standstill_filtered_speed < standstill_enter) {
            // 0속도 명령 직후 실제 raw 속도가 enter 아래로 들어오면 즉시 dwell을 시작한다.
            // 이후 0.13~0.15 잡음은 exit(0.20) 미만이므로 dwell을 깨지 않는다.
            state.standstill_latched = true;
        }
    }
    const bool standstill = state.standstill_latched;
    // 열린 safe-stop 경로는 앞쪽 감속 웨이포인트가 양수여도 말단 vx=0이 플래너의
    // 정지 의도다. launch_allowed=false 동안에는 그 중간 양수값으로 HFI를 재기동하지 않는다.
    const bool reset_requested = disengaged || target_speed <= 0.1 || !launch_allowed;
    const double relatch_time = std::max(0.0, config.relatch_time);
    const bool moving_bypass_enabled = config.moving_bypass_speed > 0.0;
    const bool moving_forward = moving_bypass_enabled &&
        current_speed >= config.moving_bypass_speed;
    const double moving_bypass_hold = std::max(0.0, config.moving_bypass_hold);
    const unsigned int max_attempts = std::max(1U, config.max_attempts);

    // 순간적인 target=0이나 모드 채터로 즉시 재무장하지 않는다. 정지 명령(또는 자율
    // 미체결)과 실제 정지가 relatch_time 동안 함께 유지되어야 다음 출발을 허용한다.
    if (reset_requested) {
        // 출발 시도/완료 뒤의 정지 요청은 relatch dwell이 끝날 때까지 다음 양의
        // 목표를 차단한다. target 채터가 0을 한 번 찍은 직후 보호 없이 재출발하는
        // 우회 경로를 없앤다. 최초 기동의 armed=true 상태에는 이 대기를 추가하지 않는다.
        // 최초 armed 상태라도 수동 주행/타력 중이면 다음 체결은 정지출발이 아니다. 첫
        // reset 사이클부터 pending을 세워 아래 signed moving bypass로만 통과시킨다.
        if (!state.armed || !standstill) state.relatch_pending = true;
        state.active = false;
        state.retry_waiting = false;
        state.post_release_monitoring = false;
        state.elapsed = 0.0;
        state.exit_hold_elapsed = 0.0;
        state.post_release_elapsed = 0.0;
        state.retry_cooldown_elapsed = 0.0;
        state.launch_signed_distance = 0.0;
        state.reverse_elapsed = 0.0;
        state.last_failure_reason = HfiLaunchFailureReason::kNone;
        state.attempt = 0;

        if (moving_forward) {
            state.moving_bypass_elapsed += safe_dt;
        } else {
            state.moving_bypass_elapsed = 0.0;
        }

        if (standstill) {
            state.relatch_elapsed += safe_dt;
            if (state.relatch_elapsed >= relatch_time) {
                const bool reset_failure = state.failure_latched;
                state.armed = true;
                state.relatch_pending = false;
                state.failure_latched = false;
                state.moving_bypass_elapsed = 0.0;
                if (reset_failure) decision.event = HfiLaunchEvent::kFailureReset;
            }
        } else {
            state.relatch_elapsed = 0.0;
            state.armed = false;
        }
        // 감속 경로는 움직이는 동안 그대로 추종하고, 실제 정지한 뒤에만 0을 고정한다.
        // 플래너가 closed handoff/creep 경로로 바꿔 launch_allowed=true가 되면 armed 상태에서
        // 정상 HFI 출발을 시작한다. 이 게이트가 없으면 safe-stop의 앞쪽 양수 waypoint만 보고
        // 4초 안전 래치를 조기에 우회할 수 있다.
        if (!launch_allowed && (standstill || abs_speed < standstill_enter)) {
            decision.force_stop = true;
        }
        return decision;
    }

    if (state.relatch_pending) {
        // 수동→자율 전환 또는 순간 target=0 뒤에도 차가 충분히 빠르게 **전진 중**이면
        // HFI 정지출발을 다시 걸 이유가 없다. abs(speed)를 쓰지 않아 역주행은 우회하지
        // 못한다. reset_requested 구간에서 이미 쌓인 dwell도 이어받아 bumpless transfer한다.
        if (!state.failure_latched && moving_forward) {
            state.relatch_elapsed = 0.0;
            state.moving_bypass_elapsed += safe_dt;
            // 1 m/s 이상은 HFI 포착구간을 충분히 벗어난 실측이므로 hold를 확인하는 동안도
            // 0/brake를 삽입하지 않는다. 그렇지 않으면 바로 그 전환 계단이 새 위험이 된다.
            if (state.moving_bypass_elapsed >= moving_bypass_hold) {
                state.relatch_pending = false;
                state.armed = false;
                state.moving_bypass_elapsed = 0.0;
                ++state.moving_bypass_count;
                decision.event = HfiLaunchEvent::kMovingBypass;
            }
            return decision;
        }

        state.moving_bypass_elapsed = 0.0;
        decision.force_stop = true;

        // 핵심 교착 수리: raw target이 다시 양수여도 이 분기 자체가 실제 발행을 0으로
        // 강제하고 있으므로, VESC도 정지해 있으면 유효한 "정지 명령+실측 정지" dwell이다.
        // 종전 코드는 양수 target에서 relatch_elapsed를 매번 0으로 만들어 영원히 못 풀렸다.
        if (standstill) {
            state.relatch_elapsed += safe_dt;
        } else {
            state.relatch_elapsed = 0.0;
        }
        if (standstill && state.relatch_elapsed >= relatch_time) {
            state.relatch_pending = false;
            state.failure_latched = false;
            state.relatch_elapsed = 0.0;
            state.armed = false;
            state.active = true;
            state.elapsed = 0.0;
            state.exit_hold_elapsed = 0.0;
            state.post_release_elapsed = 0.0;
            state.retry_cooldown_elapsed = 0.0;
            state.launch_signed_distance = 0.0;
            state.reverse_elapsed = 0.0;
            state.last_failure_reason = HfiLaunchFailureReason::kNone;
            state.attempt = 1;
            decision.force_stop = false;
            decision.constrain_to_cap = true;
            decision.event = HfiLaunchEvent::kStarted;
        }
        return decision;
    }

    state.relatch_elapsed = 0.0;
    state.moving_bypass_elapsed = 0.0;

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
            state.launch_signed_distance = 0.0;
            state.reverse_elapsed = 0.0;
            state.last_failure_reason = HfiLaunchFailureReason::kNone;
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
        state.launch_signed_distance += current_speed * safe_dt;
        if (config.reverse_abort_speed > 0.0 &&
            current_speed <= -config.reverse_abort_speed) {
            state.reverse_elapsed += safe_dt;
        } else {
            state.reverse_elapsed = 0.0;
        }

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
            state.post_release_monitoring = config.post_release_monitor_time > 0.0;
            state.post_release_elapsed = 0.0;
            ++state.release_count;
            decision.event = HfiLaunchEvent::kReleased;
            return decision;
        }

        const bool sustained_reverse = config.reverse_abort_hold > 0.0 &&
            state.reverse_elapsed >= config.reverse_abort_hold;
        const bool no_forward_progress = config.no_progress_timeout > 0.0 &&
            state.elapsed >= config.no_progress_timeout &&
            state.launch_signed_distance < config.no_progress_min_distance;
        // timeout 직전에 exit_speed를 처음 넘은 경우에는 연속 hold를 끝낼 기회를 준다.
        // 060605에서는 2차 시도가 4.00초에 +0.55 m/s까지 실제로 관통했는데, hold가
        // 한 샘플만 쌓였다는 이유로 같은 사이클에 최종 실패 래치됐다. 속도가 다시
        // 문턱 아래로 내려가면 exit_hold_elapsed가 0이 되어 다음 사이클 즉시 실패한다.
        const bool hard_timeout = config.timeout > 0.0 &&
            state.elapsed >= config.timeout && state.exit_hold_elapsed <= 0.0;
        if (sustained_reverse || no_forward_progress || hard_timeout) {
            state.active = false;
            state.armed = false;
            state.exit_hold_elapsed = 0.0;
            ++state.attempt_timeout_count;
            if (sustained_reverse) {
                state.last_failure_reason = HfiLaunchFailureReason::kSustainedReverse;
                ++state.reverse_abort_count;
            } else if (no_forward_progress) {
                state.last_failure_reason = HfiLaunchFailureReason::kNoForwardProgress;
                ++state.no_progress_abort_count;
            } else {
                state.last_failure_reason = HfiLaunchFailureReason::kTimeout;
            }
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

    // exit_speed/exit_hold를 통과한 뒤에도 짧게 실제 VESC 속도를 감시한다. 0822
    // run_152508은 +0.5 m/s를 0.14초 유지해 성공 처리됐지만 직후 +0.07 m/s로
    // 붕괴했고, armed=false 상태라 이후 13k ERPM 명령 중에도 재시도하지 못했다.
    if (state.post_release_monitoring) {
        state.post_release_elapsed += safe_dt;
        if (current_speed < std::max(0.0, config.post_release_min_speed)) {
            state.post_release_monitoring = false;
            state.post_release_elapsed = 0.0;
            state.last_failure_reason = HfiLaunchFailureReason::kPostReleaseDropout;
            ++state.attempt_timeout_count;
            ++state.post_release_dropout_count;
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
        if (state.post_release_elapsed >=
            std::max(0.0, config.post_release_monitor_time)) {
            state.post_release_monitoring = false;
            state.post_release_elapsed = 0.0;
        }
        return decision;
    }

    if (state.armed && standstill) {
        state.active = true;
        state.armed = false;
        state.elapsed = 0.0;
        state.exit_hold_elapsed = 0.0;
        state.retry_cooldown_elapsed = 0.0;
        state.launch_signed_distance = 0.0;
        state.reverse_elapsed = 0.0;
        state.last_failure_reason = HfiLaunchFailureReason::kNone;
        state.attempt = 1;
        decision.constrain_to_cap = true;
        decision.event = HfiLaunchEvent::kStarted;
    }
    return decision;
}

}  // namespace f1tenth_control
