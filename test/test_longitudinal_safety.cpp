#include <gtest/gtest.h>

#include "f1tenth_control/longitudinal_safety.hpp"

namespace fc = f1tenth_control;

TEST(LongitudinalRateLimit, TargetRiseWhileMeasuredWouldStillBeAboveTarget) {
    // 0821 23시 bag 회귀: 구 구현은 실측 2.72 > target 2.606을 감속 분기로 본 뒤
    // out < target clamp를 걸어 1.537 -> 2.606 (53.4 m/s^2) 계단을 만들었다.
    const double out = fc::rate_limit_speed_command(1.537, 2.606, 0.02, 4.1, 8.0);
    EXPECT_NEAR(out, 1.619, 1e-12);
    EXPECT_LE((out - 1.537) / 0.02, 4.1 + 1e-12);
}

TEST(LongitudinalRateLimit, StopReleaseCannotJumpToNewTarget) {
    const double out = fc::rate_limit_speed_command(0.0, 1.7, 0.02, 4.1, 8.0);
    EXPECT_NEAR(out, 0.082, 1e-12);
}

TEST(LongitudinalRateLimit, DecelerationUsesIndependentLimit) {
    const double out = fc::rate_limit_speed_command(2.0, 0.0, 0.02, 4.1, 8.0);
    EXPECT_NEAR(out, 1.84, 1e-12);
}

namespace {

fc::HfiLaunchGuardConfig test_hfi_config() {
    fc::HfiLaunchGuardConfig cfg;
    cfg.enabled = true;
    cfg.timeout = 0.06;
    cfg.exit_hold = 0.02;
    cfg.relatch_time = 0.5;
    cfg.retry_cooldown = 0.04;
    cfg.max_attempts = 2;
    return cfg;
}

}  // namespace

TEST(HfiLaunchGuard, ReleasesAtDeadlineBeforeTimeout) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    EXPECT_TRUE(d.constrain_to_cap);

    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    EXPECT_TRUE(d.constrain_to_cap);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    EXPECT_TRUE(d.constrain_to_cap);

    // 다음 호출은 timeout 경계지만 성공 검사를 먼저 하므로 release가 이긴다.
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kReleased);
    EXPECT_FALSE(d.force_stop);
    EXPECT_EQ(state.release_count, 1UL);
}

TEST(HfiLaunchGuard, ReverseMotionCanNeverReleaseForwardLaunch) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();
    cfg.timeout = 1.0;
    cfg.exit_hold = 0.04;

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kStarted);

    for (int i = 0; i < 10; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, -0.8, 0.02);
        EXPECT_TRUE(d.constrain_to_cap);
        EXPECT_NE(d.event, fc::HfiLaunchEvent::kReleased);
    }
    EXPECT_TRUE(state.active);

    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kNone);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kReleased);
}

TEST(HfiLaunchGuard, ExitSpeedMustBeContinuous) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();
    cfg.timeout = 1.0;
    cfg.exit_hold = 0.04;

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kNone);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.49, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kNone);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kNone);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kReleased);
}

TEST(HfiLaunchGuard, PositiveTargetCompletesGuardedRelatchWithoutDeadlock) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();
    cfg.timeout = 1.0;

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kReleased);

    for (int i = 0; i < 24; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, false, 0.0, 0.0, 0.02);
    }
    EXPECT_FALSE(state.armed);
    EXPECT_TRUE(state.relatch_pending);

    // 0.48초 뒤 raw 목표가 복귀해도 그 직전까지 실제 발행은 계속 0이었다. 이번 dt로
    // 0.50초 dwell이 완성되면 교착하지 않고 제한 출발로 바로 이어진다.
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    EXPECT_FALSE(d.force_stop);
    EXPECT_TRUE(d.constrain_to_cap);
    EXPECT_TRUE(state.active);
    EXPECT_FALSE(state.relatch_pending);
}

TEST(HfiLaunchGuard, PositiveTargetWaitsAtZeroUntilFullRelatchDwell) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();
    cfg.timeout = 1.0;

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kReleased);

    // Safe-stop 목표가 잠깐만 0이었다가 크립 목표로 돌아온 0822 회귀 시나리오.
    d = fc::update_hfi_launch_guard(state, cfg, false, 0.0, 0.3, 0.02);
    ASSERT_TRUE(state.relatch_pending);
    for (int i = 0; i < 24; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, false, 0.7, 0.0, 0.02);
        EXPECT_TRUE(d.force_stop) << "cycle=" << i;
        EXPECT_FALSE(d.constrain_to_cap);
    }

    d = fc::update_hfi_launch_guard(state, cfg, false, 0.7, 0.0, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    EXPECT_TRUE(d.constrain_to_cap);
    EXPECT_FALSE(d.force_stop);
}

TEST(HfiLaunchGuard, PlannerStopIntentBlocksPositivePrefixUntilHandoff) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();
    cfg.timeout = 1.0;

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kReleased);

    // 0822 safe-stop은 열린 경로 [1.90, ..., 0.00] 뒤 [0.95, 0.00]을 발행했다.
    // lookahead target 0.95만 보면 재출발하지만 말단 0은 플래너의 정지 래치다.
    d = fc::update_hfi_launch_guard(
        state, cfg, false, 1.90, 1.5, 0.02, /*launch_allowed=*/false);
    EXPECT_FALSE(d.force_stop);  // 움직이는 동안에는 플래너의 감속 경로를 그대로 추종
    for (int i = 0; i < 30; ++i) {
        d = fc::update_hfi_launch_guard(
            state, cfg, false, 0.95, 0.0, 0.02, /*launch_allowed=*/false);
        EXPECT_NE(d.event, fc::HfiLaunchEvent::kStarted);
        EXPECT_TRUE(d.force_stop);
    }

    // 플래너가 closed handoff/0.7 m/s creep 경로를 발행한 뒤에만 출발한다.
    d = fc::update_hfi_launch_guard(
        state, cfg, false, 0.7, 0.0, 0.02, /*launch_allowed=*/true);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    EXPECT_TRUE(d.constrain_to_cap);
    EXPECT_FALSE(d.force_stop);
}

TEST(HfiLaunchGuard, ForwardMotionBypassesStationaryRelatchWithoutZeroStep) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();
    cfg.timeout = 1.0;
    cfg.moving_bypass_speed = 1.0;
    cfg.moving_bypass_hold = 0.1;

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kReleased);

    // 수동 전환/순간 stop 목표 중에도 VESC가 +2 m/s로 주행했다. dwell은 reset 구간에서
    // 미리 쌓고, 자율 목표가 돌아온 사이클에는 0이나 0.7 cap을 삽입하지 않는다.
    for (int i = 0; i < 5; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, true, 0.0, 2.0, 0.02);
    }
    ASSERT_TRUE(state.relatch_pending);
    d = fc::update_hfi_launch_guard(state, cfg, false, 4.0, 2.0, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kMovingBypass);
    EXPECT_FALSE(d.force_stop);
    EXPECT_FALSE(d.constrain_to_cap);
    EXPECT_FALSE(state.relatch_pending);
    EXPECT_EQ(state.moving_bypass_count, 1UL);
}

TEST(HfiLaunchGuard, ReverseMotionCanNeverUseMovingBypass) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();
    cfg.timeout = 1.0;
    cfg.moving_bypass_speed = 1.0;
    cfg.moving_bypass_hold = 0.1;

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kReleased);
    for (int i = 0; i < 10; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, true, 0.0, -2.0, 0.02);
    }

    d = fc::update_hfi_launch_guard(state, cfg, false, 4.0, -2.0, 0.02);
    EXPECT_TRUE(d.force_stop);
    EXPECT_NE(d.event, fc::HfiLaunchEvent::kMovingBypass);
    EXPECT_TRUE(state.relatch_pending);
}

TEST(HfiLaunchGuard, DisengagedStartupCannotCreateFalseLaunch) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();
    cfg.timeout = 1.0;

    for (int i = 0; i < 50; ++i) {
        const auto d = fc::update_hfi_launch_guard(
            state, cfg, true, 3.0, 0.0, 0.02);
        EXPECT_NE(d.event, fc::HfiLaunchEvent::kStarted);
        EXPECT_FALSE(state.active);
    }
    const auto d = fc::update_hfi_launch_guard(
        state, cfg, false, 3.0, 0.0, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    EXPECT_TRUE(d.constrain_to_cap);
}

TEST(HfiLaunchGuard, RelatchStandstillDwellResetsOnWheelMotion) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();
    cfg.timeout = 1.0;

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kReleased);

    for (int i = 0; i < 20; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, false, 0.0, 0.0, 0.02);
    }
    ASSERT_FALSE(state.armed);
    // 목표는 계속 0이어도 VESC가 standstill 문턱을 벗어나면 0.4초 누적을 폐기한다.
    d = fc::update_hfi_launch_guard(state, cfg, false, 0.0, 0.11, 0.02);
    EXPECT_DOUBLE_EQ(state.relatch_elapsed, 0.0);

    for (int i = 0; i < 24; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, false, 0.0, 0.0, 0.02);
    }
    EXPECT_FALSE(state.armed);
    d = fc::update_hfi_launch_guard(state, cfg, false, 0.0, 0.0, 0.02);
    EXPECT_TRUE(state.armed);
}

TEST(HfiLaunchGuard, OneBoundedRetryThenTerminalLatch) {
    fc::HfiLaunchGuardState state;
    const auto cfg = test_hfi_config();

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    for (int i = 0; i < 3; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    }
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kRetryScheduled);
    EXPECT_TRUE(d.force_stop);
    EXPECT_TRUE(state.retry_waiting);

    // 역방향으로 흔들리는 동안에는 cooldown을 세지 않는다.
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, -0.2, 0.02);
    EXPECT_TRUE(d.force_stop);
    EXPECT_DOUBLE_EQ(state.retry_cooldown_elapsed, 0.0);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kNone);
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kRetryStarted);
    EXPECT_TRUE(d.constrain_to_cap);
    EXPECT_EQ(state.attempt, 2U);

    for (int i = 0; i < 3; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    }
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kTimedOut);
    EXPECT_TRUE(d.force_stop);
    EXPECT_TRUE(state.failure_latched);
    EXPECT_EQ(state.retry_count, 1UL);
    EXPECT_EQ(state.attempt_timeout_count, 2UL);
    EXPECT_EQ(state.failure_count, 1UL);

    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    EXPECT_TRUE(d.force_stop);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kNone);
}

TEST(HfiLaunchGuard, TerminalLatchNeedsHalfSecondStopBeforeReset) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();
    cfg.max_attempts = 1;

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    for (int i = 0; i < 3; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    }
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kTimedOut);

    d = fc::update_hfi_launch_guard(state, cfg, false, 0.0, 0.0, 0.25);
    EXPECT_TRUE(state.failure_latched);
    EXPECT_NE(d.event, fc::HfiLaunchEvent::kFailureReset);

    // 명시적인 stop 요청이 pending을 만든 뒤에는 raw 목표가 돌아와도 guard가 발행 0을
    // 유지하므로 남은 dwell을 완성할 수 있다. 완료 즉시 새 bounded 시도를 시작한다.
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.25);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    EXPECT_FALSE(state.failure_latched);
    EXPECT_TRUE(state.active);
    EXPECT_TRUE(d.constrain_to_cap);
}

TEST(HfiLaunchGuard, Observed0822CapturesUnderFourSecondsCanRelease) {
    auto cfg = test_hfi_config();
    cfg.timeout = 4.0;
    cfg.exit_hold = 0.1;
    const double observed_release_seconds[] = {
        0.340, 0.480, 0.980, 1.000, 1.040, 1.040, 2.140,
        2.256, 2.860, 3.260, 3.300, 3.380, 3.480};

    for (const double release_time : observed_release_seconds) {
        fc::HfiLaunchGuardState state;
        auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
        ASSERT_EQ(d.event, fc::HfiLaunchEvent::kStarted);

        double elapsed = 0.0;
        while (elapsed + 0.12 < release_time) {
            d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
            ASSERT_FALSE(d.force_stop) << "release_time=" << release_time;
            elapsed += 0.02;
        }
        for (int i = 0; i < 5; ++i) {
            d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.5, 0.02);
        }
        EXPECT_EQ(d.event, fc::HfiLaunchEvent::kReleased)
            << "release_time=" << release_time;
        EXPECT_FALSE(d.force_stop);
        EXPECT_LT(state.elapsed, 4.0);
    }
}

TEST(HfiLaunchGuard, ObservedLongCaptureIsCutOffAndRetried) {
    fc::HfiLaunchGuardState state;
    auto cfg = test_hfi_config();
    cfg.timeout = 4.0;
    cfg.exit_hold = 0.1;

    auto d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    ASSERT_EQ(d.event, fc::HfiLaunchEvent::kStarted);
    for (int i = 0; i < 200; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    }
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kRetryScheduled);
    EXPECT_TRUE(d.force_stop);
    EXPECT_TRUE(state.retry_waiting);
}
