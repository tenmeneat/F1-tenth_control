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

TEST(HfiLaunchGuard, RelatchNeedsContinuousStopCommandAndStandstill) {
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

    // 0.48초 뒤 목표가 튀어도 보호 없이 출발하지 않고, 재무장 대기를 처음부터 다시 센다.
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    EXPECT_TRUE(d.force_stop);
    EXPECT_FALSE(state.active);

    for (int i = 0; i < 25; ++i) {
        d = fc::update_hfi_launch_guard(state, cfg, false, 0.0, 0.0, 0.02);
    }
    EXPECT_TRUE(state.armed);
    EXPECT_FALSE(state.relatch_pending);

    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
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

    // 양의 목표 한 번으로 stop dwell이 끊기고 실패 래치가 계속 출력을 막는다.
    d = fc::update_hfi_launch_guard(state, cfg, false, 2.0, 0.0, 0.02);
    EXPECT_TRUE(d.force_stop);
    d = fc::update_hfi_launch_guard(state, cfg, false, 0.0, 0.0, 0.25);
    EXPECT_TRUE(state.failure_latched);
    d = fc::update_hfi_launch_guard(state, cfg, false, 0.0, 0.0, 0.25);
    EXPECT_EQ(d.event, fc::HfiLaunchEvent::kFailureReset);
    EXPECT_FALSE(state.failure_latched);
    EXPECT_TRUE(state.armed);
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
