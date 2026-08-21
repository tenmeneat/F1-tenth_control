#include <gtest/gtest.h>

#include <cmath>

#include "f1tenth_control/cruise_controller.hpp"

namespace
{

f1tenth_control::CruiseControllerConfig defaultConfig()
{
  f1tenth_control::CruiseControllerConfig config;
  config.maximum_speed = 12.0;
  config.emergency_stop_distance = 0.45;
  config.relative_deceleration = 2.5;
  config.proportional_gain = 1.0;
  config.integral_gain = 0.0;
  config.derivative_gain = 0.2;
  config.uncertainty_sigma = 0.0;
  return config;
}

TEST(CruiseController, MatchesOpponentAtDesiredGap)
{
  f1tenth_control::CruiseLongitudinalController controller(defaultConfig());
  const auto output = controller.update({1.5, 1.5, 4.0, 4.0, 0.0, 0.02});
  EXPECT_NEAR(output.speed_limit, 4.0, 1e-9);
}

TEST(CruiseController, AllowsCatchupButAppliesBrakingDistanceCap)
{
  f1tenth_control::CruiseLongitudinalController controller(defaultConfig());
  const auto output = controller.update({10.0, 1.5, 5.0, 4.0, 0.0, 0.02});
  EXPECT_GT(output.speed_limit, 4.0);
  EXPECT_LT(output.speed_limit, 9.0);
}

TEST(CruiseController, SlowsWhenClosingInsideDesiredGap)
{
  f1tenth_control::CruiseLongitudinalController controller(defaultConfig());
  const auto output = controller.update({1.0, 1.5, 6.0, 4.0, 0.0, 0.02});
  EXPECT_LT(output.speed_limit, 4.0);
  EXPECT_GE(output.speed_limit, 0.0);
}

TEST(CruiseController, StopsInsideEmergencyDistance)
{
  f1tenth_control::CruiseLongitudinalController controller(defaultConfig());
  const auto output = controller.update({0.4, 1.5, 5.0, 3.0, 0.0, 0.02});
  EXPECT_DOUBLE_EQ(output.speed_limit, 0.0);
}

TEST(CruiseController, PositionalUncertaintyReducesUsableGap)
{
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;
  f1tenth_control::CruiseLongitudinalController controller(config);
  const auto certain = controller.update({2.0, 1.5, 4.0, 4.0, 0.0, 0.02});
  controller.reset();
  const auto uncertain = controller.update({2.0, 1.5, 4.0, 4.0, 0.04, 0.02});
  EXPECT_LT(uncertain.speed_limit, certain.speed_limit);
}

TEST(CruiseController, DefaultHorizonZeroIsBitIdenticalWithOrWithoutSpeedVariance)
{
  // gap_uncertainty_horizon_max defaults to 0.0 -> tau_s collapses to 0 -> opponent_vs_variance
  // must have zero effect on output, matching the pre-time-propagation formula exactly.
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;
  f1tenth_control::CruiseLongitudinalController without_vs_var(config);
  const auto baseline = without_vs_var.update({2.0, 1.5, 4.0, 4.0, 0.04, 0.02, 0.0});

  f1tenth_control::CruiseLongitudinalController with_vs_var(config);
  const auto with_variance = with_vs_var.update({2.0, 1.5, 4.0, 4.0, 0.04, 0.02, 0.5});

  EXPECT_DOUBLE_EQ(with_variance.speed_limit, baseline.speed_limit);
  EXPECT_DOUBLE_EQ(with_variance.effective_gap, baseline.effective_gap);
}

TEST(CruiseController, SpeedVarianceTighensGapUnderNonzeroHorizon)
{
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;
  config.gap_uncertainty_horizon_max = 1.0;
  config.relative_deceleration = 2.5;

  f1tenth_control::CruiseLongitudinalController without_vs_var(config);
  const auto without_variance = without_vs_var.update({4.0, 1.5, 4.0, 2.0, 0.04, 0.02, 0.0});

  f1tenth_control::CruiseLongitudinalController with_vs_var(config);
  const auto with_variance = with_vs_var.update({4.0, 1.5, 4.0, 2.0, 0.04, 0.02, 0.5});

  // Non-zero opponent speed variance widens sigma_g(tau_s>0), which can only shrink (never grow)
  // the reported effective_gap relative to the same scenario without it.
  EXPECT_LT(with_variance.effective_gap, without_variance.effective_gap);
}

TEST(CruiseController, CrossCovarianceShiftsGapUnderNonzeroHorizon)
{
  // |s_vs_cov| <= sqrt(s_var*vs_var) = sqrt(0.04*0.5) = 0.1414 stays inside the detector's
  // Cauchy-Schwarz bound. A positive cov must widen sigma_g (2*tau*cov > 0) and hence shrink
  // effective_gap relative to cov=0, and a negative cov must do the opposite.
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;
  config.gap_uncertainty_horizon_max = 1.0;
  config.relative_deceleration = 2.5;

  f1tenth_control::CruiseLongitudinalController zero_cov(config);
  const auto baseline = zero_cov.update({4.0, 1.5, 4.0, 2.0, 0.04, 0.02, 0.5, 0.0});

  f1tenth_control::CruiseLongitudinalController pos_cov(config);
  const auto with_pos_cov = pos_cov.update({4.0, 1.5, 4.0, 2.0, 0.04, 0.02, 0.5, 0.1});

  f1tenth_control::CruiseLongitudinalController neg_cov(config);
  const auto with_neg_cov = neg_cov.update({4.0, 1.5, 4.0, 2.0, 0.04, 0.02, 0.5, -0.1});

  EXPECT_LT(with_pos_cov.effective_gap, baseline.effective_gap);
  EXPECT_GT(with_neg_cov.effective_gap, baseline.effective_gap);
}

TEST(CruiseController, CrossCovarianceHasNoEffectAtZeroHorizon)
{
  // tau_s collapses to 0 whenever gap_uncertainty_horizon_max is 0 (default), so the
  // 2*tau*s_vs_cov term vanishes regardless of the cov's magnitude or sign.
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;

  f1tenth_control::CruiseLongitudinalController zero_cov(config);
  const auto baseline = zero_cov.update({2.0, 1.5, 4.0, 4.0, 0.04, 0.02, 0.5, 0.0});

  f1tenth_control::CruiseLongitudinalController with_cov(config);
  const auto with_variance = with_cov.update({2.0, 1.5, 4.0, 4.0, 0.04, 0.02, 0.5, 0.1});

  EXPECT_DOUBLE_EQ(with_variance.speed_limit, baseline.speed_limit);
  EXPECT_DOUBLE_EQ(with_variance.effective_gap, baseline.effective_gap);
}

TEST(CruiseController, EmergencyStopIgnoresTimePropagation)
{
  // Emergency stop must use tau=0 (sigma_s0) regardless of gap_uncertainty_horizon_max/
  // opp_speed_confidence_z, so a large opponent speed variance must not delay the stop.
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;
  config.emergency_stop_distance = 0.45;
  config.gap_uncertainty_horizon_max = 1.0;
  config.opp_speed_confidence_z = 1.0;
  f1tenth_control::CruiseLongitudinalController controller(config);
  const auto output = controller.update({0.4, 1.5, 5.0, 3.0, 0.0, 0.02, 4.0});
  EXPECT_DOUBLE_EQ(output.speed_limit, 0.0);
}

// --- Braking-distance decomposition -----------------------------------------------------------

TEST(CruiseController, DecomposedBrakingDefaultsAreBitIdenticalToLumpedFormula)
{
  // Landing default: ego/opponent deceleration unset (0.0 -> relative_deceleration) and zero
  // latency must reproduce sqrt(v_opp^2 + 2*a*usable_gap) exactly, not just approximately.
  auto config = defaultConfig();
  f1tenth_control::CruiseLongitudinalController controller(config);
  const auto output = controller.update({10.0, 1.5, 5.0, 4.0, 0.0, 0.02});

  auto explicit_config = defaultConfig();
  explicit_config.ego_deceleration = explicit_config.relative_deceleration;
  explicit_config.opponent_deceleration = explicit_config.relative_deceleration;
  explicit_config.actuation_latency = 0.0;
  f1tenth_control::CruiseLongitudinalController explicit_controller(explicit_config);
  const auto explicit_output = explicit_controller.update({10.0, 1.5, 5.0, 4.0, 0.0, 0.02});

  const double usable_gap = output.effective_gap - config.emergency_stop_distance;
  const double legacy = std::sqrt(4.0 * 4.0 + 2.0 * config.relative_deceleration * usable_gap);
  EXPECT_DOUBLE_EQ(output.braking_speed, legacy);
  EXPECT_DOUBLE_EQ(explicit_output.braking_speed, legacy);
}

TEST(CruiseController, ActuationLatencyLowersTheBrakingCap)
{
  // tau > 0 means ego coasts before the brake acts, so the same gap supports a lower speed.
  auto config = defaultConfig();
  config.ego_deceleration = 2.5;
  config.opponent_deceleration = 2.5;
  f1tenth_control::CruiseLongitudinalController no_latency(config);
  const auto baseline = no_latency.update({10.0, 1.5, 5.0, 4.0, 0.0, 0.02});

  config.actuation_latency = 0.2;
  f1tenth_control::CruiseLongitudinalController with_latency(config);
  const auto delayed = with_latency.update({10.0, 1.5, 5.0, 4.0, 0.0, 0.02});

  EXPECT_LT(delayed.braking_speed, baseline.braking_speed);
}

TEST(CruiseController, StrongerOpponentBrakingLowersTheCap)
{
  // The opponent's own braking distance is credited to the ego. An opponent that can stop harder
  // (larger b_opp) gives away less of it, so the cap must drop; a weaker one must raise it.
  auto config = defaultConfig();
  config.ego_deceleration = 2.5;
  config.actuation_latency = 0.1;

  config.opponent_deceleration = 2.5;
  f1tenth_control::CruiseLongitudinalController matched(config);
  const auto matched_output = matched.update({10.0, 1.5, 5.0, 4.0, 0.0, 0.02});

  config.opponent_deceleration = 5.0;
  f1tenth_control::CruiseLongitudinalController stronger(config);
  const auto stronger_output = stronger.update({10.0, 1.5, 5.0, 4.0, 0.0, 0.02});

  config.opponent_deceleration = 1.25;
  f1tenth_control::CruiseLongitudinalController weaker(config);
  const auto weaker_output = weaker.update({10.0, 1.5, 5.0, 4.0, 0.0, 0.02});

  EXPECT_LT(stronger_output.braking_speed, matched_output.braking_speed);
  EXPECT_GT(weaker_output.braking_speed, matched_output.braking_speed);
}

// --- Constraint attribution (diagnostics) -----------------------------------------------------

TEST(CruiseController, ReportsWhichConstraintProducedTheLimit)
{
  auto config = defaultConfig();
  f1tenth_control::CruiseLongitudinalController controller(config);

  // Far behind a slower opponent: the braking cap is what holds the catch-up speed down.
  const auto catching_up = controller.update({10.0, 1.5, 5.0, 4.0, 0.0, 0.02});
  EXPECT_EQ(catching_up.active_constraint, f1tenth_control::CRUISE_CONSTRAINT_BRAKING);
  EXPECT_DOUBLE_EQ(catching_up.speed_limit, catching_up.braking_speed);

  // Sitting at the desired gap: the PID term is binding and equals the opponent speed.
  controller.reset();
  const auto holding = controller.update({1.5, 1.5, 4.0, 4.0, 0.0, 0.02});
  EXPECT_EQ(holding.active_constraint, f1tenth_control::CRUISE_CONSTRAINT_FEEDBACK);

  // Inside the emergency boundary: stop, and say so.
  controller.reset();
  const auto emergency = controller.update({0.4, 1.5, 5.0, 3.0, 0.0, 0.02});
  EXPECT_EQ(emergency.active_constraint, f1tenth_control::CRUISE_CONSTRAINT_EMERGENCY);
  EXPECT_DOUBLE_EQ(emergency.speed_limit, 0.0);
}

TEST(CruiseController, ExposesTheGapChainForDiagnostics)
{
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;
  f1tenth_control::CruiseLongitudinalController controller(config);
  const auto output = controller.update({3.0, 1.5, 4.0, 4.0, 0.04, 0.02});

  EXPECT_DOUBLE_EQ(output.raw_gap, 3.0);
  EXPECT_DOUBLE_EQ(output.desired_gap, 1.5);
  EXPECT_DOUBLE_EQ(output.sigma_gap, std::sqrt(0.04));
  // raw - z*sigma = effective, and effective - desired = the reported error.
  EXPECT_DOUBLE_EQ(output.effective_gap, 3.0 - 2.0 * std::sqrt(0.04));
  EXPECT_DOUBLE_EQ(output.gap_error, output.effective_gap - output.desired_gap);
}

// --- Desired gap policy -----------------------------------------------------------------------

TEST(DesiredGap, DistanceModeIsUnchanged)
{
  // Distance mode must stay max(minimum_gap, trailing_gap) at any speed.
  EXPECT_DOUBLE_EQ(f1tenth_control::desiredGap(true, 5.0, 0.8, 0.0, 0.0), 5.0);
  EXPECT_DOUBLE_EQ(f1tenth_control::desiredGap(true, 5.0, 0.8, 7.0, 0.0), 5.0);
  EXPECT_DOUBLE_EQ(f1tenth_control::desiredGap(true, 0.5, 0.8, 7.0, 0.0), 0.8);
}

TEST(DesiredGap, TimeModeIsStandstillPlusHeadway)
{
  // s0 + T*v, not max(s0, T*v): the standstill buffer survives at every speed.
  EXPECT_DOUBLE_EQ(f1tenth_control::desiredGap(false, 0.2, 0.8, 0.0, 0.0), 0.8);
  EXPECT_DOUBLE_EQ(f1tenth_control::desiredGap(false, 0.2, 0.8, 4.0, 0.0), 1.6);
  EXPECT_DOUBLE_EQ(f1tenth_control::desiredGap(false, 0.2, 0.8, 6.0, 0.0), 2.0);
  // Negative ego speed (bad odom sample) must not shrink the gap below the standstill buffer.
  EXPECT_DOUBLE_EQ(f1tenth_control::desiredGap(false, 0.2, 0.8, -3.0, 0.0), 0.8);
}

TEST(DesiredGap, ClampsToMaxDesiredGap)
{
  // The clamp exists to keep the target at or below state_machine's interference_distance_m.
  EXPECT_DOUBLE_EQ(f1tenth_control::desiredGap(false, 0.4, 0.8, 12.0, 5.0), 5.0);
  EXPECT_DOUBLE_EQ(f1tenth_control::desiredGap(false, 0.4, 0.8, 4.0, 5.0), 2.4);
  // 0.0 disables the clamp.
  EXPECT_DOUBLE_EQ(f1tenth_control::desiredGap(false, 0.4, 0.8, 12.0, 0.0), 5.6);
}

}  // namespace
