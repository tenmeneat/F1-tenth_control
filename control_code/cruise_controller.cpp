#include "f1tenth_control/cruise_controller.hpp"

namespace f1tenth_control
{

double desiredGap(
  bool distance_mode, double trailing_gap, double minimum_gap, double ego_speed,
  double max_desired_gap)
{
  const double standstill_gap = std::max(0.0, minimum_gap);
  const double gap = distance_mode ?
    std::max(standstill_gap, trailing_gap) :
    standstill_gap + std::max(0.0, trailing_gap) * std::max(0.0, ego_speed);
  if (max_desired_gap > 0.0) {
    return std::min(gap, max_desired_gap);
  }
  return gap;
}

CruiseLongitudinalController::CruiseLongitudinalController(
  const CruiseControllerConfig & config)
: config_(config)
{
  config_.maximum_speed = std::max(0.0, config_.maximum_speed);
  config_.emergency_stop_distance = std::max(0.0, config_.emergency_stop_distance);
  config_.relative_deceleration = std::max(0.01, config_.relative_deceleration);
  config_.integral_limit = std::max(0.0, config_.integral_limit);
  config_.uncertainty_sigma = std::max(0.0, config_.uncertainty_sigma);
  config_.gap_uncertainty_horizon_max = std::max(0.0, config_.gap_uncertainty_horizon_max);
  config_.opp_speed_confidence_z = std::max(0.0, config_.opp_speed_confidence_z);
  // Unset (<=0) per-side decelerations fall back to the lumped value, which is what makes the
  // shipped default reproduce the pre-decomposition cap exactly.
  if (!(config_.ego_deceleration > 0.0)) {
    config_.ego_deceleration = config_.relative_deceleration;
  }
  if (!(config_.opponent_deceleration > 0.0)) {
    config_.opponent_deceleration = config_.relative_deceleration;
  }
  config_.actuation_latency = std::max(0.0, config_.actuation_latency);
}

CruiseControllerOutput CruiseLongitudinalController::update(
  const CruiseControllerInput & input)
{
  CruiseControllerOutput output;
  const double dt = std::clamp(input.dt, 1e-3, 0.2);
  const double ego_speed = std::max(0.0, input.ego_speed);
  const double opponent_speed = std::max(0.0, input.opponent_speed);
  const double s_variance = std::max(0.0, input.opponent_s_variance);
  const double vs_variance = std::max(0.0, input.opponent_vs_variance);
  const double s_vs_cov = std::isfinite(input.opponent_s_vs_cov) ? input.opponent_s_vs_cov : 0.0;
  const double sigma_s0 = std::sqrt(s_variance);

  // Braking-time horizon for the gap covariance's CV time propagation
  // (interference_judgment_migration_proposal.md §3.4). tau_max=0.0 (default) forces tau_s=0.0
  // below, which collapses sigma_g to sigma_s0 and v_opp_lb to opponent_speed, reproducing the
  // previous formula bit-for-bit -- the 2*tau*s_vs_cov term vanishes at tau=0 regardless of cov.
  const double v_opp_lb = std::max(
    0.0, opponent_speed - config_.opp_speed_confidence_z * std::sqrt(vs_variance));
  const double tau_s = config_.gap_uncertainty_horizon_max > 0.0 ?
    std::clamp(
      (ego_speed - v_opp_lb) / config_.relative_deceleration,
      0.0, config_.gap_uncertainty_horizon_max) :
    0.0;
  // sigma_g^2(tau) = s_var + 2*tau*s_vs_cov + tau^2*vs_var, floored at 0: the detector's
  // Cauchy-Schwarz clamp (|s_vs_cov| <= 0.99*sqrt(s_var*vs_var)) keeps the discriminant
  // non-negative, but the floor guards against an unclamped producer or rounding.
  const double sigma_g = std::sqrt(
    std::max(0.0, s_variance + 2.0 * tau_s * s_vs_cov + tau_s * tau_s * vs_variance));

  output.raw_gap = input.gap;
  output.desired_gap = std::max(0.0, input.desired_gap);
  output.sigma_gap = sigma_g;
  output.horizon_tau = tau_s;
  output.effective_gap = std::max(
    0.0, input.gap - config_.uncertainty_sigma * sigma_g);
  output.gap_error = output.effective_gap - output.desired_gap;
  output.relative_speed = opponent_speed - ego_speed;

  gap_integral_ = std::clamp(
    gap_integral_ + output.gap_error * dt,
    -config_.integral_limit, config_.integral_limit);
  output.gap_integral = gap_integral_;

  // Emergency stop stays at tau=0 (immediate occupancy judgment) regardless of
  // gap_uncertainty_horizon_max -- design rule 4 in §3.4: safety uses the instant, not the
  // time-propagated, uncertainty.
  const double emergency_gap = std::max(0.0, input.gap - config_.uncertainty_sigma * sigma_s0);
  if (emergency_gap <= config_.emergency_stop_distance) {
    output.speed_limit = 0.0;
    output.active_constraint = CRUISE_CONSTRAINT_EMERGENCY;
    return output;
  }

  const double feedback_speed =
    opponent_speed +
    config_.proportional_gain * output.gap_error +
    config_.integral_gain * gap_integral_ +
    config_.derivative_gain * output.relative_speed;

  // If the opponent were to brake now, this cap leaves enough stopping distance before the
  // emergency boundary. It complements the gap feedback during high closing-speed approaches.
  // Uses v_opp_lb (opponent_speed by default) rather than the raw opponent_speed so a confident
  // opponent-speed estimate (opp_speed_confidence_z > 0) assumes the opponent could already be
  // slower than measured.
  //
  // No-collision condition, decomposed per side:
  //   v*tau + v^2/(2*b_ego)  <=  usable_gap + v_opp_lb^2/(2*b_opp)
  //   \___/   \___________/                   \________________/
  //   coast   ego braking                     opponent's own braking distance
  // Solving that quadratic for v gives v = -b_ego*tau + sqrt((b_ego*tau)^2 + 2*b_ego*reachable).
  const double usable_gap =
    std::max(0.0, output.effective_gap - config_.emergency_stop_distance);
  const double b_ego = config_.ego_deceleration;
  const double b_opp = config_.opponent_deceleration;
  const double tau = config_.actuation_latency;
  double braking_speed = 0.0;
  if (tau <= 0.0 && b_ego == b_opp) {
    // Legacy form written out explicitly so the landing default stays bit-identical to the
    // pre-decomposition build (the general branch is algebraically equal but not bit-equal).
    braking_speed = std::sqrt(v_opp_lb * v_opp_lb + 2.0 * b_ego * usable_gap);
  } else {
    const double reachable = usable_gap + (v_opp_lb * v_opp_lb) / (2.0 * b_opp);
    braking_speed = std::max(
      0.0,
      -b_ego * tau +
      std::sqrt(std::max(0.0, b_ego * b_ego * tau * tau + 2.0 * b_ego * reachable)));
  }
  output.feedback_speed = feedback_speed;
  output.braking_speed = braking_speed;

  const double unclamped = std::min(feedback_speed, braking_speed);
  output.active_constraint = braking_speed < feedback_speed ?
    CRUISE_CONSTRAINT_BRAKING : CRUISE_CONSTRAINT_FEEDBACK;
  output.speed_limit = std::clamp(unclamped, 0.0, config_.maximum_speed);
  if (output.speed_limit < unclamped) {
    output.active_constraint = CRUISE_CONSTRAINT_MAX_SPEED;
  }
  if (!config_.allow_acceleration && ego_speed < output.speed_limit) {
    output.speed_limit = ego_speed;
    output.active_constraint = CRUISE_CONSTRAINT_NO_ACCEL;
  }
  return output;
}

void CruiseLongitudinalController::reset()
{
  gap_integral_ = 0.0;
}

}  // namespace f1tenth_control
