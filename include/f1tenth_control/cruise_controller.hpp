#ifndef F1TENTH_CONTROL__CRUISE_CONTROLLER_HPP_
#define F1TENTH_CONTROL__CRUISE_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace f1tenth_control
{

// Which term produced the published speed limit. Mirrors f110_msgs/GapData.msg's
// CONSTRAINT_* constants -- keep the two lists in sync.
constexpr uint8_t CRUISE_CONSTRAINT_FEEDBACK = 0;
constexpr uint8_t CRUISE_CONSTRAINT_BRAKING = 1;
constexpr uint8_t CRUISE_CONSTRAINT_EMERGENCY = 2;
constexpr uint8_t CRUISE_CONSTRAINT_MAX_SPEED = 3;
constexpr uint8_t CRUISE_CONSTRAINT_NO_ACCEL = 4;

struct CruiseControllerConfig
{
  double maximum_speed{12.0};
  double emergency_stop_distance{0.45};
  double relative_deceleration{2.5};
  double proportional_gain{1.0};
  double integral_gain{0.0};
  double derivative_gain{0.2};
  double integral_limit{2.0};
  double uncertainty_sigma{2.0};
  bool allow_acceleration{true};
  // Braking-cap decomposition. relative_deceleration alone lumps "how much harder can ego brake
  // than the opponent" into one number, which cannot express actuation latency at all. These three
  // split it: ego brakes at ego_deceleration after coasting for actuation_latency, while the
  // opponent is assumed to brake at opponent_deceleration.
  // Landing defaults are a no-op: a non-positive deceleration falls back to relative_deceleration,
  // and with both sides equal and zero latency the cap is bit-identical to the previous formula
  // (see CruiseLongitudinalController::update()).
  double ego_deceleration{0.0};       // b_ego [m/s^2], <=0 -> relative_deceleration
  double opponent_deceleration{0.0};  // b_opp [m/s^2], <=0 -> relative_deceleration
  double actuation_latency{0.0};      // tau [s]: opponent decel observed -> ego brake actually acts
  // Time-propagation extension (interference_judgment_migration_proposal.md §3.4). Both default
  // to 0.0 = no propagation, which collapses tau_s to 0 and reproduces the pre-existing
  // position-only-uncertainty formula bit-for-bit -- see CruiseLongitudinalController::update().
  double gap_uncertainty_horizon_max{0.0};  // tau_max [s]: cap on the braking-time horizon
  double opp_speed_confidence_z{0.0};       // z_v: confidence multiplier for the opponent's
                                             // speed lower bound used by the braking cap
};

struct CruiseControllerInput
{
  double gap{0.0};
  double desired_gap{1.5};
  double ego_speed{0.0};
  double opponent_speed{0.0};
  double opponent_s_variance{0.0};
  double dt{0.02};
  // Appended after dt to keep existing positional-init call sites (tests included) valid: with
  // gap_uncertainty_horizon_max=0.0 both fields below are unused, so omitting them defaults to
  // 0.0 and changes nothing.
  double opponent_vs_variance{0.0};
  // Tracker's cov(s,vs) cross term, straight from f110_msgs/Obstacle.msg's s_vs_cov. A producer
  // that does not fill it leaves 0.0, which degrades sigma_g(tau) to the diagonal-only formula.
  double opponent_s_vs_cov{0.0};
};

struct CruiseControllerOutput
{
  double speed_limit{0.0};
  double effective_gap{0.0};
  double gap_error{0.0};
  double relative_speed{0.0};
  double gap_integral{0.0};
  // Diagnostics. Published on /cruise/gap_data so a tuning session can tell *which* term was
  // binding without re-deriving it from raw topics -- gap_error/relative_speed/gap_integral alone
  // cannot distinguish "PID held us back" from "braking cap held us back".
  double raw_gap{0.0};
  double desired_gap{0.0};
  double feedback_speed{0.0};
  double braking_speed{0.0};
  double sigma_gap{0.0};    // sigma_g(tau_s) actually subtracted from the gap
  double horizon_tau{0.0};  // tau_s used for that propagation
  uint8_t active_constraint{CRUISE_CONSTRAINT_FEEDBACK};
};

// Target following gap [m].
// - distance mode: max(minimum_gap, trailing_gap), unchanged.
// - time mode: standard ACC/IDM form s0 + T*v with s0 = minimum_gap and T = trailing_gap.
//   The previous max(s0, T*v) made s0 vanish above s0/T m/s and produced a shrinking time
//   headway with speed; s0 + T*v keeps a constant reaction-time budget on top of a fixed
//   standstill buffer.
// max_desired_gap > 0 clamps the result. It must not exceed state_machine's
// interference_distance_m, or CRUISE is exited at the very gap cruise is holding
// (enter/exit limit cycle -- see state_machine.yaml).
double desiredGap(
  bool distance_mode, double trailing_gap, double minimum_gap, double ego_speed,
  double max_desired_gap);

class CruiseLongitudinalController
{
public:
  explicit CruiseLongitudinalController(const CruiseControllerConfig & config);

  CruiseControllerOutput update(const CruiseControllerInput & input);
  void reset();

private:
  CruiseControllerConfig config_;
  double gap_integral_{0.0};
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL__CRUISE_CONTROLLER_HPP_
