# AGENTS.md

Package-specific rules for `src/f1tenth_control`.

## Scope

- `control_map_node` owns final path tracking, speed ramping, steering LUT application, and
  `/drive_autonomous` publication.
- `cruise_controller_node` may only compute a longitudinal speed limit from the selected forward
  opponent. It must not generate or modify waypoints and must not publish Ackermann commands.
- `drive_source_selector` remains the only package component forwarding `/drive_autonomous` to
  `/drive`.

## Cruise Interfaces

- Subscribe to `/opp_obs` with `f110_msgs/msg/ObstacleArray`.
- Subscribe to `/state` with transient-local QoS. Apply an opponent speed cap only in
  `STATE_CRUISE`; GLOBAL and AVOID must receive the unrestricted maximum cap.
- Subscribe to `/car_state/frenet/odom` with `nav_msgs/msg/Odometry`.
- Subscribe to `/global_waypoints` with `f110_msgs/msg/WpntArray` and transient-local QoS.
- Publish `/cruise_speed_limit` with `std_msgs/msg/Float64`.
- Publish `/cruise/gap_data` with the existing `f110_msgs/msg/GapData` for diagnostics. Keep the
  legacy `gap_diff`/`vs_diff`/`gap_int` triple first and unrenamed; append new diagnostic fields.
  Every published limit must carry an `active_constraint` value — a diagnostic that cannot say
  *which* term was binding cannot support a tuning session.
- `f110_msgs` lives at `~/2026_IFAC/f110_msgs` and nowhere else. A duplicate copy used to sit
  under `src/f1tenth_control/f110_msgs`; it was deleted on 2026-08-21 because colcon never
  crawled it (colcon stops descending once `src/f1tenth_control` is recognised as a package),
  so `.msg` edits had to be mirrored by hand or the two silently diverged. Do not re-create a
  nested copy — edit the root package only.
- A cruise speed limit is a cap only. Steering and path-source selection must remain unchanged.
- The computed `desired_gap` must never exceed `state_machine.yaml`'s `interference_distance_m`,
  or the state machine leaves CRUISE at the very gap cruise is holding (enter/exit limit cycle).
  Equality is required for the fixed-distance default; a smaller target is safe. Time-headway mode
  grows the target with speed, so pair it with `max_desired_gap` to enforce the bound.
- Time-headway mode uses `s0 + T*v` (`minimum_gap` + `trailing_gap`*v), not `max(s0, T*v)`. The
  standstill buffer must survive at every speed.
- The braking cap decomposes into `ego_deceleration` (b_ego), `opponent_deceleration` (b_opp) and
  `actuation_latency` (τ), solving `v*τ + v²/(2*b_ego) <= usable_gap + v_opp²/(2*b_opp)` for v.
  All three default to 0.0, which falls back to `relative_deceleration` with no latency and is
  bit-identical to the lumped formula — keep it that way and enable only from measured values.
  Do not "simplify" the τ=0/b_ego==b_opp branch away; it is what makes the default bit-exact.
- `CruiseLongitudinalController::update()` propagates the opponent gap covariance over a braking
  -time horizon τ_s (`gap_uncertainty_horizon_max`, default 0.0 = no propagation, bit-identical to
  the pre-2026-08-20 formula). `σ_g²(τ) = s_var + 2τ·s_vs_cov + τ²·vs_var`, where `s_vs_cov`
  comes straight from `f110_msgs/Obstacle.msg`'s `s_vs_cov` field (0.0 from a producer that
  does not fill it, which degrades to the diagonal-only formula — see
  `docs/interference_judgment_migration_proposal.md` §3.4). Emergency stop always uses τ=0
  (`sigma_s0`) regardless of `gap_uncertainty_horizon_max` — do not let the time-propagated
  `effective_gap` feed the emergency-stop comparison; that would let a large
  `opponent_vs_variance` delay an otherwise-immediate stop.

## Parameters and Launch

- Cruise parameters live in `config/cruise_controller.yaml` and must be loaded by both control
  launch entrypoints.
- Keep real/simulation topic differences in launch files. Do not hard-code environment-specific
  odometry topics in the controller implementation.
- A stale active opponent or stale CRUISE heartbeat must fail safe to the configured blind speed.
  A cruise node outside STATE_CRUISE must not alter GLOBAL/AVOID driving behavior.

## Documentation and Verification

- Keep `docs/cruise_controller_node.md` synchronized with behavior and interfaces, and
  `CRUISE_TUNING_GUIDE.md` synchronized with the formulas, parameter effects, and the
  real-car measurement procedure.
- Build with `cb --packages-select f1tenth_control` after sourcing ROS 2 Jazzy.
- Run unit tests and a ROS topic-level runtime check for clear, following, emergency-stop, and
  stale-input behavior.
