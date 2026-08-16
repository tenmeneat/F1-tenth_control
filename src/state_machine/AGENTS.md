# AGENTS.md
State machine package rules. These instructions apply to `src/state_machine`.

## Package Scope

- Keep this package focused on behavior-state decisions and final waypoint-source selection.
- `state_machine_node` publishes `f110_msgs/msg/StateMachine`, the selected
  `f110_msgs/msg/WpntArray`, and its `nav_msgs/msg/Path` visualization mirror.
- `state_machine_node` consumes `/car_state/frenet/odom`, `/global_waypoints`,
  `/avoid_waypoints`, and `/opp_obs`; it must not subscribe to `/scan`.
- This package is the only normal publisher of `/local_waypoints`. Keep
  `local_planning.publish_standalone_local` disabled during integrated operation.
- Do not reintroduce a separate state-subscriber waypoint relay. State decisions and waypoint
  selection must use the same cached inputs and validity rules in this node.

## Package Layout

- Public node declarations live in `include/state_machine/`.
- C++ node implementations live in `src/`.
- Runtime parameters live in `config/state_machine.yaml`.
- Launch entrypoints live in `launch/`.
- Node documentation lives in `docs/state_machine_node.md`, the repo `README.md`, and this `AGENTS.md`.

## Runtime Code

- Runtime code must be C++ for ROS 2 Jazzy.
- Use existing `f110_msgs` message types for project-specific interfaces.
- Transition logic is a `committed_state_`-based FSM (2026-08-16 handoff-marker contract):
  - `local_path_confirmed()` / `can_enter_avoid()`: at least M of the latest N avoid messages must
    contain a non-empty local path (default 3-of-5). AVOID has priority from GLOBAL and CRUISE.
    A path whose `ot_line == handoff_ot_line` NEVER counts toward this history and never enters
    AVOID: the handoff loop is the planner saying "avoidance is over", not a new request.
  - GLOBAL enters CRUISE when a fresh `/opp_obs` contains a dynamic obstacle with
    `is_interfering=true`.
  - CRUISE selects GLOBAL geometry and returns to GLOBAL on false, empty, or stale `/opp_obs`.
  - **AVOID -> GLOBAL requires the planner's explicit handoff marker.** The planner is the single
    owner of "no blocking cluster remains" — it publishes `ot_line=raceline_global_handoff` only
    after verifying that itself. Only while that marker is present does `enter_to_global()`
    evaluate the tail-reach / lateral-error / duration checks. A markerless non-empty path that
    happens to satisfy the geometry must NOT release AVOID: with tightly spaced obstacles the
    middle of an avoidance path can graze |d|<threshold, and releasing there sends the next
    obstacle down the raw global line. This node must never re-derive obstacle-clearance from a
    detector topic — the removed `has_front_static_obstacle`/`stopped_path_clear` machinery judged
    with the ego's CURRENT d (wrong while offset), a different topic than the planner
    (`/static_obs` vs `/confirmed_static_obs`), and could disagree with the planner about the
    same obstacle. Do not reintroduce it.
  - Feed the marker's absence through `evaluate_enter_to_global(local_available=false)` rather
    than short-circuiting around it — the duration timer must reset while the marker is away, or
    the next handoff window inherits a stale start time and skips the debounce.
  - `evaluate_avoid_path_liveness_lost()` is the ONLY non-marker escape: no non-empty avoid
    publication for `avoid_path_liveness_timeout_sec` releases AVOID regardless of obstacles. It
    is a liveness statement about the publisher, not a clearance statement. The old tail-reach
    exhaustion escape is gone — a path withdrawn near its start left the tail unreachable and the
    escape dead; conversely a planner that keeps publishing (stop paths included) never triggers
    liveness, which is correct: a car held before an obstacle is the intended outcome.
- `resolve_requested_state()`: the FSM 1-step. It reads/updates `committed_state_` and returns the state to publish. No dwell, no separate safety fallback — the state changes only when an entry or merge-back condition fires.
- `enter_to_global()` assumes the **segment publishing convention**: `/avoid_waypoints` is an
  ego→merge segment in global-raceline Frenet coordinates (`s_m`/`d_m`), tail converging to d→0.
  Its tail window is `enter_global_tail_distance_m` of ARC LENGTH walked back from the path end,
  not a waypoint-count ratio (a ratio swelled with path length: 4.3 m on the full handoff loop vs
  0.5 m on a short segment). It must equal local_planning's `state_handoff_tail_distance_m` —
  the planner rotates the handoff loop so ego sits exactly at the start of that tail.
- `/state` is timer-driven at `publish_rate_hz`; `/local_waypoints` and
  `/local_waypoints/path` are emitted only by fresh Frenet odometry callbacks.
- Global waypoints are static validated data with no use-blocking TTL. In AVOID an empty avoid
  message does NOT switch the published geometry to the global line ("global_fallback" applies
  only to states that legitimately use global geometry): the selector keeps the last non-empty
  avoid path until either the marker-gated merge or the liveness escape changes the state.
  Silently publishing global geometry while the state is still AVOID bypasses the transition
  gate kinematically and drives the obstacle-piercing raceline (2026-08-13 real car: /drive
  pulses toward the obstacle during a hold). Opponent interference is valid only for
  `opponent_stale_timeout_sec`.
- `invalid_local_path_policy=global_fallback` is the only supported first-stage policy. Do not
  invent a stop path in this selector.
- Keep refinements (dynamic obstacle prediction, score-based hysteresis) as TODOs.

## Parameters, Launch, Docs

- All topic names, frame names, publish rates, stale/diagnostic timeouts, hold durations, waypoint
  counts, and default states must be parameters with YAML defaults.
- Keep `launch/state_machine.launch.py` loading `config/state_machine.yaml`.
- Keep Korean operational documentation in `docs/state_machine_node.md` current, including both
  state and selected-waypoint interfaces.
- Update this `AGENTS.md` (and `README.md` if run/launch changes) when changing node behavior, topics, parameters, or launch usage.
