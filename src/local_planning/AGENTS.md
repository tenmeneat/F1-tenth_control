# AGENTS.md for local_planning

## Package purpose

- This package handles static-obstacle avoidance only.
- Its geometric invariant is race-line locking: every published moving path must preserve the
  ordered `/global_waypoints` samples and modify only their local Frenet `d(s)` offset.
- Never add Cartesian nearest-path search, free-space graph search, or a map shortcut that can jump
  to a different geometric track branch. This invariant is especially important on non-convex
  snake sections.
- Dynamic-opponent planning is outside this package. `obstacle_detector` publishes dynamic
  perception separately on `/opp_obs`.

## Algorithm rules

- Runtime code is C++17 for ROS 2 Jazzy.
- Use `f110_msgs/msg/ObstacleArray`, `WpntArray`, and `OTWpntArray`; do not create a new message.
- The obstacle input is `obstacles_topic`, which defaults to `/confirmed_static_obs`
  (2026-08-16). `/confirmed_static_obs` is the detector's confirmed-only Layer-2 view: it carries
  only `CONFIRMED` tracks classified `Static` by map-frame position persistence, while
  `/static_obs` also carries `CONFIRMED+UNKNOWN` provisional objects. Wall fragments and scatter
  clusters that survive a few scans reach `/static_obs` but not `/confirmed_static_obs`, and each
  one of them made the planner publish a non-empty path, which alone flips the FSM into
  `STATE_AVOID`. Both topics share the same track and physical ID space, so every ID-keyed
  contract below (observation counts, committed guards, completed IDs) is unchanged. Keep the
  topic a parameter; do not hardcode either name in the node.
- Consume the detector-owned Frenet footprint on the obstacle input without any Cartesian-to-Frenet
  conversion. Treat `s_start/s_end/d_right/d_left` as the authoritative obstacle geometry.
  Cartesian AABB fields are optional metadata and are not consumed as planner geometry.
- Obstacle bounds are raw detector geometry. Compute the tracking-error tube by bilinear
  interpolation of the configured speed-by-absolute-curvature LUT, falling back to
  `tracking_error_reserve_m` only when the LUT is intentionally empty. For obstacle target
  generation, use the maximum LUT value across the obstacle reference span. Limit the reference
  or candidate speed first with the configured jazzy_main-derived speed/lateral-acceleration table
  so `v^2 * abs(kappa) <= a_lat_max(v)`. For sampled-path, frozen-path, and raw hard-collision
  validation, use each candidate waypoint's limited speed and recomputed curvature. Obstacle
  clearance is `vehicle_half_width_m + safety_margin_m + tube`.
  The lateral-acceleration table only binds in curves, so an obstacle on a straight would otherwise
  be planned at race-line speed and reserve the widest tube in the LUT, closing gaps the car could
  hold at a lower speed. Cap each waypoint that is passing an obstacle by the gap as well: the tube
  it may spend is the lateral room left between the path and the obstacle face, and speed is
  lowered only until the tube fits, never below `avoidance_minimum_speed_mps`. Below that floor the
  maneuver is infeasible and safe-stop decides; never crawl through on a tube the car cannot hold.
  A waypoint passing no obstacle is never slowed. Keep this inversion of the LUT strict: a reduced
  speed whose tube overshoots the available room by the validator's own tolerance spends room the
  path does not have, and the validator then rejects the candidate the cap existed to enable.
  Global waypoint `d_left/d_right` are reference-to-physical-boundary distances. Validate every
  candidate's 0.56 m x 0.287 m, base-link-centred rectangular corners using candidate x/y/yaw and
  widths interpolated on the matching local reference segment. Subtract only
  `wall_safety_margin_m`, exactly once. Never add the tracking tube, obstacle margin, simulator TTC
  sweep, scan-noise guard, or another boundary/commitment/fallback margin.
- **One maneuver-scope collision horizon, used by every site that judges the same path**
  (2026-08-16). The obstacle check of a maneuver's geometry stops at
  `expanded cluster end + post_merge_lookahead_m`; track-bound and geometry checks always cover the
  whole path. That horizon must be identical at candidate validation (`buildCandidate`), fresh
  selection (`P3ManeuverLifecycle::selectFresh`), continuation revalidation
  (`continueCurrent`), and the P0 re-validation in `generateP3Candidates`. Two sites judging one
  path against different ranges is not a stricter safety check, it is an unbreakable loop: the
  wider site condemns every path the narrower one blesses, so the planner re-selects and
  re-condemns at the planning rate, and when that lands inside `safe_stop_buffer_m` it latches a
  full stop. The P3 lifecycle originally had NO horizon while `generateP3Candidates` had one; on
  the 2026-08-16 sim bag that single asymmetry produced `NO_HARD_VALID_M1_CANDIDATE` on 245/245
  cycles within 3 m of an obstacle the P0 path was planning around successfully, plus a full stop
  at the same place on every lap. Do not add a horizon to one site alone.
- **An exit ramp that still reaches the next obstacle is ranked last, never rejected**
  (2026-08-16, `exit_reaches_next_obstacle`). Measured on the post-cluster ramp only — up to the
  merge station, NOT into the post-merge global tail, which is at `d=0` and would mark every
  candidate whenever the next obstacle sits on the race line. Rejecting instead of demoting
  reinstates the 2026-08-12/08-15 regressions where every candidate died on a tail collision and
  the car crept forever; carrying the offset over closely spaced obstacles is the intended
  behaviour when nothing better exists. `P3ShadowEvaluator::betterFeasible` and `plan()`'s
  `better_candidate` must apply this term identically, or P0 commits a different path than P3
  selected.
- P3 lifecycle is continuation-first, and that is a COMPUTATION order, not only an output
  priority: an active recorded maneuver is continued (frozen path, revalidated every callback via
  guard containment + raw fallback) BEFORE any fresh M1 selection, and fresh selection runs only
  with no active maneuver or in the same callback in which continuation invalidated. Do not
  restore fresh-first ordering: it re-shapes the published path every callback while the obstacle
  envelope is still being resolved. `advanceP3Lifecycle` therefore takes a LAZY evaluator
  (`std::function<const P3ShadowResult &()>`), never a materialized result, and pulls it only
  after continuation fails to produce output or completion. Passing an already-computed
  evaluation would silently pay for up to 24 candidate constructions and their hard validations
  on every 25 ms callback while a frozen suffix is holding perfectly well.
- Committed-path retention band (`commitment_retention_reserve_fraction`, default 0.5): when
  re-validating an ALREADY COMMITTED path (P3 continuation raw fallback, P0 commitment hard
  check), the tracking-error reserve portion of the obstacle clearance is scaled by this
  fraction; the physical base clearance is never reduced and fresh planning always validates
  with the full reserve. A broken uncertainty guard or a full-margin violation inside the
  retention band holds the frozen geometry indefinitely (the old N-cycle soft-confirmation
  expiry is removed — it re-planned a nearly identical path on every progressive reveal, the
  dominant visible churn); invalidation/replacement requires an actual retention-margin
  violation, a non-obstacle failure, or completion. Do not reinstate the expiry and do not let
  fresh candidates validate with a scaled reserve.
- Localization reserve (`localization_reserve_m`, default 0.06): a constant floor added INSIDE
  `RacelineSplineParameters::trackingErrorReserve()` — the single choke point — so envelope
  expansion, hard validation, and the gap-limited speed inversion all see the same total. Do
  not add it separately at call sites (double counting) and do not remove it from the
  inversion path (the planner would pick speeds whose clearance the validator then rejects).
  Sized from sustained sim GT-vs-MCL error (per-speed-bin P95, 2026-08-13 probe); transient
  single-cycle MCL correction spikes are absorbed by the retention band, NOT this margin —
  do not resize it to the spike maximum (closes corridors). Keep it OUT of the CMA parameter
  whitelist: under GT localization the optimizer would drive it to zero. Re-measure on the
  real car (tools/mcl_gt_error_probe.py needs GT, so use MCL covariance/particle spread
  logging there) before real-car obstacle runs. On P3 completion, hand back through `activateGlobalHandoff` (the same
  closed global loop P0 uses) — never a frozen post-obstacle tail, which falls behind the ego
  and starves the FSM merge confirmation. The completion branch must re-register the completed
  maneuver's obstacle ids into `completed_obstacle_ids_` across the `clearCommitment()` wipe
  (P0-completion semantics): the detector's static occlusion hold keeps the passed track
  published, and without the exclusion the just-passed obstacle re-enters
  `buildNextManeuverInput` while ego is inside its padded span — the chained cluster then
  starts at ego and the safe-stop latch stalls the car inside its own danger region.
  Keep `maximum_exit_length_m` disabled (0) unless a
  single isolated obstacle is guaranteed: the long exit deliberately carries the offset over
  closely-following obstacles, and capping it makes the committed path dive into the next
  obstacle's inflated box the moment the detector reveals it, at a range where no fresh plan
  exists yet.
- The tracking-error LUT must stay measured, not guessed: obstacle-free laps on the clean map
  (`tools/cmaes_tuning/clean_lap_lut_traces.sh` in sim, or the real-car procedure in
  `docs/local_planner.md`) are the only valid source, and low-speed rows need dedicated
  speed-capped laps because racing never visits those cells. A monotone-filled cell silently
  becomes the span-maximum reserve and can block gaps the car could actually hold.
- `maximum_curvature_radpm` equals the vehicle's full-lock curvature (tan(0.41)/0.33 = 1.317),
  which is also the race line's own maximum. Treat it as physics, not a tunable: an obstacle
  inside or just after a maximum-curvature corner is unavoidable on both sides (the shifted
  line must exceed full lock), and safe-stop there is the correct verdict. Do not "fix" such a
  scenario by raising this limit.
- Candidates come from the P3 analytic ladder (M0-V1 → M0-V2 → M1) over the side's valid target
  domain — the minimum obstacle-clearance offset to the maximum track-bound /
  `maximum_target_offset_m` offset. Rank feasible candidates lexicographically by maximum minimum
  normalized wall/obstacle/curvature/curvature-rate slack, then minimum speed loss, then minimum
  global-line deviation. Do not replace this with a weighted sum, and never return the first
  valid candidate when a full ladder generation is available. NOTE the ranking consequence: P3
  picks the slack-maximizing plateau, not the minimum-clearance point, so on a wide track the
  selected `target_d` sits well beyond the clearance minimum. Tests that pin margins must narrow
  the track so the valid window pins the plateau (see the margin tests in
  `test/test_raceline_spline.cpp`).
- Reject a side before spline fitting when even its minimum-clearance target cannot fit the waypoint
  track widths across the expanded obstacle-cluster span. Use the remaining track-bound interval as
  the target sampling range; full sampled-path validation still applies before and after the span.
  That track-bound interval is for the vehicle CENTRE, so subtract `vehicle_half_width_m` alongside
  `wall_safety_margin_m`: the hard validator checks the rotated rectangle, and a gate that omits
  the half width offers targets the footprint check can never accept. When the race-line-speed gate
  does not fit, retry once against the gate the pass would need at `avoidance_minimum_speed_mps`
  before declaring the side blocked — a gap that is merely slower must not read as unreachable.
  ADDITIONALLY (2026-08-15): when the strict gate DOES fit the track but every ladder candidate is
  rejected by the exact validator, `evaluateP3Shadow` reruns the whole evaluation once with the
  relaxed (`avoidance_minimum_speed_mps`) gate — domain endpoints AND the corridor's per-station
  obstacle envelopes both switch to the relaxed inflation, or the corridor re-closes the window the
  domain opened. The rerun happens only on a failed strict pass, so wide-gap selections and their
  race speed are unchanged; a recovered result is tagged `+RELAXED_CLEARANCE_GATE` in
  `selected_source`. Both retries widen what is considered, never what is accepted; every candidate
  is still validated at the speed it actually ends up with.
- Build the lateral profile as P3's 5-knot C² quintic Hermite `d(s)` in unwrapped global Frenet
  `s` (knot offsets `{ego.d, d_target, d_mid, d_target, 0}`, harmonic-mean knot-derivative rule),
  keep `d`, `dd/ds`, and `d2d/ds2` continuous at the ego/target/global-line joins, and convert
  each selected global waypoint with its own normal. Preserve `s_m` and order. Entry/exit station
  lengths come from `entry_transition_fractions` × `pre_apex_distances_m` and
  `transition_distance_scales` × `post_apex_distances_m.back()` — these parameters feed P3's
  station layout and are NOT dead legacy values. Require `pre_apex_far <= detection_lookahead`.
- Validate lateral slope, recomputed Cartesian curvature, curvature rate, obstacle clearance, and
  yaw-aware rectangular-footprint track-bound clearance before publishing. Keep corner projection
  on the candidate waypoint's local track branch to avoid nearby snake-track branch aliasing.
  Hard-reject any footprint violation with `footprint_track_bound`. Track-bound VALIDATION uses
  `wall_safety_margin_m` exactly once. Do not add any further boundary, commitment, hard, or
  fallback margin to validation.
- The wall term of the RANKING slack is measured from the vehicle body plus the tracking-error
  tube (`wall_safety_margin_m + vehicle_half_width_m + avoidanceTrackingErrorReserve`), matching
  what the obstacle term already spends (`vehicle_half_width_m + safety_margin_m + tube`). This is
  a ranking quantity only: it never rejects a candidate, so the feasible set and therefore the
  avoidance/safe-stop verdict are unchanged. Do not revert it to the legacy centerline headroom
  (`wall_safety_margin_m` alone) — that made the two terms incommensurate, and maximizing their
  minimum then biased every selection toward the wall by exactly
  `(obstacleSafetyClearance - wall_safety_margin_m) / 2`, up to 0.257 m, independently of how wide
  the gap actually was (2026-08-15 measurement: on a 1.20 m obstacle-face-to-wall gap the car
  planned 0.202 m of body-to-wall room against 0.711 m at the obstacle). `wall_clearance_m` now
  carries this body-referenced value; `centerline_wall_clearance_m` keeps the legacy headroom for
  audit continuity. Regression: `RankingCentresPassBetweenObstacleAndWall` and
  `SafetySlackRejectsBarelyWallFeasibleTargetAsBest` in `test/test_raceline_spline.cpp`.
- Before the first lateral commitment, publish `ot_line=raceline_static_prepare` with a validated
  braking prefix while collecting the nearest cluster's IDs and conservative Frenet-envelope union.
  Count distinct `/confirmed_static_obs` messages, not planning ticks, and require the configured number of
  observations for every cluster ID and the configured minimum stabilization duration unless the
  maximum wait is reached. Expand the final union longitudinally by `k*sqrt(s_var)` plus the fixed
  longitudinal floor, but preserve the union's detector-owned `d_right/d_left` without lateral
  covariance inflation. Freeze that Guard with the commitment. An obstacle already inside the stop
  buffer bypasses this wait and enters safe-stop immediately.
- For a committed same-ID obstacle, replace the live envelope with the frozen Guard whenever the
  complete live uncertainty envelope remains contained in it. Never slide the Guard from one
  measurement to the next. A Guard breach must still validate the frozen path against the live
  envelope; rebuild only when that validation fails. Keep physical obstacle clearance separate
  from the uncertainty and AABB-extent margins.
- Freeze committed path geometry while its remaining forward portion is still valid against the
  latest Frenet envelopes. Replan only after that validation fails. A side may be reselected before the
  configured lateral/longitudinal engagement threshold, then it is locked for the rest of the
  maneuver. After one pre-engagement side switch, switching back is disabled until engagement or
  the next maneuver so a centred-obstacle tie cannot weave the car. Keep the commitment until its
  tail merges at `d=0`, even if perception drops the passed
  obstacle.
- After at least one valid `/confirmed_static_obs` message, treat input older than
  `obstacle_stale_timeout_sec` as degraded perception, not as proof that the track is clear. Retain
  the frozen commitment and the last valid obstacle snapshot, complete the odometry-based merge and
  GLOBAL handoff normally, and reuse the snapshot when planning on a later lap. A fresh valid
  obstacle array, including an explicitly empty array, replaces that memory. Rejecting a wrong-frame
  array must not erase it, and neither must a non-empty array whose every entry failed the Frenet
  validity check: an all-rejected array is degraded perception, and storing the empty accepted list
  would be indistinguishable from the explicitly-empty case that IS allowed to erase the memory.
  Return from such an array without touching the snapshot, sequence, source stamp, or P3 epoch. Do
  not wait for repeated observations when replanning solely from retained
  stale memory because no new samples can arrive.
- The reference-change test must compare every waypoint field the planner consumes -- `s_m`, `x_m`,
  `y_m`, `d_left`, `d_right`, `psi_rad`, `kappa_radpm`, `vx_mps` -- not only the centreline
  geometry. A boundary-only recalibration changes none of `s/x/y`, and `obstacle_detector` already
  treats `d_left/d_right` changes as a new reference; comparing fewer fields here desynchronizes
  the two nodes onto different track widths.
- Separate commitment violations into hard physical collisions and soft uncertainty-envelope
  collisions. Both checks use the same unified physical clearance. Test hard collisions against
  raw detector bounds (with the retention reserve fraction) and replan immediately; test soft
  collisions against uncertainty Guards. A soft-only collision NEVER replaces the frozen path
  (retention band, 2026-08-12): `commitment_soft_violation_confirm_cycles` only paces the
  diagnostic logging of a persistent soft violation, after which the count resets and the frozen
  geometry is explicitly kept. Never debounce track-bound, path-exhaustion, or geometry failures.
  Log the offending obstacle ID, waypoint `s/d`, obstacle `s/d` bounds, and applied clearance.
- Append a speed-aware ordered global `d=0` tail after the spline merge. After geometric merge,
  publish a full global loop with `ot_line=raceline_global_handoff`. Continue that non-empty
  handoff path until `/state` has entered `STATE_AVOID` for the commitment and subsequently
  confirms `STATE_GLOBAL`. This marker is now the FSM's REQUIRED precondition for AVOID->GLOBAL
  (2026-08-16 contract): publish it only when no unfinished blocking cluster remains, and never
  on ordinary avoidance/stop paths — a false marker releases the FSM early, a missing one keeps
  it in AVOID until the liveness escape. The handoff rotation places ego at the start of the
  last `state_handoff_tail_distance_m` metres of the loop; that value must equal the FSM's
  `enter_global_tail_distance_m` (both are arc-length metres, ratios were removed).
- **Never publish an empty path while `/state` reads `STATE_AVOID`** (2026-08-16). The FSM's
  return path (`enter_to_global`) only RUNS when the latest `/avoid_waypoints` is non-empty, and
  an empty message carries no timeout and no alternative exit: publishing empty from AVOID latches
  the FSM in AVOID permanently (sector speed scaling then stays off for the rest of the run).
  A phantom detection that vanishes before any commitment is the common way in. Two rules
  implement this: (1) when the track is clear and the planner has no commitment, treat "the last
  publication was non-empty" — not only "a preparation path was published" — as the trigger to
  hand back through `activateGlobalHandoff`, because the stabilization-time early-avoidance branch
  clears `initial_prepare_published_`; (2) at the terminal `publishEmpty` site, if `plan()`
  returned `kNoObstacle` and `/state` is still `STATE_AVOID`, publish the closed global handoff
  loop instead. Its tail sits at ego with `d=0`, which is exactly what the FSM's tail-reach and
  lateral gates need, so GLOBAL is confirmed through the normal path rather than by a timeout.
  Restrict this to `kNoObstacle`: it is the only result that proves the track is actually clear.
- Merge ramp (ego d → 0 smoothstep grafted onto the global handoff loop, 2026-08-13) is
  implemented but DEFAULT-OFF (`merge_ramp_min_length_m`/`merge_ramp_time_sec` = 0). A bare d=0
  loop delegates the return to the controller's natural convergence (real car: 0.055 m/m), but
  enabling the ramp with the current waypoint d_left/d_right shaved the sim wall pinch minimum
  from 0.117 to 0.082 m — the wall clamp cannot bind because those bounds are optimistic by a
  measured 0.16-0.23 m. Enable ONLY after the boundary data is calibrated (control team's
  per-sector lidar wall-clearance table), and re-run the lockstep baseline before adopting.
- Stabilize every non-active blocking cluster from the current ego state concurrently while the
  active maneuver runs; do not use the old `merge_s` as the next-cluster observation origin.
  Once the active Guard rear plus `chain_release_distance_m` is behind ego, allow a feasible next
  spline anchored at the current `ego.d` to preempt the old merge. Retire the completed IDs and
  release their side lock, but keep `/avoid_waypoints` non-empty and `STATE_AVOID` active. Continue
  validating the current commitment against obstacles that lie before its merge until a validated
  chained path replaces it. A post-merge controller-tail obstacle must not make the current
  maneuver fail. Hand off to GLOBAL only after no unfinished blocking cluster remains.
- **`plan()`'s only avoidance candidate generator is P3 (`generateP3Candidates`), 2026-08-15.**
  The P0 quintic grid (`generateSideCandidates`/`buildCandidate`) and its
  `p0_avoidance_candidates_enable` toggle were DELETED after on-track testing showed P3 passed
  everywhere P0 did. Every "can I plan from here?" question — safe-stop release condition B,
  `beginChainedManeuverIfNeeded()`, `tryEarlyChainedManeuver()`, and the safe-stop escape check
  (`anyFeasibleCandidateFrom`) — now flows through that single generator, so "an escape exists"
  and "plan() returns an avoidance" can no longer disagree. Do NOT reintroduce a second candidate
  generator; the 2026-08-15 permanent safe-stop deadlock was exactly plan()-vs-escape-check
  divergence. P3 candidates are still re-measured (`measureCandidate`) and exact-validated
  (`validateCandidate`) by the same safety layer P0 used; P3 trace metrics are never trusted for
  ranking or audit.
- **A value the launch file re-declares overrides the YAML silently.** The launch parameter dict
  is applied after `params_file`, so changing `config/local_planning.yaml` alone does nothing for
  those keys (`p3_mode`, `lockstep_mode`, the diagnostics toggles; verified 2026-08-15). Always
  confirm the node's startup log line rather than trusting the YAML, and keep launch defaults in
  sync with the YAML.
- Safe-stop release condition B requires the escape to target the latched obstacle ONLY while that
  obstacle is still present in the current `/confirmed_static_obs` snapshot. Once it is gone (the car
  stopped just past it and it left the FOV) the identity test is a stale bookkeeping token, while
  condition A cannot fire either because clearing the danger range by `safe_stop_buffer_m` needs
  forward motion the latch itself prevents. Keep the "latched obstacle no longer present" branch:
  without it that combination is a permanent deadlock (sim 2026-08-15: latched on obstacle 0, a
  valid left escape existed for obstacle 1, car stopped indefinitely). The escape candidate is
  exact-validated against the CURRENT raw detector geometry before it reaches the lifecycle.
- The `p3_backup_fallback_count_` ratio counts only callbacks where P3 was actually ASKED for a
  path -- a usable snapshot AND a non-empty blocking cluster. On a clear track every callback
  reaches the P0 fallback by design; counting those made an obstacle-free lap report
  "1872/1924 콜백" as if P3 had failed 97% of the time, inverting the one number this counter
  exists to produce.
- If neither side is safe, publish only a collision-checked gradual-stop prefix before the obstacle.
  During an active avoidance, derive that prefix from the remaining committed geometry so stopping
  never forces an immediate return to `d=0`. Without a usable committed prefix, keep the current
  `ego.d`; if no forward stop prefix exists, publish a zero-speed current-`d` hold rather than an
  empty path that would fall back to global. Latch safe-stop and its obstacle IDs/sequence, danger
  `s` range, stop target, activation ego `s`, and timestamp immediately. An empty `/confirmed_static_obs`
  array or an empty-cycle count is never a release condition. Release only after ego passes the
  latched danger range with margin, a hard-valid path for the same obstacle is consecutively
  confirmed and selectable in `STATE_AVOID`, or a stopped vehicle sees a persistently and
  explicitly clear forward corridor in fresh detector-health frames. While the danger remains
  logically ahead, reject global-raceline handoff. Never publish an unvalidated moving avoidance
  path merely to keep `/avoid_waypoints` non-empty.
- Treat stale Frenet odometry as a worst-case localization failure: publish a zero-speed hold at the
  last known pose without erasing a previously validated commitment. Resume ordinary validation and
  planning only after fresh odometry returns.
- Recompute heading and curvature after applying `d(s)`, cap moving-avoidance waypoint speed with
  the configured velocity-limit table, then recompute longitudinal acceleration. Do not apply this
  cap to global handoff geometry; safe-stop keeps its separate braking profile.
- Handle closed-track `s` wrap explicitly. Never encode a waypoint index in Frenet odometry fields.

## Interfaces

- Subscribe: `/global_waypoints` (`f110_msgs/msg/WpntArray`).
- Subscribe: `/confirmed_static_obs` (`f110_msgs/msg/ObstacleArray`, `obstacles_topic`); each
  obstacle must provide finite
  `s_start/s_end/d_right/d_left` fields forming a non-point Frenet footprint, with
  `d_right <= d_left`. `s_start > s_end` is valid across the closed-track wrap. Cartesian fields
  and `radius` are optional metadata and are not used as planner geometry.
- Subscribe: `/car_state/frenet/odom` (`nav_msgs/msg/Odometry`), with `position.x=s` and
  `position.y=d`.
- Subscribe: `/state` (`f110_msgs/msg/StateMachine`) for explicit AVOID-to-GLOBAL handoff
  acknowledgement.
- Publish: `/avoid_waypoints` (`f110_msgs/msg/OTWpntArray`) as an ego-to-merge segment with
  map-frame Cartesian `x_m/y_m` populated for every waypoint.
- Publish debug: `/local_planning/path` (`nav_msgs/msg/Path`) only.
- Never publish `/local_waypoints`; `state_machine_node` exclusively selects and publishes it
  according to its committed state.
- Tuning-only timing diagnostics are default-off companion messages. T0 is the first relevant
  non-empty `/confirmed_static_obs` accepted by the node and T1 is the first actual non-empty
  `/avoid_waypoints` publication. Capture the event's steady-clock time at the source operation and
  never feed `/cma_timing/events` back into perception or planning.
- Tuning-only record/replay diagnostics are also default-off. When enabled, publish passive JSON on
  `/cma_replay/planner_events` at initial-stabilization start/readiness and commitment. Include the
  source obstacle stamp and one `PLAN_CANDIDATE` record for every generated candidate: side, target,
  requested/effective entry, exit, centerline and rectangular-footprint wall clearance, footprint
  violation side/index/pose/relative heading/corner protrusion, obstacle clearance, peak
  curvature/rate, speed loss, rejection reason, and final rank. Do not introduce a decision clock
  or feed diagnostics back.
- `lockstep_mode` is CMA-only and default-off. It must suppress the wall planning timer and invoke
  the existing planner exactly once for each identical-stamp `/confirmed_static_obs` and Frenet odometry
  pair. Before processing step k after the first step, require the state-machine output from step
  k-1 so DDS arrival order cannot select a different cached state.

## Package layout and maintenance

- Node declaration: `include/local_planning/local_planner_node.hpp`.
- Algorithm declaration: `include/local_planning/raceline_spline_planner.hpp`.
- Uncertainty Guard declaration: `include/local_planning/obstacle_guard.hpp`.
- C++ sources: `src/local_planner_node.cpp`, `src/obstacle_guard.cpp`,
  `src/raceline_spline_planner.cpp`.
- Runtime parameters: `config/local_planning.yaml`.
- Launch entrypoint: `launch/local_planning.launch.py`.
- Korean node documentation: `docs/local_planner.md`.
- Algorithm tests: `test/test_raceline_spline.cpp`, including the wrong-branch snake regression.
- Guard tests: `test/test_obstacle_guard.cpp`, including variance inflation, frozen-envelope
  containment, accumulated drift rejection, invalid-variance fallback, and closed-track wrap.
- AABB projection belongs to `obstacle_detector` and must not be reintroduced here.
- Manual Frenet contract harness: `test/frenet_static_pipeline_test.py`; run it against a fresh
  `local_planner_node` with a `global_waypoints.csv` path.
- Initial-cluster harness: `test/initial_cluster_stabilization_pipeline_test.py`; run it against a
  fresh `local_planner_node` to verify the minimum stabilization time, that each late adjacent ID
  receives the configured number of real topic observations, and that it affects the first
  committed side.
- Soft-violation harness: `test/soft_violation_confirmation_pipeline_test.py`; run it against a
  fresh `local_planner_node` to verify a transient uncertainty-only collision keeps the commitment
  and a persistent soft collision replans only after the configured planning-cycle count.
- Pre-engagement switch harness: `test/pre_engagement_side_switch_pipeline_test.py`; keep ego before
  both lock thresholds and verify that an invalidated side is replaced directly by the other side.
- End-to-end detector harness: `test/static_obs_pipeline_test.py`; run it while
  `obstacle_detector_node` and `local_planner_node` are active to verify
  `/scan -> /static_obs -> /avoid_waypoints`.
- Stuck-case harness: `test/stuck_case_harness.cpp` (diagnostic, not a runtime node). Feed it the
  raceline CSV the car actually ran plus an ego/obstacle state pulled from a bag, and it prints
  every candidate's rejection reason plus the safe-stop escape verdict:
  `./build/local_planning/stuck_case_harness <csv> <ego_s> <ego_d> <ego_v> <obs_s0> <obs_s1>
  <obs_dr> <obs_dl> [reserve] [escape_check 0|1] [repeat]`. `repeat` runs `plan()` N times for
  cost measurement. **Use the live raceline** — a mismatched reference silently produces wrong
  curvature and wrong coordinates (see the reference-line note in `config/local_planning.yaml`).
  Keep the hardcoded `operationalParameters()` in sync with `config/local_planning.yaml`;
  a silently different margin makes the harness answer a question nobody asked.
- Safe-stop escape verification: `buildSafeStop` must decide the stop point with the **same**
  candidate generator `plan()` uses (`generateP3Candidates`). If a second copy of that loop is
  ever introduced, "avoidance is possible from the stop point" and the actual replan will drift
  apart and the trap comes back. Any change here needs the three regression cases in
  `test_raceline_spline.cpp` (`DensifiesShortSafeStopPrefixToMinimumPoints`,
  `ReportsWhenSafeStopPointIsNotEscapable`, `SafeStopEscapeCheckCanBeDisabled`) to stay green.
- Safe-stop/state harness: `test/safe_stop_latch_pipeline_test.py`; run it with
  `local_planner_node`, `state_machine_node`, and `wpnt_publisher` to verify the same-ID
  avoidance-to-stop latch, delayed release, and `/local_waypoints` forwarding contract.
- Sequential-obstacle harness: `test/sequential_obstacle_handoff_pipeline_test.py`; run it against
  a fresh `local_planner_node` to verify left-to-right maneuver chaining without an intermediate
  empty path or global handoff. Run it again with `--during-handoff` to verify handoff preemption.
- Post-merge-tail harness: `test/post_merge_tail_chaining_pipeline_test.py`; run it against a fresh
  `local_planner_node` to verify that a stabilized obstacle in the first path's controller-only tail
  preempts from nonzero `ego.d` without preparation, safe-stop, empty output, or GLOBAL handoff.
- Stale-perception harness: `test/stale_obstacle_memory_pipeline_test.py`; run it against a fresh
  `local_planner_node` to verify frozen-path retention beyond the stale timeout, GLOBAL handoff
  completion, and last-snapshot reuse on the next lap while `/confirmed_static_obs` remains silent.
- Keep all runtime values configurable in YAML and load that YAML from the launch file.
- Keep `timing_diagnostics_enable=false` in the operational YAML.
- Keep `replay_diagnostics_enable=false` in the operational YAML.
- Keep `lockstep_mode=false` in the operational YAML.
- `p3_mode=TEST_ACTIVE` is the operational default (2026-08-12, user decision: P3 is the primary
  driving mode with P0 as backup). `SHADOW` must never publish a P3 path or mutate the P0
  commitment/safe-stop state. Every callback must identify `P3_M1`, `P3_COMMITTED_SUFFIX`, or
  `P0_BACKUP_ONLY` ownership.
- Safe-stop escalation is graded; do not collapse the ladder: (1) a margin-only blocking cluster
  (every member's RAW envelope + `vehicle_half_width_m + safety_margin_m` stays clear of d=0)
  degrades to the capped-speed lane hold (`margin_pass_speed_cap_mps`, `margin_pass=true`
  commitment that skips margin-based validators and is instead re-checked physically every
  cycle); (2) a physically blocking cluster brakes on the collision-free stop prefix; (3) with
  no stop prefix, brake along `last_valid_guidance_path_` (`buildCommittedPathStop`, then
  `buildLastPathBrake`); (4) the in-place zero-speed emergency hold is the last resort only
  when no valid guidance path was ever published.
- Never publish a slow section as a flat step from the ego position
  (`approach_feasibility_decel_mps2`, 2026-08-14): the margin pass ramps down from the MEASURED
  ego speed, and the avoidance spline carries a backward braking ramp into the obstacle-span
  speed. A flat step saturates the service brake, slips past the friction limit and cost us
  steering authority on the real car (run_0814_111210 wall crash). Obstacle-span speeds
  themselves are reserve-backed — never raise them.
- P3/M1 is production-owned C++ in this package. External CMA/evaluator executables are parity
  oracles only and must never supply runtime local paths.
- Run P3/M1 candidate generation immediately for every authoritative non-empty snapshot that
  actually needs a path -- that is, whenever no active maneuver continues on that callback. Never
  gate it behind an additional readiness wait. Guard
  readiness is diagnostic provenance, not a standalone ownership veto: before the observation
  count/time Guard is complete, grant initial ownership only when the exact validator proves the
  selected path hard-valid against both the accumulated conservative geometry and the same
  callback's raw obstacle geometry. Do not add a second P3-only waiting parameter. After commit,
  never grow the frozen Guard by permanent lifetime union; use exact-ID containment and the
  guarded-then-raw exact-validation contract for each fresh live envelope.
- Preserve a selected P3 maneuver's immutable original geometry. Lifecycle continuation may trim
  only its passed prefix and must exact-revalidate the current suffix; it must complete after the
  expanded obstacle region is passed before a short suffix reaches the validator minimum.
- In `TEST_ACTIVE`, when current raw-obstacle validation discards a committed P3 suffix, the
  rejected suffix is never retained or published and control falls through to the existing
  `P0_BACKUP_ONLY`/safe-stop path. Do NOT re-run the evaluator on the same snapshot to retry: the
  lifecycle only reports `CURRENT_RAW_OBSTACLE_COLLISION` when the evaluation did not recover, and
  a pure re-evaluation of the same immutable snapshot returns the same verdict, so the retry can
  never select a fresh path. When the evaluation DOES recover, `advanceP3Lifecycle` already falls
  through to `selectFresh` within the same call. A same-callback replan branch existed here until
  2026-08-15 and was unreachable by construction.
- The exact validator runs once per fresh candidate. `P3ShadowResult` carries the selected
  candidate's guarded-geometry verdict as `selected_validation` /
  `selected_validation_available`, and `selectFresh` reuses it instead of repeating a bit-identical
  validation -- the `FRESH_RESULT_SNAPSHOT_LINEAGE_MISMATCH` guard above it already proves the
  inputs are the same snapshot. The subsequent RAW-geometry validation tests DIFFERENT geometry and
  must always run; never collapse the two. Regression:
  `EvaluatorCertificateReplacesRedundantGuardedValidation` and
  `CertifiedCandidateStillRejectedWhenRawGeometryCollides` in `test/test_p3_maneuver_lifecycle.cpp`.
- Update this file and the Korean documentation when behavior, topics, parameters, or launch usage
  changes.
