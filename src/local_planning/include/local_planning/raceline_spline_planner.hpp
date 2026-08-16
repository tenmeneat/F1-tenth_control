// Copyright 2026 2026_IFAC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LOCAL_PLANNING__RACELINE_SPLINE_PLANNER_HPP_
#define LOCAL_PLANNING__RACELINE_SPLINE_PLANNER_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>

#include "local_planning/p3_shadow.hpp"

namespace local_planning
{

class P3ShadowEvaluator;

struct RacelineSplineParameters
{
  double detection_lookahead_m{12.0};
  double obstacle_cluster_gap_m{0.8};
  double obstacle_longitudinal_padding_m{0.35};
  double vehicle_length_m{0.56};
  double vehicle_half_width_m{0.1435};
  // Obstacle bounds are raw detector geometry, so vehicle size, physical margin, and measured
  // closed-loop tracking error are applied exactly once. Global d_left/d_right are distances to
  // physical track boundaries; the rotated vehicle footprint and independent wall reserve are
  // therefore checked against those boundaries in the hard validator.
  double safety_margin_m{0.03};
  // Fallback used when the LUT arrays below are all empty. A configured LUT is row-major with
  // speed as the outer axis and absolute curvature as the inner axis.
  double tracking_error_reserve_m{0.14};
  std::vector<double> tracking_error_lut_speed_bins_mps;
  std::vector<double> tracking_error_lut_curvature_bins_radpm;
  std::vector<double> tracking_error_lut_values_m;
  // Speed-dependent lateral-acceleration table copied from jazzy_main velocity_limits.csv.
  // Avoidance waypoint speeds are capped so v^2 * |kappa| stays within the interpolated limit.
  std::vector<double> avoidance_velocity_limit_speed_bins_mps;
  std::vector<double> avoidance_velocity_limit_lateral_accel_mps2;
  // The lateral-acceleration cap above only binds in curves, so an obstacle sitting on a straight
  // is planned at full race-line speed and reserves the widest tracking error in the LUT -- which
  // is what makes an otherwise passable gap unusable. Slow down for the gap itself instead: the
  // reserve a waypoint may spend is whatever lateral room is left between the path and the
  // obstacle face, and speed is reduced only until the LUT reserve fits inside it. A waypoint that
  // is not passing an obstacle is never slowed, so obstacle-free laps keep their race-line speed.
  // Below this floor the maneuver is treated as infeasible rather than crawled through.
  double avoidance_minimum_speed_mps{2.0};
  double wall_safety_margin_m{0.0};
  double fallback_track_half_width_m{1.50};
  // Speed cap for the margin-only slow pass: a cluster that blocks the race line only through
  // the inflated tracking/uncertainty margin (its raw envelope plus the physical base clearance
  // never reaches d = 0) is physically passable on the line, so avoidance failure degrades to a
  // capped-speed lane hold instead of a safe stop or a zero-speed hold.
  double margin_pass_speed_cap_mps{2.0};
  // Approach feasibility ramp: a slow section that begins abruptly (margin pass flat cap from
  // ego, gap/curvature-capped obstacle span) is a speed STEP the vehicle cannot track — the
  // service brake saturates, the tires slip past the friction limit and steering authority is
  // lost (2026-08-14 real-car wall crashes: 4.4 m/s ego vs flat 2.0 m/s plan). Two shapes, one
  // rate, both only active when the ego/profile is faster than the slow section:
  //  - margin slow pass: pre-cluster waypoints may keep a profile that decelerates from the
  //    MEASURED ego speed at this rate, steepened only as much as needed to still reach the cap
  //    by the cluster start (degrades to the old flat cap when there is no room).
  //  - avoidance spline: pre-span waypoints are LOWERED onto the backward braking profile that
  //    reaches the span-start speed at this rate, so braking starts well before the span
  //    boundary instead of as a step at it. Span speeds themselves are never touched.
  // <= 0 disables both (old step behavior).
  double approach_feasibility_decel_mps2{2.0};
  // Committed-path retention band: while re-validating an ALREADY COMMITTED path (P3
  // continuation, P0 commitment hold), the tracking-error reserve portion of the obstacle
  // clearance is scaled by this fraction, so envelope growth/jitter inside the released band
  // freezes the path instead of reshaping it every callback. The physical base clearance is
  // never reduced, and fresh planning always uses the full reserve. 1.0 disables the band.
  double commitment_retention_reserve_fraction{0.5};
  // Localization (MCL vs ground-truth) lateral error reserve, added as a constant floor inside
  // trackingErrorReserve() so every consumer (envelope expansion, hard validation, gap-limited
  // speed inversion) sees the same total. Participates in the retention scaling above like the
  // rest of the reserve. Sized from sustained error (per-speed-bin P95), NOT transient MCL
  // correction spikes — those are single-cycle events the retention band absorbs. 0 disables.
  double localization_reserve_m{0.0};

  std::vector<double> pre_apex_distances_m{6.0, 4.0, 2.0};
  std::vector<double> post_apex_distances_m{1.0, 2.0, 3.0};
  // Entry and exit intentionally use separate parameter families. Entry fractions scale the
  // available-distance ratio pre_apex_far/detection_lookahead; transition scales are exit-only.
  std::vector<double> entry_transition_fractions{0.50, 0.75, 1.00};
  std::vector<double> transition_distance_scales{1.0, 1.25, 1.50};
  double outside_line_transition_scale{1.35};
  // Absolute cap on the exit segment length past the obstacle cluster. Disabled (non-positive)
  // by default: capping the exit forces the path back to the race line early, and when a second
  // obstacle sits a few metres downstream the shortened exit dives straight into its inflated
  // box the moment the detector reveals it, invalidating the committed maneuver at a range where
  // no fresh plan exists yet (regression observed 2026-08-12 with obstacles 3.6 m apart). The
  // long exit deliberately keeps the offset through closely-following obstacles; merge latency
  // is solved by the completion handback to the closed global handoff loop instead.
  double maximum_exit_length_m{0.0};
  double post_merge_lookahead_m{2.0};
  double post_merge_min_time_sec{1.0};
  // 완료 핸드오프 복귀 램프: 길이 = max(min_length, |v| * time). 너무 짧으면 램프의
  // 추가 곡률(최대 6|d0|/L^2)이 커지므로 min_length가 하한을 지킨다.
  // 🔴 기본 0 = 비활성 (2026-08-13). 시뮬 회귀에서 램프가 벽 협착부(s≈23) 최소 벽 여유를
  // 0.117→0.082 m로 깎았고, 벽 클램프는 웨이포인트 d_left/d_right가 실제 벽보다 낙관적이라
  // (제어팀 실측 0.16~0.23 m) 물리지 않았다. 제어팀 섹터별 벽 여유 실측 테이블로 경계를
  // 보정한 뒤에만 활성화할 것 — 낙관 경계로 켜면 협착부 벽 여유를 그대로 깎는다.
  double merge_ramp_min_length_m{0.0};
  double merge_ramp_time_sec{0.0};
  double minimum_target_offset_m{0.20};
  double maximum_target_offset_m{1.50};
  int target_d_candidate_count{5};
  double maximum_lateral_slope{0.65};
  double maximum_curvature_radpm{3.20};
  double maximum_curvature_rate_radpm2{20.0};

  double safe_stop_buffer_m{0.40};
  double safe_stop_deceleration_mps2{2.5};
  int minimum_path_points{8};

  // 안전정지 정지점 탈출 검증. safe_stop_buffer_m은 손으로 맞춘 상수라, 기하에 따라
  // "정지는 했는데 그 자리에서 회피 곡선을 만들 진입 거리가 없는" 영구 교착이 생긴다
  // (2026-08-14 실차: 임계 2.0~2.5 m vs 버퍼 1.20 m — 모든 안전정지가 교착이었다).
  // 켜면 정지점을 확정하기 전에 그 지점에서 v=0으로 회피가 생성되는지 확인하고,
  // 안 되면 safe_stop_escape_retreat_step_m씩 뒤로 물린다.
  bool safe_stop_escape_check_enable{true};
  // 한 스텝 후퇴 거리와 최대 후퇴 횟수. step × max_steps가 버퍼에 더해질 수 있는 최대
  // 후퇴량이다. 후보 생성을 그만큼 반복하므로 무한정 키우면 안 된다.
  double safe_stop_escape_retreat_step_m{0.30};
  int safe_stop_escape_max_retreats{8};

  bool hasTrackingErrorLut() const;
  bool trackingErrorLutValid() const;
  bool avoidanceVelocityLimitValid() const;
  double limitedAvoidanceSpeed(double requested_speed_mps, double curvature_radpm) const;
  // Largest speed not above `requested_speed_mps` whose tracking-error reserve still fits inside
  // `admissible_reserve_m`, never going below avoidance_minimum_speed_mps. The reserve is
  // monotonically non-decreasing in speed, so this is a plain monotone inversion of the LUT.
  double gapLimitedAvoidanceSpeed(
    double requested_speed_mps, double curvature_radpm, double admissible_reserve_m) const;
  // Clamp a combined (outside-multiplier-applied) exit transition scale so that
  // post_apex_distances_m.back() * scale never exceeds maximum_exit_length_m.
  double cappedCombinedExitScale(double combined_exit_scale) const;
  double trackingErrorReserve(double speed_mps, double curvature_radpm) const;
  double avoidanceTrackingErrorReserve(double speed_mps, double curvature_radpm) const;
  double obstacleBaseClearance() const;
  double obstacleSafetyClearance(
    double speed_mps, double curvature_radpm, double reserve_scale = 1.0) const;
  double trackBoundaryReserve(double speed_mps, double curvature_radpm) const;
};

struct EgoFrenetState
{
  double s{0.0};
  double d{0.0};
  double speed{0.0};
};

enum class SplinePlanKind
{
  kNoObstacle,
  kPreparation,
  kAvoidance,
  kSafeStop,
  kNoSafePath
};

struct SplineControlPoint
{
  double forward_s{0.0};
  double d{0.0};
};

// Complete, passive record of one generated candidate. Metrics are calculated before hard
// validation so rejected candidates remain auditable. final_rank is one-based for feasible
// candidates and -1 for rejected candidates.
struct SplineCandidateAudit
{
  std::size_t generation_index{0U};
  bool feasible{false};
  bool selected{false};
  bool go_left{false};
  int final_rank{-1};
  // exit 램프가 다음(비클러스터) 장애물의 물리 엔벨로프에 닿는 후보인가 — 순위 강등의
  // 원인 플래그. rank_without_exit_demotion은 그 강등 항을 뺀 순수 slack 순위로,
  // final_rank와 다르면 강등이 이 사이클의 선택을 실제로 바꿨다는 뜻이다. 감사 전용.
  bool exit_reaches_next_obstacle{false};
  int rank_without_exit_demotion{-1};
  double target_d{std::numeric_limits<double>::quiet_NaN()};
  double entry_fraction{std::numeric_limits<double>::quiet_NaN()};
  double exit_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double requested_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double effective_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double exit_length_m{std::numeric_limits<double>::quiet_NaN()};
  double centerline_wall_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double rectangular_footprint_wall_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
  bool footprint_invalid{false};
  std::string footprint_violation_side;
  std::int64_t footprint_violation_waypoint_index{-1};
  double footprint_violation_s_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_x_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_y_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double footprint_heading_relative_to_reference_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double wallward_corner_protrusion_m{std::numeric_limits<double>::quiet_NaN()};
  // Compatibility metric used by the existing candidate ranker. It remains the centerline
  // headroom so this hard-validator change does not silently alter candidate ranking semantics.
  double wall_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_rate_radpm2{std::numeric_limits<double>::quiet_NaN()};
  double velocity_loss{std::numeric_limits<double>::quiet_NaN()};
  double global_path_deviation_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_normalized_safety_slack{-std::numeric_limits<double>::infinity()};
  std::string rejection_reason;
};

struct RacelineSplineResult
{
  SplinePlanKind kind{SplinePlanKind::kNoObstacle};
  f110_msgs::msg::WpntArray path;
  bool go_left{false};
  double target_d{0.0};
  double merge_s{0.0};
  // Passive decision metadata used by deterministic replay diagnostics. These values do not
  // participate in candidate selection or path generation.
  double entry_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double exit_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double effective_entry_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double effective_exit_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double requested_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double effective_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double exit_length_m{std::numeric_limits<double>::quiet_NaN()};
  int obstacle_id{-1};
  std::vector<int> obstacle_ids;
  std::vector<SplineControlPoint> control_points;
  std::vector<SplineCandidateAudit> candidate_audits;
  // True for a margin-only slow pass: the path intentionally rides inside the inflated margin
  // band, so margin-based commitment validation must not replace it; it stays valid until an
  // obstacle's raw envelope (plus the physical base clearance) actually reaches the race line.
  bool margin_pass{false};
  // 안전정지 진단. escape_verified가 false면 정지점(그리고 자차 위치까지의 모든 후퇴
  // 지점)에서 회피 후보가 하나도 생성되지 않는다 — 전진 계획으로는 재출발할 수 없는
  // 상태이므로 노드가 이를 로그로 드러내야 한다. forward_m은 자차 기준 정지점 거리다.
  bool safe_stop_escape_verified{true};
  double safe_stop_forward_m{std::numeric_limits<double>::quiet_NaN()};
  std::string reason;
};

enum class PathValidationFailureKind
{
  kNone,
  kInput,
  kNoForwardPath,
  kTrackBoundary,
  kObstacleCollision,
  kGeometry
};

struct PathValidationFailure
{
  PathValidationFailureKind kind{PathValidationFailureKind::kNone};
  std::string reason;
  int obstacle_id{-1};
  std::size_t waypoint_index{std::numeric_limits<std::size_t>::max()};
  double waypoint_s{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_d{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_s_start{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_s_end{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_source_d_right{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_source_d_left{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_test_d_right{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_test_d_left{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_clearance{std::numeric_limits<double>::quiet_NaN()};
  double centerline_wall_clearance{std::numeric_limits<double>::quiet_NaN()};
  double rectangular_footprint_wall_clearance{
    std::numeric_limits<double>::quiet_NaN()};
  std::string footprint_violation_side;
  double waypoint_x{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_y{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_yaw{std::numeric_limits<double>::quiet_NaN()};
  double heading_relative_to_reference{std::numeric_limits<double>::quiet_NaN()};
  double wallward_corner_protrusion{std::numeric_limits<double>::quiet_NaN()};
};

// Static-obstacle planner whose only geometric reference is the ordered global race line.
// A candidate never searches the map for a shortcut: it keeps every selected global waypoint's
// s/order and changes only its local Frenet d offset before converting it back to map coordinates.
class RacelineSplinePlanner
{
public:
  explicit RacelineSplinePlanner(
    RacelineSplineParameters parameters = RacelineSplineParameters());

  void setParameters(const RacelineSplineParameters & parameters);
  bool setReference(const f110_msgs::msg::WpntArray & reference, std::string * error = nullptr);
  bool ready() const;
  double trackLength() const;
  double forwardDistance(double from_s, double to_s) const;
  std::vector<int> blockingClusterIds(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  // True when any visible obstacle's RAW envelope plus the physical base clearance
  // (vehicle half width + safety margin, no tracking-error reserve) reaches the race line —
  // i.e. the vehicle could not physically follow d = 0 without touching it. Margin-only
  // blocking (inflated interval touches the line but the raw one stays clear) returns false.
  bool obstaclesPhysicallyBlockRaceline(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  // 완료 핸드오프용 글로벌 루프. ego의 현재 횡오프셋 d에서 d=0까지 smoothstep 램프로
  // 내려가는 계획된 복귀 구간을 앞머리에 접붙인다 — 램프 없이 d=0 라인을 그대로 주면
  // 복귀가 컨트롤러의 자연 수렴에 맡겨져 실측 0.055 m/m로 느리고(2026-08-12), 연속
  // 장애물에서 오프셋이 누적된다. FSM 합류 판정(|ego_d| <= threshold 지속)은 램프와
  // 무관하게 물리적 합류를 계속 게이트한다.
  f110_msgs::msg::WpntArray buildGlobalHandoffPath(
    const EgoFrenetState & ego, double state_tail_distance_m, double speed_cap_mps) const;
  f110_msgs::msg::WpntArray buildEmergencyStopPath(const EgoFrenetState & ego) const;
  // Truncate `path` from the waypoint nearest ahead of ego and apply a braking profile that
  // stops within the configured safe-stop deceleration, without any obstacle search. Used as
  // the last-resort stop geometry when no collision-free stop prefix exists: braking along the
  // most recent vetted path beats an instantaneous zero-speed hold on a moving vehicle.
  f110_msgs::msg::WpntArray buildLastPathBrake(
    const EgoFrenetState & ego, const f110_msgs::msg::WpntArray & path) const;
  RacelineSplineResult buildCommittedPathStop(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & committed_path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  RacelineSplineResult buildPreparationStop(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;

  RacelineSplineResult plan(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::optional<bool> & preferred_left = std::nullopt,
    bool allow_side_switch = true) const;

  // Production-owned analytic P3/M1 path-family evaluation. Runtime ownership is decided by
  // LocalPlannerNode's explicit mode; this const adapter cannot alter P0 planner state.
  P3ShadowResult evaluateP3Shadow(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    std::int64_t snapshot_source_stamp_ns,
    std::uint64_t snapshot_epoch,
    std::uint64_t global_reference_generation,
    const std::string & p0_failure_reason) const;

  // Revalidate a committed P3 suffix against the current immutable planning snapshot using the
  // same production hard validator and configured minimum-path contract as fresh P3 candidates.
  // `obstacle_reserve_scale` scales only the tracking-error reserve portion of the obstacle
  // clearance (the physical base clearance is never reduced); values < 1 form the committed-path
  // retention band. Fresh planning must always validate with the default full reserve.
  // `collision_horizon` bounds the OBSTACLE check to this maneuver's responsibility range exactly
  // as `generateP3Candidates` does at selection time; track-bound and geometry checks always cover
  // the whole path. Selection and revalidation MUST pass the same horizon: an obstacle that lies
  // inside one's range and outside the other's makes every cycle select a path the next cycle
  // condemns, which is an unbreakable replan loop, not a safety check.
  P3ShadowPathEvaluation evaluateP3PathCurrent(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    double obstacle_reserve_scale = 1.0,
    const std::optional<double> & collision_horizon = std::nullopt) const;

  bool validatePath(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    std::string * error = nullptr,
    PathValidationFailure * failure = nullptr,
    const std::optional<double> & maximum_collision_forward_m = std::nullopt,
    double obstacle_reserve_scale = 1.0) const;

  double commitmentRetentionReserveFraction() const
  {
    return parameters_.commitment_retention_reserve_fraction;
  }

  // Controller tail appended after the merge. It is also the single margin every maneuver-scope
  // collision horizon adds to its cluster end, so selection and revalidation stay identical.
  double postMergeLookaheadM() const
  {
    return parameters_.post_merge_lookahead_m;
  }

  void toCartesian(double s, double d, double & x, double & y, double & yaw) const;

private:
  friend class P3ShadowEvaluator;

  struct ExpandedObstacle;
  struct Candidate;
  struct FootprintTrackBoundSample;

  double wrapS(double s) const;
  std::size_t nextReferenceIndex(double s) const;
  std::size_t nearestReferenceIndex(double s) const;
  double maximumReferenceTrackingErrorReserve(
    const EgoFrenetState & ego, double start, double end,
    double speed_cap_mps = std::numeric_limits<double>::infinity()) const;
  std::vector<ExpandedObstacle> expandVisibleObstacles(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  bool isBlockingRaceline(const ExpandedObstacle & obstacle) const;
  bool physicallyBlocksRaceline(const ExpandedObstacle & obstacle) const;
  bool clusterPhysicallyBlocksRaceline(
    const std::vector<ExpandedObstacle> & cluster) const;
  std::vector<ExpandedObstacle> nearestCluster(
    const std::vector<ExpandedObstacle> & obstacles) const;
  bool outsideIsLeft(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & cluster) const;
  // relaxed_clearance_gate=true는 strict 게이트의 전 후보가 exact validator에서 기각된 뒤의
  // 2차 시도 전용: 최소 clearance target을 avoidance_minimum_speed_mps 기준 게이트로 낮춘다.
  bool computeSideTargetRange(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & cluster,
    bool go_left,
    double & cluster_start,
    double & cluster_end,
    double & minimum_clearance_target_d,
    double & maximum_track_target_d,
    std::string & reason,
    bool relaxed_clearance_gate = false) const;
  bool targetFitsTrackBounds(
    const EgoFrenetState & ego,
    double cluster_start,
    double cluster_end,
    bool go_left,
    double target_d,
    std::string & reason,
    double * min_headroom = nullptr) const;
  void measureCandidate(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & visible,
    Candidate & candidate) const;
  FootprintTrackBoundSample measureFootprintTrackBound(
    const f110_msgs::msg::Wpnt & waypoint,
    std::size_t waypoint_index) const;
  // 이 패키지의 유일한 회피 후보 생성기 — P0 quintic 격자는 2026-08-15에 제거됐다
  // (실차 시험에서 P0가 통과 가능한 모든 곳을 P3도 통과함이 확인됨). P3(analytic
  // corridor)가 이 상태에서 만들어 낸 후보들을 Candidate로 변환해 append한다. plan()과
  // 안전정지 탈출 검증이 **같은 코드**로 후보를 만들어야 "정지점에서 회피 가능"이라는
  // 판정이 실제 재계획과 일치한다. 두 벌로 나뉘면 조용히 어긋난다. 안전 계층
  // (expandVisibleObstacles / measureCandidate / validateCandidate)과 안전정지 사다리는
  // 그대로이며, 여기서 만든 후보도 동일한 measureCandidate로 재측정한다.
  // 반환값은 이번 호출에서 생성된 feasible 후보 수.
  // stop_on_first_feasible=true면 첫 통과 후보에서 즉시 멈춘다(탈출 가능성만 물을 때).
  std::size_t generateP3Candidates(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::vector<ExpandedObstacle> & visible,
    const std::optional<bool> & preferred_left,
    bool allow_side_switch,
    bool stop_on_first_feasible,
    std::vector<Candidate> & candidates,
    std::string & reason) const;
  // 주어진 자차 상태에서 회피 경로가 하나라도 생성되는가. 경로는 만들지 않고 가능성만 본다.
  bool anyFeasibleCandidateFrom(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::vector<ExpandedObstacle> & visible,
    const std::vector<ExpandedObstacle> & cluster) const;
  // 경로 점 수가 minimum_path_points에 못 미치면 최장 구간을 반복 이등분해 채운다.
  void densifyPath(f110_msgs::msg::WpntArray & path, std::size_t minimum_points) const;
  // raw_obstacles는 **절대 s**를 담은 원본이다. ExpandedObstacle은 자차 상대거리를
  // 담으므로, 가상의 정지점 기준으로 탈출 가능성을 물으려면 그 지점 기준으로 다시
  // 확장해야 한다. 원본 없이 기존 visible/cluster를 재사용하면 장애물이 정지점에서도
  // 같은 거리에 있는 것으로 보여 검증이 통째로 무의미해진다.
  RacelineSplineResult buildSafeStop(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & visible,
    const std::vector<ExpandedObstacle> & cluster,
    const std::vector<f110_msgs::msg::Obstacle> & raw_obstacles,
    const ExpandedObstacle & blocking) const;
  RacelineSplineResult buildMarginSlowPass(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & cluster) const;
  void applyAvoidanceVelocityLimit(
    f110_msgs::msg::WpntArray & path,
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & visible) const;
  void updateGeometryAndAcceleration(f110_msgs::msg::WpntArray & path) const;
  bool validateCandidate(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<ExpandedObstacle> & visible,
    std::string & reason,
    std::size_t start_index = 0U,
    std::size_t minimum_points = 0U,
    PathValidationFailure * failure = nullptr,
    const std::optional<double> & maximum_collision_forward_m = std::nullopt,
    double obstacle_reserve_scale = 1.0) const;
  P3ShadowPlanningContext buildP3ShadowPlanningContext(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    bool relaxed_clearance_gate = false) const;
  void finalizeP3ShadowPath(
    f110_msgs::msg::WpntArray & path,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  P3ShadowPathEvaluation validateP3ShadowPath(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    double obstacle_reserve_scale = 1.0,
    const std::optional<double> & collision_horizon = std::nullopt) const;

  RacelineSplineParameters parameters_;
  f110_msgs::msg::WpntArray reference_;
  double track_length_{0.0};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__RACELINE_SPLINE_PLANNER_HPP_
