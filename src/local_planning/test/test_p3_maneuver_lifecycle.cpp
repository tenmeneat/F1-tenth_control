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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

#include "local_planning/p3_maneuver_lifecycle.hpp"

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

f110_msgs::msg::WpntArray ringReference()
{
  f110_msgs::msg::WpntArray reference;
  constexpr std::size_t kCount = 120U;
  constexpr double kRadius = 10.0;
  const double step = 2.0 * kPi * kRadius / static_cast<double>(kCount);
  reference.wpnts.reserve(kCount);
  for (std::size_t index = 0U; index < kCount; ++index) {
    const double angle = 2.0 * kPi * static_cast<double>(index) /
      static_cast<double>(kCount);
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = static_cast<std::int32_t>(index);
    waypoint.s_m = step * static_cast<double>(index);
    waypoint.x_m = kRadius * std::cos(angle);
    waypoint.y_m = kRadius * std::sin(angle);
    waypoint.psi_rad = angle + 0.5 * kPi;
    waypoint.kappa_radpm = 1.0 / kRadius;
    waypoint.vx_mps = 2.0;
    waypoint.d_left = 3.0;
    waypoint.d_right = 3.0;
    reference.wpnts.push_back(waypoint);
  }
  return reference;
}

f110_msgs::msg::Obstacle sideObstacle()
{
  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = 7;
  obstacle.s_center = 5.0;
  obstacle.s_start = 4.8;
  obstacle.s_end = 5.2;
  obstacle.d_center = 1.1;
  obstacle.d_right = 1.0;
  obstacle.d_left = 1.2;
  obstacle.size = std::hypot(0.4, 0.2);
  return obstacle;
}

f110_msgs::msg::Obstacle intrudingObstacle()
{
  auto obstacle = sideObstacle();
  obstacle.d_center = 0.0;
  obstacle.d_right = -0.2;
  obstacle.d_left = 0.2;
  return obstacle;
}

f110_msgs::msg::WpntArray forwardPath(
  const RacelineSplinePlanner & planner,
  std::size_t count = 24U)
{
  f110_msgs::msg::WpntArray path;
  path.wpnts.reserve(count);
  for (std::size_t index = 0U; index < count; ++index) {
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = static_cast<std::int32_t>(index);
    waypoint.s_m = 0.5 + 0.5 * static_cast<double>(index);
    waypoint.d_m = 0.0;
    planner.toCartesian(
      waypoint.s_m, waypoint.d_m, waypoint.x_m, waypoint.y_m, waypoint.psi_rad);
    waypoint.kappa_radpm = 0.1;
    waypoint.vx_mps = 2.0;
    waypoint.d_left = 3.0;
    waypoint.d_right = 3.0;
    path.wpnts.push_back(waypoint);
  }
  return path;
}

P3ShadowResult freshResult(
  const RacelineSplinePlanner & planner,
  const P3ManeuverSnapshot & snapshot,
  double cluster_end_forward_m,
  std::size_t path_points = 24U)
{
  P3ShadowResult result;
  result.enabled = true;
  result.invoked = true;
  result.would_recover = true;
  result.snapshot_source_stamp_ns = snapshot.source_stamp_ns;
  result.snapshot_epoch = snapshot.source_epoch;
  result.global_reference_generation = snapshot.global_reference_generation;
  result.selected_go_left = true;
  result.selected_obstacle_ids = {7};
  result.selected_cluster_end_forward_m = cluster_end_forward_m;
  result.selected_cluster_end_s = cluster_end_forward_m;
  result.selected_source = "M1";
  result.selected_source_cell = "CELL";
  result.selected_candidate_identity = "CANDIDATE";
  result.selected_logical_identity = "LOGICAL";
  result.selected_path_digest = "ORIGINAL_DIGEST";
  result.selected_path = forwardPath(planner, path_points);
  return result;
}

P3ManeuverSnapshot initialSnapshot()
{
  P3ManeuverSnapshot snapshot;
  snapshot.ego = {0.0, 0.0, 2.0};
  snapshot.obstacles = {sideObstacle()};
  snapshot.raw_obstacles = snapshot.obstacles;
  snapshot.source_stamp_ns = 100;
  snapshot.source_epoch = 3U;
  snapshot.global_reference_generation = 4U;
  snapshot.obstacle_sequence = 5U;
  return snapshot;
}

RacelineSplinePlanner plannerWithReference()
{
  RacelineSplinePlanner planner;
  std::string error;
  EXPECT_TRUE(planner.setReference(ringReference(), &error)) << error;
  return planner;
}

TEST(P3ManeuverLifecycle, ExpandedRegionCompletionPrecedesShortSuffixValidation)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  const auto fresh = lifecycle.selectFresh(
    snapshot, freshResult(planner, snapshot, 1.0, 24U), planner);
  ASSERT_TRUE(fresh.has_output);
  ASSERT_EQ(fresh.state, P3ManeuverLifecycleState::kFreshSelected);

  snapshot.ego.s = 1.01;
  snapshot.obstacles.clear();
  snapshot.source_stamp_ns = 101;
  const auto completion = lifecycle.continueCurrent(snapshot, planner, 3);
  EXPECT_TRUE(completion.complete);
  EXPECT_FALSE(completion.invalidated);
  EXPECT_FALSE(completion.suffix_revalidated);
  EXPECT_EQ(completion.state, P3ManeuverLifecycleState::kComplete);
  EXPECT_EQ(completion.reason, "EXPANDED_OBSTACLE_REGION_FULLY_PASSED");
  ASSERT_TRUE(completion.completion_handoff_available);
  ASSERT_FALSE(completion.completion_handoff_path.wpnts.empty());
  EXPECT_EQ(completion.original_path_digest, "ORIGINAL_DIGEST");
  EXPECT_NE(completion.completion_handoff_path_digest, "NONE");
  EXPECT_GE(completion.completion_handoff_path.wpnts.front().s_m, 1.0 - 1.0e-9);
  EXPECT_DOUBLE_EQ(completion.completion_handoff_path.wpnts.back().s_m, 12.0);
}

TEST(P3ManeuverLifecycle, GuardFormationAllowsDualExactValidFreshCandidate)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  snapshot.selection_guard_ready = false;
  const auto candidate = freshResult(planner, snapshot, 10.0);
  ASSERT_TRUE(candidate.invoked);
  ASSERT_TRUE(candidate.would_recover);

  P3ManeuverLifecycle lifecycle;
  const auto decision = lifecycle.selectFresh(snapshot, candidate, planner);
  EXPECT_TRUE(decision.has_output);
  EXPECT_TRUE(decision.fresh_selected);
  EXPECT_TRUE(decision.guarded_validation_attempted);
  EXPECT_TRUE(decision.guarded_validation_hard_valid);
  EXPECT_TRUE(decision.raw_validation_attempted);
  EXPECT_TRUE(decision.raw_validation_hard_valid);
  EXPECT_EQ(decision.reason, "FRESH_HARD_VALID_P3_M1");
  EXPECT_TRUE(lifecycle.active());
  EXPECT_EQ(lifecycle.state(), P3ManeuverLifecycleState::kFreshSelected);
}

TEST(P3ManeuverLifecycle, EvaluatorCertificateReplacesRedundantGuardedValidation)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  auto certified_candidate = freshResult(planner, snapshot, 10.0);
  // Exactly what the evaluator measures while building the candidate: same ego, same guarded
  // obstacles, same path, same default reserve scale.
  certified_candidate.selected_validation = planner.evaluateP3PathCurrent(
    snapshot.ego, certified_candidate.selected_path, snapshot.obstacles);
  certified_candidate.selected_validation_available = true;
  ASSERT_TRUE(certified_candidate.selected_validation.hard_valid);

  P3ManeuverLifecycle certified;
  const auto reused = certified.selectFresh(snapshot, certified_candidate, planner);

  auto uncertified_candidate = certified_candidate;
  uncertified_candidate.selected_validation_available = false;
  P3ManeuverLifecycle uncertified;
  const auto revalidated = uncertified.selectFresh(snapshot, uncertified_candidate, planner);

  EXPECT_TRUE(reused.guarded_validation_reused_certificate);
  EXPECT_FALSE(revalidated.guarded_validation_reused_certificate);
  // Reusing the certificate must be observationally identical to re-running the validation.
  EXPECT_EQ(reused.has_output, revalidated.has_output);
  EXPECT_EQ(reused.fresh_selected, revalidated.fresh_selected);
  EXPECT_EQ(reused.suffix_hard_valid, revalidated.suffix_hard_valid);
  EXPECT_EQ(reused.guarded_validation_hard_valid, revalidated.guarded_validation_hard_valid);
  EXPECT_EQ(reused.guarded_validation_rejection, revalidated.guarded_validation_rejection);
  EXPECT_EQ(reused.reason, revalidated.reason);
  // The raw-geometry check tests different geometry and must survive in BOTH paths.
  EXPECT_TRUE(reused.raw_validation_attempted);
  EXPECT_TRUE(revalidated.raw_validation_attempted);
  EXPECT_EQ(reused.raw_validation_hard_valid, revalidated.raw_validation_hard_valid);
}

TEST(P3ManeuverLifecycle, CertifiedCandidateStillRejectedWhenRawGeometryCollides)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  auto candidate = freshResult(planner, snapshot, 10.0);
  candidate.selected_validation = planner.evaluateP3PathCurrent(
    snapshot.ego, candidate.selected_path, snapshot.obstacles);
  candidate.selected_validation_available = true;
  ASSERT_TRUE(candidate.selected_validation.hard_valid);
  // A guarded certificate says nothing about the raw detector geometry. Put a blocking obstacle
  // only in raw_obstacles: the guarded step is skipped by the certificate, so this proves the raw
  // check is what still rejects the candidate.
  auto blocking = snapshot.obstacles.front();
  blocking.d_right = -1.0;
  blocking.d_left = 1.0;
  snapshot.raw_obstacles = {blocking};

  P3ManeuverLifecycle lifecycle;
  const auto decision = lifecycle.selectFresh(snapshot, candidate, planner);
  EXPECT_TRUE(decision.guarded_validation_reused_certificate);
  EXPECT_TRUE(decision.guarded_validation_hard_valid);
  EXPECT_TRUE(decision.raw_validation_attempted);
  EXPECT_FALSE(decision.raw_validation_hard_valid);
  EXPECT_FALSE(decision.has_output);
  EXPECT_FALSE(lifecycle.active());
}

TEST(P3ManeuverLifecycle, GuardFormationRejectsFreshCandidateThatIsRawHardInvalid)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  snapshot.selection_guard_ready = false;
  snapshot.raw_obstacles = {intrudingObstacle()};
  const auto candidate = freshResult(planner, snapshot, 10.0);

  P3ManeuverLifecycle lifecycle;
  const auto decision = lifecycle.selectFresh(snapshot, candidate, planner);
  EXPECT_FALSE(decision.has_output);
  EXPECT_FALSE(decision.fresh_selected);
  EXPECT_TRUE(decision.guarded_validation_hard_valid);
  EXPECT_TRUE(decision.raw_validation_attempted);
  EXPECT_FALSE(decision.raw_validation_hard_valid);
  EXPECT_EQ(decision.reason,
    "FRESH_RAW_EXACT_HARD_INVALID:d-offset intersects an inflated static-obstacle box");
  EXPECT_FALSE(lifecycle.active());
}

TEST(P3ManeuverLifecycle, CompletionHandoffIsFrozenPostObstacleOriginalTail)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  const auto selected = freshResult(planner, snapshot, 2.0, 24U);
  ASSERT_TRUE(lifecycle.selectFresh(snapshot, selected, planner).has_output);

  snapshot.ego.s = 2.01;
  snapshot.obstacles.clear();
  snapshot.source_stamp_ns = 101;
  const auto completion = lifecycle.continueCurrent(snapshot, planner, 3);
  ASSERT_TRUE(completion.complete);
  ASSERT_TRUE(completion.completion_handoff_available);
  ASSERT_FALSE(completion.completion_handoff_path.wpnts.empty());
  EXPECT_FALSE(completion.suffix_revalidated);
  EXPECT_EQ(completion.completion_handoff_path.header.frame_id,
    selected.selected_path.header.frame_id);
  for (std::size_t index = 0U; index < completion.completion_handoff_path.wpnts.size(); ++index) {
    const auto & actual = completion.completion_handoff_path.wpnts[index];
    const auto expected = std::find_if(
      selected.selected_path.wpnts.begin(), selected.selected_path.wpnts.end(),
      [&actual](const auto & waypoint) {return waypoint.id == actual.id;});
    ASSERT_NE(expected, selected.selected_path.wpnts.end());
    EXPECT_DOUBLE_EQ(actual.s_m, expected->s_m);
    EXPECT_DOUBLE_EQ(actual.d_m, expected->d_m);
    EXPECT_DOUBLE_EQ(actual.x_m, expected->x_m);
    EXPECT_DOUBLE_EQ(actual.y_m, expected->y_m);
    EXPECT_DOUBLE_EQ(actual.psi_rad, expected->psi_rad);
    EXPECT_DOUBLE_EQ(actual.kappa_radpm, expected->kappa_radpm);
    EXPECT_DOUBLE_EQ(actual.vx_mps, expected->vx_mps);
    EXPECT_DOUBLE_EQ(actual.ax_mps2, expected->ax_mps2);
  }
}

TEST(P3ManeuverLifecycle, SourceEpochChangeInvalidatesImmediately)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  ++snapshot.source_epoch;
  snapshot.source_stamp_ns = 101;
  const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);
  EXPECT_TRUE(decision.invalidated);
  EXPECT_FALSE(decision.has_output);
  EXPECT_EQ(decision.reason, "SOURCE_EPOCH_MISMATCH");
  EXPECT_FALSE(lifecycle.active());
}

TEST(P3ManeuverLifecycle, StaleSourceInvalidatesWithoutSuffixPublication)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  snapshot.source_stale = true;
  const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);
  EXPECT_TRUE(decision.invalidated);
  EXPECT_FALSE(decision.has_output);
  EXPECT_FALSE(decision.suffix_revalidated);
  EXPECT_EQ(decision.reason, "STALE_SOURCE");
}

TEST(P3ManeuverLifecycle, CurrentExactHardInvalidSuffixIsNeverPublished)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  snapshot.ego.s = 0.1;
  snapshot.source_stamp_ns = 101;
  snapshot.obstacles.front().s_center = 1.0;
  snapshot.obstacles.front().s_start = 0.8;
  snapshot.obstacles.front().s_end = 1.2;
  snapshot.obstacles.front().d_center = 0.0;
  snapshot.obstacles.front().d_right = -0.2;
  snapshot.obstacles.front().d_left = 0.2;
  snapshot.raw_obstacles = snapshot.obstacles;
  const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);
  EXPECT_TRUE(decision.suffix_revalidated);
  EXPECT_FALSE(decision.suffix_hard_valid);
  EXPECT_TRUE(decision.invalidated);
  EXPECT_FALSE(decision.has_output);
  EXPECT_EQ(decision.state, P3ManeuverLifecycleState::kInvalidated);
  EXPECT_NE(decision.reason.find("CURRENT_RAW_OBSTACLE_COLLISION:"), std::string::npos);
}

TEST(P3ManeuverLifecycle, CurrentExactValidationUsesTrimmedCopyAndPreservesIdentity)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  snapshot.ego.s = 0.6;
  snapshot.source_stamp_ns = 101;
  const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);
  ASSERT_TRUE(decision.suffix_revalidated);
  ASSERT_TRUE(decision.suffix_hard_valid) << decision.validation.rejection_reason;
  EXPECT_TRUE(decision.has_output);
  EXPECT_EQ(decision.state, P3ManeuverLifecycleState::kCommitted);
  EXPECT_EQ(decision.original_candidate_identity, "CANDIDATE");
  EXPECT_EQ(decision.original_path_digest, "ORIGINAL_DIGEST");
  EXPECT_LE(decision.output_path.wpnts.size(), 24U);
  EXPECT_NE(decision.output_path_digest, decision.original_path_digest);
}

TEST(P3ManeuverLifecycle, ReferenceGenerationChangeInvalidatesImmediately)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  ++snapshot.global_reference_generation;
  snapshot.source_stamp_ns = 101;
  const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);
  EXPECT_TRUE(decision.invalidated);
  EXPECT_FALSE(decision.has_output);
  EXPECT_EQ(decision.reason, "GLOBAL_REFERENCE_GENERATION_MISMATCH");
}

TEST(P3ManeuverLifecycle, SourceStampRegressionInvalidatesImmediately)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  snapshot.source_stamp_ns = 99;
  const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);
  EXPECT_TRUE(decision.invalidated);
  EXPECT_FALSE(decision.has_output);
  EXPECT_EQ(decision.reason, "SOURCE_STAMP_REGRESSION");
}

TEST(P3ManeuverLifecycle, MissingOriginalObstacleIdentityInvalidatesImmediately)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  snapshot.source_stamp_ns = 101;
  snapshot.obstacles.front().id = 8;
  const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);
  EXPECT_TRUE(decision.invalidated);
  EXPECT_FALSE(decision.has_output);
  EXPECT_EQ(decision.reason, "OBSTACLE_MANEUVER_IDENTITY_MISMATCH");
}

TEST(P3ManeuverLifecycle, FreshEmptyObstacleSnapshotContinuesImmutableSuffix)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  snapshot.ego.s = 0.6;
  snapshot.source_stamp_ns = 101;
  snapshot.obstacles.clear();
  snapshot.raw_obstacles.clear();
  const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);
  EXPECT_FALSE(decision.invalidated);
  EXPECT_TRUE(decision.suffix_revalidated);
  EXPECT_TRUE(decision.suffix_hard_valid) << decision.validation.rejection_reason;
  EXPECT_TRUE(decision.has_output);
  EXPECT_EQ(decision.state, P3ManeuverLifecycleState::kCommitted);
  EXPECT_EQ(decision.original_candidate_identity, "CANDIDATE");
  EXPECT_EQ(decision.original_path_digest, "ORIGINAL_DIGEST");
}

TEST(P3ManeuverLifecycle, BackwardProgressInvalidatesInsteadOfWrapping)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  snapshot.source_stamp_ns = 101;
  snapshot.ego.s = planner.trackLength() - 0.1;
  const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);
  EXPECT_TRUE(decision.invalidated);
  EXPECT_FALSE(decision.has_output);
  EXPECT_EQ(decision.reason, "WRAP_OR_BACKWARD_PROGRESS");
}

TEST(P3ManeuverLifecycle, FreshSelectionReplacesPreviousImmutableIdentity)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  snapshot.source_stamp_ns = 101;
  auto replacement = freshResult(planner, snapshot, 11.0);
  replacement.selected_candidate_identity = "REPLACEMENT_CANDIDATE";
  replacement.selected_logical_identity = "REPLACEMENT_LOGICAL";
  replacement.selected_path_digest = "REPLACEMENT_DIGEST";
  replacement.selected_path.wpnts.front().ax_mps2 = 1.25;
  const auto fresh = lifecycle.selectFresh(snapshot, replacement, planner);
  ASSERT_TRUE(fresh.fresh_selected);
  EXPECT_EQ(fresh.original_candidate_identity, "REPLACEMENT_CANDIDATE");
  EXPECT_EQ(fresh.original_path_digest, "REPLACEMENT_DIGEST");

  snapshot.source_stamp_ns = 102;
  snapshot.ego.s = 0.6;
  const auto continuation = lifecycle.continueCurrent(snapshot, planner, 3);
  ASSERT_TRUE(continuation.has_output);
  EXPECT_EQ(continuation.original_candidate_identity, "REPLACEMENT_CANDIDATE");
  EXPECT_EQ(continuation.original_path_digest, "REPLACEMENT_DIGEST");
}

TEST(P3ManeuverLifecycle, SameIdEnvelopeInsideFrozenGuardKeepsImmutableSuffix)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  snapshot.source_stamp_ns = 101;
  snapshot.ego.s = 0.1;
  snapshot.obstacles.front().s_start = 4.85;
  snapshot.obstacles.front().s_end = 5.15;
  snapshot.obstacles.front().d_right = 1.02;
  snapshot.obstacles.front().d_left = 1.18;
  snapshot.raw_obstacles = snapshot.obstacles;
  const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);

  EXPECT_TRUE(decision.has_output);
  EXPECT_TRUE(decision.suffix_hard_valid) << decision.validation.rejection_reason;
  EXPECT_FALSE(decision.guard_raw_revalidated);
  EXPECT_EQ(decision.guard_contained_same_id_count, 1U);
  EXPECT_EQ(decision.original_path_digest, "ORIGINAL_DIGEST");
  EXPECT_EQ(decision.state, P3ManeuverLifecycleState::kCommitted);
}

TEST(P3ManeuverLifecycle, GuardOnlyCollisionHoldsFrozenPathWithoutExpiry)
{
  // Retention contract (2026-08-12): a broken uncertainty guard with a raw-hard-valid suffix is
  // NOT a margin violation, so the frozen path is held indefinitely — the previous N-cycle
  // confirmation expiry re-planned a nearly identical geometry on every progressive envelope
  // reveal. The counter keeps accumulating for diagnostics only.
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  snapshot.ego.s = 0.1;
  snapshot.obstacles = {intrudingObstacle()};
  snapshot.raw_obstacles = {sideObstacle()};
  for (int count = 1; count <= 6; ++count) {  // twice the old 3-cycle confirmation window
    snapshot.source_stamp_ns = 100 + count;
    const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);
    ASSERT_TRUE(decision.has_output);
    EXPECT_TRUE(decision.suffix_hard_valid) << decision.validation.rejection_reason;
    EXPECT_TRUE(decision.guard_raw_revalidated);
    EXPECT_TRUE(decision.guard_soft_violation_pending);
    EXPECT_EQ(decision.guard_soft_violation_count, count);
    EXPECT_EQ(
      decision.reason,
      "GUARD_SOFT_OBSTACLE_VIOLATION_PENDING_RAW_HARD_VALID");
    EXPECT_FALSE(decision.invalidated);
  }
}

TEST(P3ManeuverLifecycle, RawSameIdCollisionInvalidatesImmediately)
{
  auto planner = plannerWithReference();
  auto snapshot = initialSnapshot();
  P3ManeuverLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.selectFresh(
      snapshot, freshResult(planner, snapshot, 10.0), planner).has_output);

  snapshot.source_stamp_ns = 101;
  snapshot.ego.s = 0.1;
  snapshot.obstacles = {intrudingObstacle()};
  snapshot.raw_obstacles = snapshot.obstacles;
  const auto decision = lifecycle.continueCurrent(snapshot, planner, 3);

  EXPECT_TRUE(decision.guard_raw_revalidated);
  EXPECT_TRUE(decision.invalidated);
  EXPECT_FALSE(decision.has_output);
  EXPECT_FALSE(decision.suffix_hard_valid);
  EXPECT_EQ(decision.reason.find("CURRENT_RAW_OBSTACLE_COLLISION:"), 0U);
}

}  // namespace
}  // namespace local_planning
