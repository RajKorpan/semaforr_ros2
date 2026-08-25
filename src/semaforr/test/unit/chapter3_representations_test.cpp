/**
 * @file chapter3_representations_test.cpp
 * @brief Chapter3 representations test responsibilities.
 *
 * @details This file exercises chapter3 representations test behavior for automated
 * verification and regression testing. It centers on
 * `RetainsSelectionExecutionOutcomesAndInterruptions`,
 * `SelectsHandComputedHistoricalVisibilityMarkers`,
 * `UsesOnlyActualReachedTranslationGeometry`,
 * `SuccessfulRotationIsNotTraversedPathGeometry`,
 * `RepeatedSuccessfulTrailsIncreaseCellStrength`,
 * `FailedTargetTraversalAddsNoFrequency`,
 * `UsesMinimumRangeAndDeterministicReconciliation`,
 * `BuildsFirstClassExitsAndExitDerivedArc`. Its package-relative location
 * is `test/unit/chapter3_representations_test.cpp`.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <semaforr/domain/completed_path.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/reactive_planner.hpp>
#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learners/conveyor_learner.hpp>
#include <semaforr/spatial/learners/hallway_learner.hpp>

namespace {

constexpr double pi = 3.14159265358979323846;

/**
 * @brief Performs the observation operation for this subsystem.
 *
 * Arguments:
 * - @p x: Supplies x input to the operation.
 * - @p y: Supplies y input to the operation.
 * - @p range: Supplies range input to the operation.
 *
 * Returns:
 * - `semaforr::domain::RobotObservation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::RobotObservation observation(double x, double y,
                                                double range = 10.0) {
  semaforr::domain::RobotObservation result;
  result.pose = {{x, y}, semaforr::domain::Angle::zero()};
  result.laser.angle_min = semaforr::domain::Angle(-pi);
  result.laser.angle_increment = semaforr::domain::Angle(pi / 4.0);
  result.laser.minimum_range = semaforr::domain::Distance(0.05);
  result.laser.maximum_range = semaforr::domain::Distance(10.0);
  result.laser.ranges_m.assign(9U, range);
  return result;
}

/**
 * @brief Performs the path point operation for this subsystem.
 *
 * Arguments:
 * - @p id: Supplies id input to the operation.
 * - @p start: Supplies start input to the operation.
 * - @p finish: Supplies finish input to the operation.
 * - @p status: Supplies status input to the operation.
 *
 * Returns:
 * - `semaforr::domain::PathDecisionPoint` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::PathDecisionPoint pathPoint(
    std::uint64_t id, semaforr::domain::Point2D start,
    semaforr::domain::Point2D finish,
    semaforr::domain::ExecutionCompletionStatus status =
        semaforr::domain::ExecutionCompletionStatus::Succeeded) {
  semaforr::domain::PathDecisionPoint point;
  point.selection.decision_id = id;
  point.selection.action_id = id;
  point.selection.task_id = 1U;
  point.selection.expected_start = {start, semaforr::domain::Angle::zero()};
  point.selection.action =
      semaforr::domain::Action(semaforr::domain::ActionType::Forward, 1U);
  point.execution.decision_id = id;
  point.execution.action_id = id;
  point.execution.task_id = 1U;
  point.execution.status = status;
  point.execution.start_pose = point.selection.expected_start;
  point.execution.final_pose = {finish, semaforr::domain::Angle::zero()};
  point.execution.distance_achieved_m =
      semaforr::domain::distance(start, finish).meters();
  point.executed_action = point.selection.action;
  point.decision_observation = observation(start.x_m, start.y_m);
  return point;
}

/**
 * @brief Performs the path operation for this subsystem.
 *
 * Arguments:
 * - @p id: Supplies id input to the operation.
 * - @p points: Supplies points input to the operation.
 * - @p target: Supplies target input to the operation.
 *
 * Returns:
 * - `semaforr::domain::CompletedPath` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::CompletedPath path(
    semaforr::domain::PathId id,
    std::vector<semaforr::domain::PathDecisionPoint> points,
    semaforr::domain::Point2D target) {
  semaforr::domain::CompletedPath result;
  result.id = id;
  result.task_id = 1U;
  result.target = target;
  result.target_reached = true;
  result.decision_points = std::move(points);
  return result;
}

/**
 * @brief Performs the straight trail operation for this subsystem.
 *
 * Arguments:
 * - @p id: Supplies id input to the operation.
 * - @p y: Supplies y input to the operation.
 *
 * Returns:
 * - `semaforr::domain::LearnedTrail` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::LearnedTrail straightTrail(semaforr::domain::TrailId id,
                                              double y) {
  semaforr::domain::LearnedTrail trail;
  trail.id = id;
  trail.source_path = id;
  trail.markers.push_back({0U, {{0.0, y}, semaforr::domain::Angle::zero()},
                           observation(0.0, y).laser, std::nullopt});
  trail.markers.push_back({1U, {{4.0, y}, semaforr::domain::Angle::zero()},
                           observation(4.0, y).laser, std::nullopt});
  return trail;
}

}  // namespace

TEST(CompletedPath, RetainsSelectionExecutionOutcomesAndInterruptions) {
  semaforr::domain::PathHistory history;
  history.begin(7U, 1U, semaforr::domain::Point2D{2.0, 0.0});
  auto successful = pathPoint(1U, {0.0, 0.0}, {1.0, 0.0});
  history.record(successful);
  auto failed = pathPoint(
      2U, {1.0, 0.0}, {1.2, 0.0},
      semaforr::domain::ExecutionCompletionStatus::SafetyInterrupted);
  failed.interrupted = true;
  failed.executed_action.reset();
  history.record(failed);
  const auto completed = history.finish(false, true, {});
  ASSERT_TRUE(completed);
  ASSERT_EQ(completed->decision_points.size(), 2U);
  EXPECT_TRUE(completed->decision_points.front().successfulTraversal());
  EXPECT_FALSE(completed->decision_points.back().successfulTraversal());
  EXPECT_TRUE(completed->decision_points.back().execution.moved());
  EXPECT_TRUE(completed->decision_points.back().interrupted);
  EXPECT_FALSE(completed->decision_points.back().executed_action);
  EXPECT_EQ(completed->decision_points.back().selection.action,
            semaforr::domain::Action(semaforr::domain::ActionType::Forward,
                                     1U));
}

TEST(TrailCompatibility, SelectsHandComputedHistoricalVisibilityMarkers) {
  auto completed = path(
      11U,
      {pathPoint(1U, {0.0, 0.0}, {1.0, 1.0}),
       pathPoint(2U, {1.0, 1.0}, {1.0, 1.0},
                 semaforr::domain::ExecutionCompletionStatus::NoMovement),
       pathPoint(3U, {1.0, 1.0}, {2.0, -1.0}),
       pathPoint(4U, {2.0, -1.0}, {4.0, 0.0})},
      {4.0, 0.0});
  const auto trail = semaforr::spatial::learnVisibilityTrail(completed, 5U);
  ASSERT_EQ(trail.markers.size(), 2U);
  EXPECT_EQ(trail.markers.front().pose.position,
            (semaforr::domain::Point2D{0.0, 0.0}));
  EXPECT_EQ(trail.markers.back().pose.position,
            (semaforr::domain::Point2D{4.0, 0.0}));
  ASSERT_TRUE(trail.markers.front().visibility_to_next);
  EXPECT_EQ(trail.source_path, 11U);
  EXPECT_EQ(trail.subtrail_geometry.size(), 1U);
}

TEST(TrailCompatibility, UsesOnlyActualReachedTranslationGeometry) {
  auto failed = pathPoint(
      1U, {0.0, 0.0}, {8.0, 0.0},
      semaforr::domain::ExecutionCompletionStatus::ControllerFailure);
  // The controller reported no achieved translation; an erroneous intended
  // final pose must not become learned geometry.
  failed.execution.distance_achieved_m = 0.0;
  failed.executed_action.reset();
  auto failed_path = path(12U, {failed}, {8.0, 0.0});
  EXPECT_TRUE(
      semaforr::spatial::learnVisibilityTrail(failed_path, 12U).markers.empty());

  auto partial = pathPoint(
      2U, {0.0, 0.0}, {0.4, 0.0},
      semaforr::domain::ExecutionCompletionStatus::PartialMovement);
  partial.execution.distance_achieved_m = 0.4;
  const auto partial_path = path(13U, {partial}, {2.0, 0.0});
  semaforr::spatial::TrailLearningConfiguration configuration;
  configuration.include_partial_movement = true;
  const auto learned = semaforr::spatial::learnVisibilityTrail(
      partial_path, 13U, configuration);
  ASSERT_EQ(learned.markers.size(), 2U);
  EXPECT_EQ(learned.markers.front().pose.position,
            (semaforr::domain::Point2D{0.0, 0.0}));
  EXPECT_EQ(learned.markers.back().pose.position,
            (semaforr::domain::Point2D{0.4, 0.0}));
}

TEST(TrailCompatibility, SuccessfulRotationIsNotTraversedPathGeometry) {
  auto rotation = pathPoint(1U, {0.0, 0.0}, {0.0, 0.0});
  rotation.selection.action = semaforr::domain::Action(
      semaforr::domain::ActionType::TurnLeft, 1U);
  rotation.executed_action = rotation.selection.action;
  rotation.execution.rotation_achieved_rad = 0.5;
  rotation.execution.final_pose.heading = semaforr::domain::Angle(0.5);
  EXPECT_TRUE(rotation.execution.moved());
  EXPECT_FALSE(rotation.execution.translated());
  EXPECT_FALSE(rotation.successfulTraversal());
  EXPECT_TRUE(semaforr::spatial::learnVisibilityTrail(
                  path(14U, {rotation}, {1.0, 0.0}), 14U)
                  .markers.empty());
}

TEST(ConveyorCompatibility, RepeatedSuccessfulTrailsIncreaseCellStrength) {
  const auto one = semaforr::spatial::learnConveyorGrid({straightTrail(1U, 0.0)});
  const auto two = semaforr::spatial::learnConveyorGrid(
      {straightTrail(1U, 0.0), straightTrail(2U, 0.0)});
  ASSERT_FALSE(one.grid.cells.empty());
  ASSERT_EQ(one.grid.cells.size(), two.grid.cells.size());
  EXPECT_EQ(one.grid.maximum_frequency, 1U);
  EXPECT_EQ(two.grid.maximum_frequency, 2U);
  for (const auto& cell : two.grid.cells)
    EXPECT_EQ(cell.traversal_frequency, 2U);

  semaforr::domain::SpatialModel spatial;
  spatial.conveyor_grid = two.grid;
  semaforr::planning::PlanningRequest request;
  request.start.position = {0.0, 0.0};
  request.spatial_model = &spatial;
  const auto preferred = semaforr::planning::evaluatePathObjectives(
      request, {{1.0, 0.0}, {2.0, 0.0}});
  const auto unsupported = semaforr::planning::evaluatePathObjectives(
      request, {{1.0, 3.0}, {2.0, 3.0}});
  EXPECT_LT(preferred.at(semaforr::planning::PlanObjective::ConveyorPreference),
            unsupported.at(
                semaforr::planning::PlanObjective::ConveyorPreference));
}

TEST(ConveyorCompatibility, FailedTargetTraversalAddsNoFrequency) {
  semaforr::spatial::ConveyorLearner learner(
      0.05, semaforr::spatial::SpatialLearningMode::Compatibility);
  semaforr::spatial::NavigationEpisode episode;
  episode.sequence = 1U;
  episode.observation = observation(0.0, 0.0);
  episode.active_task = 7U;
  episode.active_target = semaforr::domain::Point2D{2.0, 0.0};
  const auto failed = pathPoint(
      1U, {0.0, 0.0}, {0.3, 0.0},
      semaforr::domain::ExecutionCompletionStatus::SafetyInterrupted);
  episode.selection = failed.selection;
  episode.execution_result = failed.execution;
  episode.action_started = true;
  learner.observe(episode);
  learner.rebuild();
  const auto model =
      std::get<semaforr::spatial::ConveyorModel>(learner.snapshot().payload);
  EXPECT_TRUE(model.grid.cells.empty());
  EXPECT_TRUE(model.flows.empty());

  semaforr::spatial::ConveyorLearner successful(
      0.05, semaforr::spatial::SpatialLearningMode::Compatibility);
  auto completed = episode;
  completed.execution_result =
      pathPoint(2U, {0.0, 0.0}, {2.0, 0.0}).execution;
  completed.selection =
      pathPoint(2U, {0.0, 0.0}, {2.0, 0.0}).selection;
  completed.target_reached = true;
  completed.task_finished = true;
  completed.action_started = true;
  successful.observe(completed);
  successful.rebuild();
  const auto successful_model = std::get<semaforr::spatial::ConveyorModel>(
      successful.snapshot().payload);
  EXPECT_FALSE(successful_model.grid.cells.empty());
}

TEST(RegionCompatibility, UsesMinimumRangeAndDeterministicReconciliation) {
  std::vector<semaforr::spatial::NavigationEpisode> episodes;
  for (std::size_t index = 0U; index < 2U; ++index) {
    semaforr::spatial::NavigationEpisode episode;
    episode.sequence = index + 1U;
    episode.observation = observation(0.0, 0.0, index == 0U ? 2.0 : 1.0);
    episode.selection.emplace();
    episode.selection->decision_id = index + 1U;
    episodes.push_back(std::move(episode));
  }
  const auto regions = semaforr::spatial::learnDecisionRegions(episodes);
  ASSERT_EQ(regions.learned_regions.size(), 1U);
  EXPECT_EQ(regions.learned_regions.front().id, 1U);
  EXPECT_NEAR(regions.learned_regions.front().boundary.radius.meters(), 1.0,
              1.0e-9);
  EXPECT_EQ(regions.learned_regions.front().supporting_observation.pose.position,
            (semaforr::domain::Point2D{0.0, 0.0}));
  EXPECT_TRUE(std::any_of(
      regions.learned_regions.front().visibility.begin(),
      regions.learned_regions.front().visibility.end(),
      [](const auto& bin) { return bin.known; }));
}

TEST(DoorCompatibility, BuildsFirstClassExitsAndExitDerivedArc) {
  semaforr::spatial::RegionModel regions;
  semaforr::domain::LearnedRegion region;
  region.id = 1U;
  region.boundary = {{0.0, 0.0}, semaforr::domain::Distance(1.0)};
  regions.learned_regions.push_back(region);
  regions.regions.push_back(region.boundary);
  const auto east = path(1U, {pathPoint(1U, {0.0, 0.0}, {2.0, 0.0})},
                         {2.0, 0.0});
  const auto northeast = path(
      2U, {pathPoint(2U, {0.0, 0.0}, {2.0, 0.4})}, {2.0, 0.4});
  semaforr::spatial::DoorLearningConfiguration configuration;
  configuration.exit_merge_angle_rad = 0.05;
  const auto doors = semaforr::spatial::learnRegionExitsAndDoors(
      regions, {east, northeast}, configuration);
  ASSERT_GE(doors.exits.size(), 2U);
  ASSERT_FALSE(doors.doors.empty());
  EXPECT_EQ(doors.doors.front().region, 1U);
  EXPECT_GE(doors.doors.front().exits.size(), 2U);
  EXPECT_FALSE(doors.openings.empty());
}

TEST(DoorCompatibility, AttemptedButUnexecutedCrossingCreatesNoExit) {
  semaforr::spatial::RegionModel regions;
  semaforr::domain::LearnedRegion region;
  region.id = 1U;
  region.boundary = {{0.0, 0.0}, semaforr::domain::Distance(1.0)};
  regions.learned_regions.push_back(region);
  auto rejected = pathPoint(
      1U, {0.0, 0.0}, {2.0, 0.0},
      semaforr::domain::ExecutionCompletionStatus::ControllerRejected);
  rejected.executed_action.reset();
  rejected.execution.distance_achieved_m = 0.0;
  const auto model = semaforr::spatial::learnRegionExitsAndDoors(
      regions, {path(15U, {rejected}, {2.0, 0.0})});
  EXPECT_TRUE(model.exits.empty());
  EXPECT_TRUE(model.doors.empty());
}

TEST(HallwayCompatibility, RunsDirectionalInferenceAndPublishesAreaAndWidth) {
  std::vector<semaforr::domain::CompletedPath> paths;
  for (std::size_t index = 0U; index < 4U; ++index) {
    const double y = static_cast<double>(index) * 0.4;
    paths.push_back(path(index + 1U,
                         {pathPoint(index + 1U, {0.0, y}, {5.0, y})},
                         {5.0, y}));
  }
  auto configuration = semaforr::spatial::HallwayLearningConfiguration{};
  configuration.initial_sigma = 0.0;
  const auto hallways =
      semaforr::spatial::learnCompatibilityHallways(paths, configuration);
  ASSERT_FALSE(hallways.hallways.empty());
  EXPECT_EQ(hallways.hallways.front().direction,
            semaforr::domain::HallwayDirection::Horizontal);
  EXPECT_GT(hallways.hallways.front().width_m, 0.0);
  EXPECT_GT(hallways.hallways.front().extent_m, 0.0);
  EXPECT_FALSE(hallways.hallways.front().connected_area.empty());
}

TEST(HallwayModernized, UsesSuccessfulExecutionSegmentNotDecisionPoseDelta) {
  using namespace semaforr;
  spatial::HallwayLearner learner(
      0.1, spatial::SpatialLearningMode::Modernized);
  spatial::NavigationEpisode successful;
  successful.sequence = 1U;
  successful.observation = observation(100.0, 100.0);
  successful.active_task = 1U;
  successful.action_started = true;
  successful.execution_result.emplace();
  successful.execution_result->status =
      domain::ExecutionCompletionStatus::Succeeded;
  successful.execution_result->start_pose =
      {{-2.0, 1.0}, domain::Angle::zero()};
  successful.execution_result->final_pose =
      {{1.0, 1.0}, domain::Angle::zero()};
  successful.execution_result->distance_achieved_m = 3.0;
  learner.observe(successful);

  auto rejected = successful;
  rejected.sequence = 2U;
  rejected.execution_result->status =
      domain::ExecutionCompletionStatus::ControllerRejected;
  rejected.action_started = false;
  rejected.execution_result->start_pose.position = {1.0, 1.0};
  rejected.execution_result->final_pose.position = {50.0, 50.0};
  learner.observe(rejected);
  learner.rebuild();
  const auto model = std::get<spatial::HallwayModel>(
      learner.snapshot().payload);
  ASSERT_EQ(model.centerlines.size(), 1U);
  EXPECT_EQ(model.centerlines.front().start,
            (domain::Point2D{-2.0, 1.0}));
  EXPECT_EQ(model.centerlines.front().end,
            (domain::Point2D{1.0, 1.0}));
}

TEST(SkeletonCompatibility, NodesAreRegionsAndEdgesCarryShortestSubtrails) {
  semaforr::spatial::RegionModel regions;
  for (std::size_t id = 0U; id < 2U; ++id) {
    semaforr::domain::LearnedRegion region;
    region.id = id + 10U;
    region.boundary = {{static_cast<double>(id) * 4.0, 0.0},
                       semaforr::domain::Distance(1.25)};
    regions.learned_regions.push_back(region);
    regions.regions.push_back(region.boundary);
  }
  const auto traveled = path(
      1U,
      {pathPoint(1U, {0.0, 0.0}, {1.0, 0.0}),
       pathPoint(2U, {1.0, 0.0}, {2.0, 0.0}),
       pathPoint(3U, {2.0, 0.0}, {3.0, 0.0}),
       pathPoint(4U, {3.0, 0.0}, {4.0, 0.0})},
      {4.0, 0.0});
  const auto second_traversal = path(
      3U,
      {pathPoint(5U, {0.0, 0.0}, {1.0, 0.2}),
       pathPoint(6U, {1.0, 0.2}, {2.0, -0.2}),
       pathPoint(7U, {2.0, -0.2}, {4.0, 0.0})},
      {4.0, 0.0});
  const auto interrupted = path(
      2U,
      {pathPoint(2U, {0.0, 0.0}, {4.0, 0.0},
                 semaforr::domain::ExecutionCompletionStatus::SafetyInterrupted)},
      {4.0, 0.0});
  const auto failed_skeleton = semaforr::spatial::learnRegionSkeleton(
      regions, {}, {interrupted});
  EXPECT_TRUE(failed_skeleton.region_edges.empty());
  auto cancelled = pathPoint(
      8U, {0.0, 0.0}, {4.0, 0.0},
      semaforr::domain::ExecutionCompletionStatus::Cancelled);
  cancelled.execution.distance_achieved_m = 0.0;
  const auto cancelled_skeleton = semaforr::spatial::learnRegionSkeleton(
      regions, {}, {path(4U, {cancelled}, {4.0, 0.0})});
  EXPECT_TRUE(cancelled_skeleton.region_edges.empty());
  const auto skeleton = semaforr::spatial::learnRegionSkeleton(
      regions, {straightTrail(1U, 0.0)}, {traveled, second_traversal});
  ASSERT_EQ(skeleton.region_nodes.size(), 2U);
  ASSERT_EQ(skeleton.nodes.size(), regions.learned_regions.size());
  ASSERT_EQ(skeleton.region_edges.size(), 1U);
  ASSERT_EQ(skeleton.region_edges.front().supporting_trails.size(), 2U);
  ASSERT_TRUE(skeleton.region_edges.front().operational_trail_id);
  EXPECT_EQ(skeleton.region_edges.front().supporting_subtrail.size(), 2U);
  EXPECT_LT(skeleton.region_edges.front().supporting_subtrail.size(),
            traveled.decision_points.size() + 1U);
  EXPECT_EQ(skeleton.region_edges.front().supporting_subtrail.front(),
            (semaforr::domain::Point2D{0.0, 0.0}));
  EXPECT_EQ(skeleton.region_edges.front().supporting_subtrail.back(),
            (semaforr::domain::Point2D{3.0, 0.0}));
  EXPECT_GT(skeleton.region_edges.front().length_m, 0.0);

  semaforr::domain::SpatialModel spatial;
  spatial.learned_regions = regions.regions;
  spatial.regions = regions.learned_regions;
  spatial.skeleton_nodes = skeleton.nodes;
  spatial.skeleton_edges = {{0U, 1U}};
  spatial.region_skeleton_nodes = skeleton.region_nodes;
  spatial.region_skeleton_edges = skeleton.region_edges;
  semaforr::planning::PlanningRequest request{
      {{0.0, 0.0}, semaforr::domain::Angle::zero()}, {4.0, 0.0}, &spatial};
  auto plan = semaforr::planning::SkeletonPlan{}.plan(request);
  ASSERT_TRUE(plan.succeeded());
  ASSERT_TRUE(plan.hierarchical);
  EXPECT_TRUE(std::any_of(
      plan.hierarchical->steps.begin(), plan.hierarchical->steps.end(),
      [](const auto& step) {
        return std::holds_alternative<
            semaforr::planning::SkeletonTransitionStep>(step);
      }));
  EXPECT_NE(plan.explanation.find("region-skeleton"), std::string::npos);
}

TEST(SkeletonSurrogates, UsesRetainedContainmentBeforeOtherChoices) {
  semaforr::domain::SpatialModel spatial;
  for (std::size_t index = 0U; index < 2U; ++index) {
    semaforr::domain::LearnedRegion region;
    region.id = 10U + index;
    region.boundary = {{5.0 * static_cast<double>(index), 0.0},
                       semaforr::domain::Distance(1.0)};
    spatial.regions.push_back(region);
    spatial.learned_regions.push_back(region.boundary);
    spatial.region_skeleton_nodes.push_back(
        {index, region.id, region.boundary.center, {}});
  }
  spatial.region_skeleton_edges.emplace_back(
      0U, 1U, std::vector<semaforr::domain::Point2D>{{0.0, 0.0}, {5.0, 0.0}},
      5.0, 1U);
  const auto result = semaforr::planning::SkeletonPlan{}.plan(
      {{{0.2, 0.0}, semaforr::domain::Angle::zero()}, {4.8, 0.0}, &spatial});
  ASSERT_TRUE(result.succeeded());
  ASSERT_TRUE(result.hierarchical);
  EXPECT_NE(std::find(result.hierarchical->diagnostics.begin(),
                      result.hierarchical->diagnostics.end(),
                      "surrogate:contained:region=0"),
            result.hierarchical->diagnostics.end());
  EXPECT_FALSE(std::any_of(
      result.hierarchical->steps.begin(), result.hierarchical->steps.end(),
      [](const auto& step) {
        return std::holds_alternative<
            semaforr::planning::VisibilityConnectionStep>(step);
      }));
}

TEST(SkeletonSurrogates, RetainsStartAndGoalVisibilityRaysAsPlanSteps) {
  semaforr::domain::SpatialModel spatial;
  for (std::size_t index = 0U; index < 2U; ++index) {
    semaforr::domain::LearnedRegion region;
    region.id = 20U + index;
    region.boundary = {{5.0 * static_cast<double>(index), 0.0},
                       semaforr::domain::Distance(0.25)};
    spatial.regions.push_back(region);
    spatial.learned_regions.push_back(region.boundary);
    spatial.region_skeleton_nodes.push_back(
        {index, region.id, region.boundary.center, {}});
  }
  spatial.region_skeleton_nodes[0].visibility[180] =
      {true, 3.0, {0.0, 0.0}, {-3.0, 0.0}, 41U};
  spatial.region_skeleton_nodes[1].visibility[0] =
      {true, 3.0, {5.0, 0.0}, {8.0, 0.0}, 42U};
  spatial.region_skeleton_edges.emplace_back(
      0U, 1U, std::vector<semaforr::domain::Point2D>{{0.0, 0.0}, {5.0, 0.0}},
      5.0, 1U);
  const auto result = semaforr::planning::SkeletonPlan{}.plan(
      {{{-2.0, 0.0}, semaforr::domain::Angle::zero()}, {7.0, 0.0}, &spatial});
  ASSERT_TRUE(result.succeeded());
  ASSERT_TRUE(result.hierarchical);
  std::vector<semaforr::planning::VisibilityConnectionStep> connections;
  for (const auto& step : result.hierarchical->steps)
    if (const auto* connection = std::get_if<
            semaforr::planning::VisibilityConnectionStep>(&step))
      connections.push_back(*connection);
  ASSERT_EQ(connections.size(), 2U);
  EXPECT_TRUE(connections.front().toward_region);
  EXPECT_FALSE(connections.back().toward_region);
  EXPECT_EQ(connections.front().supporting_decision, 41U);
  EXPECT_EQ(connections.back().supporting_decision, 42U);
  EXPECT_EQ(connections.front().evidence_ray_end,
            (semaforr::domain::Point2D{-3.0, 0.0}));
}

TEST(SkeletonSurrogates, CombinesDistanceAndDegreeDeterministically) {
  semaforr::domain::SpatialModel spatial;
  const std::array<double, 4U> x{1.2, 2.0, 4.0, 2.0};
  const std::array<double, 4U> y{0.0, 0.0, 0.0, 2.0};
  for (std::size_t index = 0U; index < x.size(); ++index) {
    semaforr::domain::LearnedRegion region;
    region.id = 30U + index;
    region.boundary = {{x[index], y[index]}, semaforr::domain::Distance(0.1)};
    spatial.regions.push_back(region);
    spatial.learned_regions.push_back(region.boundary);
    spatial.region_skeleton_nodes.push_back(
        {index, region.id, region.boundary.center, {}});
  }
  const auto add_edge = [&](std::size_t from, std::size_t to) {
    spatial.region_skeleton_edges.emplace_back(
        from, to,
        std::vector<semaforr::domain::Point2D>{
            spatial.region_skeleton_nodes[from].center,
            spatial.region_skeleton_nodes[to].center},
        1.0, 1U);
  };
  add_edge(1U, 0U);
  add_edge(1U, 2U);
  add_edge(1U, 3U);
  const auto result = semaforr::planning::SkeletonPlan{}.plan(
      {{{0.0, 0.0}, semaforr::domain::Angle::zero()}, {4.0, 0.0}, &spatial});
  ASSERT_TRUE(result.succeeded());
  ASSERT_TRUE(result.hierarchical);
  EXPECT_NE(std::find(result.hierarchical->diagnostics.begin(),
                      result.hierarchical->diagnostics.end(),
                      "surrogate:degree_distance:selected=1"),
            result.hierarchical->diagnostics.end());
  const auto first_region = std::find_if(
      result.hierarchical->steps.begin(), result.hierarchical->steps.end(),
      [](const auto& step) {
        return std::holds_alternative<semaforr::planning::RegionStep>(step);
      });
  ASSERT_NE(first_region, result.hierarchical->steps.end());
  EXPECT_EQ(std::get<semaforr::planning::RegionStep>(*first_region).region_id,
            1U);
}

TEST(RegionVisibilityCompatibility, SuppliesLowLevelExplorationCandidates) {
  semaforr::domain::WorldModel world;
  world.mission = semaforr::domain::Mission({{1U, {10.0, 0.0}}}, 20U);
  ASSERT_TRUE(world.mission.activate_next());
  world.robot.pose = {{0.0, 0.0}, semaforr::domain::Angle::zero()};
  world.robot.laser = observation(0.0, 0.0).laser;
  semaforr::domain::LearnedRegion region;
  region.id = 8U;
  region.boundary = {{8.0, 0.0}, semaforr::domain::Distance(1.0)};
  region.visibility[0] = {true, 2.0, {8.0, 0.0}, {10.0, 0.0}, 4U};
  region.visibility_revision = 4U;
  world.spatial.regions.push_back(region);
  const semaforr::domain::ActionSpace action_space({0.25}, {0.2});
  semaforr::planning::LowLevelExplorer explorer;
  ASSERT_EQ(explorer.evaluate({world, action_space}).status,
            semaforr::planning::ReactiveStatus::Action);
  EXPECT_TRUE(std::any_of(
      explorer.candidates().begin(), explorer.candidates().end(),
      [](const auto& candidate) {
        return candidate.source ==
               semaforr::planning::LLECandidateSource::RegionVisibility;
      }));
}
