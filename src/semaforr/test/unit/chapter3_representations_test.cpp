#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <semaforr/domain/completed_path.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/reactive_planner.hpp>
#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learners/conveyor_learner.hpp>

namespace {

constexpr double pi = 3.14159265358979323846;

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
      1U, {pathPoint(1U, {0.0, 0.0}, {4.0, 0.0})}, {4.0, 0.0});
  const auto interrupted = path(
      2U,
      {pathPoint(2U, {0.0, 0.0}, {4.0, 0.0},
                 semaforr::domain::ExecutionCompletionStatus::SafetyInterrupted)},
      {4.0, 0.0});
  const auto failed_skeleton = semaforr::spatial::learnRegionSkeleton(
      regions, {}, {interrupted});
  EXPECT_TRUE(failed_skeleton.region_edges.empty());
  const auto skeleton = semaforr::spatial::learnRegionSkeleton(
      regions, {straightTrail(1U, 0.0)}, {traveled});
  ASSERT_EQ(skeleton.region_nodes.size(), 2U);
  ASSERT_EQ(skeleton.nodes.size(), regions.learned_regions.size());
  ASSERT_EQ(skeleton.region_edges.size(), 1U);
  EXPECT_EQ(skeleton.region_edges.front().supporting_subtrail.size(), 2U);
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
        return std::holds_alternative<semaforr::planning::SubtrailStep>(step);
      }));
  EXPECT_NE(plan.explanation.find("region-skeleton"), std::string::npos);
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
