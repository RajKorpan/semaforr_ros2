#include <gtest/gtest.h>

#include <algorithm>
#include <semaforr/exploration/high_level_explorer.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/traversability.hpp>
#include <semaforr/spatial/chapter3_learning.hpp>
#include <vector>

namespace {

semaforr::domain::RobotObservation observation(double x, double y,
                                                double heading = 0.0) {
  semaforr::domain::RobotObservation result;
  result.pose = {{x, y}, semaforr::domain::Angle(heading)};
  result.laser.angle_min = semaforr::domain::Angle(-0.61);
  result.laser.angle_increment = semaforr::domain::Angle(0.01);
  result.laser.minimum_range = semaforr::domain::Distance(0.1);
  result.laser.maximum_range = semaforr::domain::Distance(10.0);
  result.laser.ranges_m.assign(123U, 4.0);
  return result;
}

semaforr::domain::RobotObservation pathObservation(double x, double y) {
  auto result = observation(x, y);
  result.laser.angle_min = semaforr::domain::Angle(-3.14159265358979323846);
  result.laser.angle_increment =
      semaforr::domain::Angle(3.14159265358979323846 / 4.0);
  result.laser.ranges_m.assign(9U, 10.0);
  return result;
}

semaforr::domain::PathDecisionPoint traversal(std::uint64_t id,
                                               double start_x,
                                               double start_y,
                                               double finish_x,
                                               double finish_y) {
  semaforr::domain::PathDecisionPoint point;
  point.selection.decision_id = id;
  point.selection.action_id = id;
  point.selection.action =
      {semaforr::domain::ActionType::Forward, 1U};
  point.selection.expected_start =
      {{start_x, start_y}, semaforr::domain::Angle::zero()};
  point.execution.decision_id = id;
  point.execution.action_id = id;
  point.execution.status =
      semaforr::domain::ExecutionCompletionStatus::Succeeded;
  point.execution.start_pose = point.selection.expected_start;
  point.execution.final_pose =
      {{finish_x, finish_y}, semaforr::domain::Angle::zero()};
  point.execution.distance_achieved_m =
      semaforr::domain::distance({start_x, start_y},
                                 {finish_x, finish_y})
          .meters();
  point.executed_action = point.selection.action;
  point.decision_observation = pathObservation(start_x, start_y);
  return point;
}

semaforr::domain::CompletedPath path(std::uint64_t id,
                                     semaforr::domain::PathDecisionPoint point) {
  semaforr::domain::CompletedPath result;
  result.id = id;
  result.target_reached = true;
  result.decision_points.push_back(std::move(point));
  return result;
}

TEST(EnvironmentRegression, HallwayNetworkLearnsBothTravelOrientations) {
  std::vector<semaforr::domain::CompletedPath> horizontal;
  std::vector<semaforr::domain::CompletedPath> vertical;
  for (std::size_t index = 0U; index < 4U; ++index) {
    const double offset = static_cast<double>(index) * 0.4;
    horizontal.push_back(path(index + 1U,
                              traversal(index + 1U, -4.0, offset,
                                        4.0, offset)));
    vertical.push_back(path(index + 10U,
                            traversal(index + 10U, offset, -4.0,
                                      offset, 4.0)));
  }
  semaforr::spatial::HallwayLearningConfiguration configuration;
  configuration.initial_sigma = 0.0;
  const auto horizontal_model =
      semaforr::spatial::learnCompatibilityHallways(horizontal, configuration);
  const auto vertical_model =
      semaforr::spatial::learnCompatibilityHallways(vertical, configuration);
  EXPECT_TRUE(std::any_of(horizontal_model.hallways.begin(),
                          horizontal_model.hallways.end(),
                          [](const auto& hallway) {
                            return hallway.direction ==
                                   semaforr::domain::HallwayDirection::Horizontal;
                          }));
  EXPECT_TRUE(std::any_of(vertical_model.hallways.begin(),
                          vertical_model.hallways.end(),
                          [](const auto& hallway) {
                            return hallway.direction ==
                                   semaforr::domain::HallwayDirection::Vertical;
                          }));
}

TEST(EnvironmentRegression, HighwayCrossingOperationalizesThroughIntersection) {
  semaforr::domain::SpatialModel spatial;
  spatial.highways.graph.vertices = {
      {0U, {0, 0}, {-4.0, 0.0}, true},
      {1U, {0, 4}, {0.0, 0.0}, false},
      {2U, {0, 8}, {4.0, 0.0}, true},
      {3U, {-4, 4}, {0.0, -4.0}, true},
      {4U, {4, 4}, {0.0, 4.0}, true}};
  spatial.highways.graph.edges = {
      {0U, 1U, 10U, 4.0, {10U}, {{-4.0, 0.0}, {0.0, 0.0}}},
      {1U, 2U, 11U, 4.0, {11U}, {{0.0, 0.0}, {4.0, 0.0}}},
      {3U, 1U, 12U, 4.0, {12U}, {{0.0, -4.0}, {0.0, 0.0}}},
      {1U, 4U, 13U, 4.0, {13U}, {{0.0, 0.0}, {0.0, 4.0}}}};
  spatial.highways.revision = 3U;
  spatial.revisions[semaforr::domain::ModelDependency::Highways] = 3U;
  spatial.revisions[semaforr::domain::ModelDependency::HighwayGraph] = 3U;
  semaforr::planning::HighwayPlan planner;
  const auto result = planner.plan(
      {{{-4.0, 0.0}, semaforr::domain::Angle::zero()},
       {0.0, 4.0}, &spatial});
  ASSERT_TRUE(result.succeeded());
  ASSERT_TRUE(result.hierarchical);
  EXPECT_TRUE(std::any_of(
      result.hierarchical->steps.begin(), result.hierarchical->steps.end(),
      [](const auto& step) {
        const auto* intersection =
            std::get_if<semaforr::planning::IntersectionStep>(&step);
        return intersection != nullptr &&
               intersection->intersection_id == 1U;
      }));
}

TEST(EnvironmentRegression, LargeRoomIsAnExplicitExplorationCue) {
  auto view = observation(0.0, 0.0);
  view.laser.angle_min =
      semaforr::domain::Angle(-1.5707963267948966);
  view.laser.angle_increment = semaforr::domain::Angle(
      3.1415926535897932 / 359.0);
  view.laser.ranges_m.assign(360U, 8.0);
  semaforr::exploration::HighLevelExplorationConfiguration configuration;
  configuration.behavior_policy =
      semaforr::exploration::HleBehaviorPolicy::Compatibility;
  configuration.large_room_width = semaforr::domain::Distance(3.0);
  const auto candidates =
      semaforr::exploration::HighLevelExplorer::discoverCandidates(
          view, configuration);
  ASSERT_FALSE(candidates.empty());
  EXPECT_TRUE(std::all_of(candidates.begin(), candidates.end(),
                          [](const auto& candidate) {
                            return candidate.kind ==
                                   semaforr::exploration::PassageKind::LargeRoom;
                          }));
}

TEST(EnvironmentRegression,
     DynamicObstacleBlocksRouteWithoutChangingStaticOccupancy) {
  semaforr::domain::StaticMap map;
  map.bounds = {{0.0, 0.0}, {3.0, 1.0}};
  map.walls = {{{0.0, 0.0}, {3.0, 0.0}}};
  map.occupancy =
      {3U, 1U, 1.0, {0.0, 0.0},
       std::vector<semaforr::domain::StaticOccupancyState>(
           3U, semaforr::domain::StaticOccupancyState::StaticFree)};
  semaforr::domain::SensedOccupancyGrid sensed;
  sensed.geometry = {3U, 1U, 1.0, {0.0, 0.0}};
  sensed.cells.resize(3U);
  for (auto& cell : sensed.cells)
    cell.state = semaforr::domain::SensedOccupancyState::ObservedFree;
  sensed.cells[1].state =
      semaforr::domain::SensedOccupancyState::ObservedOccupied;
  sensed.cells[1].dynamic = true;
  sensed.cells[1].source =
      semaforr::domain::OccupancyEvidenceSource::DynamicObstacle;
  semaforr::planning::TraversabilityConfiguration policy;
  policy.robot_radius_m = policy.safety_clearance_m =
      policy.localization_uncertainty_m = policy.dynamic_obstacle_margin_m = 0.0;
  const auto blocked = semaforr::planning::deriveTraversability(
      semaforr::planning::OccupancySourceMode::StaticMapWithSensors,
      &map, &sensed, policy);
  EXPECT_EQ(blocked.grid.cells[1].state,
            semaforr::domain::TraversabilityState::NonTraversable);
  EXPECT_EQ(map.occupancy.cells[1],
            semaforr::domain::StaticOccupancyState::StaticFree);
}

TEST(EnvironmentRegression, FailedMovementRemainsNonTraversalEvidence) {
  auto failed = traversal(7U, 0.0, 0.0, 0.2, 0.0);
  failed.execution.status =
      semaforr::domain::ExecutionCompletionStatus::ControllerFailure;
  EXPECT_FALSE(failed.successfulTraversal());
  EXPECT_TRUE(failed.execution.moved());
  const auto completed = path(7U, std::move(failed));
  const auto trail = semaforr::spatial::learnVisibilityTrail(completed, 7U);
  EXPECT_TRUE(trail.markers.empty());
}

TEST(EnvironmentRegression, NegativeCoordinateMapRoutesWithinBounds) {
  semaforr::domain::StaticMap map;
  map.bounds = {{-3.0, -2.0}, {3.0, 2.0}};
  map.walls = {{{0.0, -1.0}, {0.0, 1.0}}};
  map.occupancy =
      {6U, 4U, 1.0, {-3.0, -2.0},
       std::vector<semaforr::domain::StaticOccupancyState>(
           24U, semaforr::domain::StaticOccupancyState::StaticFree)};
  map.occupancy.cells[9U] =
      semaforr::domain::StaticOccupancyState::StaticOccupied;
  map.occupancy.cells[15U] =
      semaforr::domain::StaticOccupancyState::StaticOccupied;
  semaforr::planning::DomainPlanner planner(
      "negative_distance", semaforr::planning::PlanObjective::Distance);
  const auto result = planner.plan(
      {{{-2.5, -0.5}, semaforr::domain::Angle::zero()},
       {2.5, -0.5}, nullptr, nullptr, &map});
  ASSERT_TRUE(result.succeeded());
  for (const auto& point : result.path) {
    EXPECT_TRUE(map.bounds.contains(point));
    const auto index = map.occupancy.geometry.index(point);
    ASSERT_TRUE(index);
    EXPECT_NE(map.occupancy.cells[*index],
              semaforr::domain::StaticOccupancyState::StaticOccupied);
  }
}

}  // namespace
