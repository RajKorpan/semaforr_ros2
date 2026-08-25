/**
 * @file domain_planning_test.cpp
 * @brief Domain planning test responsibilities.
 *
 * @details This file exercises domain planning test behavior for automated
 * verification and regression testing. It centers on
 * `RepeatedSearchDoesNotMutateGraphState`,
 * `IdenticalStartAndGoalReturnsSingleVertex`,
 * `DisconnectedGoalIsTypedAsUnreachable`,
 * `InvalidVertexIsReportedWithoutThrowing`,
 * `RequiresStaticMapAndReturnsTypedResults`,
 * `SearchesValidatedStaticOccupancy`,
 * `AffordancePlannerNeverSubstitutesTheLearnedSkeleton`. Its
 * package-relative location is `test/unit/domain_planning_test.cpp`.
 */
#include <gtest/gtest.h>

#include <limits>
#include <semaforr/planning/astar.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <stdexcept>

namespace {

/**
 * @brief Performs the line graph operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `semaforr::planning::Graph` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::planning::Graph lineGraph() {
  semaforr::planning::Graph graph;
  const auto first = graph.addVertex({0.0, 0.0});
  const auto second = graph.addVertex({1.0, 0.0});
  const auto third = graph.addVertex({2.0, 0.0});
  graph.addUndirectedEdge(first, second, {1.0, 0.0, 0.0});
  graph.addUndirectedEdge(second, third, {1.0, 0.0, 0.0});
  return graph;
}

/**
 * @brief Performs the open map operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `semaforr::domain::StaticMap` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::StaticMap openMap() {
  semaforr::domain::StaticMap map;
  map.source = "unit-test";
  map.format = "test";
  map.bounds = {{0.0, 0.0}, {3.0, 2.0}};
  map.walls = {{{0.0, 0.0}, {3.0, 0.0}}};
  map.occupancy = {
      3U, 2U, 1.0, {0.0, 0.0},
      std::vector<semaforr::domain::StaticOccupancyState>(
          6U, semaforr::domain::StaticOccupancyState::StaticFree)};
  return map;
}

TEST(DomainAStar, RepeatedSearchDoesNotMutateGraphState) {
  const auto graph = lineGraph();
  semaforr::planning::AStar search;
  const auto first = search.search(graph, 0U, 2U);
  const auto second = search.search(graph, 0U, 2U);
  EXPECT_TRUE(first.succeeded());
  EXPECT_EQ(first.vertices,
            (std::vector<semaforr::planning::VertexId>{0U, 1U, 2U}));
  EXPECT_EQ(first.vertices, second.vertices);
  EXPECT_DOUBLE_EQ(first.cost, 2.0);
}

TEST(DomainAStar, IdenticalStartAndGoalReturnsSingleVertex) {
  const auto graph = lineGraph();
  const auto result = semaforr::planning::AStar().search(graph, 1U, 1U);
  EXPECT_EQ(result.status, semaforr::planning::PathStatus::Success);
  EXPECT_EQ(result.vertices, (std::vector<semaforr::planning::VertexId>{1U}));
  EXPECT_DOUBLE_EQ(result.cost, 0.0);
}

TEST(DomainAStar, DisconnectedGoalIsTypedAsUnreachable) {
  auto graph = lineGraph();
  const auto disconnected = graph.addVertex({10.0, 10.0});
  const auto result =
      semaforr::planning::AStar().search(graph, 0U, disconnected);
  EXPECT_EQ(result.status, semaforr::planning::PathStatus::Unreachable);
  EXPECT_TRUE(result.vertices.empty());
}

TEST(DomainAStar, InvalidVertexIsReportedWithoutThrowing) {
  const auto graph = lineGraph();
  const auto result = semaforr::planning::AStar().search(graph, 0U, 99U);
  EXPECT_EQ(result.status, semaforr::planning::PathStatus::InvalidVertex);
}

TEST(DomainPlanner, RequiresStaticMapAndReturnsTypedResults) {
  semaforr::planning::DomainPlanner direct(
      "distance", semaforr::planning::PlannerObjective::Distance);
  const auto unavailable =
      direct.plan({{{0.5, 0.5}, semaforr::domain::Angle::zero()}, {2.5, 0.5}});
  EXPECT_EQ(unavailable.status,
            semaforr::planning::PlanStatus::PlannerUnavailable);
  const auto map = openMap();
  const auto direct_result = direct.plan(
      {{{0.5, 0.5}, semaforr::domain::Angle::zero()}, {2.5, 0.5}, nullptr,
       nullptr, &map});
  ASSERT_TRUE(direct_result.succeeded());
  EXPECT_DOUBLE_EQ(direct_result.cost_m, 2.0);

  const auto invalid =
      direct.plan({{{std::numeric_limits<double>::quiet_NaN(), 0.0},
                    semaforr::domain::Angle::zero()},
                   {2.0, 0.0}, nullptr, nullptr, &map});
  EXPECT_EQ(invalid.status, semaforr::planning::PlanStatus::InvalidRequest);
  EXPECT_THROW(semaforr::planning::DomainPlanner(
                   "", semaforr::planning::PlannerObjective::Distance),
               std::invalid_argument);
}

TEST(DomainPlanner, SearchesValidatedStaticOccupancy) {
  const auto map = openMap();
  semaforr::planning::DomainPlanner planner(
      "distance", semaforr::planning::PlannerObjective::Distance);
  const auto result = planner.plan(
      {{{0.5, 0.5}, semaforr::domain::Angle::zero()}, {2.5, 0.5}, nullptr,
       nullptr, &map});
  ASSERT_TRUE(result.succeeded());
  EXPECT_DOUBLE_EQ(result.cost_m, 2.0);
  ASSERT_FALSE(result.path.empty());
  EXPECT_EQ(result.path.back(), (semaforr::domain::Point2D{2.5, 0.5}));

  auto invalid_map = map;
  invalid_map.occupancy.cells.clear();
  const auto invalid = planner.plan({
      {{0.5, 0.5}, semaforr::domain::Angle::zero()}, {2.5, 0.5}, nullptr,
      nullptr, &invalid_map});
  EXPECT_EQ(invalid.status,
            semaforr::planning::PlanStatus::PlannerUnavailable);
}

TEST(DomainPlanner, AffordancePlannerNeverSubstitutesTheLearnedSkeleton) {
  semaforr::domain::SpatialModel spatial;
  spatial.skeleton_nodes = {{0.5, 0.5}, {1.5, 0.5}, {2.5, 0.5}};
  spatial.skeleton_edges = {{0U, 1U}, {1U, 2U}};
  spatial.learned_regions = {
      {{1.5, 0.5}, semaforr::domain::Distance(1.0)}};
  semaforr::planning::DomainPlanner planner(
      "region", semaforr::planning::PlanObjective::RegionPreference,
      semaforr::planning::OccupancySourceMode::
          StaticOrSensorDerived);
  const auto unavailable = planner.plan(
      {{{0.5, 0.5}, semaforr::domain::Angle::zero()}, {2.5, 0.5}, &spatial});
  EXPECT_EQ(unavailable.status,
            semaforr::planning::PlanStatus::PlannerUnavailable);

  spatial.sensed_occupancy.geometry = {3U, 1U, 1.0, {0.0, 0.0}};
  spatial.sensed_occupancy.cells.resize(3U);
  for (auto& cell : spatial.sensed_occupancy.cells)
    cell.state = semaforr::domain::SensedOccupancyState::ObservedFree;
  const auto result = planner.plan(
      {{{0.5, 0.5}, semaforr::domain::Angle::zero()}, {2.5, 0.5}, &spatial});
  ASSERT_TRUE(result.succeeded());
  EXPECT_NE(result.explanation.find("sensor-derived"), std::string::npos);
}

}  // namespace
