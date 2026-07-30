#include <gtest/gtest.h>

#include <limits>
#include <semaforr/planning/astar.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <stdexcept>

namespace {

semaforr::planning::Graph lineGraph() {
  semaforr::planning::Graph graph;
  const auto first = graph.addVertex({0.0, 0.0});
  const auto second = graph.addVertex({1.0, 0.0});
  const auto third = graph.addVertex({2.0, 0.0});
  graph.addUndirectedEdge(first, second, {1.0, 0.0, 0.0});
  graph.addUndirectedEdge(second, third, {1.0, 0.0, 0.0});
  return graph;
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

TEST(DomainPlanner, ReturnsTypedDirectAndUnavailableResults) {
  semaforr::planning::DomainPlanner direct(
      "distance", semaforr::planning::PlannerObjective::Distance);
  const auto direct_result =
      direct.plan({{{0.0, 0.0}, semaforr::domain::Angle::zero()}, {2.0, 0.0}});
  ASSERT_TRUE(direct_result.succeeded());
  EXPECT_DOUBLE_EQ(direct_result.cost_m, 2.0);

  const auto invalid =
      direct.plan({{{std::numeric_limits<double>::quiet_NaN(), 0.0},
                    semaforr::domain::Angle::zero()},
                   {2.0, 0.0}});
  EXPECT_EQ(invalid.status, semaforr::planning::PlanStatus::InvalidRequest);

  semaforr::planning::DomainPlanner skeleton(
      "skeleton", semaforr::planning::PlannerObjective::SkeletonDistance);
  const auto unavailable = skeleton.plan(
      {{{0.0, 0.0}, semaforr::domain::Angle::zero()}, {2.0, 0.0}});
  EXPECT_EQ(unavailable.status,
            semaforr::planning::PlanStatus::PlannerUnavailable);
  EXPECT_THROW(semaforr::planning::DomainPlanner(
                   "", semaforr::planning::PlannerObjective::Distance),
               std::invalid_argument);
}

TEST(DomainPlanner, SearchesValidatedSpatialSkeleton) {
  semaforr::domain::SpatialModel spatial;
  spatial.skeleton_nodes = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  spatial.skeleton_edges = {{0U, 1U}, {1U, 2U}};
  semaforr::planning::DomainPlanner planner(
      "skeleton", semaforr::planning::PlannerObjective::SkeletonDistance);
  const auto result = planner.plan(
      {{{0.0, 0.0}, semaforr::domain::Angle::zero()}, {2.0, 0.0}, &spatial});
  ASSERT_TRUE(result.succeeded());
  EXPECT_DOUBLE_EQ(result.cost_m, 2.0);
  ASSERT_FALSE(result.path.empty());
  EXPECT_EQ(result.path.back(), (semaforr::domain::Point2D{2.0, 0.0}));

  spatial.skeleton_edges.push_back({2U, 99U});
  const auto invalid = planner.plan(
      {{{0.0, 0.0}, semaforr::domain::Angle::zero()}, {2.0, 0.0}, &spatial});
  EXPECT_EQ(invalid.status, semaforr::planning::PlanStatus::InvalidRequest);
}

}  // namespace
