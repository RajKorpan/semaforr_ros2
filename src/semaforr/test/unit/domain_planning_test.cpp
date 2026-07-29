#include <gtest/gtest.h>

#include <semaforr/planning/astar.hpp>

namespace {

semaforr::planning::Graph lineGraph()
{
  semaforr::planning::Graph graph;
  const auto first = graph.addVertex({0.0, 0.0});
  const auto second = graph.addVertex({1.0, 0.0});
  const auto third = graph.addVertex({2.0, 0.0});
  graph.addUndirectedEdge(first, second, {1.0, 0.0, 0.0});
  graph.addUndirectedEdge(second, third, {1.0, 0.0, 0.0});
  return graph;
}

TEST(DomainAStar, RepeatedSearchDoesNotMutateGraphState)
{
  const auto graph = lineGraph();
  semaforr::planning::AStar search;
  const auto first = search.search(graph, 0U, 2U);
  const auto second = search.search(graph, 0U, 2U);
  EXPECT_TRUE(first.succeeded());
  EXPECT_EQ(first.vertices, (std::vector<semaforr::planning::VertexId>{0U, 1U, 2U}));
  EXPECT_EQ(first.vertices, second.vertices);
  EXPECT_DOUBLE_EQ(first.cost, 2.0);
}

TEST(DomainAStar, IdenticalStartAndGoalReturnsSingleVertex)
{
  const auto graph = lineGraph();
  const auto result = semaforr::planning::AStar().search(graph, 1U, 1U);
  EXPECT_EQ(result.status, semaforr::planning::PathStatus::Success);
  EXPECT_EQ(result.vertices, (std::vector<semaforr::planning::VertexId>{1U}));
  EXPECT_DOUBLE_EQ(result.cost, 0.0);
}

TEST(DomainAStar, DisconnectedGoalIsTypedAsUnreachable)
{
  auto graph = lineGraph();
  const auto disconnected = graph.addVertex({10.0, 10.0});
  const auto result = semaforr::planning::AStar().search(graph, 0U, disconnected);
  EXPECT_EQ(result.status, semaforr::planning::PathStatus::Unreachable);
  EXPECT_TRUE(result.vertices.empty());
}

TEST(DomainAStar, InvalidVertexIsReportedWithoutThrowing)
{
  const auto graph = lineGraph();
  const auto result = semaforr::planning::AStar().search(graph, 0U, 99U);
  EXPECT_EQ(result.status, semaforr::planning::PathStatus::InvalidVertex);
}

}  // namespace
