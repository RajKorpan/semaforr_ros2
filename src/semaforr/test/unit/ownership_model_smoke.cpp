#include <semaforr/decision/AgentState.hpp>
#include <semaforr/decision/Controller.hpp>
#include <semaforr/navigation/Graph.hpp>
#include <semaforr/navigation/PathPlanner.hpp>
#include <semaforr/navigation/astar.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>

#ifndef SEMAFORR_TEST_SOURCE_DIR
#error "SEMAFORR_TEST_SOURCE_DIR must name the package source directory"
#endif

int main()
{
  static_assert(!std::is_copy_constructible<AgentState>::value,
                "AgentState uniquely owns its tasks");
  static_assert(!std::is_copy_constructible<Controller>::value,
                "Controller uniquely owns its components");
  static_assert(!std::is_copy_constructible<Graph>::value,
                "Graph uniquely owns its nodes and edges");
  static_assert(!std::is_copy_constructible<PathPlanner>::value,
                "PathPlanner can uniquely own its graphs");

  Graph graph(100, 500, 500);
  const auto require = [](bool condition, const char* message) {
    if (!condition) {
      std::cerr << message << '\n';
      std::exit(1);
    }
  };

  require(graph.addNode(100, 100, 0.0, 0), "failed to add first node");
  require(graph.addNode(200, 100, 0.0, 1), "failed to add second node");
  graph.addEdge(
    0,
    1,
    100.0,
    {CartesianPoint(1.0, 1.0), CartesianPoint(2.0, 1.0)});

  require(graph.numNodes() == 2, "wrong node count");
  require(graph.numEdges() == 1, "wrong edge count");
  require(graph.getEdge(0, 1)->getFrom() == 0, "edge lookup failed");
  require(
    graph.getEdge(0, 0)->getFrom() == Node::invalid_node_index,
    "missing edge lookup was not safe");

  astar search(&graph);
  require(search.search(0, 1, "ownership-smoke"), "A* search failed");
  require(search.isPathFound(), "A* did not record a path");

  double moves[] = {0.0, 0.5, 1.0};
  double rotations[] = {0.0, 0.25, 0.5};
  AgentState state(moves, rotations, 3, 3);
  state.addTask(3.0F, 4.0F, 10, 10);
  state.addTask(5.0F, 6.0F, 10, 10);
  require(state.getAgenda().size() == 2U, "agenda ownership failed");
  require(state.getAllAgenda().size() == 2U, "all-agenda ownership failed");
  require(state.getNextTask()->getTaskX() == 3.0, "task order changed");

  const std::string source_dir = SEMAFORR_TEST_SOURCE_DIR;
  const std::string config_dir = source_dir + "/config";
  const std::string tutorial_dir = config_dir + "/stage_tutorial";
  {
    Controller controller(
      config_dir + "/advisors.conf",
      config_dir + "/params.conf",
      tutorial_dir + "/stage_tutorialS.xml",
      tutorial_dir + "/target.conf",
      tutorial_dir + "/dimensions.conf");
    require(controller.getBeliefs() != nullptr, "beliefs were not constructed");
    require(
      controller.getBeliefs()->getAgentState() != nullptr,
      "agent state was not constructed");
    require(
      !controller.getBeliefs()->getAgentState()->getAgenda().empty(),
      "configured mission was not loaded");
  }

  return 0;
}
