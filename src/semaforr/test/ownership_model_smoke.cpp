#include <semaforr/decision/AgentState.h>
#include <semaforr/decision/Controller.h>
#include <semaforr/navigation/Graph.h>
#include <semaforr/navigation/PathPlanner.h>
#include <semaforr/navigation/astar.h>

#include <cassert>
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
  assert(graph.addNode(100, 100, 0.0, 0));
  assert(graph.addNode(200, 100, 0.0, 1));
  graph.addEdge(
    0,
    1,
    100.0,
    {CartesianPoint(1.0, 1.0), CartesianPoint(2.0, 1.0)});

  assert(graph.numNodes() == 2);
  assert(graph.numEdges() == 1);
  assert(graph.getEdge(0, 1)->getFrom() == 0);
  assert(graph.getEdge(0, 0)->getFrom() == Node::invalid_node_index);

  astar search(&graph);
  assert(search.search(0, 1, "ownership-smoke"));
  assert(search.isPathFound());

  double moves[] = {0.0, 0.5, 1.0};
  double rotations[] = {0.0, 0.25, 0.5};
  AgentState state(moves, rotations, 3, 3);
  state.addTask(3.0F, 4.0F, 10, 10);
  state.addTask(5.0F, 6.0F, 10, 10);
  assert(state.getAgenda().size() == 2U);
  assert(state.getAllAgenda().size() == 2U);
  assert(state.getNextTask()->getTaskX() == 3.0);

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
    assert(controller.getBeliefs() != nullptr);
    assert(controller.getBeliefs()->getAgentState() != nullptr);
    assert(!controller.getBeliefs()->getAgentState()->getAgenda().empty());
  }

  return 0;
}
