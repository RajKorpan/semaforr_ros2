/**
 * @file domain_astar.cpp
 * @brief Domain astar responsibilities.
 *
 * @details This file implements domain astar behavior for path planning and
 * hierarchical plan construction. It centers on `QueueEntry`,
 * `LowestEstimateFirst`. Its package-relative location is
 * `src/planning/domain_astar.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <semaforr/planning/astar.hpp>
#include <utility>

namespace semaforr::planning {
namespace {

/**
 * @brief Encapsulates queue entry state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct QueueEntry {
  double estimate;
  VertexId vertex;
};

/**
 * @brief Encapsulates lowest estimate first state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct LowestEstimateFirst {
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator()(const QueueEntry& left, const QueueEntry& right) const {
    if (left.estimate == right.estimate) {
      return left.vertex > right.vertex;
    }
    return left.estimate > right.estimate;
  }
};

/**
 * @brief Performs the heuristic operation for this subsystem.
 *
 * Arguments:
 * - @p graph: Supplies graph input to the operation.
 * - @p from: Supplies from input to the operation.
 * - @p goal: Supplies goal input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double heuristic(const Graph& graph, VertexId from, VertexId goal) {
  return domain::distance(graph.position(from), graph.position(goal)).meters();
}

}  // namespace

/**
 * @brief Performs the search operation for this subsystem.
 *
 * Arguments:
 * - @p graph: Supplies graph input to the operation.
 * - @p start: Supplies start input to the operation.
 * - @p goal: Supplies goal input to the operation.
 *
 * Returns:
 * - `PathResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PathResult AStar::search(const Graph& graph, VertexId start,
                         VertexId goal) const {
  if (start >= graph.size() || goal >= graph.size()) {
    return {PathStatus::InvalidVertex,
            {},
            0.0,
            "start or goal vertex is outside the graph"};
  }
  if (start == goal) {
    return {PathStatus::Success, {start}, 0.0, {}};
  }

  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> cost(graph.size(), infinity);
  std::vector<std::optional<VertexId>> predecessor(graph.size());
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, LowestEstimateFirst>
      frontier;
  cost[start] = 0.0;
  frontier.push({heuristic(graph, start, goal), start});

  while (!frontier.empty()) {
    const QueueEntry current = frontier.top();
    frontier.pop();
    if (current.vertex == goal) {
      break;
    }
    if (current.estimate >
        cost[current.vertex] + heuristic(graph, current.vertex, goal)) {
      continue;
    }
    for (const auto& edge : graph.edges(current.vertex)) {
      const double next_cost = cost[current.vertex] + edge.cost.total();
      if (next_cost < cost[edge.target]) {
        cost[edge.target] = next_cost;
        predecessor[edge.target] = current.vertex;
        frontier.push(
            {next_cost + heuristic(graph, edge.target, goal), edge.target});
      }
    }
  }

  if (!std::isfinite(cost[goal])) {
    return {PathStatus::Unreachable,
            {},
            0.0,
            "goal is disconnected from the start"};
  }

  std::vector<VertexId> path;
  for (VertexId vertex = goal;; vertex = *predecessor[vertex]) {
    path.push_back(vertex);
    if (vertex == start) {
      break;
    }
  }
  std::reverse(path.begin(), path.end());
  return {PathStatus::Success, std::move(path), cost[goal], {}};
}

}  // namespace semaforr::planning
