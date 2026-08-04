#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <semaforr/planning/astar.hpp>
#include <utility>

namespace semaforr::planning {
namespace {

struct QueueEntry {
  double estimate;
  VertexId vertex;
};

struct LowestEstimateFirst {
  bool operator()(const QueueEntry& left, const QueueEntry& right) const {
    if (left.estimate == right.estimate) {
      return left.vertex > right.vertex;
    }
    return left.estimate > right.estimate;
  }
};

double heuristic(const Graph& graph, VertexId from, VertexId goal) {
  return domain::distance(graph.position(from), graph.position(goal)).meters();
}

}  // namespace

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
