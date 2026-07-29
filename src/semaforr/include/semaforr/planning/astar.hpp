#ifndef SEMAFORR_PLANNING_ASTAR_HPP
#define SEMAFORR_PLANNING_ASTAR_HPP

#include <string>
#include <vector>

#include <semaforr/planning/graph.hpp>

namespace semaforr::planning {

enum class PathStatus {
  Success,
  Unreachable,
  InvalidVertex
};

struct PathResult {
  PathStatus status{PathStatus::Unreachable};
  std::vector<VertexId> vertices;
  double cost{0.0};
  std::string explanation;

  bool succeeded() const noexcept { return status == PathStatus::Success; }
};

class AStar {
public:
  PathResult search(
    const Graph& graph,
    VertexId start,
    VertexId goal) const;
};

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_ASTAR_HPP
