#ifndef SEMAFORR_PLANNING_GRAPH_HPP
#define SEMAFORR_PLANNING_GRAPH_HPP

#include <cstddef>
#include <semaforr/domain/geometry.hpp>
#include <stdexcept>
#include <vector>

namespace semaforr::planning {

using VertexId = std::size_t;

struct CostComponents {
  double distance_m{0.0};
  double crowd_penalty{0.0};
  double risk_penalty{0.0};

  double total() const noexcept {
    return distance_m + crowd_penalty + risk_penalty;
  }
};

struct GraphEdge {
  VertexId target;
  CostComponents cost;
};

class Graph {
 public:
  VertexId addVertex(domain::Point2D position);
  void addDirectedEdge(VertexId source, VertexId target, CostComponents cost);
  void addUndirectedEdge(VertexId first, VertexId second, CostComponents cost);

  const domain::Point2D& position(VertexId vertex) const;
  const std::vector<GraphEdge>& edges(VertexId vertex) const;
  std::size_t size() const noexcept { return positions_.size(); }

 private:
  void validateVertex(VertexId vertex) const;
  static void validateCost(const CostComponents& cost);

  std::vector<domain::Point2D> positions_;
  std::vector<std::vector<GraphEdge>> adjacency_;
};

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_GRAPH_HPP
