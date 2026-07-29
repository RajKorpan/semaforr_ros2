#include <semaforr/planning/graph.hpp>

#include <cmath>

namespace semaforr::planning {

VertexId Graph::addVertex(domain::Point2D position)
{
  positions_.push_back(position);
  adjacency_.emplace_back();
  return positions_.size() - 1U;
}

void Graph::validateVertex(VertexId vertex) const
{
  if (vertex >= positions_.size()) {
    throw std::out_of_range("graph vertex is out of range");
  }
}

void Graph::validateCost(const CostComponents& cost)
{
  if (!std::isfinite(cost.distance_m) || cost.distance_m < 0.0 ||
      !std::isfinite(cost.crowd_penalty) || cost.crowd_penalty < 0.0 ||
      !std::isfinite(cost.risk_penalty) || cost.risk_penalty < 0.0) {
    throw std::invalid_argument(
      "graph cost components must be finite and nonnegative");
  }
}

void Graph::addDirectedEdge(
  VertexId source,
  VertexId target,
  CostComponents cost)
{
  validateVertex(source);
  validateVertex(target);
  validateCost(cost);
  const double straight_line =
    domain::distance(positions_[source], positions_[target]).meters();
  if (cost.distance_m + domain::geometry_tolerance_m < straight_line) {
    throw std::invalid_argument(
      "edge distance cost cannot be shorter than its geometry");
  }
  adjacency_[source].push_back({target, cost});
}

void Graph::addUndirectedEdge(
  VertexId first,
  VertexId second,
  CostComponents cost)
{
  addDirectedEdge(first, second, cost);
  addDirectedEdge(second, first, cost);
}

const domain::Point2D& Graph::position(VertexId vertex) const
{
  validateVertex(vertex);
  return positions_[vertex];
}

const std::vector<GraphEdge>& Graph::edges(VertexId vertex) const
{
  validateVertex(vertex);
  return adjacency_[vertex];
}

}  // namespace semaforr::planning
