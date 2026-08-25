/**
 * @file domain_graph.cpp
 * @brief Domain graph responsibilities.
 *
 * @details This file implements domain graph behavior for path planning and
 * hierarchical plan construction. It records the declarations, settings,
 * fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/planning/domain_graph.cpp`.
 */
#include <cmath>
#include <semaforr/planning/graph.hpp>

namespace semaforr::planning {

/**
 * @brief Performs the add vertex operation for this subsystem.
 *
 * Arguments:
 * - @p position: Supplies position input to the operation.
 *
 * Returns:
 * - `VertexId` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
VertexId Graph::addVertex(domain::Point2D position) {
  positions_.push_back(position);
  adjacency_.emplace_back();
  return positions_.size() - 1U;
}

/**
 * @brief Validates vertex for this subsystem.
 *
 * Arguments:
 * - @p vertex: Supplies vertex input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void Graph::validateVertex(VertexId vertex) const {
  if (vertex >= positions_.size()) {
    throw std::out_of_range("graph vertex is out of range");
  }
}

/**
 * @brief Validates cost for this subsystem.
 *
 * Arguments:
 * - @p cost: Supplies cost input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void Graph::validateCost(const CostComponents& cost) {
  if (!std::isfinite(cost.distance_m) || cost.distance_m < 0.0 ||
      !std::isfinite(cost.crowd_penalty) || cost.crowd_penalty < 0.0 ||
      !std::isfinite(cost.risk_penalty) || cost.risk_penalty < 0.0) {
    throw std::invalid_argument(
        "graph cost components must be finite and nonnegative");
  }
}

/**
 * @brief Performs the add directed edge operation for this subsystem.
 *
 * Arguments:
 * - @p source: Supplies source input to the operation.
 * - @p target: Supplies target input to the operation.
 * - @p cost: Supplies cost input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void Graph::addDirectedEdge(VertexId source, VertexId target,
                            CostComponents cost) {
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

/**
 * @brief Performs the add undirected edge operation for this subsystem.
 *
 * Arguments:
 * - @p first: Supplies first input to the operation.
 * - @p second: Supplies second input to the operation.
 * - @p cost: Supplies cost input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void Graph::addUndirectedEdge(VertexId first, VertexId second,
                              CostComponents cost) {
  addDirectedEdge(first, second, cost);
  addDirectedEdge(second, first, cost);
}

/**
 * @brief Performs the position operation for this subsystem.
 *
 * Arguments:
 * - @p vertex: Supplies vertex input to the operation.
 *
 * Returns:
 * - `const domain::Point2D&` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const domain::Point2D& Graph::position(VertexId vertex) const {
  validateVertex(vertex);
  return positions_[vertex];
}

/**
 * @brief Performs the edges operation for this subsystem.
 *
 * Arguments:
 * - @p vertex: Supplies vertex input to the operation.
 *
 * Returns:
 * - `const std::vector<GraphEdge>&` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const std::vector<GraphEdge>& Graph::edges(VertexId vertex) const {
  validateVertex(vertex);
  return adjacency_[vertex];
}

}  // namespace semaforr::planning
