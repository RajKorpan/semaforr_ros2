/**
 * @file graph.hpp
 * @brief Graph responsibilities.
 *
 * @details This file defines graph behavior for path planning and hierarchical plan
 * construction. It centers on `CostComponents`, `GraphEdge`, `Graph`. Its
 * package-relative location is `include/semaforr/planning/graph.hpp`.
 */
#ifndef SEMAFORR_PLANNING_GRAPH_HPP
#define SEMAFORR_PLANNING_GRAPH_HPP

#include <cstddef>
#include <semaforr/domain/geometry.hpp>
#include <stdexcept>
#include <vector>

namespace semaforr::planning {

using VertexId = std::size_t;

/**
 * @brief Encapsulates cost components state and behavior for this
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
struct CostComponents {
  double distance_m{0.0};
  double crowd_penalty{0.0};
  double risk_penalty{0.0};

  /**
   * @brief Converts tal for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double total() const noexcept {
    return distance_m + crowd_penalty + risk_penalty;
  }
};

/**
 * @brief Encapsulates graph edge state and behavior for this subsystem.
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
struct GraphEdge {
  VertexId target;
  CostComponents cost;
};

/**
 * @brief Encapsulates graph state and behavior for this subsystem.
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
class Graph {
 public:
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
  VertexId addVertex(domain::Point2D position);
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
  void addDirectedEdge(VertexId source, VertexId target, CostComponents cost);
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
  void addUndirectedEdge(VertexId first, VertexId second, CostComponents cost);

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
  const domain::Point2D& position(VertexId vertex) const;
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
  const std::vector<GraphEdge>& edges(VertexId vertex) const;
  /**
   * @brief Performs the size operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t size() const noexcept { return positions_.size(); }

 private:
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
  void validateVertex(VertexId vertex) const;
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
  static void validateCost(const CostComponents& cost);

  std::vector<domain::Point2D> positions_;
  std::vector<std::vector<GraphEdge>> adjacency_;
};

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_GRAPH_HPP
