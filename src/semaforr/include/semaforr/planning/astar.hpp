/**
 * @file astar.hpp
 * @brief Astar responsibilities.
 *
 * @details This file defines astar behavior for path planning and hierarchical
 * plan construction. It centers on `PathStatus`, `PathResult`, `AStar`. Its
 * package-relative location is `include/semaforr/planning/astar.hpp`.
 */
#ifndef SEMAFORR_PLANNING_ASTAR_HPP
#define SEMAFORR_PLANNING_ASTAR_HPP

#include <semaforr/planning/graph.hpp>
#include <string>
#include <vector>

namespace semaforr::planning {

/**
 * @brief Enumerates the supported path status values used by this
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
enum class PathStatus { Success, Unreachable, InvalidVertex };

/**
 * @brief Encapsulates path result state and behavior for this subsystem.
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
struct PathResult {
  PathStatus status{PathStatus::Unreachable};
  std::vector<VertexId> vertices;
  double cost{0.0};
  std::string explanation;

  /**
   * @brief Performs the succeeded operation for this subsystem.
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
  bool succeeded() const noexcept { return status == PathStatus::Success; }
};

/**
 * @brief Encapsulates astar state and behavior for this subsystem.
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
class AStar {
 public:
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
  PathResult search(const Graph& graph, VertexId start, VertexId goal) const;
};

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_ASTAR_HPP
