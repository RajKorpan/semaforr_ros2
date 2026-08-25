/**
 * @file highway.hpp
 * @brief Highway responsibilities.
 *
 * @details This file defines highway behavior for ROS-independent domain state and
 * value types. It centers on `Axis`, `GridCell`, `Intersection`,
 * `HighwayEdge`, `Graph`, `Highway`, `HighwayIntersection`,
 * `HighwayGraph`. Its package-relative location is
 * `include/semaforr/domain/highway.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_HIGHWAY_HPP
#define SEMAFORR_DOMAIN_HIGHWAY_HPP

#include <cstddef>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/grid_geometry.hpp>
#include <string>
#include <utility>
#include <vector>

namespace semaforr::domain {

using HighwayId = std::size_t;
using IntersectionId = std::size_t;
using ModelRevision = std::size_t;
using TrailId = std::size_t;

/**
 * @brief Enumerates the supported axis values used by this subsystem.
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
enum class Axis { Horizontal, Vertical };

/**
 * @brief Encapsulates grid cell state and behavior for this subsystem.
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
struct GridCell {
  int row = 0;
  int column = 0;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const GridCell&) const = default;
};

/**
 * @brief Encapsulates intersection state and behavior for this subsystem.
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
struct Intersection {
  IntersectionId id = 0U;
  GridCell cell;
  Point2D position;
  bool terminal_access = false;
};

/**
 * @brief Encapsulates highway edge state and behavior for this subsystem.
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
struct HighwayEdge {
  IntersectionId from = 0U;
  IntersectionId to = 0U;
  HighwayId highway = 0U;
  double length_m = 0.0;
  std::vector<TrailId> trail_labels;
  // Execution-confirmed exploration poses from `from` to `to`.
  std::vector<Point2D> operational_subtrail;
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
template <typename Vertex, typename Edge>
struct Graph {
  std::vector<Vertex> vertices;
  std::vector<Edge> edges;
};

/**
 * @brief Encapsulates highway state and behavior for this subsystem.
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
struct Highway {
  HighwayId id = 0U;
  Axis axis = Axis::Horizontal;
  std::vector<GridCell> cells;
  std::vector<IntersectionId> endpoints;
};

/**
 * @brief Encapsulates highway intersection state and behavior for this
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
struct HighwayIntersection {
  std::size_t node = 0U;
  std::size_t degree = 0U;
};

/**
 * @brief Encapsulates highway graph state and behavior for this subsystem.
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
struct HighwayGraph {
  static constexpr std::size_t schema_version = 2U;
  Graph<Intersection, HighwayEdge> graph;
  std::vector<Highway> highways;
  ModelRevision revision = 0U;
  std::size_t serialized_schema_version = schema_version;
  std::vector<Point2D> nodes;
  std::vector<std::pair<std::size_t, std::size_t>> edges;
  std::vector<HighwayIntersection> intersections;
  GridGeometry geometry;
  std::string smoothing_policy{"von_neumann_three_of_four"};
  std::string component_selection_policy{"most_intersections"};
};

}  // namespace semaforr::domain

#endif
