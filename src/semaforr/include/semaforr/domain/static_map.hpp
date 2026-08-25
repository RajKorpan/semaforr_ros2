/**
 * @file static_map.hpp
 * @brief Static map responsibilities.
 *
 * @details This file defines static map behavior for ROS-independent domain state
 * and value types. It centers on `GeometryProvenance`, `MapBounds`,
 * `StaticOccupancyGrid`, `StaticMap`, `MapCapabilities`. Its
 * package-relative location is `include/semaforr/domain/static_map.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_STATIC_MAP_HPP
#define SEMAFORR_DOMAIN_STATIC_MAP_HPP

#include <algorithm>
#include <cstdint>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/grid_layers.hpp>
#include <semaforr/domain/model_revision.hpp>
#include <string>
#include <vector>

namespace semaforr::domain {

/**
 * @brief Enumerates the supported geometry provenance values used by this
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
enum class GeometryProvenance { StaticMap, LiveSensor, LearnedModel };

/**
 * @brief Encapsulates map bounds state and behavior for this subsystem.
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
struct MapBounds {
  Point2D minimum;
  Point2D maximum;

  /**
   * @brief Performs the valid operation for this subsystem.
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
  bool valid() const noexcept {
    return minimum.finite() && maximum.finite() &&
           maximum.x_m > minimum.x_m && maximum.y_m > minimum.y_m;
  }
  /**
   * @brief Performs the contains operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool contains(Point2D point) const noexcept {
    return point.x_m >= minimum.x_m && point.x_m <= maximum.x_m &&
           point.y_m >= minimum.y_m && point.y_m <= maximum.y_m;
  }
};

/**
 * @brief Encapsulates static occupancy grid state and behavior for this
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
struct StaticOccupancyGrid {
  GridGeometry geometry;
  // Immutable prior occupancy. Inflation belongs to derived traversability.
  std::vector<StaticOccupancyState> cells;

  /**
   * @brief Performs the static occupancy grid operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  StaticOccupancyGrid() = default;
  /**
   * @brief Performs the static occupancy grid operation for this subsystem.
   *
   * Arguments:
   * - @p columns: Supplies columns input to the operation.
   * - @p rows: Supplies rows input to the operation.
   * - @p resolution_m: Supplies resolution m input to the operation.
   * - @p origin: Supplies origin input to the operation.
   * - @p occupancy_cells: Supplies occupancy cells input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  StaticOccupancyGrid(std::size_t columns, std::size_t rows,
                      double resolution_m, Point2D origin,
                      std::vector<StaticOccupancyState> occupancy_cells)
      : geometry(columns, rows, resolution_m, origin, GridExtentMode::Fixed,
                 GridExtentSource::StaticMapBounds),
        cells(std::move(occupancy_cells)) {}

  /**
   * @brief Performs the valid operation for this subsystem.
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
  bool valid() const noexcept {
    return geometry.valid() && cells.size() == geometry.cellCount();
  }
};

// Constructed once during startup and thereafter shared read-only.
/**
 * @brief Encapsulates static map state and behavior for this subsystem.
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
struct StaticMap {
  std::string source;
  std::string checksum;
  std::string format;
  MapBounds bounds;
  std::vector<Segment2D> walls;
  std::vector<Polygon> obstacle_polygons;
  StaticOccupancyGrid occupancy;
  GeometryProvenance provenance = GeometryProvenance::StaticMap;
  Revision geometry_revision = 1U;
  Revision occupancy_revision = 1U;
  // Compatibility identifier for the immutable loaded artifact.
  std::size_t revision = 1U;

  /**
   * @brief Performs the geometry available operation for this subsystem.
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
  bool geometryAvailable() const noexcept {
    return bounds.valid() && !walls.empty();
  }
  /**
   * @brief Performs the occupancy available operation for this subsystem.
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
  bool occupancyAvailable() const noexcept { return occupancy.valid(); }
  /**
   * @brief Performs the line of sight operation for this subsystem.
   *
   * Arguments:
   * - @p ray: Supplies ray input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool lineOfSight(const Segment2D& ray) const noexcept {
    if (!bounds.contains(ray.start) || !bounds.contains(ray.end)) return false;
    return std::none_of(walls.begin(), walls.end(), [&](const auto& wall) {
      return intersects(ray, wall);
    });
  }
};

/**
 * @brief Encapsulates map capabilities state and behavior for this
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
struct MapCapabilities {
  bool map_available = false;
  bool map_geometry_available = false;
  bool map_occupancy_available = false;
  bool map_based_planning_available = false;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_STATIC_MAP_HPP
