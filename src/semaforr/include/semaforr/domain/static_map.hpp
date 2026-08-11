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

enum class GeometryProvenance { StaticMap, LiveSensor, LearnedModel };

struct MapBounds {
  Point2D minimum;
  Point2D maximum;

  bool valid() const noexcept {
    return minimum.finite() && maximum.finite() &&
           maximum.x_m > minimum.x_m && maximum.y_m > minimum.y_m;
  }
  bool contains(Point2D point) const noexcept {
    return point.x_m >= minimum.x_m && point.x_m <= maximum.x_m &&
           point.y_m >= minimum.y_m && point.y_m <= maximum.y_m;
  }
};

struct StaticOccupancyGrid {
  GridGeometry geometry;
  // Immutable prior occupancy. Inflation belongs to derived traversability.
  std::vector<StaticOccupancyState> cells;

  StaticOccupancyGrid() = default;
  StaticOccupancyGrid(std::size_t columns, std::size_t rows,
                      double resolution_m, Point2D origin,
                      std::vector<StaticOccupancyState> occupancy_cells)
      : geometry(columns, rows, resolution_m, origin, GridExtentMode::Fixed,
                 GridExtentSource::StaticMapBounds),
        cells(std::move(occupancy_cells)) {}

  bool valid() const noexcept {
    return geometry.valid() && cells.size() == geometry.cellCount();
  }
};

// Constructed once during startup and thereafter shared read-only.
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

  bool geometryAvailable() const noexcept {
    return bounds.valid() && !walls.empty();
  }
  bool occupancyAvailable() const noexcept { return occupancy.valid(); }
  bool lineOfSight(const Segment2D& ray) const noexcept {
    if (!bounds.contains(ray.start) || !bounds.contains(ray.end)) return false;
    return std::none_of(walls.begin(), walls.end(), [&](const auto& wall) {
      return intersects(ray, wall);
    });
  }
};

struct MapCapabilities {
  bool map_available = false;
  bool map_geometry_available = false;
  bool map_occupancy_available = false;
  bool map_based_planning_available = false;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_STATIC_MAP_HPP
