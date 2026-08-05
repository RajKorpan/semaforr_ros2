#ifndef SEMAFORR_DOMAIN_GRID_GEOMETRY_HPP
#define SEMAFORR_DOMAIN_GRID_GEOMETRY_HPP

#include <cstddef>
#include <optional>
#include <semaforr/domain/geometry.hpp>
#include <string>

namespace semaforr::domain {

enum class CellBoundaryConvention { HalfOpen };
enum class GridOutOfBoundsBehavior { Reject, ExpandBeforeInsert, NonTraversable };
enum class GridExtentMode { Fixed, Expandable };
enum class GridExtentSource {
  StaticMapBounds,
  InferredMapBounds,
  ConfiguredMaplessInitialBounds,
  SensorDerivedExpansion,
  RepresentationLocalBounds
};

// Shared geometry only: the contents and semantics remain owned by each layer.
struct GridGeometry {
  std::string frame_id{"map"};
  Point2D minimum;
  Point2D maximum;
  double resolution_m{1.0};
  std::size_t columns{0U};
  std::size_t rows{0U};
  Point2D origin;
  CellBoundaryConvention boundary_convention =
      CellBoundaryConvention::HalfOpen;
  GridOutOfBoundsBehavior out_of_bounds =
      GridOutOfBoundsBehavior::NonTraversable;
  GridExtentMode extent_mode = GridExtentMode::Fixed;
  std::size_t geometry_revision{0U};
  GridExtentSource extent_source = GridExtentSource::RepresentationLocalBounds;
  std::string map_identifier;

  GridGeometry() = default;
  GridGeometry(std::size_t column_count, std::size_t row_count,
               double resolution, Point2D grid_origin,
               GridExtentMode mode = GridExtentMode::Expandable,
               GridExtentSource source =
                   GridExtentSource::ConfiguredMaplessInitialBounds);
  GridGeometry(std::string frame, double width_m, double height_m,
               double resolution, double origin_x_m, double origin_y_m);

  static GridGeometry fromBounds(
      std::string frame, Point2D minimum, Point2D maximum, double resolution,
      GridExtentMode mode, GridExtentSource source,
      GridOutOfBoundsBehavior out_of_bounds_behavior,
      std::size_t revision = 1U, std::string map_id = {});

  bool valid() const noexcept;
  void validate() const;
  double widthMeters() const noexcept;
  double heightMeters() const noexcept;
  std::size_t cellCount() const;
  bool contains(Point2D point) const noexcept;
  std::optional<std::size_t> index(Point2D point) const noexcept;
  std::optional<std::pair<std::size_t, std::size_t>> cell(
      Point2D point) const noexcept;
  Point2D center(std::size_t index) const;
  Point2D center(std::size_t column, std::size_t row) const;

  bool operator==(const GridGeometry&) const = default;
};

struct GridExpansionPolicy {
  double margin_m{0.0};
  std::size_t increment_cells{32U};
  double maximum_width_m{0.0};
  double maximum_height_m{0.0};
  std::size_t memory_limit_cells{10'000'000U};
};

struct GridExpansionResult {
  GridGeometry geometry;
  bool expanded{false};
  bool resource_limited{false};
  std::string diagnostic;
};

GridExpansionResult expandToInclude(const GridGeometry& geometry,
                                    Point2D point,
                                    const GridExpansionPolicy& policy);

std::string serializeGridGeometry(const GridGeometry& geometry);
GridGeometry deserializeGridGeometry(const std::string& serialized);
const char* toString(GridExtentSource source) noexcept;

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_GRID_GEOMETRY_HPP
