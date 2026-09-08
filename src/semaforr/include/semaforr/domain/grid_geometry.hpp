/**
 * @file grid_geometry.hpp
 * @brief Grid geometry responsibilities.
 *
 * @details This file defines grid geometry behavior for ROS-independent domain
 * state and value types. It centers on `CellBoundaryConvention`,
 * `GridOutOfBoundsBehavior`, `GridExtentMode`, `GridExtentSource`,
 * `GridGeometry`, `GridExpansionPolicy`, `GridExpansionResult`. Its
 * package-relative location is
 * `include/semaforr/domain/grid_geometry.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_GRID_GEOMETRY_HPP
#define SEMAFORR_DOMAIN_GRID_GEOMETRY_HPP

#include <cstddef>
#include <optional>
#include <semaforr/domain/geometry.hpp>
#include <string>

namespace semaforr::domain {

/**
 * @brief Enumerates the supported cell boundary convention values used by
 * this subsystem.
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
enum class CellBoundaryConvention { HalfOpen };
/**
 * @brief Enumerates the supported grid out of bounds behavior values used
 * by this subsystem.
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
enum class GridOutOfBoundsBehavior {
  Reject,
  ExpandBeforeInsert,
  NonTraversable
};
/**
 * @brief Enumerates the supported grid extent mode values used by this
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
enum class GridExtentMode { Fixed, Expandable };
/**
 * @brief Enumerates the supported grid extent source values used by this
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
enum class GridExtentSource {
  StaticMapBounds,
  InferredMapBounds,
  ConfiguredMaplessInitialBounds,
  SensorDerivedExpansion,
  RepresentationLocalBounds
};

// Shared geometry only: the contents and semantics remain owned by each layer.
/**
 * @brief Encapsulates grid geometry state and behavior for this subsystem.
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
struct GridGeometry {
  std::string frame_id{"map"};
  Point2D minimum;
  Point2D maximum;
  double resolution_m{1.0};
  std::size_t columns{0U};
  std::size_t rows{0U};
  Point2D origin;
  CellBoundaryConvention boundary_convention = CellBoundaryConvention::HalfOpen;
  GridOutOfBoundsBehavior out_of_bounds =
      GridOutOfBoundsBehavior::NonTraversable;
  GridExtentMode extent_mode = GridExtentMode::Fixed;
  std::size_t geometry_revision{0U};
  GridExtentSource extent_source = GridExtentSource::RepresentationLocalBounds;
  std::string map_identifier;

  /**
   * @brief Performs the grid geometry operation for this subsystem.
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
  GridGeometry() = default;
  /**
   * @brief Performs the grid geometry operation for this subsystem.
   *
   * Arguments:
   * - @p column_count: Supplies column count input to the operation.
   * - @p row_count: Supplies row count input to the operation.
   * - @p resolution: Supplies resolution input to the operation.
   * - @p grid_origin: Supplies grid origin input to the operation.
   * - @p mode: Supplies mode input to the operation.
   * - @p source: Supplies source input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  GridGeometry(std::size_t column_count, std::size_t row_count,
               double resolution, Point2D grid_origin,
               GridExtentMode mode = GridExtentMode::Expandable,
               GridExtentSource source =
                   GridExtentSource::ConfiguredMaplessInitialBounds);
  /**
   * @brief Performs the grid geometry operation for this subsystem.
   *
   * Arguments:
   * - @p frame: Supplies frame input to the operation.
   * - @p width_m: Supplies width m input to the operation.
   * - @p height_m: Supplies height m input to the operation.
   * - @p resolution: Supplies resolution input to the operation.
   * - @p origin_x_m: Supplies origin x m input to the operation.
   * - @p origin_y_m: Supplies origin y m input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  GridGeometry(std::string frame, double width_m, double height_m,
               double resolution, double origin_x_m, double origin_y_m);

  /**
   * @brief Constructs bounds for this subsystem.
   *
   * Arguments:
   * - @p frame: Supplies frame input to the operation.
   * - @p minimum: Supplies minimum input to the operation.
   * - @p maximum: Supplies maximum input to the operation.
   * - @p resolution: Supplies resolution input to the operation.
   * - @p mode: Supplies mode input to the operation.
   * - @p source: Supplies source input to the operation.
   * - @p out_of_bounds_behavior: Supplies out of bounds behavior input to
   * the operation.
   * - @p revision: Supplies revision input to the operation.
   * - @p map_id: Supplies map id input to the operation.
   *
   * Returns:
   * - `GridGeometry` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static GridGeometry fromBounds(std::string frame, Point2D minimum,
                                 Point2D maximum, double resolution,
                                 GridExtentMode mode, GridExtentSource source,
                                 GridOutOfBoundsBehavior out_of_bounds_behavior,
                                 std::size_t revision = 1U,
                                 std::string map_id = {});

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
  bool valid() const noexcept;
  /**
   * @brief Validates package content for this subsystem.
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
  void validate() const;
  /**
   * @brief Performs the width meters operation for this subsystem.
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
  double widthMeters() const noexcept;
  /**
   * @brief Performs the height meters operation for this subsystem.
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
  double heightMeters() const noexcept;
  /**
   * @brief Performs the cell count operation for this subsystem.
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
  std::size_t cellCount() const;
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
  bool contains(Point2D point) const noexcept;
  /**
   * @brief Performs the index operation for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   *
   * Returns:
   * - `std::optional<std::size_t>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<std::size_t> index(Point2D point) const noexcept;
  std::optional<std::pair<std::size_t, std::size_t>> cell(
      Point2D point) const noexcept;
  /**
   * @brief Performs the center operation for this subsystem.
   *
   * Arguments:
   * - @p index: Supplies index input to the operation.
   *
   * Returns:
   * - `Point2D` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Point2D center(std::size_t index) const;
  /**
   * @brief Performs the center operation for this subsystem.
   *
   * Arguments:
   * - @p column: Supplies column input to the operation.
   * - @p row: Supplies row input to the operation.
   *
   * Returns:
   * - `Point2D` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Point2D center(std::size_t column, std::size_t row) const;

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
  bool operator==(const GridGeometry&) const = default;
};

/**
 * @brief Encapsulates grid expansion policy state and behavior for this
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
struct GridExpansionPolicy {
  double margin_m{0.0};
  std::size_t increment_cells{32U};
  double maximum_width_m{0.0};
  double maximum_height_m{0.0};
  std::size_t memory_limit_cells{10'000'000U};
};

/**
 * @brief Encapsulates grid expansion result state and behavior for this
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
struct GridExpansionResult {
  GridGeometry geometry;
  bool expanded{false};
  bool resource_limited{false};
  std::string diagnostic;
};

/**
 * @brief Performs the expand to include operation for this subsystem.
 *
 * Arguments:
 * - @p geometry: Supplies geometry input to the operation.
 * - @p point: Supplies point input to the operation.
 * - @p policy: Supplies policy input to the operation.
 *
 * Returns:
 * - `GridExpansionResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
GridExpansionResult expandToInclude(const GridGeometry& geometry, Point2D point,
                                    const GridExpansionPolicy& policy);

/**
 * @brief Performs the serialize grid geometry operation for this subsystem.
 *
 * Arguments:
 * - @p geometry: Supplies geometry input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string serializeGridGeometry(const GridGeometry& geometry);
/**
 * @brief Performs the deserialize grid geometry operation for this
 * subsystem.
 *
 * Arguments:
 * - @p serialized: Supplies serialized input to the operation.
 *
 * Returns:
 * - `GridGeometry` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
GridGeometry deserializeGridGeometry(const std::string& serialized);
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p source: Supplies source input to the operation.
 *
 * Returns:
 * - `const char*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const char* toString(GridExtentSource source) noexcept;

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_GRID_GEOMETRY_HPP
