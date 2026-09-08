/**
 * @file grid_geometry.cpp
 * @brief Grid geometry responsibilities.
 *
 * @details This file implements grid geometry behavior for ROS-independent
 * domain state and value types. It records the declarations, settings,
 * fixtures, or guidance needed by that responsibility. Its package-relative
 * location is `src/domain/grid_geometry.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <semaforr/domain/grid_geometry.hpp>
#include <sstream>
#include <stdexcept>

namespace semaforr::domain {
namespace {

/**
 * @brief Performs the checked cell count operation for this subsystem.
 *
 * Arguments:
 * - @p columns: Supplies columns input to the operation.
 * - @p rows: Supplies rows input to the operation.
 *
 * Returns:
 * - `std::size_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::size_t checkedCellCount(std::size_t columns, std::size_t rows) {
  if (columns != 0U && rows > std::numeric_limits<std::size_t>::max() / columns)
    throw std::overflow_error("grid cell count overflow");
  return columns * rows;
}

/**
 * @brief Performs the aligned cells operation for this subsystem.
 *
 * Arguments:
 * - @p amount_m: Supplies amount m input to the operation.
 * - @p resolution_m: Supplies resolution m input to the operation.
 * - @p increment: Supplies increment input to the operation.
 *
 * Returns:
 * - `std::size_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::size_t alignedCells(double amount_m, double resolution_m,
                         std::size_t increment) {
  const auto needed = static_cast<std::size_t>(
      std::ceil(std::max(0.0, amount_m) / resolution_m));
  const auto chunk = std::max<std::size_t>(1U, increment);
  return ((needed + chunk - 1U) / chunk) * chunk;
}

}  // namespace

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
GridGeometry::GridGeometry(std::size_t column_count, std::size_t row_count,
                           double resolution, Point2D grid_origin,
                           GridExtentMode mode, GridExtentSource source)
    : frame_id("map"),
      minimum(grid_origin),
      maximum{grid_origin.x_m + static_cast<double>(column_count) * resolution,
              grid_origin.y_m + static_cast<double>(row_count) * resolution},
      resolution_m(resolution),
      columns(column_count),
      rows(row_count),
      origin(grid_origin),
      out_of_bounds(mode == GridExtentMode::Expandable
                        ? GridOutOfBoundsBehavior::ExpandBeforeInsert
                        : GridOutOfBoundsBehavior::NonTraversable),
      extent_mode(mode),
      geometry_revision(1U),
      extent_source(source) {
  if (column_count > 0U && row_count > 0U) validate();
}

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
GridGeometry::GridGeometry(std::string frame, double width_m, double height_m,
                           double resolution, double origin_x_m,
                           double origin_y_m)
    : GridGeometry(fromBounds(std::move(frame), {origin_x_m, origin_y_m},
                              {origin_x_m + width_m, origin_y_m + height_m},
                              resolution, GridExtentMode::Fixed,
                              GridExtentSource::RepresentationLocalBounds,
                              GridOutOfBoundsBehavior::NonTraversable)) {}

/**
 * @brief Constructs bounds for this subsystem.
 *
 * Arguments:
 * - @p frame: Supplies frame input to the operation.
 * - @p grid_minimum: Supplies grid minimum input to the operation.
 * - @p requested_maximum: Supplies requested maximum input to the
 * operation.
 * - @p resolution: Supplies resolution input to the operation.
 * - @p mode: Supplies mode input to the operation.
 * - @p source: Supplies source input to the operation.
 * - @p out_of_bounds_behavior: Supplies out of bounds behavior input to the
 * operation.
 * - @p revision: Supplies revision input to the operation.
 * - @p map_id: Supplies map id input to the operation.
 *
 * Returns:
 * - `GridGeometry` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
GridGeometry GridGeometry::fromBounds(
    std::string frame, Point2D grid_minimum, Point2D requested_maximum,
    double resolution, GridExtentMode mode, GridExtentSource source,
    GridOutOfBoundsBehavior out_of_bounds_behavior, std::size_t revision,
    std::string map_id) {
  if (frame.empty() || !grid_minimum.finite() || !requested_maximum.finite() ||
      !std::isfinite(resolution) || resolution <= 0.0 ||
      requested_maximum.x_m <= grid_minimum.x_m ||
      requested_maximum.y_m <= grid_minimum.y_m)
    throw std::invalid_argument(
        "grid bounds, frame, and resolution are invalid");
  const auto column_count = static_cast<std::size_t>(
      std::ceil((requested_maximum.x_m - grid_minimum.x_m) / resolution));
  const auto row_count = static_cast<std::size_t>(
      std::ceil((requested_maximum.y_m - grid_minimum.y_m) / resolution));
  GridGeometry result;
  result.frame_id = std::move(frame);
  result.minimum = grid_minimum;
  result.maximum = {
      grid_minimum.x_m + static_cast<double>(column_count) * resolution,
      grid_minimum.y_m + static_cast<double>(row_count) * resolution};
  result.resolution_m = resolution;
  result.columns = column_count;
  result.rows = row_count;
  result.origin = grid_minimum;
  result.extent_mode = mode;
  result.extent_source = source;
  result.out_of_bounds = out_of_bounds_behavior;
  result.geometry_revision = revision;
  result.map_identifier = std::move(map_id);
  result.validate();
  return result;
}

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
bool GridGeometry::valid() const noexcept {
  if (frame_id.empty() || !minimum.finite() || !maximum.finite() ||
      !origin.finite() || !std::isfinite(resolution_m) || resolution_m <= 0.0 ||
      columns == 0U || rows == 0U || origin != minimum)
    return false;
  const double expected_x =
      minimum.x_m + static_cast<double>(columns) * resolution_m;
  const double expected_y =
      minimum.y_m + static_cast<double>(rows) * resolution_m;
  return std::abs(expected_x - maximum.x_m) <= geometry_tolerance_m &&
         std::abs(expected_y - maximum.y_m) <= geometry_tolerance_m;
}

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
void GridGeometry::validate() const {
  if (!valid()) throw std::invalid_argument("grid geometry is inconsistent");
  if (cellCount() > 100'000'000U)
    throw std::invalid_argument("grid geometry exceeds the hard cell limit");
}

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
double GridGeometry::widthMeters() const noexcept {
  return maximum.x_m - minimum.x_m;
}
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
double GridGeometry::heightMeters() const noexcept {
  return maximum.y_m - minimum.y_m;
}
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
std::size_t GridGeometry::cellCount() const {
  return checkedCellCount(columns, rows);
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
bool GridGeometry::contains(Point2D point) const noexcept {
  return point.finite() && point.x_m >= minimum.x_m &&
         point.y_m >= minimum.y_m && point.x_m < maximum.x_m &&
         point.y_m < maximum.y_m;
}
std::optional<std::pair<std::size_t, std::size_t>> GridGeometry::cell(
    Point2D point) const noexcept {
  if (!contains(point)) return std::nullopt;
  const auto column = static_cast<std::size_t>(
      /**
       * @brief Performs the floor operation for this subsystem.
       *
       * Arguments:
       * - @p resolution_m: Supplies resolution m input to the operation.
       *
       * Returns:
       * - No value; effects are applied to owned state or outputs.
       *
       * Exceptions:
       * - None documented; validation or dependency failures may propagate.
       */
      std::floor((point.x_m - origin.x_m) / resolution_m));
  const auto row = static_cast<std::size_t>(
      /**
       * @brief Performs the floor operation for this subsystem.
       *
       * Arguments:
       * - @p resolution_m: Supplies resolution m input to the operation.
       *
       * Returns:
       * - No value; effects are applied to owned state or outputs.
       *
       * Exceptions:
       * - None documented; validation or dependency failures may propagate.
       */
      std::floor((point.y_m - origin.y_m) / resolution_m));
  if (column >= columns || row >= rows) return std::nullopt;
  return std::pair{column, row};
}
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
std::optional<std::size_t> GridGeometry::index(Point2D point) const noexcept {
  const auto grid_cell = cell(point);
  if (!grid_cell) return std::nullopt;
  return grid_cell->second * columns + grid_cell->first;
}
/**
 * @brief Performs the center operation for this subsystem.
 *
 * Arguments:
 * - @p index_value: Supplies index value input to the operation.
 *
 * Returns:
 * - `Point2D` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
Point2D GridGeometry::center(std::size_t index_value) const {
  if (index_value >= cellCount())
    throw std::out_of_range("grid cell index is out of range");
  return center(index_value % columns, index_value / columns);
}
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
Point2D GridGeometry::center(std::size_t column, std::size_t row) const {
  if (column >= columns || row >= rows)
    throw std::out_of_range("grid row or column is out of range");
  return {origin.x_m + (static_cast<double>(column) + 0.5) * resolution_m,
          origin.y_m + (static_cast<double>(row) + 0.5) * resolution_m};
}

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
                                    const GridExpansionPolicy& policy) {
  geometry.validate();
  if (geometry.extent_mode != GridExtentMode::Expandable)
    return {geometry, false, false,
            "fixed grid rejected out-of-bounds evidence"};
  const double margin = std::max(0.0, policy.margin_m);
  if (point.x_m >= geometry.minimum.x_m + margin &&
      point.y_m >= geometry.minimum.y_m + margin &&
      point.x_m < geometry.maximum.x_m - margin &&
      point.y_m < geometry.maximum.y_m - margin)
    return {geometry, false, false, {}};
  const auto left = alignedCells(geometry.minimum.x_m + margin - point.x_m,
                                 geometry.resolution_m, policy.increment_cells);
  const auto right =
      alignedCells(point.x_m + margin - geometry.maximum.x_m,
                   geometry.resolution_m, policy.increment_cells);
  const auto bottom =
      alignedCells(geometry.minimum.y_m + margin - point.y_m,
                   geometry.resolution_m, policy.increment_cells);
  const auto top = alignedCells(point.y_m + margin - geometry.maximum.y_m,
                                geometry.resolution_m, policy.increment_cells);
  if (left == 0U && right == 0U && bottom == 0U && top == 0U)
    return {geometry, false, false, {}};
  const auto new_columns = geometry.columns + left + right;
  const auto new_rows = geometry.rows + bottom + top;
  const double new_width =
      static_cast<double>(new_columns) * geometry.resolution_m;
  const double new_height =
      static_cast<double>(new_rows) * geometry.resolution_m;
  const bool exceeds_width =
      policy.maximum_width_m > 0.0 && new_width > policy.maximum_width_m;
  const bool exceeds_height =
      policy.maximum_height_m > 0.0 && new_height > policy.maximum_height_m;
  const bool exceeds_memory =
      checkedCellCount(new_columns, new_rows) > policy.memory_limit_cells;
  if (exceeds_width || exceeds_height || exceeds_memory)
    return {geometry, false, true,
            "grid expansion rejected by configured extent or memory limit"};
  auto result = geometry;
  result.minimum = {
      geometry.minimum.x_m - static_cast<double>(left) * geometry.resolution_m,
      geometry.minimum.y_m -
          static_cast<double>(bottom) * geometry.resolution_m};
  result.origin = result.minimum;
  result.columns = new_columns;
  result.rows = new_rows;
  result.maximum = {result.minimum.x_m + new_width,
                    result.minimum.y_m + new_height};
  result.geometry_revision = geometry.geometry_revision + 1U;
  result.extent_source = GridExtentSource::SensorDerivedExpansion;
  return {std::move(result), true, false, "grid geometry expanded"};
}

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
std::string serializeGridGeometry(const GridGeometry& geometry) {
  geometry.validate();
  std::ostringstream output;
  output << std::setprecision(17) << "SEMAFORR_GRID_GEOMETRY_V1 "
         << std::quoted(geometry.frame_id) << ' ' << geometry.minimum.x_m << ' '
         << geometry.minimum.y_m << ' ' << geometry.maximum.x_m << ' '
         << geometry.maximum.y_m << ' ' << geometry.resolution_m << ' '
         << geometry.columns << ' ' << geometry.rows << ' '
         << static_cast<int>(geometry.boundary_convention) << ' '
         << static_cast<int>(geometry.out_of_bounds) << ' '
         << static_cast<int>(geometry.extent_mode) << ' '
         << geometry.geometry_revision << ' '
         << static_cast<int>(geometry.extent_source) << ' '
         << std::quoted(geometry.map_identifier);
  return output.str();
}

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
GridGeometry deserializeGridGeometry(const std::string& serialized) {
  std::istringstream input(serialized);
  std::string magic;
  GridGeometry result;
  int boundary = 0, out_of_bounds = 0, mode = 0, source = 0;
  input >> magic >> std::quoted(result.frame_id) >> result.minimum.x_m >>
      result.minimum.y_m >> result.maximum.x_m >> result.maximum.y_m >>
      result.resolution_m >> result.columns >> result.rows >> boundary >>
      out_of_bounds >> mode >> result.geometry_revision >> source >>
      std::quoted(result.map_identifier);
  if (!input || magic != "SEMAFORR_GRID_GEOMETRY_V1")
    throw std::runtime_error("unsupported or malformed grid geometry snapshot");
  result.origin = result.minimum;
  result.boundary_convention = static_cast<CellBoundaryConvention>(boundary);
  result.out_of_bounds = static_cast<GridOutOfBoundsBehavior>(out_of_bounds);
  result.extent_mode = static_cast<GridExtentMode>(mode);
  result.extent_source = static_cast<GridExtentSource>(source);
  result.validate();
  return result;
}

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
const char* toString(GridExtentSource source) noexcept {
  switch (source) {
    case GridExtentSource::StaticMapBounds:
      return "static_map_bounds";
    case GridExtentSource::InferredMapBounds:
      return "inferred_map_bounds";
    case GridExtentSource::ConfiguredMaplessInitialBounds:
      return "configured_mapless_initial_bounds";
    case GridExtentSource::SensorDerivedExpansion:
      return "sensor_derived_expansion";
    case GridExtentSource::RepresentationLocalBounds:
      return "representation_local_bounds";
  }
  return "unknown";
}

}  // namespace semaforr::domain
