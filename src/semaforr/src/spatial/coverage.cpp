/**
 * @file coverage.cpp
 * @brief Coverage responsibilities.
 *
 * @details This file implements coverage behavior for learned spatial
 * representations and their lifecycle. It centers on `CoverageGrid`. Its
 * package-relative location is `src/spatial/coverage.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <semaforr/spatial/coverage.hpp>
#include <unordered_set>

namespace semaforr::spatial {
namespace {

constexpr double coverage_resolution_m = 1.0;

/**
 * @brief Encapsulates coverage grid state and behavior for this subsystem.
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
struct CoverageGrid {
  std::size_t columns = 0U;
  std::size_t rows = 0U;
  domain::Point2D origin;
};

/**
 * @brief Performs the grid for operation for this subsystem.
 *
 * Arguments:
 * - @p model: Supplies model input to the operation.
 *
 * Returns:
 * - `CoverageGrid` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
CoverageGrid gridFor(const domain::SpatialModel& model) {
  if (model.inclusion_grid.columns != 0U)
    return {static_cast<std::size_t>(
                std::ceil(static_cast<double>(model.inclusion_grid.columns) *
                          model.inclusion_grid.resolution_m)),
            static_cast<std::size_t>(
                std::ceil(static_cast<double>(model.inclusion_grid.rows) *
                          model.inclusion_grid.resolution_m)),
            model.inclusion_grid.origin};
  return {static_cast<std::size_t>(
              std::ceil(static_cast<double>(model.known_grid.columns) *
                        model.known_grid.resolution_m)),
          static_cast<std::size_t>(
              std::ceil(static_cast<double>(model.known_grid.rows) *
                        model.known_grid.resolution_m)),
          model.known_grid.origin};
}

/**
 * @brief Performs the mark operation for this subsystem.
 *
 * Arguments:
 * - @p grid: Supplies grid input to the operation.
 * - @p point: Supplies point input to the operation.
 * - @p cells: Supplies cells input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void mark(const CoverageGrid& grid, domain::Point2D point,
          std::unordered_set<std::size_t>& cells) {
  const auto column = static_cast<long long>(
      std::floor((point.x_m - grid.origin.x_m) / coverage_resolution_m));
  const auto row = static_cast<long long>(
      std::floor((point.y_m - grid.origin.y_m) / coverage_resolution_m));
  if (column < 0 || row < 0 || column >= static_cast<long long>(grid.columns) ||
      row >= static_cast<long long>(grid.rows))
    return;
  cells.insert(static_cast<std::size_t>(row) * grid.columns +
               static_cast<std::size_t>(column));
}

}  // namespace

/**
 * @brief Performs the represented coverage cells operation for this
 * subsystem.
 *
 * Arguments:
 * - @p model: Supplies model input to the operation.
 *
 * Returns:
 * - `std::size_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::size_t representedCoverageCells(const domain::SpatialModel& model) {
  const CoverageGrid grid = gridFor(model);
  if (grid.columns == 0U || grid.rows == 0U) return 0U;

  std::unordered_set<std::size_t> cells;
  for (const auto& region : model.learned_regions) {
    const double radius = region.radius.meters();
    const auto first_column = static_cast<long long>(
        std::floor(region.center.x_m - radius - grid.origin.x_m));
    const auto last_column = static_cast<long long>(
        std::floor(region.center.x_m + radius - grid.origin.x_m));
    const auto first_row = static_cast<long long>(
        std::floor(region.center.y_m - radius - grid.origin.y_m));
    const auto last_row = static_cast<long long>(
        std::floor(region.center.y_m + radius - grid.origin.y_m));
    for (long long row = std::max(0LL, first_row);
         row <= std::min(last_row, static_cast<long long>(grid.rows) - 1LL);
         ++row) {
      for (long long column = std::max(0LL, first_column);
           column <=
           std::min(last_column, static_cast<long long>(grid.columns) - 1LL);
           ++column) {
        const double minimum_x = grid.origin.x_m + static_cast<double>(column);
        const double minimum_y = grid.origin.y_m + static_cast<double>(row);
        const double nearest_x = std::clamp(region.center.x_m, minimum_x,
                                            minimum_x + coverage_resolution_m);
        const double nearest_y = std::clamp(region.center.y_m, minimum_y,
                                            minimum_y + coverage_resolution_m);
        const double dx = nearest_x - region.center.x_m;
        const double dy = nearest_y - region.center.y_m;
        if (dx * dx + dy * dy <= radius * radius)
          cells.insert(static_cast<std::size_t>(row) * grid.columns +
                       static_cast<std::size_t>(column));
      }
    }
  }

  constexpr double trail_sample_m = coverage_resolution_m / 4.0;
  for (const auto& trail : model.trails) {
    if (trail.size() == 1U) mark(grid, trail.front(), cells);
    for (std::size_t index = 1U; index < trail.size(); ++index) {
      const auto& from = trail[index - 1U];
      const auto& to = trail[index];
      const double length = domain::distance(from, to).meters();
      const std::size_t samples = std::max<std::size_t>(
          1U, static_cast<std::size_t>(std::ceil(length / trail_sample_m)));
      for (std::size_t sample = 0U; sample <= samples; ++sample) {
        const double fraction =
            static_cast<double>(sample) / static_cast<double>(samples);
        mark(grid,
             {from.x_m + fraction * (to.x_m - from.x_m),
              from.y_m + fraction * (to.y_m - from.y_m)},
             cells);
      }
    }
  }
  return cells.size();
}

}  // namespace semaforr::spatial
