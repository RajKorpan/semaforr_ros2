/**
 * @file coverage.hpp
 * @brief Coverage responsibilities.
 *
 * @details This file defines coverage behavior for learned spatial representations
 * and their lifecycle. It records the declarations, settings, fixtures, or
 * guidance needed by that responsibility. Its package-relative location is
 * `include/semaforr/spatial/coverage.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_COVERAGE_HPP
#define SEMAFORR_SPATIAL_COVERAGE_HPP

#include <cstddef>
#include <semaforr/domain/world_model.hpp>

namespace semaforr::spatial {

// Counts one-metre cells overlapped by the union of learned regions and trails.
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
std::size_t representedCoverageCells(const domain::SpatialModel& model);

}  // namespace semaforr::spatial

#endif
