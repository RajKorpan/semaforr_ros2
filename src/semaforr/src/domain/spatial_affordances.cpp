/**
 * @file spatial_affordances.cpp
 * @brief Spatial affordances responsibilities.
 *
 * @details This file implements spatial affordances behavior for
 * ROS-independent domain state and value types. It records the declarations,
 * settings, fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/domain/spatial_affordances.cpp`.
 */
#include <algorithm>
#include <semaforr/domain/spatial_affordances.hpp>

namespace semaforr::domain {

/**
 * @brief Performs the at operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `const ConveyorCell*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const ConveyorCell* ConveyorGrid::at(Point2D point) const noexcept {
  const auto index = geometry.index(point);
  if (!index) return nullptr;
  const auto found =
      std::lower_bound(cells.begin(), cells.end(), *index,
                       [](const ConveyorCell& cell, std::size_t value) {
                         return cell.index < value;
                       });
  return found != cells.end() && found->index == *index ? &*found : nullptr;
}

}  // namespace semaforr::domain
