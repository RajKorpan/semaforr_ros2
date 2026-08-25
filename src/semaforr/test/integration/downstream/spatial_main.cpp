/**
 * @file spatial_main.cpp
 * @brief Spatial main responsibilities.
 *
 * @details This file exercises spatial main behavior for automated verification and
 * regression testing. It records the declarations, settings, fixtures, or
 * guidance needed by that responsibility. Its package-relative location is
 * `test/integration/downstream/spatial_main.cpp`.
 */
#include <semaforr/spatial/spatial_learning_coordinator.hpp>

/**
 * @brief Performs the main operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `int` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
int main() {
  const auto learning =
      semaforr::spatial::SpatialLearningCoordinator::defaults();
  return learning.learnerCount() == 10U ? 0 : 1;
}
