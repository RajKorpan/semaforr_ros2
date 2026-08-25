/**
 * @file main.cpp
 * @brief Main responsibilities.
 *
 * @details This file exercises main behavior for automated verification and
 * regression testing. It records the declarations, settings, fixtures, or
 * guidance needed by that responsibility. Its package-relative location is
 * `test/integration/downstream/main.cpp`.
 */
#include <semaforr/domain/action.hpp>

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
  const semaforr::domain::Action action(semaforr::domain::ActionType::Forward,
                                        1U);
  return action.type() == semaforr::domain::ActionType::Forward ? 0 : 1;
}
