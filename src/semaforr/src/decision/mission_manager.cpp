/**
 * @file mission_manager.cpp
 * @brief Mission manager responsibilities.
 *
 * @details This file implements mission manager behavior for tiered decision making
 * and action arbitration. It records the declarations, settings, fixtures,
 * or guidance needed by that responsibility. Its package-relative location
 * is `src/decision/mission_manager.cpp`.
 */
#include <semaforr/decision/mission_manager.hpp>

namespace semaforr::decision {

/**
 * @brief Performs the prepare decision operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `MissionStep` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
MissionStep MissionManager::prepareDecision() {
  if (!mission_.active()) {
    if (!mission_.activate_next()) {
      return MissionStep::Complete;
    }
    return MissionStep::ActivatedTask;
  }
  if (mission_.decisions_for_active() >= mission_.decision_limit()) {
    mission_.skip_active();
    if (!mission_.activate_next()) {
      return MissionStep::Complete;
    }
    return MissionStep::SkippedTask;
  }
  return MissionStep::Ready;
}

/**
 * @brief Records decision for this subsystem.
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
void MissionManager::recordDecision() { mission_.record_decision(); }

/**
 * @brief Performs the complete active task operation for this subsystem.
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
bool MissionManager::completeActiveTask() { return mission_.complete_active(); }

/**
 * @brief Performs the skip active task operation for this subsystem.
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
bool MissionManager::skipActiveTask() { return mission_.skip_active(); }

/**
 * @brief Performs the install plan operation for this subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void MissionManager::installPlan(std::vector<domain::Point2D> plan) {
  mission_.install_active_plan(std::move(plan));
}

/**
 * @brief Performs the prepend plan operation for this subsystem.
 *
 * Arguments:
 * - @p prefix: Supplies prefix input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void MissionManager::prependPlan(std::vector<domain::Point2D> prefix) {
  if (!mission_.active())
    throw std::logic_error("cannot prepend a plan without an active task");
  const auto& task = *mission_.active();
  prefix.insert(prefix.end(),
                task.plan.begin() +
                    static_cast<std::ptrdiff_t>(task.waypoint_index),
                task.plan.end());
  mission_.install_active_plan(std::move(prefix));
}

/**
 * @brief Clears plan for this subsystem.
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
void MissionManager::clearPlan() { mission_.install_active_plan({}); }

/**
 * @brief Performs the advance waypoint operation for this subsystem.
 *
 * Arguments:
 * - @p pose: Supplies pose input to the operation.
 * - @p tolerance: Supplies tolerance input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool MissionManager::advanceWaypoint(const domain::Pose2D& pose,
                                     domain::Distance tolerance) {
  return mission_.advance_waypoint(pose, tolerance);
}

/**
 * @brief Performs the complete operation for this subsystem.
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
bool MissionManager::complete() const noexcept { return mission_.finished(); }

}  // namespace semaforr::decision
