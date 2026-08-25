/**
 * @file mission_manager.hpp
 * @brief Mission manager responsibilities.
 *
 * @details This file defines mission manager behavior for tiered decision making
 * and action arbitration. It centers on `MissionStep`, `MissionManager`.
 * Its package-relative location is
 * `include/semaforr/decision/mission_manager.hpp`.
 */
#ifndef SEMAFORR_DECISION_MISSION_MANAGER_HPP
#define SEMAFORR_DECISION_MISSION_MANAGER_HPP

#include <semaforr/domain/mission.hpp>
#include <vector>

namespace semaforr::decision {

/**
 * @brief Enumerates the supported mission step values used by this
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
enum class MissionStep { Ready, ActivatedTask, SkippedTask, Complete };

/**
 * @brief Encapsulates mission manager state and behavior for this
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
class MissionManager {
 public:
  /**
   * @brief Performs the mission manager operation for this subsystem.
   *
   * Arguments:
   * - @p mission: Supplies mission input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit MissionManager(domain::Mission& mission) : mission_(mission) {}

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
  MissionStep prepareDecision();
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
  void recordDecision();
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
  bool completeActiveTask();
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
  bool skipActiveTask();
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
  void installPlan(std::vector<domain::Point2D> plan);
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
  void prependPlan(std::vector<domain::Point2D> prefix);
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
  void clearPlan();
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
  bool advanceWaypoint(const domain::Pose2D& pose, domain::Distance tolerance);
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
  bool complete() const noexcept;

 private:
  domain::Mission& mission_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_MISSION_MANAGER_HPP
