/**
 * @file completed_path.hpp
 * @brief Completed path responsibilities.
 *
 * @details This file defines completed path behavior for ROS-independent domain
 * state and value types. It centers on `PathDecisionPoint`,
 * `CompletedPath`, `PathHistory`. Its package-relative location is
 * `include/semaforr/domain/completed_path.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_COMPLETED_PATH_HPP
#define SEMAFORR_DOMAIN_COMPLETED_PATH_HPP

#include <cstdint>
#include <optional>
#include <semaforr/domain/action_execution.hpp>
#include <semaforr/domain/observation.hpp>
#include <vector>

namespace semaforr::domain {

using PathId = std::uint64_t;

/**
 * @brief Encapsulates path decision point state and behavior for this
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
struct PathDecisionPoint {
  SelectedActionRecord selection;
  ActionExecutionResult execution;
  RobotObservation decision_observation;
  std::optional<Action> executed_action;
  std::optional<Point2D> target;
  bool task_started{false};
  bool task_finished{false};
  bool interrupted{false};

  // Selection, command acceptance, and terminal execution are deliberately
  // separate. A terminal record without executed_action was selected but was
  // never accepted by the controller.
  /**
   * @brief Performs the selected operation for this subsystem.
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
  bool selected() const noexcept {
    return selection.decision_id != 0U && selection.action_id != 0U;
  }
  /**
   * @brief Performs the started operation for this subsystem.
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
  bool started() const noexcept { return executed_action.has_value(); }
  /**
   * @brief Performs the completed operation for this subsystem.
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
  bool completed() const noexcept {
    return execution.status == ExecutionCompletionStatus::Succeeded;
  }
  /**
   * @brief Performs the partially completed operation for this subsystem.
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
  bool partiallyCompleted() const noexcept {
    return execution.status == ExecutionCompletionStatus::PartialMovement;
  }
  /**
   * @brief Performs the failed operation for this subsystem.
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
  bool failed() const noexcept {
    switch (execution.status) {
      case ExecutionCompletionStatus::NoMovement:
      case ExecutionCompletionStatus::ControllerRejected:
      case ExecutionCompletionStatus::ControllerFailure:
      case ExecutionCompletionStatus::SensorLost:
      case ExecutionCompletionStatus::Shutdown:
      case ExecutionCompletionStatus::ClockReset:
      case ExecutionCompletionStatus::OdometryReset: return true;
      default: return false;
    }
  }
  /**
   * @brief Performs the cancelled operation for this subsystem.
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
  bool cancelled() const noexcept {
    return execution.status == ExecutionCompletionStatus::Cancelled;
  }
  /**
   * @brief Performs the timed out operation for this subsystem.
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
  bool timedOut() const noexcept {
    return execution.status == ExecutionCompletionStatus::TimedOut ||
           execution.timed_out;
  }
  /**
   * @brief Performs the safety interrupted operation for this subsystem.
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
  bool safetyInterrupted() const noexcept {
    return execution.status == ExecutionCompletionStatus::SafetyInterrupted ||
           execution.safety_interruption;
  }
  /**
   * @brief Performs the preempted operation for this subsystem.
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
  bool preempted() const noexcept {
    return execution.status == ExecutionCompletionStatus::GoalPreempted ||
           execution.status ==
               ExecutionCompletionStatus::NavigationModeTransition;
  }
  /**
   * @brief Performs the actual reached pose operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const Pose2D&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const Pose2D& actualReachedPose() const noexcept {
    return execution.final_pose;
  }
  /**
   * @brief Performs the successful traversal operation for this subsystem.
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
  bool successfulTraversal() const noexcept {
    return started() && execution.successful() && execution.translated();
  }
  /**
   * @brief Performs the partial traversal operation for this subsystem.
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
  bool partialTraversal() const noexcept {
    return started() && partiallyCompleted() && execution.translated();
  }
};

/**
 * @brief Encapsulates completed path state and behavior for this subsystem.
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
struct CompletedPath {
  PathId id{0U};
  std::optional<TaskId> task_id;
  std::optional<Point2D> target;
  std::vector<PathDecisionPoint> decision_points;
  ExecutionTimestamp started_at{};
  ExecutionTimestamp finished_at{};
  bool target_reached{false};
  bool task_skipped{false};

  /**
   * @brief Performs the empty operation for this subsystem.
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
  bool empty() const noexcept { return decision_points.empty(); }
};

/**
 * @brief Encapsulates path history state and behavior for this subsystem.
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
class PathHistory {
 public:
  /**
   * @brief Performs the begin operation for this subsystem.
   *
   * Arguments:
   * - @p id: Supplies id input to the operation.
   * - @p task_id: Supplies task id input to the operation.
   * - @p target: Supplies target input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void begin(PathId id, std::optional<TaskId> task_id,
             std::optional<Point2D> target);
  /**
   * @brief Records package content for this subsystem.
   *
   * Arguments:
   * - @p point: Supplies point input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void record(PathDecisionPoint point);
  /**
   * @brief Performs the finish operation for this subsystem.
   *
   * Arguments:
   * - @p target_reached: Supplies target reached input to the operation.
   * - @p task_skipped: Supplies task skipped input to the operation.
   * - @p when: Supplies when input to the operation.
   *
   * Returns:
   * - `std::optional<CompletedPath>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<CompletedPath> finish(bool target_reached, bool task_skipped,
                                      ExecutionTimestamp when);

  /**
   * @brief Performs the active operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::optional<CompletedPath>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::optional<CompletedPath>& active() const noexcept {
    return active_;
  }
  /**
   * @brief Performs the completed operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<CompletedPath>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<CompletedPath>& completed() const noexcept {
    return completed_;
  }
  /**
   * @brief Performs the terminal events operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<PathDecisionPoint>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<PathDecisionPoint>& terminalEvents() const noexcept {
    return terminal_events_;
  }

 private:
  std::optional<CompletedPath> active_;
  std::vector<CompletedPath> completed_;
  std::vector<PathDecisionPoint> terminal_events_;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_COMPLETED_PATH_HPP
