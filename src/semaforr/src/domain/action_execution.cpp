/**
 * @file action_execution.cpp
 * @brief Action execution responsibilities.
 *
 * @details This file implements action execution behavior for ROS-independent
 * domain state and value types. It records the declarations, settings,
 * fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/domain/action_execution.cpp`.
 */
#include <semaforr/domain/action_execution.hpp>

namespace semaforr::domain {

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p status: Supplies status input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(ExecutionCompletionStatus status) noexcept {
  switch (status) {
    case ExecutionCompletionStatus::Succeeded:
      return "succeeded";
    case ExecutionCompletionStatus::PartialMovement:
      return "partial_movement";
    case ExecutionCompletionStatus::NoMovement:
      return "no_movement";
    case ExecutionCompletionStatus::TimedOut:
      return "timed_out";
    case ExecutionCompletionStatus::Cancelled:
      return "cancelled";
    case ExecutionCompletionStatus::SafetyInterrupted:
      return "safety_interrupted";
    case ExecutionCompletionStatus::ControllerRejected:
      return "controller_rejected";
    case ExecutionCompletionStatus::ControllerFailure:
      return "controller_failure";
    case ExecutionCompletionStatus::GoalPreempted:
      return "goal_preempted";
    case ExecutionCompletionStatus::NavigationModeTransition:
      return "navigation_mode_transition";
    case ExecutionCompletionStatus::SensorLost:
      return "sensor_lost";
    case ExecutionCompletionStatus::Shutdown:
      return "shutdown";
    case ExecutionCompletionStatus::ClockReset:
      return "clock_reset";
    case ExecutionCompletionStatus::OdometryReset:
      return "odometry_reset";
  }
  return "unknown";
}

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p disposition: Supplies disposition input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(FeedbackDisposition disposition) noexcept {
  switch (disposition) {
    case FeedbackDisposition::Accepted:
      return "accepted";
    case FeedbackDisposition::Duplicate:
      return "duplicate";
    case FeedbackDisposition::UnknownAction:
      return "unknown_action";
    case FeedbackDisposition::StaleDecision:
      return "stale_decision";
    case FeedbackDisposition::TaskMismatch:
      return "task_mismatch";
    case FeedbackDisposition::NotStarted:
      return "not_started";
    case FeedbackDisposition::AlreadyStarted:
      return "already_started";
  }
  return "unknown";
}

}  // namespace semaforr::domain
