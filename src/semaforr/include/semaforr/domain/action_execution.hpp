/**
 * @file action_execution.hpp
 * @brief Action execution responsibilities.
 *
 * @details This file defines action execution behavior for ROS-independent domain
 * state and value types. It centers on `SelectedActionRecord`,
 * `ExecutionCompletionStatus`, `ActionStartedEvent`,
 * `ActionProgressEvent`, `ActionExecutionResult`, `FeedbackDisposition`.
 * Its package-relative location is
 * `include/semaforr/domain/action_execution.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_ACTION_EXECUTION_HPP
#define SEMAFORR_DOMAIN_ACTION_EXECUTION_HPP

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/mission.hpp>
#include <string>
#include <string_view>

namespace semaforr::domain {

using DecisionId = std::uint64_t;
using ActionId = std::uint64_t;
using ExecutionTimestamp = std::chrono::steady_clock::time_point;

/**
 * @brief Encapsulates selected action record state and behavior for this
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
struct SelectedActionRecord {
  DecisionId decision_id{0U};
  ActionId action_id{0U};
  std::optional<TaskId> task_id;
  ExecutionTimestamp selected_at{};
  Pose2D expected_start;
  /**
   * @brief Performs the pause operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `Action action{` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Action action{Action::pause()};
  std::string selected_tier;
  std::string provenance;
  double intended_distance_m{0.0};
  double intended_rotation_rad{0.0};
  double intended_duration_s{0.0};
};

/**
 * @brief Enumerates the supported execution completion status values used
 * by this subsystem.
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
enum class ExecutionCompletionStatus {
  Succeeded,
  PartialMovement,
  NoMovement,
  TimedOut,
  Cancelled,
  SafetyInterrupted,
  ControllerRejected,
  ControllerFailure,
  GoalPreempted,
  NavigationModeTransition,
  SensorLost,
  Shutdown,
  ClockReset,
  OdometryReset
};

/**
 * @brief Encapsulates action started event state and behavior for this
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
struct ActionStartedEvent {
  DecisionId decision_id{0U};
  ActionId action_id{0U};
  ExecutionTimestamp started_at{};
  Pose2D start_pose;
};

/**
 * @brief Encapsulates action progress event state and behavior for this
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
struct ActionProgressEvent {
  DecisionId decision_id{0U};
  ActionId action_id{0U};
  ExecutionTimestamp observed_at{};
  Pose2D pose;
  double distance_achieved_m{0.0};
  double rotation_achieved_rad{0.0};
};

/**
 * @brief Encapsulates action execution result state and behavior for this
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
struct ActionExecutionResult {
  DecisionId decision_id{0U};
  ActionId action_id{0U};
  std::optional<TaskId> task_id;
  ExecutionTimestamp started_at{};
  ExecutionTimestamp finished_at{};
  ExecutionCompletionStatus status{ExecutionCompletionStatus::NoMovement};
  Pose2D start_pose;
  Pose2D final_pose;
  double distance_achieved_m{0.0};
  double rotation_achieved_rad{0.0};
  bool timed_out{false};
  std::string cancellation_reason;
  bool safety_interruption{false};
  bool controller_failure{false};
  bool collision{false};
  bool near_collision{false};

  /**
   * @brief Performs the terminal operation for this subsystem.
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
  bool terminal() const noexcept { return true; }
  /**
   * @brief Performs the successful operation for this subsystem.
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
  bool successful() const noexcept {
    return status == ExecutionCompletionStatus::Succeeded;
  }
  /**
   * @brief Performs the moved operation for this subsystem.
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
  bool moved() const noexcept {
    return distance_achieved_m > geometry_tolerance_m ||
           rotation_achieved_rad > geometry_tolerance_m;
  }
  /**
   * @brief Performs the translated operation for this subsystem.
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
  bool translated() const noexcept {
    return distance_achieved_m > geometry_tolerance_m ||
           distance(start_pose.position, final_pose.position).meters() >
               geometry_tolerance_m;
  }
  /**
   * @brief Performs the rotated operation for this subsystem.
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
  bool rotated() const noexcept {
    return rotation_achieved_rad > geometry_tolerance_m ||
           std::abs(Angle::normalize(final_pose.heading.radians() -
                                     start_pose.heading.radians())) >
               geometry_tolerance_m;
  }
};

/**
 * @brief Enumerates the supported feedback disposition values used by this
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
enum class FeedbackDisposition {
  Accepted,
  Duplicate,
  UnknownAction,
  StaleDecision,
  TaskMismatch,
  NotStarted,
  AlreadyStarted
};

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
std::string_view toString(ExecutionCompletionStatus status) noexcept;
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
std::string_view toString(FeedbackDisposition disposition) noexcept;

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_ACTION_EXECUTION_HPP
