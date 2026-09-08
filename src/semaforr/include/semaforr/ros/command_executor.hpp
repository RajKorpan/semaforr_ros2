/**
 * @file command_executor.hpp
 * @brief Command executor responsibilities.
 *
 * @details This file defines command executor behavior for the ROS 2
 * composition and message-adaptation boundary. It centers on
 * `CommandExecutorConfiguration`, `ActionExecutionRequest`,
 * `ActionExecutionStatus`, `ActionExecutionUpdate`, `CommandExecutor`. Its
 * package-relative location is
 * `include/semaforr/ros/command_executor.hpp`.
 */
#ifndef SEMAFORR_ROS_COMMAND_EXECUTOR_HPP
#define SEMAFORR_ROS_COMMAND_EXECUTOR_HPP

#include <cstddef>
#include <optional>
#include <rclcpp/time.hpp>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/action_execution.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/observation.hpp>
#include <string_view>

namespace semaforr::ros {

/**
 * @brief Encapsulates command executor configuration state and behavior for
 * this subsystem.
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
struct CommandExecutorConfiguration {
  double linear_velocity_mps{0.5};
  double angular_velocity_radps{0.5};
  double turn_linear_velocity_mps{0.01};
  double maximum_linear_velocity_mps{0.5};
  double maximum_angular_velocity_radps{0.5};
  double maximum_linear_acceleration_mps2{1.0};
  double maximum_angular_acceleration_radps2{1.0};
  std::size_t maximum_move_action_index{
      domain::Action::maximum_magnitude_index};
  std::size_t maximum_rotation_action_index{
      domain::Action::maximum_magnitude_index};
  double distance_tolerance_m{0.06};
  double angle_tolerance_rad{0.11};
  double timeout_multiplier{1.5};
  double minimum_timeout_s{0.1};
  double odometry_reset_distance_m{2.0};
  double odometry_reset_angle_rad{2.8};
};

/**
 * @brief Encapsulates action execution request state and behavior for this
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
struct ActionExecutionRequest {
  /**
   * @brief Performs the pause operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `domain::Action action{` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Action action{domain::Action::pause()};
  double target_distance_m{0.0};
  double target_angle_rad{0.0};
  domain::DecisionId decision_id{0U};
  domain::ActionId action_id{0U};

  /**
   * @brief Performs the action execution request operation for this
   * subsystem.
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
  ActionExecutionRequest() = default;
  /**
   * @brief Performs the action execution request operation for this
   * subsystem.
   *
   * Arguments:
   * - @p requested_action: Supplies requested action input to the
   * operation.
   * - @p requested_distance_m: Supplies requested distance m input to the
   * operation.
   * - @p requested_angle_rad: Supplies requested angle rad input to the
   * operation.
   * - @p requested_decision_id: Supplies requested decision id input to the
   * operation.
   * - @p requested_action_id: Supplies requested action id input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ActionExecutionRequest(domain::Action requested_action,
                         double requested_distance_m,
                         double requested_angle_rad,
                         domain::DecisionId requested_decision_id = 0U,
                         domain::ActionId requested_action_id = 0U)
      : action(requested_action),
        target_distance_m(requested_distance_m),
        target_angle_rad(requested_angle_rad),
        decision_id(requested_decision_id),
        action_id(requested_action_id) {}
};

/**
 * @brief Enumerates the supported action execution status values used by
 * this subsystem.
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
enum class ActionExecutionStatus {
  Idle,
  Executing,
  Completed,
  TimedOut,
  OdometryReset,
  ClockReset,
  Cancelled,
  SafetyInterrupted,
  ControllerRejected,
  ControllerFailure,
  GoalPreempted,
  NavigationModeTransition,
  SensorLost,
  Shutdown
};

/**
 * @brief Encapsulates action execution update state and behavior for this
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
struct ActionExecutionUpdate {
  ActionExecutionStatus status{ActionExecutionStatus::Idle};
  domain::VelocityCommand command;
  double progress{0.0};
  double target{0.0};
  domain::DecisionId decision_id{0U};
  domain::ActionId action_id{0U};
  domain::Pose2D start_pose;
  domain::Pose2D final_pose;
  double distance_achieved_m{0.0};
  double rotation_achieved_rad{0.0};
};

/**
 * @brief Encapsulates command executor state and behavior for this
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
class CommandExecutor {
 public:
  /**
   * @brief Performs the command executor operation for this subsystem.
   *
   * Arguments:
   * - @p configuration: Supplies configuration input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit CommandExecutor(CommandExecutorConfiguration configuration);

  /**
   * @brief Performs the start operation for this subsystem.
   *
   * Arguments:
   * - @p request: Supplies request input to the operation.
   * - @p pose: Supplies pose input to the operation.
   * - @p now: Supplies now input to the operation.
   *
   * Returns:
   * - `ActionExecutionUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ActionExecutionUpdate start(const ActionExecutionRequest& request,
                              const domain::Pose2D& pose,
                              const rclcpp::Time& now);
  /**
   * @brief Updates package content for this subsystem.
   *
   * Arguments:
   * - @p pose: Supplies pose input to the operation.
   * - @p now: Supplies now input to the operation.
   *
   * Returns:
   * - `ActionExecutionUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ActionExecutionUpdate update(const domain::Pose2D& pose,
                               const rclcpp::Time& now);
  /**
   * @brief Performs the cancel operation for this subsystem.
   *
   * Arguments:
   * - @p status: Supplies status input to the operation.
   *
   * Returns:
   * - `ActionExecutionUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ActionExecutionUpdate cancel(
      ActionExecutionStatus status = ActionExecutionStatus::Cancelled) noexcept;

  /**
   * @brief Performs the status operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `ActionExecutionStatus` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ActionExecutionStatus status() const noexcept { return status_; }
  /**
   * @brief Performs the executing operation for this subsystem.
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
  bool executing() const noexcept {
    return status_ == ActionExecutionStatus::Executing;
  }
  /**
   * @brief Performs the command operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const domain::VelocityCommand&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const domain::VelocityCommand& command() const noexcept { return command_; }

 private:
  /**
   * @brief Performs the terminal operation for this subsystem.
   *
   * Arguments:
   * - @p status: Supplies status input to the operation.
   *
   * Returns:
   * - `ActionExecutionUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ActionExecutionUpdate terminal(ActionExecutionStatus status) noexcept;
  /**
   * @brief Performs the timeout seconds operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double timeoutSeconds() const noexcept;
  /**
   * @brief Performs the target operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double target() const noexcept;

  CommandExecutorConfiguration configuration_;
  std::optional<ActionExecutionRequest> request_;
  std::optional<domain::Pose2D> previous_pose_;
  std::optional<domain::Pose2D> start_pose_;
  std::optional<rclcpp::Time> started_at_;
  std::optional<rclcpp::Time> command_updated_at_;
  ActionExecutionStatus status_{ActionExecutionStatus::Idle};
  domain::VelocityCommand command_;
  double progress_{0.0};
  double distance_achieved_m_{0.0};
  double rotation_achieved_rad_{0.0};
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
std::string_view toString(ActionExecutionStatus status) noexcept;

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_COMMAND_EXECUTOR_HPP
