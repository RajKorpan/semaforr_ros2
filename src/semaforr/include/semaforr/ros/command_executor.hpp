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

struct ActionExecutionRequest {
  domain::Action action{domain::Action::pause()};
  double target_distance_m{0.0};
  double target_angle_rad{0.0};
  domain::DecisionId decision_id{0U};
  domain::ActionId action_id{0U};

  ActionExecutionRequest() = default;
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

class CommandExecutor {
 public:
  explicit CommandExecutor(CommandExecutorConfiguration configuration);

  ActionExecutionUpdate start(const ActionExecutionRequest& request,
                              const domain::Pose2D& pose,
                              const rclcpp::Time& now);
  ActionExecutionUpdate update(const domain::Pose2D& pose,
                               const rclcpp::Time& now);
  ActionExecutionUpdate cancel(
      ActionExecutionStatus status = ActionExecutionStatus::Cancelled) noexcept;

  ActionExecutionStatus status() const noexcept { return status_; }
  bool executing() const noexcept {
    return status_ == ActionExecutionStatus::Executing;
  }
  const domain::VelocityCommand& command() const noexcept { return command_; }

 private:
  ActionExecutionUpdate terminal(ActionExecutionStatus status) noexcept;
  double timeoutSeconds() const noexcept;
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

std::string_view toString(ActionExecutionStatus status) noexcept;

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_COMMAND_EXECUTOR_HPP
