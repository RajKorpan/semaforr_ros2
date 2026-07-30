#ifndef SEMAFORR_ROS_COMMAND_EXECUTOR_HPP
#define SEMAFORR_ROS_COMMAND_EXECUTOR_HPP

#include <optional>
#include <rclcpp/time.hpp>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/observation.hpp>
#include <string_view>

namespace semaforr::ros {

struct CommandExecutorConfiguration {
  double linear_velocity_mps{0.5};
  double angular_velocity_radps{0.5};
  double turn_linear_velocity_mps{0.01};
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
};

enum class ActionExecutionStatus {
  Idle,
  Executing,
  Completed,
  TimedOut,
  OdometryReset,
  ClockReset,
  Cancelled
};

struct ActionExecutionUpdate {
  ActionExecutionStatus status{ActionExecutionStatus::Idle};
  domain::VelocityCommand command;
  double progress{0.0};
  double target{0.0};
};

class CommandExecutor {
 public:
  explicit CommandExecutor(CommandExecutorConfiguration configuration);

  ActionExecutionUpdate start(const ActionExecutionRequest& request,
                              const domain::Pose2D& pose,
                              const rclcpp::Time& now);
  ActionExecutionUpdate update(const domain::Pose2D& pose,
                               const rclcpp::Time& now);
  ActionExecutionUpdate cancel() noexcept;

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
  std::optional<rclcpp::Time> started_at_;
  ActionExecutionStatus status_{ActionExecutionStatus::Idle};
  domain::VelocityCommand command_;
  double progress_{0.0};
};

std::string_view toString(ActionExecutionStatus status) noexcept;

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_COMMAND_EXECUTOR_HPP
