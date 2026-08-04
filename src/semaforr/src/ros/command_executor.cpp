#include <algorithm>
#include <cmath>
#include <semaforr/ros/command_executor.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace semaforr::ros {
namespace {

void requirePositiveFinite(double value, std::string_view name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and positive");
  }
}

domain::VelocityCommand commandFor(
    const domain::Action& action,
    const CommandExecutorConfiguration& configuration) {
  switch (action.type()) {
    case domain::ActionType::Forward:
      return {configuration.linear_velocity_mps, 0.0};
    case domain::ActionType::TurnRight:
      return {configuration.turn_linear_velocity_mps,
              -configuration.angular_velocity_radps};
    case domain::ActionType::TurnLeft:
      return {configuration.turn_linear_velocity_mps,
              configuration.angular_velocity_radps};
    case domain::ActionType::Pause:
      return {};
  }
  return {};
}

double approach(double current, double target, double maximum_delta) {
  return current +
         std::clamp(target - current, -maximum_delta, maximum_delta);
}

}  // namespace

CommandExecutor::CommandExecutor(CommandExecutorConfiguration configuration)
    : configuration_(std::move(configuration)) {
  requirePositiveFinite(configuration_.linear_velocity_mps, "linear velocity");
  requirePositiveFinite(configuration_.angular_velocity_radps,
                        "angular velocity");
  if (!std::isfinite(configuration_.turn_linear_velocity_mps) ||
      configuration_.turn_linear_velocity_mps < 0.0) {
    throw std::invalid_argument(
        "turn linear velocity must be finite and non-negative");
  }
  requirePositiveFinite(configuration_.maximum_linear_velocity_mps,
                        "maximum linear velocity");
  requirePositiveFinite(configuration_.maximum_angular_velocity_radps,
                        "maximum angular velocity");
  requirePositiveFinite(configuration_.maximum_linear_acceleration_mps2,
                        "maximum linear acceleration");
  requirePositiveFinite(configuration_.maximum_angular_acceleration_radps2,
                        "maximum angular acceleration");
  if (configuration_.linear_velocity_mps >
          configuration_.maximum_linear_velocity_mps ||
      configuration_.turn_linear_velocity_mps >
          configuration_.maximum_linear_velocity_mps ||
      configuration_.angular_velocity_radps >
          configuration_.maximum_angular_velocity_radps)
    throw std::invalid_argument(
        "command velocities must not exceed configured platform bounds");
  requirePositiveFinite(configuration_.distance_tolerance_m,
                        "distance tolerance");
  requirePositiveFinite(configuration_.angle_tolerance_rad, "angle tolerance");
  requirePositiveFinite(configuration_.timeout_multiplier,
                        "timeout multiplier");
  requirePositiveFinite(configuration_.minimum_timeout_s, "minimum timeout");
  requirePositiveFinite(configuration_.odometry_reset_distance_m,
                        "odometry reset distance");
  requirePositiveFinite(configuration_.odometry_reset_angle_rad,
                        "odometry reset angle");
}

ActionExecutionUpdate CommandExecutor::start(
    const ActionExecutionRequest& request, const domain::Pose2D& pose,
    const rclcpp::Time& now) {
  if (!pose.position.finite()) {
    throw std::invalid_argument("action start pose must be finite");
  }
  if (!std::isfinite(request.target_distance_m) ||
      request.target_distance_m < 0.0 ||
      !std::isfinite(request.target_angle_rad) ||
      request.target_angle_rad < 0.0) {
    throw std::invalid_argument(
        "action targets must be finite and non-negative");
  }
  const std::size_t magnitude = request.action.magnitude_index();
  if ((request.action.type() == domain::ActionType::Forward &&
       magnitude > configuration_.maximum_move_action_index) ||
      ((request.action.type() == domain::ActionType::TurnLeft ||
        request.action.type() == domain::ActionType::TurnRight) &&
       magnitude > configuration_.maximum_rotation_action_index))
    throw std::invalid_argument(
        "action magnitude index is outside the configured action space");
  if (request.action.type() == domain::ActionType::Forward &&
      request.target_distance_m <= 0.0) {
    throw std::invalid_argument(
        "forward action requires a positive distance target");
  }
  if ((request.action.type() == domain::ActionType::TurnLeft ||
       request.action.type() == domain::ActionType::TurnRight) &&
      request.target_angle_rad <= 0.0) {
    throw std::invalid_argument("turn action requires a positive angle target");
  }

  request_ = request;
  previous_pose_ = pose;
  started_at_ = now;
  command_updated_at_ = now;
  status_ = ActionExecutionStatus::Executing;
  progress_ = 0.0;
  command_ = {};
  return {status_, command_, progress_, target()};
}

ActionExecutionUpdate CommandExecutor::update(const domain::Pose2D& pose,
                                              const rclcpp::Time& now) {
  if (!executing() || !request_ || !previous_pose_ || !started_at_ ||
      !command_updated_at_) {
    return {status_, command_, progress_, target()};
  }
  if (now.get_clock_type() != started_at_->get_clock_type() ||
      now < *started_at_) {
    return terminal(ActionExecutionStatus::ClockReset);
  }

  const double translation =
      std::hypot(pose.position.x_m - previous_pose_->position.x_m,
                 pose.position.y_m - previous_pose_->position.y_m);
  const double rotation = std::fabs(domain::Angle::normalize(
      pose.heading.radians() - previous_pose_->heading.radians()));
  if (!std::isfinite(translation) || !std::isfinite(rotation) ||
      translation > configuration_.odometry_reset_distance_m ||
      rotation > configuration_.odometry_reset_angle_rad) {
    return terminal(ActionExecutionStatus::OdometryReset);
  }

  switch (request_->action.type()) {
    case domain::ActionType::Forward:
      progress_ += translation;
      break;
    case domain::ActionType::TurnRight:
    case domain::ActionType::TurnLeft:
      progress_ += rotation;
      break;
    case domain::ActionType::Pause:
      break;
  }
  previous_pose_ = pose;

  const double elapsed_s = (now - *started_at_).seconds();
  bool completed = false;
  switch (request_->action.type()) {
    case domain::ActionType::Forward:
      completed = progress_ + configuration_.distance_tolerance_m >=
                  request_->target_distance_m;
      break;
    case domain::ActionType::TurnRight:
    case domain::ActionType::TurnLeft: {
      const double tolerance = std::min(configuration_.angle_tolerance_rad,
                                        request_->target_angle_rad * 0.25);
      completed = progress_ + tolerance >= request_->target_angle_rad;
      break;
    }
    case domain::ActionType::Pause:
      completed = elapsed_s >= configuration_.minimum_timeout_s;
      break;
  }
  if (completed) {
    return terminal(ActionExecutionStatus::Completed);
  }
  if (elapsed_s >= timeoutSeconds()) {
    return terminal(ActionExecutionStatus::TimedOut);
  }
  const double command_elapsed_s = (now - *command_updated_at_).seconds();
  if (!std::isfinite(command_elapsed_s) || command_elapsed_s < 0.0)
    return terminal(ActionExecutionStatus::ClockReset);
  const auto desired = commandFor(request_->action, configuration_);
  command_.linear_mps =
      approach(command_.linear_mps, desired.linear_mps,
               configuration_.maximum_linear_acceleration_mps2 *
                   command_elapsed_s);
  command_.angular_radps =
      approach(command_.angular_radps, desired.angular_radps,
               configuration_.maximum_angular_acceleration_radps2 *
                   command_elapsed_s);
  command_updated_at_ = now;
  if (!command_.finite() ||
      std::abs(command_.linear_mps) >
          configuration_.maximum_linear_velocity_mps ||
      std::abs(command_.angular_radps) >
          configuration_.maximum_angular_velocity_radps)
    return terminal(ActionExecutionStatus::Cancelled);
  return {status_, command_, progress_, target()};
}

ActionExecutionUpdate CommandExecutor::cancel() noexcept {
  return terminal(ActionExecutionStatus::Cancelled);
}

ActionExecutionUpdate CommandExecutor::terminal(
    ActionExecutionStatus status) noexcept {
  status_ = status;
  command_ = {};
  return {status_, command_, progress_, target()};
}

double CommandExecutor::timeoutSeconds() const noexcept {
  if (!request_) {
    return configuration_.minimum_timeout_s;
  }
  double nominal_s = configuration_.minimum_timeout_s;
  switch (request_->action.type()) {
    case domain::ActionType::Forward:
      nominal_s =
          request_->target_distance_m / configuration_.linear_velocity_mps;
      nominal_s += configuration_.linear_velocity_mps /
                   configuration_.maximum_linear_acceleration_mps2;
      break;
    case domain::ActionType::TurnRight:
    case domain::ActionType::TurnLeft:
      nominal_s =
          request_->target_angle_rad / configuration_.angular_velocity_radps;
      nominal_s += configuration_.angular_velocity_radps /
                   configuration_.maximum_angular_acceleration_radps2;
      break;
    case domain::ActionType::Pause:
      break;
  }
  return std::max(configuration_.minimum_timeout_s,
                  nominal_s * configuration_.timeout_multiplier);
}

double CommandExecutor::target() const noexcept {
  if (!request_) {
    return 0.0;
  }
  return request_->action.type() == domain::ActionType::Forward
             ? request_->target_distance_m
             : request_->target_angle_rad;
}

std::string_view toString(ActionExecutionStatus status) noexcept {
  switch (status) {
    case ActionExecutionStatus::Idle:
      return "idle";
    case ActionExecutionStatus::Executing:
      return "executing";
    case ActionExecutionStatus::Completed:
      return "completed";
    case ActionExecutionStatus::TimedOut:
      return "timed_out";
    case ActionExecutionStatus::OdometryReset:
      return "odometry_reset";
    case ActionExecutionStatus::ClockReset:
      return "clock_reset";
    case ActionExecutionStatus::Cancelled:
      return "cancelled";
  }
  return "unknown";
}

}  // namespace semaforr::ros
