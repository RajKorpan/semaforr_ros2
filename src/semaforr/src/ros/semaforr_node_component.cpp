#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hunav_msgs/msg/agents.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/qos.hpp>
#include <semaforr/ros/parameter_configuration.hpp>
#include <semaforr/ros/command_executor.hpp>
#include <semaforr/ros/navigation_engine_adapter.hpp>
#include <semaforr/ros/semaforr_node.hpp>
#include <semaforr/validation/allocation_probe.hpp>
#include <semaforr/ros/sensor_synchronizer.hpp>
#include <semaforr/ros/social_observation_buffer.hpp>
#include <semaforr/ros/visualization_publisher.hpp>
#include <semaforr_msgs/msg/navigation_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <social_context_msgs/msg/formation_group_array.hpp>
#include <social_context_msgs/msg/tracked_person_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

namespace semaforr::ros {
namespace {

struct QosConfiguration {
  std::size_t depth{10U};
  std::string reliability{"reliable"};
  std::string durability{"volatile"};
};

struct RuntimeConfiguration {
  std::string pose_topic{"pose"};
  std::string scan_topic{"scan_raw"};
  std::string command_topic{"cmd_vel"};
  std::string state_topic{"navigation_state"};
  std::string decision_topic{"decision_records"};
  std::string tracked_people_topic{"/human_poses_3d_tracked_global"};
  std::string tracked_predictions_topic{"/pedestrian_predictions_tracked"};
  std::string hunav_agents_topic{"/human_states"};
  std::string hunav_predictions_topic{"/pedestrian_predictions"};
  std::string formations_topic{"/formation_groups"};
  QosConfiguration sensor_qos;
  QosConfiguration command_qos{1U, "reliable", "volatile"};
  SensorSynchronizerConfiguration sensors;
  SocialObservationConfiguration social;
  bool social_observations_enabled{true};
  bool formations_enabled{true};
  CommandExecutorConfiguration commands;
  double control_rate_hz{30.0};
  double transform_timeout_s{0.05};
};

std::string requireNonEmpty(std::string value, std::string_view name) {
  if (value.empty()) {
    throw std::runtime_error(std::string(name) + " must not be empty");
  }
  return value;
}

void requirePositive(double value, std::string_view name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(std::string(name) +
                             " must be finite and positive");
  }
}

void declareRuntimeParameters(rclcpp::Node& node) {
  node.declare_parameter("topics.pose", std::string{"pose"});
  node.declare_parameter("topics.scan", std::string{"scan_raw"});
  node.declare_parameter("topics.command", std::string{"cmd_vel"});
  node.declare_parameter("topics.navigation_state",
                         std::string{"navigation_state"});
  node.declare_parameter("topics.decision_records",
                         std::string{"decision_records"});
  node.declare_parameter("topics.crowd_density", std::string{"crowd_density"});
  node.declare_parameter("topics.crowd_risk", std::string{"crowd_risk"});
  node.declare_parameter("topics.crowd_flow", std::string{"crowd_flow"});
  node.declare_parameter("topics.crowd_people", std::string{"crowd_people"});
  node.declare_parameter("topics.crowd_predictions",
                         std::string{"crowd_predictions"});
  node.declare_parameter("topics.crowd_formations",
                         std::string{"crowd_formations"});
  node.declare_parameter("social.input.tracked_people_topic",
                         std::string{"/human_poses_3d_tracked_global"});
  node.declare_parameter("social.input.tracked_predictions_topic",
                         std::string{"/pedestrian_predictions_tracked"});
  node.declare_parameter("social.input.hunav_agents_topic",
                         std::string{"/human_states"});
  node.declare_parameter("social.input.hunav_predictions_topic",
                         std::string{"/pedestrian_predictions"});
  node.declare_parameter("social.input.formations_topic",
                         std::string{"/formation_groups"});

  node.declare_parameter("qos.sensors.depth", 10);
  node.declare_parameter("qos.sensors.reliability", std::string{"reliable"});
  node.declare_parameter("qos.sensors.durability", std::string{"volatile"});
  node.declare_parameter("qos.command.depth", 1);
  node.declare_parameter("qos.command.reliability", std::string{"reliable"});
  node.declare_parameter("qos.command.durability", std::string{"volatile"});

  node.declare_parameter("frames.global", std::string{"map"});
  node.declare_parameter("frames.scan", std::string{"base_laser_link"});
  node.declare_parameter("frames.transform_timeout_s", 0.05);

  node.declare_parameter("timing.control_rate_hz", 30.0);
  node.declare_parameter("timing.sensor_timeout_s", 0.5);
  node.declare_parameter("timing.sensor_sync_tolerance_s", 0.1);
  node.declare_parameter("social.input.mode", std::string{"tracked"});
  node.declare_parameter("social.input.coordinate_frame", std::string{"map"});
  node.declare_parameter("social.input.current_maximum_age_s", 0.75);
  node.declare_parameter("social.input.prediction_maximum_age_s", 6.0);
  node.declare_parameter("social.input.minimum_confidence", 0.25);
  node.declare_parameter("social.input.prediction_step_s", 1.0);
  node.declare_parameter("social.input.prediction_steps", 5);
  node.declare_parameter("social.input.fallback_prediction",
                         std::string{"constant_velocity"});
  node.declare_parameter("social.input.history_step_s", 0.1);
  node.declare_parameter("social.input.default_position_variance", 0.09);
  node.declare_parameter("social.input.minimum_covariance_confidence", 0.05);
  node.declare_parameter("social.input.hunav_confidence", 1.0);
  node.declare_parameter("social.formations.enabled", true);
  node.declare_parameter("social.formations.maximum_age_s", 1.0);
  node.declare_parameter("social.formations.minimum_confidence", 0.5);
  node.declare_parameter("social.visualizations.enabled", true);
  node.declare_parameter("social.learning.enabled", true);
  node.declare_parameter("social.learning.estimator",
                         std::string{"count_exposure"});
  node.declare_parameter("social.learning.resolution_m", 1.0);
  node.declare_parameter("social.learning.origin_x_m", 0.0);
  node.declare_parameter("social.learning.origin_y_m", 0.0);
  node.declare_parameter("social.learning.discount_factor", 0.7);
  node.declare_parameter("social.learning.minimum_update_period_s", 1.0);
  node.declare_parameter("social.learning.encounter_radius_m", 1.0);
  node.declare_parameter("social.learning.minimum_flow_speed_mps", 0.05);
  node.declare_parameter("social.learning.confidence_exposures", 10.0);
  node.declare_parameter("social.learning.cusum_increase", 4.0);
  node.declare_parameter("social.learning.cusum_decrease", -3.0);
  node.declare_parameter("social.learning.cusum_threshold", 10.0);
  node.declare_parameter("social.learning.random_seed", 0);

  node.declare_parameter("command.linear_velocity_mps", 0.5);
  node.declare_parameter("command.angular_velocity_radps", 0.5);
  node.declare_parameter("command.turn_linear_velocity_mps", 0.01);
  node.declare_parameter("command.maximum_linear_velocity_mps", 0.5);
  node.declare_parameter("command.maximum_angular_velocity_radps", 0.5);
  node.declare_parameter("command.maximum_linear_acceleration_mps2", 1.0);
  node.declare_parameter("command.maximum_angular_acceleration_radps2", 1.0);
  node.declare_parameter("command.distance_tolerance_m", 0.06);
  node.declare_parameter("command.angle_tolerance_rad", 0.11);
  node.declare_parameter("command.timeout_multiplier", 1.5);
  node.declare_parameter("command.minimum_timeout_s", 0.1);
  node.declare_parameter("command.odometry_reset_distance_m", 2.0);
  node.declare_parameter("command.odometry_reset_angle_rad", 2.8);
}

QosConfiguration readQos(rclcpp::Node& node, const std::string& prefix) {
  const auto depth = node.get_parameter(prefix + ".depth").as_int();
  if (depth <= 0) {
    throw std::runtime_error(prefix + ".depth must be positive");
  }
  return {static_cast<std::size_t>(depth),
          node.get_parameter(prefix + ".reliability").as_string(),
          node.get_parameter(prefix + ".durability").as_string()};
}

rclcpp::QoS makeQos(const QosConfiguration& configuration) {
  rclcpp::QoS qos(rclcpp::KeepLast(configuration.depth));
  if (configuration.reliability == "reliable") {
    qos.reliable();
  } else if (configuration.reliability == "best_effort") {
    qos.best_effort();
  } else {
    throw std::runtime_error(
        "QoS reliability must be 'reliable' or 'best_effort'");
  }
  if (configuration.durability == "volatile") {
    qos.durability_volatile();
  } else if (configuration.durability == "transient_local") {
    qos.transient_local();
  } else {
    throw std::runtime_error(
        "QoS durability must be 'volatile' or 'transient_local'");
  }
  return qos;
}

RuntimeConfiguration readRuntimeConfiguration(rclcpp::Node& node) {
  RuntimeConfiguration configuration;
  configuration.pose_topic = requireNonEmpty(
      node.get_parameter("topics.pose").as_string(), "topics.pose");
  configuration.scan_topic = requireNonEmpty(
      node.get_parameter("topics.scan").as_string(), "topics.scan");
  configuration.command_topic = requireNonEmpty(
      node.get_parameter("topics.command").as_string(), "topics.command");
  configuration.state_topic =
      requireNonEmpty(node.get_parameter("topics.navigation_state").as_string(),
                      "topics.navigation_state");
  configuration.decision_topic =
      requireNonEmpty(node.get_parameter("topics.decision_records").as_string(),
                      "topics.decision_records");
  configuration.tracked_people_topic =
      node.get_parameter("social.input.tracked_people_topic").as_string();
  configuration.tracked_predictions_topic =
      node.get_parameter("social.input.tracked_predictions_topic").as_string();
  configuration.hunav_agents_topic =
      node.get_parameter("social.input.hunav_agents_topic").as_string();
  configuration.hunav_predictions_topic =
      node.get_parameter("social.input.hunav_predictions_topic").as_string();
  configuration.formations_topic =
      node.get_parameter("social.input.formations_topic").as_string();
  configuration.sensor_qos = readQos(node, "qos.sensors");
  configuration.command_qos = readQos(node, "qos.command");
  configuration.sensors.pose_frame = requireNonEmpty(
      node.get_parameter("frames.global").as_string(), "frames.global");
  configuration.sensors.scan_frame = requireNonEmpty(
      node.get_parameter("frames.scan").as_string(), "frames.scan");
  configuration.social.frame =
      node.get_parameter("social.input.coordinate_frame").as_string();
  configuration.social.input_mode = socialInputModeFromString(
      node.get_parameter("social.input.mode").as_string());
  configuration.social_observations_enabled =
      node.get_parameter("social.enabled").as_bool() &&
      configuration.social.input_mode != SocialInputMode::None;
  configuration.formations_enabled =
      configuration.social_observations_enabled &&
      configuration.social.input_mode == SocialInputMode::Tracked &&
      node.get_parameter("social.formations.enabled").as_bool();
  if (configuration.social.frame.empty()) {
    if (configuration.social_observations_enabled)
      throw std::runtime_error(
          "social.input.coordinate_frame must not be empty for an active "
          "social input mode");
    configuration.social.frame = configuration.sensors.pose_frame;
  }
  if (configuration.social_observations_enabled) {
    if (configuration.social.input_mode == SocialInputMode::Tracked) {
      configuration.tracked_people_topic = requireNonEmpty(
          configuration.tracked_people_topic,
          "social.input.tracked_people_topic");
      configuration.tracked_predictions_topic = requireNonEmpty(
          configuration.tracked_predictions_topic,
          "social.input.tracked_predictions_topic");
      if (configuration.formations_enabled)
        configuration.formations_topic = requireNonEmpty(
            configuration.formations_topic,
            "social.input.formations_topic");
    } else {
      configuration.hunav_agents_topic = requireNonEmpty(
          configuration.hunav_agents_topic,
          "social.input.hunav_agents_topic");
      configuration.hunav_predictions_topic = requireNonEmpty(
          configuration.hunav_predictions_topic,
          "social.input.hunav_predictions_topic");
    }
  }
  configuration.social.current_maximum_age_s = node
      .get_parameter("social.input.current_maximum_age_s")
      .as_double();
  configuration.social.prediction_maximum_age_s = node
      .get_parameter("social.input.prediction_maximum_age_s")
      .as_double();
  configuration.social.formation_maximum_age_s =
      node.get_parameter("social.formations.maximum_age_s").as_double();
  configuration.social.minimum_confidence =
      node.get_parameter("social.input.minimum_confidence").as_double();
  configuration.social.minimum_formation_confidence =
      node.get_parameter("social.formations.minimum_confidence").as_double();
  configuration.social.prediction_step_s =
      node.get_parameter("social.input.prediction_step_s").as_double();
  const auto prediction_steps =
      node.get_parameter("social.input.prediction_steps").as_int();
  if (prediction_steps <= 0) {
    throw std::runtime_error("social.input.prediction_steps must be positive");
  }
  configuration.social.prediction_steps =
      static_cast<std::size_t>(prediction_steps);
  const auto fallback =
      node.get_parameter("social.input.fallback_prediction").as_string();
  if (fallback == "constant_velocity") {
    configuration.social.constant_velocity_fallback = true;
  } else if (fallback == "none") {
    configuration.social.constant_velocity_fallback = false;
  } else {
    throw std::runtime_error(
        "social.input.fallback_prediction must be 'constant_velocity' or "
        "'none'");
  }
  configuration.social.adapter.history_step_s =
      node.get_parameter("social.input.history_step_s").as_double();
  configuration.social.adapter.default_position_variance =
      node.get_parameter("social.input.default_position_variance").as_double();
  configuration.social.adapter.minimum_covariance_confidence =
      node.get_parameter("social.input.minimum_covariance_confidence")
          .as_double();
  configuration.social.adapter.hunav_confidence =
      node.get_parameter("social.input.hunav_confidence").as_double();
  configuration.transform_timeout_s =
      node.get_parameter("frames.transform_timeout_s").as_double();
  configuration.control_rate_hz =
      node.get_parameter("timing.control_rate_hz").as_double();
  configuration.sensors.maximum_age_s =
      node.get_parameter("timing.sensor_timeout_s").as_double();
  configuration.sensors.maximum_skew_s =
      node.get_parameter("timing.sensor_sync_tolerance_s").as_double();

  configuration.commands.linear_velocity_mps =
      node.get_parameter("command.linear_velocity_mps").as_double();
  configuration.commands.angular_velocity_radps =
      node.get_parameter("command.angular_velocity_radps").as_double();
  configuration.commands.turn_linear_velocity_mps =
      node.get_parameter("command.turn_linear_velocity_mps").as_double();
  configuration.commands.maximum_linear_velocity_mps =
      node.get_parameter("command.maximum_linear_velocity_mps").as_double();
  configuration.commands.maximum_angular_velocity_radps =
      node.get_parameter("command.maximum_angular_velocity_radps").as_double();
  configuration.commands.maximum_linear_acceleration_mps2 =
      node.get_parameter("command.maximum_linear_acceleration_mps2")
          .as_double();
  configuration.commands.maximum_angular_acceleration_radps2 =
      node.get_parameter("command.maximum_angular_acceleration_radps2")
          .as_double();
  configuration.commands.maximum_move_action_index =
      node.get_parameter("actions.move_distances_m").as_double_array().size();
  configuration.commands.maximum_rotation_action_index =
      node.get_parameter("actions.rotation_angles_rad").as_double_array().size();
  configuration.commands.distance_tolerance_m =
      node.get_parameter("command.distance_tolerance_m").as_double();
  configuration.commands.angle_tolerance_rad =
      node.get_parameter("command.angle_tolerance_rad").as_double();
  configuration.commands.timeout_multiplier =
      node.get_parameter("command.timeout_multiplier").as_double();
  configuration.commands.minimum_timeout_s =
      node.get_parameter("command.minimum_timeout_s").as_double();
  configuration.commands.odometry_reset_distance_m =
      node.get_parameter("command.odometry_reset_distance_m").as_double();
  configuration.commands.odometry_reset_angle_rad =
      node.get_parameter("command.odometry_reset_angle_rad").as_double();

  requirePositive(configuration.transform_timeout_s,
                  "frames.transform_timeout_s");
  requirePositive(configuration.control_rate_hz, "timing.control_rate_hz");
  // Constructing these value objects performs the remainder of validation.
  (void)SensorSynchronizer(configuration.sensors);
  (void)SocialObservationBuffer(configuration.social);
  (void)CommandExecutor(configuration.commands);
  (void)makeQos(configuration.sensor_qos);
  (void)makeQos(configuration.command_qos);
  return configuration;
}

geometry_msgs::msg::Twist toRos(const domain::VelocityCommand& command) {
  geometry_msgs::msg::Twist message;
  message.linear.x = command.linear_mps;
  message.angular.z = command.angular_radps;
  return message;
}

bool isWaitingStatus(SensorStatus status) noexcept {
  return status == SensorStatus::WaitingForPose ||
         status == SensorStatus::WaitingForScan;
}

std::string_view actionName(domain::ActionType type) noexcept {
  switch (type) {
    case domain::ActionType::Forward:
      return "forward";
    case domain::ActionType::TurnRight:
      return "turn_right";
    case domain::ActionType::TurnLeft:
      return "turn_left";
    case domain::ActionType::Pause:
      return "pause";
  }
  return "unknown";
}

decision::ActionOutcome toOutcome(ActionExecutionStatus status) noexcept {
  switch (status) {
    case ActionExecutionStatus::Completed:
      return decision::ActionOutcome::Completed;
    case ActionExecutionStatus::TimedOut:
      return decision::ActionOutcome::TimedOut;
    case ActionExecutionStatus::OdometryReset:
      return decision::ActionOutcome::OdometryReset;
    case ActionExecutionStatus::ClockReset:
      return decision::ActionOutcome::ClockReset;
    case ActionExecutionStatus::Cancelled:
      return decision::ActionOutcome::Cancelled;
    case ActionExecutionStatus::SafetyInterrupted:
      return decision::ActionOutcome::SafetyInterrupted;
    case ActionExecutionStatus::ControllerRejected:
      return decision::ActionOutcome::ControllerRejected;
    case ActionExecutionStatus::ControllerFailure:
      return decision::ActionOutcome::ControllerFailure;
    case ActionExecutionStatus::GoalPreempted:
      return decision::ActionOutcome::GoalPreempted;
    case ActionExecutionStatus::NavigationModeTransition:
      return decision::ActionOutcome::NavigationModeTransition;
    case ActionExecutionStatus::SensorLost:
      return decision::ActionOutcome::SensorLost;
    case ActionExecutionStatus::Shutdown:
      return decision::ActionOutcome::Shutdown;
    case ActionExecutionStatus::Idle:
    case ActionExecutionStatus::Executing:
      return decision::ActionOutcome::Pending;
  }
  return decision::ActionOutcome::Cancelled;
}

domain::ExecutionCompletionStatus toExecutionStatus(
    ActionExecutionStatus status, decision::ActionOutcome outcome) noexcept {
  switch (outcome) {
    case decision::ActionOutcome::Completed:
      return domain::ExecutionCompletionStatus::Succeeded;
    case decision::ActionOutcome::TimedOut:
      return domain::ExecutionCompletionStatus::TimedOut;
    case decision::ActionOutcome::OdometryReset:
      return domain::ExecutionCompletionStatus::OdometryReset;
    case decision::ActionOutcome::ClockReset:
      return domain::ExecutionCompletionStatus::ClockReset;
    case decision::ActionOutcome::SensorLost:
      return domain::ExecutionCompletionStatus::SensorLost;
    case decision::ActionOutcome::Shutdown:
      return domain::ExecutionCompletionStatus::Shutdown;
    case decision::ActionOutcome::PartialMovement:
      return domain::ExecutionCompletionStatus::PartialMovement;
    case decision::ActionOutcome::NoMovement:
      return domain::ExecutionCompletionStatus::NoMovement;
    case decision::ActionOutcome::SafetyInterrupted:
      return domain::ExecutionCompletionStatus::SafetyInterrupted;
    case decision::ActionOutcome::ControllerRejected:
      return domain::ExecutionCompletionStatus::ControllerRejected;
    case decision::ActionOutcome::ControllerFailure:
      return domain::ExecutionCompletionStatus::ControllerFailure;
    case decision::ActionOutcome::GoalPreempted:
      return domain::ExecutionCompletionStatus::GoalPreempted;
    case decision::ActionOutcome::NavigationModeTransition:
      return domain::ExecutionCompletionStatus::NavigationModeTransition;
    case decision::ActionOutcome::Cancelled:
    case decision::ActionOutcome::Pending:
      break;
  }
  return status == ActionExecutionStatus::TimedOut
             ? domain::ExecutionCompletionStatus::TimedOut
             : domain::ExecutionCompletionStatus::Cancelled;
}

std::uint8_t toMessage(NavigationNodeState state) noexcept {
  switch (state) {
    case NavigationNodeState::WaitingForSensors:
      return semaforr_msgs::msg::NavigationState::WAITING_FOR_SENSORS;
    case NavigationNodeState::ReadyToDecide:
      return semaforr_msgs::msg::NavigationState::READY_TO_DECIDE;
    case NavigationNodeState::ExecutingAction:
      return semaforr_msgs::msg::NavigationState::EXECUTING_ACTION;
    case NavigationNodeState::Stopped:
      return semaforr_msgs::msg::NavigationState::STOPPED;
  }
  return semaforr_msgs::msg::NavigationState::STOPPED;
}

std::uint8_t toMessage(navigation::NavigationPhase phase) noexcept {
  switch (phase) {
    case navigation::NavigationPhase::InitialExploration:
      return semaforr_msgs::msg::NavigationState::PHASE_INITIAL_EXPLORATION;
    case navigation::NavigationPhase::TargetNavigation:
      return semaforr_msgs::msg::NavigationState::PHASE_TARGET_NAVIGATION;
    case navigation::NavigationPhase::MissionComplete:
      return semaforr_msgs::msg::NavigationState::PHASE_MISSION_COMPLETE;
  }
  return semaforr_msgs::msg::NavigationState::PHASE_MISSION_COMPLETE;
}

void rotatePositionCovariance(
    std::array<double, 4>& covariance,
    const geometry_msgs::msg::TransformStamped& transform) {
  const auto& quaternion = transform.transform.rotation;
  const double sin_yaw =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw =
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  const double yaw = std::atan2(sin_yaw, cos_yaw);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const auto source = covariance;
  covariance[0] = cosine * cosine * source[0] -
                  cosine * sine * (source[1] + source[2]) +
                  sine * sine * source[3];
  covariance[1] = cosine * sine * source[0] - sine * sine * source[2] +
                  cosine * cosine * source[1] - cosine * sine * source[3];
  covariance[2] = cosine * sine * source[0] + cosine * cosine * source[2] -
                  sine * sine * source[1] - cosine * sine * source[3];
  covariance[3] = sine * sine * source[0] +
                  cosine * sine * (source[1] + source[2]) +
                  cosine * cosine * source[3];
}

void transformCrowdObservation(
    domain::CrowdObservation& observation,
    const std_msgs::msg::Header& source_header,
    const geometry_msgs::msg::TransformStamped& transform,
    const std::string& target_frame) {
  for (auto& pedestrian : observation.pedestrians) {
    geometry_msgs::msg::PointStamped point;
    point.header = source_header;
    point.point.x = pedestrian.position.x_m;
    point.point.y = pedestrian.position.y_m;
    geometry_msgs::msg::PointStamped transformed_point;
    tf2::doTransform(point, transformed_point, transform);
    pedestrian.position = {transformed_point.point.x,
                           transformed_point.point.y};

    geometry_msgs::msg::Vector3Stamped velocity;
    velocity.header = source_header;
    velocity.vector.x = pedestrian.velocity_mps.x_m;
    velocity.vector.y = pedestrian.velocity_mps.y_m;
    geometry_msgs::msg::Vector3Stamped transformed_velocity;
    tf2::doTransform(velocity, transformed_velocity, transform);
    pedestrian.velocity_mps = {transformed_velocity.vector.x,
                               transformed_velocity.vector.y};
    rotatePositionCovariance(pedestrian.position_covariance, transform);
  }
  observation.frame_id = target_frame;
}

}  // namespace

class SemaFORRNode::Impl {
 public:
  explicit Impl(SemaFORRNode& node)
      : node_(node), transform_buffer_(node.get_clock()) {
    declareConfigurationParameters(node_);
    declareRuntimeParameters(node_);
    runtime_ = readRuntimeConfiguration(node_);

    config::Configuration navigation_configuration =
        configurationFromParameters(node_);
    if (navigation_configuration.navigation.crowd_learning.enabled &&
        !runtime_.social_observations_enabled) {
      throw std::runtime_error(
          "social.learning.enabled requires social.enabled=true and "
          "social.input.mode set to 'tracked' or 'hunav'");
    }
    if (navigation_configuration.navigation.crowd_learning.enabled &&
        runtime_.social.frame != runtime_.sensors.pose_frame) {
      throw std::runtime_error(
          "social.input.coordinate_frame must equal frames.global while "
          "social learning is enabled so robot, laser, and people evidence "
          "share one frame");
    }
    navigation_engine_ = std::make_unique<NavigationEngineAdapter>(
        std::move(navigation_configuration));
    for (const auto& diagnostic : navigation_engine_->startupDiagnostics())
      RCLCPP_INFO(node_.get_logger(), "SemaFORR startup: %s",
                  diagnostic.c_str());
    synchronizer_ = std::make_unique<SensorSynchronizer>(runtime_.sensors);
    social_buffer_ = std::make_unique<SocialObservationBuffer>(runtime_.social);
    executor_ = std::make_unique<CommandExecutor>(runtime_.commands);
    visualization_ = std::make_unique<VisualizationPublisher>(
        node_, navigation_engine_->worldModel());

    command_publisher_ = node_.create_publisher<geometry_msgs::msg::Twist>(
        runtime_.command_topic, makeQos(runtime_.command_qos));
    state_publisher_ =
        node_.create_publisher<semaforr_msgs::msg::NavigationState>(
            runtime_.state_topic, makeQos(runtime_.command_qos));
    const bool tracked =
        runtime_.social.input_mode == SocialInputMode::Tracked;
    const char* current_topic =
        !runtime_.social_observations_enabled
            ? "disabled"
            : (tracked ? runtime_.tracked_people_topic.c_str()
                       : runtime_.hunav_agents_topic.c_str());
    const char* prediction_topic =
        !runtime_.social_observations_enabled
            ? "disabled"
            : (tracked ? runtime_.tracked_predictions_topic.c_str()
                       : runtime_.hunav_predictions_topic.c_str());
    RCLCPP_INFO(
        node_.get_logger(),
        "Social boundary: enabled=%s mode=%s current=%s predictions=%s "
        "formations=%s",
        runtime_.social_observations_enabled ? "true" : "false",
        std::string(toString(runtime_.social.input_mode)).c_str(),
        current_topic, prediction_topic,
        runtime_.formations_enabled ? runtime_.formations_topic.c_str()
                                    : "disabled");
  }

  void start() {
    std::scoped_lock lock(mutex_);
    if (started_ || state_ == NavigationNodeState::Stopped) {
      return;
    }
    const auto node_shared = node_.shared_from_this();
    transform_listener_ = std::make_unique<tf2_ros::TransformListener>(
        transform_buffer_, node_shared, false);
    const rclcpp::QoS sensor_qos = makeQos(runtime_.sensor_qos);
    pose_subscription_ =
        node_.create_subscription<geometry_msgs::msg::PoseStamped>(
            runtime_.pose_topic, sensor_qos,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
              onPose(*message);
            });
    scan_subscription_ = node_.create_subscription<sensor_msgs::msg::LaserScan>(
        runtime_.scan_topic, sensor_qos,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
          onScan(*message);
        });
    if (runtime_.social_observations_enabled) {
      const bool tracked =
          runtime_.social.input_mode == SocialInputMode::Tracked;
      if (tracked) {
        tracked_people_subscription_ = node_.create_subscription<
            social_context_msgs::msg::TrackedPersonArray>(
            runtime_.tracked_people_topic, sensor_qos,
            [this](social_context_msgs::msg::TrackedPersonArray::ConstSharedPtr
                       message) { onTrackedPeople(*message); });
      } else {
        hunav_agents_subscription_ =
            node_.create_subscription<hunav_msgs::msg::Agents>(
                runtime_.hunav_agents_topic, sensor_qos,
                [this](hunav_msgs::msg::Agents::ConstSharedPtr message) {
                  onHunavAgents(*message);
                });
      }
      prediction_subscription_ =
          node_.create_subscription<geometry_msgs::msg::PoseStamped>(
              tracked ? runtime_.tracked_predictions_topic
                      : runtime_.hunav_predictions_topic,
              sensor_qos,
              [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
                onSocialPrediction(*message);
              });
      if (runtime_.formations_enabled) {
        formation_subscription_ = node_.create_subscription<
            social_context_msgs::msg::FormationGroupArray>(
            runtime_.formations_topic, sensor_qos,
            [this](social_context_msgs::msg::FormationGroupArray::ConstSharedPtr
                       message) { onFormations(*message); });
      }
    }
    timer_ = rclcpp::create_timer(
        node_shared, node_.get_clock(),
        rclcpp::Duration::from_seconds(1.0 / runtime_.control_rate_hz),
        [this]() {
          try {
            controlTick();
          } catch (const std::exception& error) {
            handleRuntimeError(error.what());
          } catch (...) {
            handleRuntimeError("unknown invariant failure");
          }
        });
    started_ = true;
    transition(NavigationNodeState::WaitingForSensors, "started");
  }

  void stop() {
    std::scoped_lock lock(mutex_);
    if (state_ == NavigationNodeState::Stopped) {
      return;
    }
    if (timer_) {
      timer_->cancel();
    }
    if (executor_ && pending_decision_) {
      const ActionExecutionUpdate update =
          executor_->cancel(ActionExecutionStatus::Shutdown);
      completeDecision(node_.now(), decision::ActionOutcome::Shutdown, update,
                       "shutdown");
    }
    publishZero(true);
    transition(NavigationNodeState::Stopped, "shutdown");
  }

  NavigationNodeState state() const noexcept {
    std::scoped_lock lock(mutex_);
    return state_;
  }

  std::string lastFailure() const {
    std::scoped_lock lock(mutex_);
    return last_failure_;
  }

 private:
  void handleRuntimeError(const std::string& detail) {
    std::scoped_lock lock(mutex_);
    RCLCPP_ERROR(node_.get_logger(), "Navigation invariant failed: %s",
                 detail.c_str());
    last_failure_ = "invariant_failure: " + detail;
    if (pending_decision_) {
      const ActionExecutionUpdate update =
          executor_->cancel(ActionExecutionStatus::SafetyInterrupted);
      completeDecision(node_.now(), decision::ActionOutcome::SafetyInterrupted,
                       update,
                       last_failure_);
    }
    publishZero(true);
    transition(NavigationNodeState::Stopped, last_failure_, true);
  }

  void onPose(const geometry_msgs::msg::PoseStamped& message) {
    std::scoped_lock lock(mutex_);
    if (state_ == NavigationNodeState::Stopped) {
      return;
    }
    const rclcpp::Time received_at = node_.now();
    if (message.header.frame_id == runtime_.sensors.pose_frame) {
      synchronizer_->acceptPose(message, received_at);
      return;
    }
    try {
      const auto normalized = transform_buffer_.transform(
          message, runtime_.sensors.pose_frame,
          tf2::durationFromSec(runtime_.transform_timeout_s));
      synchronizer_->acceptPose(normalized, received_at);
    } catch (const tf2::TransformException& error) {
      synchronizer_->acceptPose(message, received_at);
      last_failure_ = "pose transform unavailable from '" +
                      message.header.frame_id + "' to '" +
                      runtime_.sensors.pose_frame + "': " + error.what();
      RCLCPP_WARN(node_.get_logger(), "%s", last_failure_.c_str());
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan& message) {
    std::scoped_lock lock(mutex_);
    if (state_ != NavigationNodeState::Stopped) {
      synchronizer_->acceptScan(message, node_.now());
    }
  }

  bool normalizeSocialObservation(domain::CrowdObservation& observation,
                                  const std_msgs::msg::Header& header) {
    if (observation.frame_id == runtime_.social.frame) return true;
    try {
      const auto transform = transform_buffer_.lookupTransform(
          runtime_.social.frame, observation.frame_id,
          rclcpp::Time(header.stamp, node_.get_clock()->get_clock_type()),
          tf2::durationFromSec(runtime_.transform_timeout_s));
      transformCrowdObservation(observation, header, transform,
                                runtime_.social.frame);
      return true;
    } catch (const tf2::TransformException& error) {
      last_social_failure_ = "social transform unavailable from '" +
                             observation.frame_id + "' to '" +
                             runtime_.social.frame + "': " + error.what();
      RCLCPP_WARN(node_.get_logger(), "%s", last_social_failure_.c_str());
      return false;
    }
  }

  void finishCurrentSocialObservation(domain::CrowdObservation observation,
                                      const std_msgs::msg::Header& header,
                                      const rclcpp::Time& received_at) {
    if (!normalizeSocialObservation(observation, header)) return;
    if (!social_buffer_->accept(std::move(observation), received_at)) {
      last_social_failure_ =
          "social_" +
          std::string(toString(social_buffer_->status(received_at)));
      RCLCPP_WARN(node_.get_logger(), "%s", last_social_failure_.c_str());
      return;
    }
    if (const auto current = social_buffer_->snapshot(received_at)) {
      crowd_state_.update(*current);
    }
    for (const auto& event : social_buffer_->takeLifecycleEvents()) {
      RCLCPP_INFO(node_.get_logger(), "social track %s: %s",
                  event.pedestrian_id.c_str(),
                  std::string(toString(event.type)).c_str());
    }
    last_social_failure_.clear();
  }

  void onTrackedPeople(
      const social_context_msgs::msg::TrackedPersonArray& message) {
    std::scoped_lock lock(mutex_);
    if (state_ == NavigationNodeState::Stopped) return;
    const rclcpp::Time received_at = node_.now();
    try {
      finishCurrentSocialObservation(
          trackedPeopleToDomain(message, received_at, runtime_.social.adapter),
          message.header, received_at);
    } catch (const std::exception& error) {
      last_social_failure_ = "social_tracked_invalid: " +
                             std::string(error.what());
      RCLCPP_WARN(node_.get_logger(), "%s", last_social_failure_.c_str());
    }
  }

  void onHunavAgents(const hunav_msgs::msg::Agents& message) {
    std::scoped_lock lock(mutex_);
    if (state_ == NavigationNodeState::Stopped) return;
    const rclcpp::Time received_at = node_.now();
    try {
      finishCurrentSocialObservation(
          hunavAgentsToDomain(message, received_at, runtime_.social.adapter),
          message.header, received_at);
    } catch (const std::exception& error) {
      last_social_failure_ = "social_hunav_invalid: " +
                             std::string(error.what());
      RCLCPP_WARN(node_.get_logger(), "%s", last_social_failure_.c_str());
    }
  }

  void onSocialPrediction(const geometry_msgs::msg::PoseStamped& message) {
    std::scoped_lock lock(mutex_);
    if (state_ != NavigationNodeState::Stopped &&
        !social_buffer_->acceptPrediction(message, node_.now())) {
      RCLCPP_DEBUG(node_.get_logger(),
                   "rejected malformed, duplicate, or out-of-range social "
                   "prediction '%s'",
                   message.header.frame_id.c_str());
    }
  }

  void onFormations(
      const social_context_msgs::msg::FormationGroupArray& message) {
    std::scoped_lock lock(mutex_);
    if (state_ == NavigationNodeState::Stopped) return;
    auto normalized = message;
    if (message.header.frame_id != runtime_.social.frame) {
      try {
        const auto transform = transform_buffer_.lookupTransform(
            runtime_.social.frame, message.header.frame_id,
            rclcpp::Time(message.header.stamp,
                         node_.get_clock()->get_clock_type()),
            tf2::durationFromSec(runtime_.transform_timeout_s));
        for (auto& group : normalized.groups) {
          geometry_msgs::msg::PointStamped source;
          source.header = message.header;
          source.point.x = group.center_x;
          source.point.y = group.center_y;
          geometry_msgs::msg::PointStamped target;
          tf2::doTransform(source, target, transform);
          group.center_x = static_cast<float>(target.point.x);
          group.center_y = static_cast<float>(target.point.y);
        }
        normalized.header.frame_id = runtime_.social.frame;
      } catch (const tf2::TransformException& error) {
        RCLCPP_WARN(node_.get_logger(), "formation transform unavailable: %s",
                    error.what());
        return;
      }
    }
    if (!social_buffer_->acceptFormations(normalized, node_.now())) {
      RCLCPP_DEBUG(node_.get_logger(), "rejected invalid social formations");
    }
  }

  void controlTick() {
    std::scoped_lock lock(mutex_);
    if (!started_ || state_ == NavigationNodeState::Stopped) {
      return;
    }
    const rclcpp::Time now = node_.now();
    const SensorStatus sensor_status = synchronizer_->status(now);
    const auto sensors = synchronizer_->snapshot(now);
    if (!sensors) {
      handleUnavailableSensors(sensor_status, now);
      return;
    }
    zero_latched_ = false;
    ever_had_sensors_ = true;

    switch (state_) {
      case NavigationNodeState::WaitingForSensors:
        updateNavigationEngine(*sensors, now);
        last_failure_.clear();
        transition(NavigationNodeState::ReadyToDecide, "sensors_ready");
        decide(*sensors, now);
        return;
      case NavigationNodeState::ReadyToDecide:
        decide(*sensors, now);
        return;
      case NavigationNodeState::ExecutingAction:
        execute(*sensors, now);
        return;
      case NavigationNodeState::Stopped:
        return;
    }
  }

  void handleUnavailableSensors(SensorStatus status, const rclcpp::Time& now) {
    if (status == SensorStatus::ClockReset) {
      synchronizer_->clear();
    }
    if (isWaitingStatus(status) && !ever_had_sensors_) {
      return;
    }

    if (state_ == NavigationNodeState::ExecutingAction) {
      const ActionExecutionUpdate update = executor_->cancel(
          status == SensorStatus::ClockReset
              ? ActionExecutionStatus::ClockReset
              : ActionExecutionStatus::SensorLost);
      const decision::ActionOutcome outcome =
          status == SensorStatus::ClockReset
              ? decision::ActionOutcome::ClockReset
              : decision::ActionOutcome::SensorLost;
      completeDecision(now, outcome, update,
                       "sensor_" + std::string(toString(status)));
    }
    last_failure_ = "sensor_" + std::string(toString(status));
    publishZero();
    transition(NavigationNodeState::WaitingForSensors, last_failure_, true);
  }

  void updateNavigationEngine(const SynchronizedSensors& sensors,
                              const rclcpp::Time& now) {
    domain::CrowdState effective_crowd = crowd_state_;
    if (const auto observation = social_buffer_->snapshot(now)) {
      effective_crowd.replaceCurrent(*observation);
      last_social_failure_.clear();
    } else {
      effective_crowd.clearCurrent();
      const auto status = social_buffer_->status(now);
      if (status != SocialObservationStatus::NoData) {
        const std::string failure = "social_" + std::string(toString(status));
        if (failure != last_social_failure_) {
          RCLCPP_WARN(node_.get_logger(), "%s", failure.c_str());
        }
        last_social_failure_ = failure;
      }
    }
    navigation_engine_->observe(sensors, effective_crowd);
    visualization_->publishSnapshot();
    last_observation_generation_ = sensors.generation;
  }

  void decide(const SynchronizedSensors& sensors, const rclcpp::Time& now) {
    if (navigation_engine_->missionComplete()) {
      publishZero();
      transition(NavigationNodeState::Stopped, "mission_complete");
      return;
    }

    const rclcpp::Time computation_started = node_.now();
    const auto allocations_before = validation::allocationSnapshot();
    pending_decision_ = navigation_engine_->decide();
    const auto allocation_delta = validation::allocationDifference(
        allocations_before, validation::allocationSnapshot());
    const rclcpp::Time computation_finished = node_.now();
    computation_time_s_ =
        std::max(0.0, (computation_finished - computation_started).seconds());
    pending_decision_->decision_latency_s = computation_time_s_;
    pending_decision_->allocation_count = allocation_delta.count;
    pending_decision_->allocation_bytes = allocation_delta.bytes;

    const ActionExecutionRequest request =
        navigation_engine_->executionRequest(*pending_decision_);
    pending_decision_->action_lifecycle_status = "selected";
    visualization_->publishDecision(*pending_decision_);

    ActionExecutionUpdate update;
    try {
      pending_decision_->action_lifecycle_status = "commanded";
      visualization_->publishDecision(*pending_decision_);
      update = executor_->start(request, sensors.pose, now);
    } catch (const std::exception& error) {
      update.status = ActionExecutionStatus::Cancelled;
      update.decision_id = request.decision_id;
      update.action_id = request.action_id;
      update.start_pose = sensors.pose;
      update.final_pose = sensors.pose;
      completeDecision(now, decision::ActionOutcome::ControllerRejected,
                       update, error.what());
      throw;
    }
    if (navigation_engine_->onActionStarted(update) !=
        domain::FeedbackDisposition::Accepted)
      throw std::runtime_error("navigation engine rejected action-start feedback");
    pending_decision_->action_progress = update.progress;
    pending_decision_->action_target = update.target;
    pending_decision_->action_lifecycle_status = "started";
    visualization_->publishDecision(*pending_decision_);
    action_started_at_ = now;
    publishCommand(update.command);
    transition(NavigationNodeState::ExecutingAction,
               std::string("action_") +
                   std::string(actionName(pending_decision_->action.type())));
  }

  void execute(const SynchronizedSensors& sensors, const rclcpp::Time& now) {
    const ActionExecutionUpdate update = executor_->update(sensors.pose, now);
    if (update.status == ActionExecutionStatus::Executing) {
      const auto disposition = navigation_engine_->onActionProgress(update);
      if (disposition != domain::FeedbackDisposition::Accepted)
        throw std::runtime_error(
            "navigation engine rejected action-progress feedback: " +
            std::string(domain::toString(disposition)));
      publishCommand(update.command);
      return;
    }

    const bool succeeded = update.status == ActionExecutionStatus::Completed;
    if (!succeeded) {
      last_failure_ = "action_" + std::string(toString(update.status));
      publishZero();
    } else {
      last_failure_.clear();
    }
    completeDecision(now, toOutcome(update.status), update,
                     succeeded ? "completed" : last_failure_);
    updateNavigationEngine(sensors, now);

    if (navigation_engine_->missionComplete()) {
      publishZero();
      transition(NavigationNodeState::Stopped, "mission_complete");
    } else {
      transition(NavigationNodeState::ReadyToDecide,
                 succeeded ? "action_completed" : last_failure_, !succeeded);
      if (succeeded) {
        decide(sensors, now);
      }
    }
  }

  void completeDecision(const rclcpp::Time& now,
                        decision::ActionOutcome outcome,
                        const ActionExecutionUpdate& update,
                        std::string detail) {
    if (!pending_decision_) {
      return;
    }
    pending_decision_->action_outcome = outcome;
    pending_decision_->action_progress = update.progress;
    pending_decision_->action_target = update.target;
    pending_decision_->action_duration_s =
        action_started_at_
            ? std::max(0.0, (now - *action_started_at_).seconds())
            : 0.0;
    pending_decision_->outcome_detail = std::move(detail);
    pending_decision_->action_lifecycle_status =
        outcome == decision::ActionOutcome::Completed
            ? "completed"
            : std::string(decision::toString(outcome));
    domain::ActionExecutionResult execution_trace;
    execution_trace.decision_id = update.decision_id;
    execution_trace.action_id = update.action_id;
    if (pending_decision_->task)
      execution_trace.task_id =
          domain::TaskId{pending_decision_->task->task_index};
    execution_trace.status = toExecutionStatus(update.status, outcome);
    execution_trace.start_pose = update.start_pose;
    execution_trace.final_pose = update.final_pose;
    execution_trace.distance_achieved_m = update.distance_achieved_m;
    execution_trace.rotation_achieved_rad = update.rotation_achieved_rad;
    execution_trace.timed_out =
        execution_trace.status ==
        domain::ExecutionCompletionStatus::TimedOut;
    execution_trace.cancellation_reason = pending_decision_->outcome_detail;
    execution_trace.safety_interruption =
        execution_trace.status ==
        domain::ExecutionCompletionStatus::SafetyInterrupted;
    execution_trace.controller_failure =
        execution_trace.status ==
            domain::ExecutionCompletionStatus::ControllerFailure ||
        execution_trace.status ==
            domain::ExecutionCompletionStatus::ControllerRejected;
    pending_decision_->execution_result = execution_trace;
    if (pending_decision_->plan_id &&
        outcome != decision::ActionOutcome::Completed) {
      ++pending_decision_->plan_revision;
      pending_decision_->plan_status = "stale";
      pending_decision_->plan_execution_events.push_back(
          "execution_invalidated_remaining_route:" +
          std::string(domain::toString(execution_trace.status)));
    }
    const auto disposition = navigation_engine_->onActionTerminal(
        update, toExecutionStatus(update.status, outcome),
        pending_decision_->outcome_detail);
    if (disposition != domain::FeedbackDisposition::Accepted) {
      RCLCPP_ERROR(node_.get_logger(),
                   "Navigation engine rejected terminal feedback: %s",
                   std::string(domain::toString(disposition)).c_str());
      last_failure_ = "execution_feedback_" +
                      std::string(domain::toString(disposition));
    }
    visualization_->publishDecision(*pending_decision_);
    pending_decision_.reset();
    action_started_at_.reset();
  }

  void publishCommand(const domain::VelocityCommand& command) {
    if (!command.finite() ||
        std::abs(command.linear_mps) >
            runtime_.commands.maximum_linear_velocity_mps ||
        std::abs(command.angular_radps) >
            runtime_.commands.maximum_angular_velocity_radps)
      throw std::runtime_error(
          "command executor produced a non-finite or out-of-bounds command");
    command_publisher_->publish(toRos(command));
    zero_latched_ = command.linear_mps == 0.0 && command.angular_radps == 0.0;
  }

  void publishZero(bool force = false) {
    if ((force || !zero_latched_) && command_publisher_) {
      command_publisher_->publish(geometry_msgs::msg::Twist{});
      zero_latched_ = true;
    }
  }

  void transition(NavigationNodeState next, const std::string& detail,
                  bool failure = false) {
    if (state_ == next && last_transition_detail_ == detail) {
      return;
    }
    state_ = next;
    last_transition_detail_ = detail;
    semaforr_msgs::msg::NavigationState message;
    message.header.stamp = node_.now();
    message.header.frame_id = runtime_.sensors.pose_frame;
    message.transition_sequence = ++transition_sequence_;
    message.state = toMessage(state_);
    message.navigation_phase =
        navigation_engine_
            ? toMessage(navigation_engine_->phase())
            : semaforr_msgs::msg::NavigationState::PHASE_TARGET_NAVIGATION;
    message.detail = detail;
    message.failure = failure;
    if (state_publisher_) {
      state_publisher_->publish(message);
    }
    if (failure) {
      RCLCPP_WARN(node_.get_logger(), "Navigation state: %s (%s)",
                  std::string(toString(state_)).c_str(), detail.c_str());
    } else {
      RCLCPP_INFO(node_.get_logger(), "Navigation state: %s (%s)",
                  std::string(toString(state_)).c_str(), detail.c_str());
    }
  }

  SemaFORRNode& node_;
  mutable std::mutex mutex_;
  RuntimeConfiguration runtime_;
  std::unique_ptr<NavigationEngineAdapter> navigation_engine_;
  std::unique_ptr<SensorSynchronizer> synchronizer_;
  std::unique_ptr<SocialObservationBuffer> social_buffer_;
  std::unique_ptr<CommandExecutor> executor_;
  std::unique_ptr<VisualizationPublisher> visualization_;

  tf2_ros::Buffer transform_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Subscription<social_context_msgs::msg::TrackedPersonArray>::SharedPtr
      tracked_people_subscription_;
  rclcpp::Subscription<hunav_msgs::msg::Agents>::SharedPtr
      hunav_agents_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      prediction_subscription_;
  rclcpp::Subscription<
      social_context_msgs::msg::FormationGroupArray>::SharedPtr
      formation_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<semaforr_msgs::msg::NavigationState>::SharedPtr
      state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  domain::CrowdState crowd_state_;
  std::optional<decision::DecisionResult> pending_decision_;
  std::optional<rclcpp::Time> action_started_at_;
  double computation_time_s_{0.0};
  std::size_t last_observation_generation_{0U};
  NavigationNodeState state_{NavigationNodeState::WaitingForSensors};
  std::string last_failure_;
  std::string last_social_failure_;
  std::string last_transition_detail_;
  bool started_{false};
  bool ever_had_sensors_{false};
  bool zero_latched_{true};
  std::uint64_t transition_sequence_{0U};
};

std::string_view toString(NavigationNodeState state) noexcept {
  switch (state) {
    case NavigationNodeState::WaitingForSensors:
      return "WaitingForSensors";
    case NavigationNodeState::ReadyToDecide:
      return "ReadyToDecide";
    case NavigationNodeState::ExecutingAction:
      return "ExecutingAction";
    case NavigationNodeState::Stopped:
      return "Stopped";
  }
  return "Unknown";
}

SemaFORRNode::SemaFORRNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("semaforr", options), impl_(std::make_unique<Impl>(*this)) {}

SemaFORRNode::~SemaFORRNode() { impl_->stop(); }

void SemaFORRNode::start() { impl_->start(); }

void SemaFORRNode::stop() { impl_->stop(); }

NavigationNodeState SemaFORRNode::state() const noexcept {
  return impl_->state();
}

std::string SemaFORRNode::lastFailure() const { return impl_->lastFailure(); }

}  // namespace semaforr::ros
