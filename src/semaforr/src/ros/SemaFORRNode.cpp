#include <semaforr/ros/semaforr_node.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <semaforr_msgs/msg/navigation_state.hpp>
#include <social_context_msgs/msg/social_observation.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <semaforr/ros/ParameterConfiguration.hpp>
#include <semaforr/ros/command_executor.hpp>
#include <semaforr/ros/navigation_engine_adapter.hpp>
#include <semaforr/ros/sensor_synchronizer.hpp>
#include <semaforr/ros/social_observation_buffer.hpp>
#include <semaforr/ros/visualization_publisher.hpp>

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
  std::string social_topic{"social_observations"};
  QosConfiguration sensor_qos;
  QosConfiguration command_qos{1U, "reliable", "volatile"};
  SensorSynchronizerConfiguration sensors;
  SocialObservationConfiguration social;
  CommandExecutorConfiguration commands;
  double control_rate_hz{30.0};
  double transform_timeout_s{0.05};
};

std::string requireNonEmpty(std::string value, std::string_view name)
{
  if (value.empty()) {
    throw std::runtime_error(std::string(name) + " must not be empty");
  }
  return value;
}

void requirePositive(double value, std::string_view name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(
      std::string(name) + " must be finite and positive");
  }
}

void declareRuntimeParameters(rclcpp::Node& node)
{
  node.declare_parameter("topics.pose", std::string{"pose"});
  node.declare_parameter("topics.scan", std::string{"scan_raw"});
  node.declare_parameter("topics.command", std::string{"cmd_vel"});
  node.declare_parameter(
    "topics.navigation_state", std::string{"navigation_state"});
  node.declare_parameter(
    "topics.decision_records", std::string{"decision_records"});
  node.declare_parameter(
    "topics.social_observations", std::string{"social_observations"});
  node.declare_parameter(
    "topics.crowd_field", std::string{"crowd_field"});

  node.declare_parameter("qos.sensors.depth", 10);
  node.declare_parameter(
    "qos.sensors.reliability", std::string{"reliable"});
  node.declare_parameter(
    "qos.sensors.durability", std::string{"volatile"});
  node.declare_parameter("qos.command.depth", 1);
  node.declare_parameter(
    "qos.command.reliability", std::string{"reliable"});
  node.declare_parameter(
    "qos.command.durability", std::string{"volatile"});

  node.declare_parameter("frames.global", std::string{"map"});
  node.declare_parameter(
    "frames.scan", std::string{"base_laser_link"});
  node.declare_parameter("frames.transform_timeout_s", 0.05);

  node.declare_parameter("timing.control_rate_hz", 30.0);
  node.declare_parameter("timing.sensor_timeout_s", 0.5);
  node.declare_parameter("timing.sensor_sync_tolerance_s", 0.1);
  node.declare_parameter("social.maximum_age_s", 0.75);
  node.declare_parameter("social.minimum_confidence", 0.25);
  node.declare_parameter("social.learning.enabled", true);
  node.declare_parameter(
    "social.learning.estimator", std::string{"count_exposure"});
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
  node.declare_parameter("command.distance_tolerance_m", 0.06);
  node.declare_parameter("command.angle_tolerance_rad", 0.11);
  node.declare_parameter("command.timeout_multiplier", 1.5);
  node.declare_parameter("command.minimum_timeout_s", 0.1);
  node.declare_parameter("command.odometry_reset_distance_m", 2.0);
  node.declare_parameter("command.odometry_reset_angle_rad", 2.8);
}

QosConfiguration readQos(rclcpp::Node& node, const std::string& prefix)
{
  const auto depth = node.get_parameter(prefix + ".depth").as_int();
  if (depth <= 0) {
    throw std::runtime_error(prefix + ".depth must be positive");
  }
  return {
    static_cast<std::size_t>(depth),
    node.get_parameter(prefix + ".reliability").as_string(),
    node.get_parameter(prefix + ".durability").as_string()};
}

rclcpp::QoS makeQos(const QosConfiguration& configuration)
{
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

RuntimeConfiguration readRuntimeConfiguration(rclcpp::Node& node)
{
  RuntimeConfiguration configuration;
  configuration.pose_topic = requireNonEmpty(
    node.get_parameter("topics.pose").as_string(), "topics.pose");
  configuration.scan_topic = requireNonEmpty(
    node.get_parameter("topics.scan").as_string(), "topics.scan");
  configuration.command_topic = requireNonEmpty(
    node.get_parameter("topics.command").as_string(), "topics.command");
  configuration.state_topic = requireNonEmpty(
    node.get_parameter("topics.navigation_state").as_string(),
    "topics.navigation_state");
  configuration.decision_topic = requireNonEmpty(
    node.get_parameter("topics.decision_records").as_string(),
    "topics.decision_records");
  configuration.social_topic = requireNonEmpty(
    node.get_parameter("topics.social_observations").as_string(),
    "topics.social_observations");
  configuration.sensor_qos = readQos(node, "qos.sensors");
  configuration.command_qos = readQos(node, "qos.command");
  configuration.sensors.pose_frame = requireNonEmpty(
    node.get_parameter("frames.global").as_string(), "frames.global");
  configuration.sensors.scan_frame = requireNonEmpty(
    node.get_parameter("frames.scan").as_string(), "frames.scan");
  configuration.social.frame = configuration.sensors.pose_frame;
  configuration.social.maximum_age_s =
    node.get_parameter("social.maximum_age_s").as_double();
  configuration.social.minimum_confidence =
    node.get_parameter("social.minimum_confidence").as_double();
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

  requirePositive(
    configuration.transform_timeout_s, "frames.transform_timeout_s");
  requirePositive(
    configuration.control_rate_hz, "timing.control_rate_hz");
  // Constructing these value objects performs the remainder of validation.
  (void)SensorSynchronizer(configuration.sensors);
  (void)SocialObservationBuffer(configuration.social);
  (void)CommandExecutor(configuration.commands);
  (void)makeQos(configuration.sensor_qos);
  (void)makeQos(configuration.command_qos);
  return configuration;
}

geometry_msgs::msg::Twist toRos(const domain::VelocityCommand& command)
{
  geometry_msgs::msg::Twist message;
  message.linear.x = command.linear_mps;
  message.angular.z = command.angular_radps;
  return message;
}

domain::Pose2D toDomainPose(const Position& pose)
{
  return {
    {pose.getX(), pose.getY()},
    domain::Angle(pose.getTheta())};
}

bool isWaitingStatus(SensorStatus status) noexcept
{
  return status == SensorStatus::WaitingForPose ||
    status == SensorStatus::WaitingForScan;
}

std::string_view actionName(domain::ActionType type) noexcept
{
  switch (type) {
    case domain::ActionType::Forward: return "forward";
    case domain::ActionType::TurnRight: return "turn_right";
    case domain::ActionType::TurnLeft: return "turn_left";
    case domain::ActionType::Pause: return "pause";
  }
  return "unknown";
}

decision::ActionOutcome toOutcome(ActionExecutionStatus status) noexcept
{
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
    case ActionExecutionStatus::Idle:
    case ActionExecutionStatus::Executing:
      return decision::ActionOutcome::Pending;
  }
  return decision::ActionOutcome::Cancelled;
}

std::uint8_t toMessage(NavigationNodeState state) noexcept
{
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

void rotatePositionCovariance(
  social_context_msgs::msg::PedestrianObservation& pedestrian,
  const geometry_msgs::msg::TransformStamped& transform)
{
  const auto& quaternion = transform.transform.rotation;
  const double sin_yaw = 2.0 * (
    quaternion.w * quaternion.z +
    quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 * (
    quaternion.y * quaternion.y +
    quaternion.z * quaternion.z);
  const double yaw = std::atan2(sin_yaw, cos_yaw);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const auto covariance = pedestrian.position_covariance;
  pedestrian.position_covariance[0] =
    cosine * cosine * covariance[0] -
    cosine * sine * (covariance[1] + covariance[2]) +
    sine * sine * covariance[3];
  pedestrian.position_covariance[1] =
    cosine * sine * covariance[0] -
    sine * sine * covariance[2] +
    cosine * cosine * covariance[1] -
    cosine * sine * covariance[3];
  pedestrian.position_covariance[2] =
    cosine * sine * covariance[0] +
    cosine * cosine * covariance[2] -
    sine * sine * covariance[1] -
    cosine * sine * covariance[3];
  pedestrian.position_covariance[3] =
    sine * sine * covariance[0] +
    cosine * sine * (covariance[1] + covariance[2]) +
    cosine * cosine * covariance[3];
}

}  // namespace

class SemaFORRNode::Impl {
public:
  explicit Impl(SemaFORRNode& node)
    : node_(node),
      transform_buffer_(node.get_clock())
  {
    declareConfigurationParameters(node_);
    declareRuntimeParameters(node_);
    runtime_ = readRuntimeConfiguration(node_);

    config::Configuration controller_configuration =
      configurationFromParameters(node_);
    navigation_engine_ = std::make_unique<NavigationEngineAdapter>(
      std::move(controller_configuration));
    synchronizer_ =
      std::make_unique<SensorSynchronizer>(runtime_.sensors);
    social_buffer_ =
      std::make_unique<SocialObservationBuffer>(runtime_.social);
    executor_ = std::make_unique<CommandExecutor>(runtime_.commands);
    visualization_ =
      std::make_unique<VisualizationPublisher>(
        node_, navigation_engine_->visualizationModel());

    command_publisher_ =
      node_.create_publisher<geometry_msgs::msg::Twist>(
        runtime_.command_topic, makeQos(runtime_.command_qos));
    state_publisher_ =
      node_.create_publisher<semaforr_msgs::msg::NavigationState>(
        runtime_.state_topic, makeQos(runtime_.command_qos));
    mission_started_at_ = node_.now();
  }

  void start()
  {
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
        runtime_.pose_topic,
        sensor_qos,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          onPose(*message);
        });
    scan_subscription_ =
      node_.create_subscription<sensor_msgs::msg::LaserScan>(
        runtime_.scan_topic,
        sensor_qos,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
          onScan(*message);
        });
    social_subscription_ =
      node_.create_subscription<
        social_context_msgs::msg::SocialObservation>(
        runtime_.social_topic,
        sensor_qos,
        [this](
          social_context_msgs::msg::SocialObservation::ConstSharedPtr
            message) {
          onSocialObservation(*message);
        });
    timer_ = rclcpp::create_timer(
      node_shared,
      node_.get_clock(),
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

  void stop()
  {
    std::scoped_lock lock(mutex_);
    if (state_ == NavigationNodeState::Stopped) {
      return;
    }
    if (timer_) {
      timer_->cancel();
    }
    if (executor_ && pending_decision_) {
      const ActionExecutionUpdate update = executor_->cancel();
      completeDecision(
        node_.now(), decision::ActionOutcome::Shutdown, update, "shutdown");
    }
    publishZero(true);
    transition(NavigationNodeState::Stopped, "shutdown");
  }

  NavigationNodeState state() const noexcept
  {
    std::scoped_lock lock(mutex_);
    return state_;
  }

  std::string lastFailure() const
  {
    std::scoped_lock lock(mutex_);
    return last_failure_;
  }

private:
  void handleRuntimeError(const std::string& detail)
  {
    std::scoped_lock lock(mutex_);
    RCLCPP_ERROR(
      node_.get_logger(), "Navigation invariant failed: %s",
      detail.c_str());
    last_failure_ = "invariant_failure: " + detail;
    if (pending_decision_) {
      const ActionExecutionUpdate update = executor_->cancel();
      completeDecision(
        node_.now(),
        decision::ActionOutcome::Cancelled,
        update,
        last_failure_);
    }
    publishZero(true);
    transition(NavigationNodeState::Stopped, last_failure_, true);
  }

  void onPose(const geometry_msgs::msg::PoseStamped& message)
  {
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
        message,
        runtime_.sensors.pose_frame,
        tf2::durationFromSec(runtime_.transform_timeout_s));
      synchronizer_->acceptPose(normalized, received_at);
    } catch (const tf2::TransformException& error) {
      synchronizer_->acceptPose(message, received_at);
      last_failure_ =
        "pose transform unavailable from '" + message.header.frame_id +
        "' to '" + runtime_.sensors.pose_frame + "': " + error.what();
      RCLCPP_WARN(node_.get_logger(), "%s", last_failure_.c_str());
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan& message)
  {
    std::scoped_lock lock(mutex_);
    if (state_ != NavigationNodeState::Stopped) {
      synchronizer_->acceptScan(message, node_.now());
    }
  }

  void onSocialObservation(
    const social_context_msgs::msg::SocialObservation& message)
  {
    std::scoped_lock lock(mutex_);
    if (state_ == NavigationNodeState::Stopped) {
      return;
    }
    const rclcpp::Time received_at = node_.now();
    social_context_msgs::msg::SocialObservation normalized = message;
    if (message.header.frame_id != runtime_.social.frame) {
      try {
        const auto transform = transform_buffer_.lookupTransform(
          runtime_.social.frame,
          message.header.frame_id,
          rclcpp::Time(
            message.header.stamp, received_at.get_clock_type()),
          tf2::durationFromSec(runtime_.transform_timeout_s));
        normalized.header.frame_id = runtime_.social.frame;
        for (auto& pedestrian : normalized.pedestrians) {
          rotatePositionCovariance(pedestrian, transform);
          geometry_msgs::msg::PointStamped point;
          point.header = message.header;
          point.point = pedestrian.position;
          geometry_msgs::msg::PointStamped transformed_point;
          tf2::doTransform(point, transformed_point, transform);
          pedestrian.position = transformed_point.point;

          geometry_msgs::msg::Vector3Stamped velocity;
          velocity.header = message.header;
          velocity.vector = pedestrian.velocity;
          geometry_msgs::msg::Vector3Stamped transformed_velocity;
          tf2::doTransform(velocity, transformed_velocity, transform);
          pedestrian.velocity = transformed_velocity.vector;

          for (auto& prediction : pedestrian.predicted_positions) {
            point.point = prediction;
            tf2::doTransform(point, transformed_point, transform);
            prediction = transformed_point.point;
          }
        }
      } catch (const tf2::TransformException& error) {
        last_social_failure_ =
          "social transform unavailable from '" +
          message.header.frame_id + "' to '" + runtime_.social.frame +
          "': " + error.what();
        RCLCPP_WARN(
          node_.get_logger(), "%s", last_social_failure_.c_str());
        return;
      }
    }
    if (social_buffer_->accept(normalized, received_at)) {
      if (const auto observation = social_buffer_->snapshot(received_at)) {
        crowd_state_.update(*observation);
      }
      last_social_failure_.clear();
    } else {
      last_social_failure_ =
        "social_" +
        std::string(toString(social_buffer_->status(received_at)));
      RCLCPP_WARN(node_.get_logger(), "%s", last_social_failure_.c_str());
    }
  }

  void controlTick()
  {
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

  void handleUnavailableSensors(
    SensorStatus status,
    const rclcpp::Time& now)
  {
    if (status == SensorStatus::ClockReset) {
      synchronizer_->clear();
    }
    if (isWaitingStatus(status) && !ever_had_sensors_) {
      return;
    }

    if (state_ == NavigationNodeState::ExecutingAction) {
      const ActionExecutionUpdate update = executor_->cancel();
      const decision::ActionOutcome outcome =
        status == SensorStatus::ClockReset
        ? decision::ActionOutcome::ClockReset
        : decision::ActionOutcome::SensorLost;
      completeDecision(
        now, outcome, update,
        "sensor_" + std::string(toString(status)));
    }
    last_failure_ = "sensor_" + std::string(toString(status));
    publishZero();
    transition(
      NavigationNodeState::WaitingForSensors,
      last_failure_,
      true);
  }

  void updateNavigationEngine(
    const SynchronizedSensors& sensors,
    const rclcpp::Time& now)
  {
    domain::CrowdState effective_crowd = crowd_state_;
    if (const auto observation = social_buffer_->snapshot(now)) {
      effective_crowd.replaceCurrent(*observation);
      last_social_failure_.clear();
    } else {
      effective_crowd.clearCurrent();
      const auto status = social_buffer_->status(now);
      if (status != SocialObservationStatus::NoData) {
        const std::string failure =
          "social_" + std::string(toString(status));
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

  void decide(
    const SynchronizedSensors& sensors,
    const rclcpp::Time& now)
  {
    if (navigation_engine_->missionComplete()) {
      publishZero();
      transition(NavigationNodeState::Stopped, "mission_complete");
      return;
    }

    const rclcpp::Time computation_started = node_.now();
    pending_decision_ = navigation_engine_->decide();
    const rclcpp::Time computation_finished = node_.now();
    computation_time_s_ =
      std::max(0.0, (computation_finished - computation_started).seconds());
    pending_decision_->decision_latency_s = computation_time_s_;

    const domain::Action& action = pending_decision_->action;
    const ActionExecutionRequest request =
      navigation_engine_->executionRequest(action);

    const ActionExecutionUpdate update =
      executor_->start(request, toDomainPose(sensors.pose), now);
    pending_decision_->action_progress = update.progress;
    pending_decision_->action_target = update.target;
    action_started_at_ = now;
    publishCommand(update.command);
    transition(
      NavigationNodeState::ExecutingAction,
      std::string("action_") +
        std::string(actionName(action.type())));
  }

  void execute(
    const SynchronizedSensors& sensors,
    const rclcpp::Time& now)
  {
    const ActionExecutionUpdate update =
      executor_->update(toDomainPose(sensors.pose), now);
    if (update.status == ActionExecutionStatus::Executing) {
      publishCommand(update.command);
      return;
    }

    const bool succeeded =
      update.status == ActionExecutionStatus::Completed;
    if (!succeeded) {
      last_failure_ = "action_" + std::string(toString(update.status));
      publishZero();
    } else {
      last_failure_.clear();
    }
    completeDecision(
      now,
      toOutcome(update.status),
      update,
      succeeded ? "completed" : last_failure_);
    updateNavigationEngine(sensors, now);

    if (navigation_engine_->missionComplete()) {
      publishZero();
      transition(NavigationNodeState::Stopped, "mission_complete");
    } else {
      transition(
        NavigationNodeState::ReadyToDecide,
        succeeded ? "action_completed" : last_failure_,
        !succeeded);
      if (succeeded) {
        decide(sensors, now);
      }
    }
  }

  void completeDecision(
    const rclcpp::Time& now,
    decision::ActionOutcome outcome,
    const ActionExecutionUpdate& update,
    std::string detail)
  {
    if (!pending_decision_) {
      return;
    }
    pending_decision_->action_outcome = outcome;
    pending_decision_->action_progress = update.progress;
    pending_decision_->action_target = update.target;
    pending_decision_->action_duration_s = action_started_at_
      ? std::max(0.0, (now - *action_started_at_).seconds())
      : 0.0;
    pending_decision_->outcome_detail = std::move(detail);
    visualization_->publishDecision(*pending_decision_);
    const double mission_time_s =
      std::max(0.0, (now - mission_started_at_).seconds());
    navigation_engine_->markDecisionComplete(mission_time_s);
    pending_decision_.reset();
    action_started_at_.reset();
  }

  void publishCommand(const domain::VelocityCommand& command)
  {
    command_publisher_->publish(toRos(command));
    zero_latched_ =
      command.linear_mps == 0.0 && command.angular_radps == 0.0;
  }

  void publishZero(bool force = false)
  {
    if ((force || !zero_latched_) && command_publisher_) {
      command_publisher_->publish(geometry_msgs::msg::Twist{});
      zero_latched_ = true;
    }
  }

  void transition(
    NavigationNodeState next,
    const std::string& detail,
    bool failure = false)
  {
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
    message.detail = detail;
    message.failure = failure;
    if (state_publisher_) {
      state_publisher_->publish(message);
    }
    if (failure) {
      RCLCPP_WARN(
        node_.get_logger(), "Navigation state: %s (%s)",
        std::string(toString(state_)).c_str(), detail.c_str());
    } else {
      RCLCPP_INFO(
        node_.get_logger(), "Navigation state: %s (%s)",
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
  rclcpp::Subscription<
    social_context_msgs::msg::SocialObservation>::SharedPtr
    social_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    command_publisher_;
  rclcpp::Publisher<semaforr_msgs::msg::NavigationState>::SharedPtr
    state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  domain::CrowdState crowd_state_;
  std::optional<decision::DecisionResult> pending_decision_;
  std::optional<rclcpp::Time> action_started_at_;
  rclcpp::Time mission_started_at_;
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

std::string_view toString(NavigationNodeState state) noexcept
{
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
  : rclcpp::Node("semaforr", options),
    impl_(std::make_unique<Impl>(*this))
{
}

SemaFORRNode::~SemaFORRNode()
{
  impl_->stop();
}

void SemaFORRNode::start()
{
  impl_->start();
}

void SemaFORRNode::stop()
{
  impl_->stop();
}

NavigationNodeState SemaFORRNode::state() const noexcept
{
  return impl_->state();
}

std::string SemaFORRNode::lastFailure() const
{
  return impl_->lastFailure();
}

}  // namespace semaforr::ros
