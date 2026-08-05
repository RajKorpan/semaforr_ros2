#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <semaforr/ros/visualization_publisher.hpp>
#include <semaforr_msgs/msg/decision_record.hpp>
#include <social_context_msgs/msg/crowd_field.hpp>
#include <string>
#include <utility>
#include <visualization_msgs/msg/marker.hpp>

namespace semaforr::ros {
namespace {

builtin_interfaces::msg::Time toRosTime(std::int64_t nanoseconds) {
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(nanoseconds / 1'000'000'000LL);
  result.nanosec = static_cast<std::uint32_t>(nanoseconds % 1'000'000'000LL);
  return result;
}

semaforr_msgs::msg::DecisionAction toMessage(const domain::Action& source) {
  semaforr_msgs::msg::DecisionAction result;
  switch (source.type()) {
    case domain::ActionType::Forward:
      result.type = semaforr_msgs::msg::DecisionAction::FORWARD;
      break;
    case domain::ActionType::TurnRight:
      result.type = semaforr_msgs::msg::DecisionAction::TURN_RIGHT;
      break;
    case domain::ActionType::TurnLeft:
      result.type = semaforr_msgs::msg::DecisionAction::TURN_LEFT;
      break;
    case domain::ActionType::Pause:
      result.type = semaforr_msgs::msg::DecisionAction::PAUSE;
      break;
  }
  result.magnitude_index = static_cast<std::uint32_t>(source.magnitude_index());
  return result;
}

std::uint8_t toMessage(decision::DecisionTier tier) {
  switch (tier) {
    case decision::DecisionTier::TierOne:
      return semaforr_msgs::msg::DecisionRecord::TIER_ONE;
    case decision::DecisionTier::TierTwo:
      return semaforr_msgs::msg::DecisionRecord::TIER_TWO;
    case decision::DecisionTier::TierThree:
      return semaforr_msgs::msg::DecisionRecord::TIER_THREE;
    case decision::DecisionTier::Exploration:
      return semaforr_msgs::msg::DecisionRecord::EXPLORATION;
    case decision::DecisionTier::Fallback:
      return semaforr_msgs::msg::DecisionRecord::FALLBACK;
    case decision::DecisionTier::SafeStop:
      return semaforr_msgs::msg::DecisionRecord::SAFE_STOP;
  }
  return semaforr_msgs::msg::DecisionRecord::SAFE_STOP;
}

std::uint8_t toMessage(decision::DecisionSource source) {
  switch (source) {
    case decision::DecisionSource::MandatoryRule:
      return semaforr_msgs::msg::DecisionRecord::SOURCE_MANDATORY_RULE;
    case decision::DecisionSource::TierThreeAdvisor:
      return semaforr_msgs::msg::DecisionRecord::SOURCE_TIER_THREE_ADVISOR;
    case decision::DecisionSource::Planner:
      return semaforr_msgs::msg::DecisionRecord::SOURCE_PLANNER;
    case decision::DecisionSource::Exploration:
      return semaforr_msgs::msg::DecisionRecord::SOURCE_EXPLORATION;
    case decision::DecisionSource::Fallback:
      return semaforr_msgs::msg::DecisionRecord::SOURCE_FALLBACK;
    case decision::DecisionSource::SafeStop:
      return semaforr_msgs::msg::DecisionRecord::SOURCE_SAFE_STOP;
  }
  return semaforr_msgs::msg::DecisionRecord::SOURCE_SAFE_STOP;
}

std::uint8_t toMessage(decision::ActionOutcome outcome) {
  switch (outcome) {
    case decision::ActionOutcome::Pending:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_PENDING;
    case decision::ActionOutcome::Completed:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_COMPLETED;
    case decision::ActionOutcome::TimedOut:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_TIMED_OUT;
    case decision::ActionOutcome::OdometryReset:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_ODOMETRY_RESET;
    case decision::ActionOutcome::ClockReset:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_CLOCK_RESET;
    case decision::ActionOutcome::Cancelled:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_CANCELLED;
    case decision::ActionOutcome::SensorLost:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_SENSOR_LOST;
    case decision::ActionOutcome::Shutdown:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_SHUTDOWN;
  }
  return semaforr_msgs::msg::DecisionRecord::OUTCOME_CANCELLED;
}

std::uint8_t toMessage(navigation::NavigationPhase phase) {
  switch (phase) {
    case navigation::NavigationPhase::InitialExploration:
      return semaforr_msgs::msg::DecisionRecord::PHASE_INITIAL_EXPLORATION;
    case navigation::NavigationPhase::TargetNavigation:
      return semaforr_msgs::msg::DecisionRecord::PHASE_TARGET_NAVIGATION;
    case navigation::NavigationPhase::MissionComplete:
      return semaforr_msgs::msg::DecisionRecord::PHASE_MISSION_COMPLETE;
  }
  return semaforr_msgs::msg::DecisionRecord::PHASE_MISSION_COMPLETE;
}

semaforr_msgs::msg::DecisionRecord toMessage(
    const decision::DecisionResult& source, const rclcpp::Time& stamp,
    const std::string& frame_id) {
  semaforr_msgs::msg::DecisionRecord result;
  result.header.stamp = stamp;
  result.header.frame_id = frame_id;
  result.sequence = source.sequence;
  result.navigation_phase = toMessage(source.navigation_phase);
  result.configuration_fingerprint = source.configuration_fingerprint;
  result.component_manifest = source.component_manifest;
  result.phase_events = source.phase_events;
  result.robot_pose.x = source.robot_pose.position.x_m;
  result.robot_pose.y = source.robot_pose.position.y_m;
  result.robot_pose.theta = source.robot_pose.heading.radians();
  result.has_task = source.task.has_value();
  if (source.task) {
    result.task.task_index = source.task->task_index;
    result.task.decision_count = source.task->decision_count;
    result.task.target.x = source.task->target.x_m;
    result.task.target.y = source.task->target.y_m;
    result.task.has_waypoint = source.task->waypoint.has_value();
    if (source.task->waypoint) {
      result.task.waypoint.x = source.task->waypoint->x_m;
      result.task.waypoint.y = source.task->waypoint->y_m;
    }
  }
  result.candidates.reserve(source.candidates.size());
  for (const auto& candidate : source.candidates) {
    result.candidates.push_back(toMessage(candidate));
  }
  result.vetoes.reserve(source.vetoes.size());
  for (const auto& source_veto : source.vetoes) {
    semaforr_msgs::msg::DecisionVeto veto;
    veto.action = toMessage(source_veto.action);
    veto.rule = source_veto.rule;
    veto.explanation = source_veto.explanation;
    result.vetoes.push_back(std::move(veto));
  }
  result.advisor_contributions.reserve(source.contributions.size());
  for (const auto& source_contribution : source.contributions) {
    semaforr_msgs::msg::AdvisorContribution contribution;
    contribution.advisor = source_contribution.advisor;
    contribution.action = toMessage(source_contribution.action);
    contribution.raw_score = source_contribution.raw_score;
    contribution.weight = source_contribution.weight;
    contribution.weighted_score = source_contribution.weighted_score;
    contribution.explanation = source_contribution.explanation;
    result.advisor_contributions.push_back(std::move(contribution));
  }
  result.selected_tier = toMessage(source.tier);
  result.selected_source = toMessage(source.source);
  result.selected_policy = source.selected_policy;
  result.has_planner = source.planner.has_value();
  result.selected_planner = source.planner.value_or("");
  result.selected_action = toMessage(source.action);
  result.decision_latency_s = source.decision_latency_s;
  result.planning_latency_s = source.planning_latency_s;
  result.model_update_cost_s = source.model_update_cost_s;
  result.allocation_count = source.allocation_count;
  result.allocation_bytes = source.allocation_bytes;
  result.covered_cells = source.covered_cells;
  result.action_outcome = toMessage(source.action_outcome);
  result.action_duration_s = source.action_duration_s;
  result.action_progress = source.action_progress;
  result.action_target = source.action_target;
  result.outcome_detail = source.outcome_detail;
  return result;
}

}  // namespace

class VisualizationPublisher::Impl {
 public:
  Impl(rclcpp::Node& node, const domain::WorldModel& world)
      : node_(node),
        world_(world),
        frame_id_(node.get_parameter("frames.global").as_string()),
        crowd_field_publisher_(
            node.create_publisher<social_context_msgs::msg::CrowdField>(
                node.get_parameter("topics.crowd_field").as_string(),
                rclcpp::QoS(1).transient_local().reliable())),
        decision_publisher_(
            node.create_publisher<semaforr_msgs::msg::DecisionRecord>(
                node.get_parameter("topics.decision_records").as_string(),
                rclcpp::QoS(10).reliable())),
        target_publisher_(
            node.create_publisher<geometry_msgs::msg::PointStamped>(
                "target_point", rclcpp::QoS(1).transient_local().reliable())),
        waypoint_publisher_(
            node.create_publisher<geometry_msgs::msg::PointStamped>(
                "waypoint", rclcpp::QoS(1).transient_local().reliable())),
        plan_publisher_(node.create_publisher<nav_msgs::msg::Path>(
            "plan", rclcpp::QoS(1).transient_local().reliable())),
        static_map_publisher_(
            node.create_publisher<visualization_msgs::msg::Marker>(
                "static_map_geometry",
                rclcpp::QoS(1).transient_local().reliable())),
        map_visualization_enabled_(
            node.get_parameter("map.visualizations.enabled").as_bool()),
        pose_publisher_(node.create_publisher<geometry_msgs::msg::PoseStamped>(
            "decision_pose", rclcpp::QoS(10).reliable())) {}

  void publishDecision(const decision::DecisionResult& result) {
    decision_publisher_->publish(toMessage(result, node_.now(), frame_id_));
    if (result.task &&
        (!last_task_index_ || *last_task_index_ != result.task->task_index)) {
      RCLCPP_INFO(node_.get_logger(), "Task %lu active: target=(%.3f, %.3f)",
                  static_cast<unsigned long>(result.task->task_index),
                  result.task->target.x_m, result.task->target.y_m);
      last_task_index_ = result.task->task_index;
    }
    if (result.selected_policy == "get_out" ||
        result.selected_policy == "find_a_way") {
      RCLCPP_WARN(node_.get_logger(),
                  "Recovery decision %lu selected policy=%s",
                  static_cast<unsigned long>(result.sequence),
                  result.selected_policy.c_str());
    }
    for (const auto& contribution : result.contributions) {
      RCLCPP_DEBUG(node_.get_logger(),
                   "decision=%lu advisor=%s action=%u:%zu raw=%.6f weight=%.6f "
                   "weighted=%.6f",
                   static_cast<unsigned long>(result.sequence),
                   contribution.advisor.c_str(),
                   static_cast<unsigned>(contribution.action.type()),
                   contribution.action.magnitude_index(),
                   contribution.raw_score, contribution.weight,
                   contribution.weighted_score);
    }
    const std::string planner = result.planner.value_or("none");
    RCLCPP_INFO(node_.get_logger(),
                "Decision %lu: tier=%s policy=%s planner=%s action=%u:%zu "
                "latency=%.6fs outcome=%s duration=%.3fs",
                static_cast<unsigned long>(result.sequence),
                std::string(decision::toString(result.tier)).c_str(),
                result.selected_policy.c_str(), planner.c_str(),
                static_cast<unsigned>(result.action.type()),
                result.action.magnitude_index(), result.decision_latency_s,
                std::string(decision::toString(result.action_outcome)).c_str(),
                result.action_duration_s);
    if (result.action_outcome != decision::ActionOutcome::Completed) {
      RCLCPP_WARN(
          node_.get_logger(), "Decision %lu action ended with %s: %s",
          static_cast<unsigned long>(result.sequence),
          std::string(decision::toString(result.action_outcome)).c_str(),
          result.outcome_detail.c_str());
    }
  }

  void publishCrowdField() {
    const auto& snapshot = world_.crowd.learned();
    if (!snapshot.available() || snapshot.version == last_crowd_version_) {
      return;
    }
    social_context_msgs::msg::CrowdField message;
    message.header.frame_id = snapshot.geometry.frame_id;
    message.header.stamp = toRosTime(snapshot.generated_at.count());
    message.width_m = snapshot.geometry.width_m;
    message.height_m = snapshot.geometry.height_m;
    message.resolution_m = snapshot.geometry.resolution_m;
    message.origin_x_m = snapshot.geometry.origin_x_m;
    message.origin_y_m = snapshot.geometry.origin_y_m;
    message.columns = static_cast<std::uint32_t>(snapshot.geometry.columns());
    message.rows = static_cast<std::uint32_t>(snapshot.geometry.rows());
    message.estimator = snapshot.estimator;
    message.version = snapshot.version;
    message.cells.reserve(snapshot.cells.size());
    for (const auto& source : snapshot.cells) {
      social_context_msgs::msg::CrowdFieldCell cell;
      cell.density = source.density;
      cell.learned_encounter_risk = source.learned_encounter_risk;
      cell.directional_flow = source.directional_flow;
      cell.visibility_exposures = source.visibility_exposures;
      cell.pedestrian_hits = source.pedestrian_hits;
      cell.risk_encounters = source.risk_encounters;
      cell.risk_experiences = source.risk_experiences;
      cell.last_updated = toRosTime(source.last_updated.count());
      cell.confidence = source.confidence;
      message.cells.push_back(std::move(cell));
    }
    crowd_field_publisher_->publish(message);
    last_crowd_version_ = snapshot.version;
  }

  void publishSnapshot() {
    const rclcpp::Time stamp = node_.now();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = world_.robot.pose.position.x_m;
    pose.pose.position.y = world_.robot.pose.position.y_m;
    const double half_heading = world_.robot.pose.heading.radians() * 0.5;
    pose.pose.orientation.z = std::sin(half_heading);
    pose.pose.orientation.w = std::cos(half_heading);
    pose_publisher_->publish(pose);

    if (map_visualization_enabled_ && !static_map_published_ &&
        world_.static_map && world_.static_map->geometryAvailable()) {
      visualization_msgs::msg::Marker marker;
      marker.header = pose.header;
      marker.ns = "semaforr_static_map";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.04;
      marker.color.r = 0.15F;
      marker.color.g = 0.15F;
      marker.color.b = 0.15F;
      marker.color.a = 1.0F;
      marker.points.reserve(world_.static_map->walls.size() * 2U);
      for (const auto& wall : world_.static_map->walls) {
        geometry_msgs::msg::Point start;
        start.x = wall.start.x_m;
        start.y = wall.start.y_m;
        geometry_msgs::msg::Point end;
        end.x = wall.end.x_m;
        end.y = wall.end.y_m;
        marker.points.push_back(start);
        marker.points.push_back(end);
      }
      static_map_publisher_->publish(marker);
      static_map_published_ = true;
    }

    if (world_.mission.active()) {
      geometry_msgs::msg::PointStamped target;
      target.header = pose.header;
      target.point.x = world_.mission.active()->target.x_m;
      target.point.y = world_.mission.active()->target.y_m;
      target_publisher_->publish(target);

      if (const auto waypoint = world_.mission.active()->waypoint()) {
        geometry_msgs::msg::PointStamped message;
        message.header = pose.header;
        message.point.x = waypoint->x_m;
        message.point.y = waypoint->y_m;
        waypoint_publisher_->publish(message);
      }

      nav_msgs::msg::Path path;
      path.header = pose.header;
      for (std::size_t index = world_.mission.active()->waypoint_index;
           index < world_.mission.active()->plan.size(); ++index) {
        geometry_msgs::msg::PoseStamped waypoint_pose;
        waypoint_pose.header = path.header;
        waypoint_pose.pose.position.x =
            world_.mission.active()->plan[index].x_m;
        waypoint_pose.pose.position.y =
            world_.mission.active()->plan[index].y_m;
        waypoint_pose.pose.orientation.w = 1.0;
        path.poses.push_back(std::move(waypoint_pose));
      }
      plan_publisher_->publish(path);
    }
    publishCrowdField();
  }

  rclcpp::Node& node_;
  const domain::WorldModel& world_;
  std::string frame_id_;
  rclcpp::Publisher<social_context_msgs::msg::CrowdField>::SharedPtr
      crowd_field_publisher_;
  rclcpp::Publisher<semaforr_msgs::msg::DecisionRecord>::SharedPtr
      decision_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      target_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      waypoint_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      static_map_publisher_;
  bool map_visualization_enabled_ = false;
  bool static_map_published_ = false;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  std::uint64_t last_crowd_version_{0U};
  std::optional<std::uint64_t> last_task_index_;
};

VisualizationPublisher::VisualizationPublisher(rclcpp::Node& node,
                                               const domain::WorldModel& world)
    : impl_(std::make_unique<Impl>(node, world)) {}

VisualizationPublisher::~VisualizationPublisher() = default;
VisualizationPublisher::VisualizationPublisher(
    VisualizationPublisher&&) noexcept = default;
VisualizationPublisher& VisualizationPublisher::operator=(
    VisualizationPublisher&&) noexcept = default;

void VisualizationPublisher::publishSnapshot() { impl_->publishSnapshot(); }

void VisualizationPublisher::publishDecision(
    const decision::DecisionResult& result) {
  impl_->publishDecision(result);
}

}  // namespace semaforr::ros
