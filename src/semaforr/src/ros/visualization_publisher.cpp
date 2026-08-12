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
#include <type_traits>
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

geometry_msgs::msg::Point toMessage(const domain::Point2D& source) {
  geometry_msgs::msg::Point result;
  result.x = source.x_m;
  result.y = source.y_m;
  return result;
}

geometry_msgs::msg::Pose2D toMessage(const domain::Pose2D& source) {
  geometry_msgs::msg::Pose2D result;
  result.x = source.position.x_m;
  result.y = source.position.y_m;
  result.theta = source.heading.radians();
  return result;
}

semaforr_msgs::msg::PlannerMetadata toMessage(
    const planning::PlannerMetadata& source) {
  semaforr_msgs::msg::PlannerMetadata result;
  result.planner_name = source.name;
  result.plan_type = std::string(planning::toString(source.plan_family));
  result.objective_name = source.objective_name;
  result.objective_description = source.objective_description;
  result.representation_dependencies = source.representation_dependencies;
  result.requires_static_map = source.requires_static_map;
  result.supports_mapless_operation = source.supports_mapless_operation;
  return result;
}

std::vector<semaforr_msgs::msg::ModelRevision> toMessage(
    const domain::DependencyRevisions& source) {
  std::vector<semaforr_msgs::msg::ModelRevision> result;
  for (const auto& [dependency, revision] : source) {
    semaforr_msgs::msg::ModelRevision item;
    item.representation = std::string(domain::toString(dependency));
    item.revision = revision;
    result.push_back(std::move(item));
  }
  return result;
}

semaforr_msgs::msg::PlanStepTrace toMessage(
    const planning::PlanStep& source, std::size_t index) {
  semaforr_msgs::msg::PlanStepTrace result;
  result.step_id = index;
  std::visit(
      [&](const auto& step) {
        using T = std::decay_t<decltype(step)>;
        if constexpr (std::is_same_v<T, planning::WaypointStep>) {
          result.step_type = "waypoint";
          result.has_target = true;
          result.target = toMessage(step.target);
        } else if constexpr (std::is_same_v<T, planning::SubtrailStep>) {
          result.step_type = "subtrail";
          result.primary_entity_id = step.trail_id.value_or(0U);
          for (const auto& point : step.waypoints)
            result.geometry.push_back(toMessage(point));
        } else if constexpr (std::is_same_v<T, planning::RegionStep>) {
          result.step_type = "region";
          result.primary_entity_id = step.region_id;
          result.has_target = true;
          result.target = toMessage(step.center);
        } else if constexpr (std::is_same_v<T, planning::HighwayStep>) {
          result.step_type = "highway";
          result.primary_entity_id = step.highway_id;
          result.secondary_entity_id = step.to;
          for (const auto& point : step.fallback_subtrail)
            result.geometry.push_back(toMessage(point));
          result.execution_event = "intersection_from=" +
                                   std::to_string(step.from) + ",to=" +
                                   std::to_string(step.to);
        } else if constexpr (std::is_same_v<T, planning::IntersectionStep>) {
          result.step_type = "intersection";
          result.primary_entity_id = step.intersection_id;
          result.has_target = true;
          result.target = toMessage(step.centroid);
        } else if constexpr (std::is_same_v<T, planning::HighwayEntryStep>) {
          result.step_type = "highway_entry";
          result.primary_entity_id = step.highway_id;
          result.has_target = true;
          result.target = toMessage(step.entry);
          for (const auto& point : step.supporting_subtrail)
            result.geometry.push_back(toMessage(point));
        } else if constexpr (std::is_same_v<T, planning::HighwayExitStep>) {
          result.step_type = "highway_exit";
          result.primary_entity_id = step.highway_id;
          result.has_target = true;
          result.target = toMessage(step.exit);
          for (const auto& point : step.supporting_subtrail)
            result.geometry.push_back(toMessage(point));
        } else if constexpr (
            std::is_same_v<T, planning::SkeletonTransitionStep>) {
          result.step_type = "skeleton_transition";
          result.primary_entity_id = step.from_region;
          result.secondary_entity_id = step.to_region;
          for (const auto& point : step.supporting_subtrail)
            result.geometry.push_back(toMessage(point));
        } else if constexpr (std::is_same_v<T, planning::FinalTargetStep>) {
          result.step_type = "final_target";
          result.has_target = true;
          result.target = toMessage(step.target);
        }
      },
      source);
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
    case decision::ActionOutcome::PartialMovement:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_PARTIAL_MOVEMENT;
    case decision::ActionOutcome::NoMovement:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_NO_MOVEMENT;
    case decision::ActionOutcome::SafetyInterrupted:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_SAFETY_INTERRUPTED;
    case decision::ActionOutcome::ControllerRejected:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_CONTROLLER_REJECTED;
    case decision::ActionOutcome::ControllerFailure:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_CONTROLLER_FAILURE;
    case decision::ActionOutcome::GoalPreempted:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_GOAL_PREEMPTED;
    case decision::ActionOutcome::NavigationModeTransition:
      return semaforr_msgs::msg::DecisionRecord::OUTCOME_NAVIGATION_MODE_TRANSITION;
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
  result.decision_id = source.decision_id;
  result.action_id = source.action_id;
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
  for (const auto& candidate : source.viable_actions)
    result.viable_actions.push_back(toMessage(candidate));
  for (const auto& prediction : source.predicted_actions) {
    semaforr_msgs::msg::PredictedActionResult item;
    item.action = toMessage(prediction.action);
    item.predicted_pose.x = prediction.predicted_pose.position.x_m;
    item.predicted_pose.y = prediction.predicted_pose.position.y_m;
    item.predicted_pose.theta = prediction.predicted_pose.heading.radians();
    item.viable = prediction.viable;
    item.evidence_source = prediction.evidence_source;
    result.predicted_actions.push_back(std::move(item));
  }
  result.vetoes.reserve(source.vetoes.size());
  for (const auto& source_veto : source.vetoes) {
    semaforr_msgs::msg::DecisionVeto veto;
    veto.action = toMessage(source_veto.action);
    veto.rule = source_veto.rule;
    veto.explanation = source_veto.explanation;
    veto.reason_code = source_veto.reason_code;
    veto.rejection_kind =
        std::string(decision::toString(source_veto.rejection_kind));
    veto.explanation_category =
        std::string(decision::toString(source_veto.category));
    result.vetoes.push_back(std::move(veto));
  }
  for (const auto& source_event : source.decision_cycle) {
    semaforr_msgs::msg::DecisionCycleEvent event;
    event.order = source_event.order;
    event.tier = source_event.tier;
    event.component = source_event.component;
    for (const auto& action : source_event.input_actions)
      event.input_actions.push_back(toMessage(action));
    event.has_mandate = source_event.mandate.has_value();
    if (source_event.mandate)
      event.mandate = toMessage(*source_event.mandate);
    for (const auto& source_veto : source_event.vetoes) {
      semaforr_msgs::msg::DecisionVeto veto;
      veto.action = toMessage(source_veto.action);
      veto.rule = source_veto.rule;
      veto.explanation = source_veto.explanation;
      veto.reason_code = source_veto.reason_code;
      veto.rejection_kind =
          std::string(decision::toString(source_veto.rejection_kind));
      veto.explanation_category =
          std::string(decision::toString(source_veto.category));
      event.vetoes.push_back(std::move(veto));
    }
    for (const auto& action : source_event.remaining_actions)
      event.remaining_actions.push_back(toMessage(action));
    event.outcome = source_event.outcome;
    event.returned_to_earlier_tier =
        source_event.returned_to_earlier_tier;
    event.has_final_attribution = source_event.final_attribution.has_value();
    if (source_event.final_attribution)
      event.final_attribution = toMessage(*source_event.final_attribution);
    event.reason_code = source_event.reason_code;
    result.decision_cycle.push_back(std::move(event));
  }
  result.advisor_contributions.reserve(source.contributions.size());
  for (const auto& source_contribution : source.contributions) {
    semaforr_msgs::msg::AdvisorContribution contribution;
    contribution.advisor = source_contribution.advisor;
    contribution.action = toMessage(source_contribution.action);
    contribution.raw_score = source_contribution.raw_score;
    contribution.normalized_score = source_contribution.normalized_score;
    contribution.advisor_mean = source_contribution.advisor_mean;
    contribution.advisor_standard_deviation =
        source_contribution.advisor_standard_deviation;
    contribution.relative_support = source_contribution.relative_support;
    contribution.weight = source_contribution.weight;
    contribution.weighted_score = source_contribution.weighted_score;
    contribution.viable = source_contribution.viable;
    contribution.final_total = source_contribution.final_total;
    contribution.explanation = source_contribution.explanation;
    result.advisor_contributions.push_back(std::move(contribution));
  }
  result.tier_three_scoring_policy = source.tier_three_scoring_policy;
  for (const auto& source_total : source.tier_three_totals) {
    semaforr_msgs::msg::TierThreeActionTotal total;
    total.action = toMessage(source_total.action);
    total.total = source_total.total;
    total.viable = source_total.viable;
    total.scored = source_total.scored;
    result.tier_three_action_totals.push_back(std::move(total));
  }
  result.tier_three_tie_policy = source.tier_three_tie_policy;
  result.tier_three_tie_tolerance = source.tier_three_tie_tolerance;
  result.tier_three_random_seed = source.tier_three_random_seed;
  for (const auto& candidate : source.tier_three_tie_candidates)
    result.tier_three_tie_candidates.push_back(toMessage(candidate));
  result.tier_three_random_selection_used =
      source.tier_three_random_selection_used;
  result.has_tier_three_random_selection_index =
      source.tier_three_random_selection_index.has_value();
  result.tier_three_random_selection_index =
      source.tier_three_random_selection_index.value_or(0U);
  result.decision_gini_agreement =
      source.decision_confidence.gini_agreement;
  result.decision_standardized_total =
      source.decision_confidence.standardized_total;
  result.decision_relative_support =
      source.decision_confidence.relative_support;
  result.decision_confidence_category =
      source.decision_confidence.category;
  result.selected_tier = toMessage(source.tier);
  result.selected_source = toMessage(source.source);
  result.selected_policy = source.selected_policy;
  result.has_planner = source.planner.has_value();
  result.selected_planner = source.planner.value_or("");
  result.has_plan = source.plan_id.has_value();
  result.active_plan_id = source.plan_id.value_or(0U);
  result.active_plan_revision = source.plan_revision;
  result.has_planning_episode = source.planning_episode_id.has_value();
  result.planning_episode_id = source.planning_episode_id.value_or(0U);
  result.plan_family = source.plan_family
                           ? std::string(planning::toString(*source.plan_family))
                           : "";
  result.enforcer_mode = source.enforcer_mode.value_or("");
  result.active_plan_step = source.active_plan_step.value_or(0U);
  result.has_operational_target = source.operational_target.has_value();
  if (source.operational_target) {
    result.operational_target.x = source.operational_target->x_m;
    result.operational_target.y = source.operational_target->y_m;
  }
  result.enforcer_reason = source.enforcer_reason;
  result.plan_status = source.plan_status;
  result.plan_execution_events = source.plan_execution_events;
  result.planning_candidates.reserve(source.planning_candidates.size());
  for (const auto& candidate : source.planning_candidates) {
    semaforr_msgs::msg::PlanCandidateDiagnostic diagnostic;
    diagnostic.plan_id = candidate.plan_id;
    diagnostic.planner = candidate.planner;
    diagnostic.plan_family = std::string(planning::toString(candidate.family));
    for (const auto& [objective, cost] : candidate.raw_costs) {
      diagnostic.objectives.emplace_back(planning::toString(objective));
      diagnostic.raw_costs.push_back(cost);
      const auto normalized = candidate.normalized_costs.find(objective);
      diagnostic.normalized_costs.push_back(
          normalized == candidate.normalized_costs.end() ? 0.0
                                                         : normalized->second);
    }
    diagnostic.summed_score = candidate.summed_score;
    diagnostic.tied_for_best = candidate.tied_for_best;
    diagnostic.metadata = toMessage(candidate.metadata);
    for (const auto& point : candidate.geometry)
      diagnostic.geometry.push_back(toMessage(point));
    for (std::size_t index = 0U; index < candidate.typed_steps.size(); ++index)
      diagnostic.typed_steps.push_back(
          toMessage(candidate.typed_steps[index], index));
    diagnostic.dependency_revisions =
        toMessage(candidate.dependency_revisions);
    diagnostic.planner_configuration_revision =
        candidate.planner_configuration_revision;
    diagnostic.operating_mode =
        std::string(planning::toString(candidate.operating_mode));
    diagnostic.static_map_contributed = candidate.static_map_contributed;
    result.planning_candidates.push_back(std::move(diagnostic));
  }
  result.planning_tie_candidates = source.planning_tie_candidates;
  result.planning_tie_break_reason = source.planning_tie_break_reason;
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
  result.execution_id = source.execution_id;
  result.action_lifecycle_status = source.action_lifecycle_status;
  result.has_execution_result = source.execution_result.has_value();
  if (source.execution_result) {
    result.execution_start_pose = toMessage(source.execution_result->start_pose);
    result.execution_final_pose = toMessage(source.execution_result->final_pose);
    result.distance_achieved_m = source.execution_result->distance_achieved_m;
    result.rotation_achieved_rad =
        source.execution_result->rotation_achieved_rad;
    result.execution_timed_out = source.execution_result->timed_out;
    result.execution_cancellation_reason =
        source.execution_result->cancellation_reason;
    result.safety_interruption = source.execution_result->safety_interruption;
    result.controller_failure = source.execution_result->controller_failure;
    result.collision = source.execution_result->collision;
    result.near_collision = source.execution_result->near_collision;
  }
  if (!source.source_provenance.empty()) {
    result.source_provenance = source.source_provenance;
  } else if (source.predicted_actions.empty()) {
    result.source_provenance.push_back("no_spatial_evidence_recorded");
  } else {
    result.source_provenance.push_back(
        source.predicted_actions.front().evidence_source);
  }
  if (source.plan_id) result.source_provenance.push_back("active_plan_structure");
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
        familiarity_publisher_(
            node.create_publisher<visualization_msgs::msg::Marker>(
                "familiarity_grid",
                rclcpp::QoS(1).transient_local().reliable())),
        sensed_free_publisher_(
            node.create_publisher<visualization_msgs::msg::Marker>(
                "sensed_occupancy_free",
                rclcpp::QoS(1).transient_local().reliable())),
        sensed_occupied_publisher_(
            node.create_publisher<visualization_msgs::msg::Marker>(
                "sensed_occupancy_occupied",
                rclcpp::QoS(1).transient_local().reliable())),
        static_occupancy_publisher_(
            node.create_publisher<visualization_msgs::msg::Marker>(
                "static_map_occupancy",
                rclcpp::QoS(1).transient_local().reliable())),
        map_visualization_enabled_(
            node.get_parameter("map.visualizations.enabled").as_bool()),
        grid_visualization_enabled_(
            node.get_parameter("grids.visualizations.enabled").as_bool()),
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
                   "decision=%lu advisor=%s action=%u:%zu raw=%.6f "
                   "transformed=%.6f weight=%.6f weighted=%.6f total=%.6f",
                   static_cast<unsigned long>(result.sequence),
                   contribution.advisor.c_str(),
                   static_cast<unsigned>(contribution.action.type()),
                   contribution.action.magnitude_index(),
                   contribution.raw_score, contribution.normalized_score,
                   contribution.weight, contribution.weighted_score,
                   contribution.final_total);
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
    message.width_m = snapshot.geometry.widthMeters();
    message.height_m = snapshot.geometry.heightMeters();
    message.resolution_m = snapshot.geometry.resolution_m;
    message.origin_x_m = snapshot.geometry.origin.x_m;
    message.origin_y_m = snapshot.geometry.origin.y_m;
    message.columns = static_cast<std::uint32_t>(snapshot.geometry.columns);
    message.rows = static_cast<std::uint32_t>(snapshot.geometry.rows);
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

    if (grid_visualization_enabled_) publishGridLayers(pose.header);

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

  visualization_msgs::msg::Marker gridMarker(
      const std_msgs::msg::Header& header, const std::string& name,
      double resolution, float red, float green, float blue) const {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 0.75F;
    return marker;
  }

  void publishGridLayers(const std_msgs::msg::Header& header) {
    const auto& familiarity = world_.spatial.known_grid;
    if (familiarity.valid() &&
        familiarity.revision != last_familiarity_revision_) {
      auto marker = gridMarker(header, "familiarity", familiarity.resolution_m,
                               0.1F, 0.35F, 1.0F);
      const auto append_familiarity = [&](std::size_t index) {
        const auto center = familiarity.extent().center(index);
        geometry_msgs::msg::Point point;
        point.x = center.x_m;
        point.y = center.y_m;
        marker.points.push_back(point);
      };
      if (familiarity.cells.empty()) {
        for (const auto& cell : familiarity.sparseCells())
          if (cell.value != 0U) append_familiarity(cell.index);
      } else {
        for (std::size_t index = 0U; index < familiarity.cells.size(); ++index)
          if (familiarity.cells[index] != 0U) append_familiarity(index);
      }
      familiarity_publisher_->publish(marker);
      last_familiarity_revision_ = familiarity.revision;
    }

    const auto& sensed = world_.spatial.sensed_occupancy;
    if (sensed.valid() && sensed.revision != last_sensed_revision_) {
      auto free = gridMarker(header, "sensed_free", sensed.geometry.resolution_m,
                             0.1F, 0.8F, 0.25F);
      auto occupied = gridMarker(header, "sensed_occupied",
                                 sensed.geometry.resolution_m, 0.9F, 0.1F,
                                 0.1F);
      const auto append_sensed = [&](std::size_t index,
                                     domain::SensedOccupancyState state) {
        if (state == domain::SensedOccupancyState::Unknown) return;
        const auto center = sensed.geometry.center(index);
        geometry_msgs::msg::Point point;
        point.x = center.x_m;
        point.y = center.y_m;
        (state == domain::SensedOccupancyState::ObservedOccupied ? occupied
                                                                 : free)
            .points.push_back(point);
      };
      if (sensed.cells.empty()) {
        for (const auto& cell : sensed.sparseCells())
          append_sensed(cell.index, cell.value.state);
      } else {
        for (std::size_t index = 0U; index < sensed.cells.size(); ++index)
          append_sensed(index, sensed.cells[index].state);
      }
      sensed_free_publisher_->publish(free);
      sensed_occupied_publisher_->publish(occupied);
      last_sensed_revision_ = sensed.revision;
    }

    if (!static_occupancy_published_ && world_.static_map &&
        world_.static_map->occupancyAvailable()) {
      const auto& grid = world_.static_map->occupancy;
      auto marker = gridMarker(header, "static_occupancy",
                               grid.geometry.resolution_m,
                               0.2F, 0.2F, 0.2F);
      const auto& geometry = grid.geometry;
      for (std::size_t index = 0U; index < grid.cells.size(); ++index) {
        if (grid.cells[index] != domain::StaticOccupancyState::StaticOccupied)
          continue;
        const auto center = geometry.center(index);
        geometry_msgs::msg::Point point;
        point.x = center.x_m;
        point.y = center.y_m;
        marker.points.push_back(point);
      }
      static_occupancy_publisher_->publish(marker);
      static_occupancy_published_ = true;
    }
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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      familiarity_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      sensed_free_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      sensed_occupied_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      static_occupancy_publisher_;
  bool map_visualization_enabled_ = false;
  bool grid_visualization_enabled_ = false;
  bool static_map_published_ = false;
  bool static_occupancy_published_ = false;
  std::size_t last_familiarity_revision_ = 0U;
  std::size_t last_sensed_revision_ = 0U;
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
