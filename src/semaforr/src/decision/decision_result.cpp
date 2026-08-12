#include <semaforr/decision/decision_result.hpp>

namespace semaforr::decision {

std::string_view toString(DecisionSource source) noexcept {
  switch (source) {
    case DecisionSource::MandatoryRule:
      return "mandatory_rule";
    case DecisionSource::TierThreeAdvisor:
      return "tier_three_advisor";
    case DecisionSource::Planner:
      return "planner";
    case DecisionSource::Exploration:
      return "exploration";
    case DecisionSource::Fallback:
      return "fallback";
    case DecisionSource::SafeStop:
      return "safe_stop";
  }
  return "unknown";
}

std::string_view toString(DecisionTier tier) noexcept {
  switch (tier) {
    case DecisionTier::TierOne:
      return "tier_one";
    case DecisionTier::TierTwo:
      return "tier_two";
    case DecisionTier::TierThree:
      return "tier_three";
    case DecisionTier::Exploration:
      return "exploration";
    case DecisionTier::Fallback:
      return "fallback";
    case DecisionTier::SafeStop:
      return "safe_stop";
  }
  return "unknown";
}

std::string_view toString(ActionOutcome outcome) noexcept {
  switch (outcome) {
    case ActionOutcome::Pending:
      return "pending";
    case ActionOutcome::Completed:
      return "completed";
    case ActionOutcome::TimedOut:
      return "timed_out";
    case ActionOutcome::OdometryReset:
      return "odometry_reset";
    case ActionOutcome::ClockReset:
      return "clock_reset";
    case ActionOutcome::Cancelled:
      return "cancelled";
    case ActionOutcome::SensorLost:
      return "sensor_lost";
    case ActionOutcome::Shutdown:
      return "shutdown";
    case ActionOutcome::PartialMovement:
      return "partial_movement";
    case ActionOutcome::NoMovement:
      return "no_movement";
    case ActionOutcome::SafetyInterrupted:
      return "safety_interrupted";
    case ActionOutcome::ControllerRejected:
      return "controller_rejected";
    case ActionOutcome::ControllerFailure:
      return "controller_failure";
    case ActionOutcome::GoalPreempted:
      return "goal_preempted";
    case ActionOutcome::NavigationModeTransition:
      return "navigation_mode_transition";
  }
  return "unknown";
}

std::string_view toString(RejectionKind kind) noexcept {
  switch (kind) {
    case RejectionKind::Safety: return "safety_rejection";
    case RejectionKind::Cognitive: return "cognitive_veto";
    case RejectionKind::NotViable: return "not_viable";
  }
  return "not_viable";
}

std::string_view toString(VetoCategory category) noexcept {
  switch (category) {
    case VetoCategory::Unsafe: return "unsafe";
    case VetoCategory::ObstacleConflict: return "obstacle_conflict";
    case VetoCategory::OpposesRecentOrientation:
      return "opposes_recent_orientation";
    case VetoCategory::IneffectivePrecedent:
      return "previously_ineffective";
    case VetoCategory::ReturnsToVisitedSpace:
      return "returns_to_visited_space";
    case VetoCategory::ActivePlanConflict: return "active_plan_conflict";
    case VetoCategory::NoUsefulProgress: return "no_useful_progress";
    case VetoCategory::NotViable: return "not_viable";
    case VetoCategory::ReactiveControl: return "reactive_control";
    case VetoCategory::ExplorationPreference:
      return "exploration_preference";
    case VetoCategory::CaseBasedPrecedent: return "case_based_precedent";
    case VetoCategory::PlanEnforcement: return "plan_enforcement";
    case VetoCategory::InvalidNavigationState:
      return "invalid_navigation_state";
  }
  return "invalid_navigation_state";
}

}  // namespace semaforr::decision
