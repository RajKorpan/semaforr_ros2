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
  }
  return "unknown";
}

}  // namespace semaforr::decision
