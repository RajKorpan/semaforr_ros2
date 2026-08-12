#include <algorithm>
#include <chrono>
#include <semaforr/decision/hard_safety_filter.hpp>
#include <set>

namespace semaforr::decision {

SafetyFilterResult HardSafetyFilter::filter(
    const DecisionContext& context,
    std::span<const domain::Action> candidates) const {
  SafetyFilterResult result;
  const auto now = std::chrono::steady_clock::now();
  const auto observed_at = context.world.robot.observed_at;
  const bool fresh =
      observed_at != std::chrono::steady_clock::time_point{} &&
      observed_at <= now && now - observed_at <= sensor_freshness_timeout_;
  if (fresh && context.world.robot.laser) {
    result.vetoes = obstacle_filter_.evaluate(context);
    for (auto& veto : result.vetoes) {
      veto.rule = "HardSafetyFilter";
      veto.explanation = "hard_safety:collision_clearance";
    }
  }
  std::set<domain::Action> unsafe;
  for (const auto& veto : result.vetoes) unsafe.insert(veto.action);
  for (const auto& action : candidates) {
    if (!action_space_.contains(action)) {
      result.vetoes.push_back(
          {action, "HardSafetyFilter", "hard_safety:invalid_action_index"});
      continue;
    }
    if ((!fresh || !context.world.robot.laser) &&
        action.type() != domain::ActionType::Pause) {
      result.vetoes.push_back(
          {action, "HardSafetyFilter",
           "hard_safety:sensor_stale_or_missing"});
      continue;
    }
    if (!unsafe.contains(action)) result.safe_actions.push_back(action);
  }
  std::sort(result.safe_actions.begin(), result.safe_actions.end());
  result.safe_actions.erase(
      std::unique(result.safe_actions.begin(), result.safe_actions.end()),
      result.safe_actions.end());
  return result;
}

}  // namespace semaforr::decision
