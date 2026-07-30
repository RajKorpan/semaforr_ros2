#include <algorithm>
#include <semaforr/decision/hard_safety_filter.hpp>
#include <set>

namespace semaforr::decision {

SafetyFilterResult HardSafetyFilter::filter(
    const DecisionContext& context,
    std::span<const domain::Action> candidates) const {
  SafetyFilterResult result;
  result.vetoes = obstacle_filter_.evaluate(context);
  std::set<domain::Action> unsafe;
  for (const auto& veto : result.vetoes) unsafe.insert(veto.action);
  for (const auto& action : candidates)
    if (!unsafe.contains(action)) result.safe_actions.push_back(action);
  std::sort(result.safe_actions.begin(), result.safe_actions.end());
  result.safe_actions.erase(
      std::unique(result.safe_actions.begin(), result.safe_actions.end()),
      result.safe_actions.end());
  return result;
}

}  // namespace semaforr::decision
