#include <semaforr/decision/Arbitration.hpp>

#include <cmath>

namespace semaforr {
namespace decision {

ActionArbitrationResult selectHighestScoringAction(
  const std::map<FORRAction, double>& scores) {
  ActionArbitrationResult result;
  for (const auto& candidate : scores) {
    if (!std::isfinite(candidate.second)) {
      continue;
    }
    if (!result.selected || candidate.second > result.score) {
      result.selected = true;
      result.action = candidate.first;
      result.score = candidate.second;
    }
  }
  return result;
}

PlanArbitrationResult selectLowestCostPlan(
  const std::vector<double>& costs) {
  PlanArbitrationResult result;
  for (std::size_t index = 0; index < costs.size(); ++index) {
    if (!std::isfinite(costs[index])) {
      continue;
    }
    if (!result.selected || costs[index] < result.cost) {
      result.selected = true;
      result.index = index;
      result.cost = costs[index];
    }
  }
  return result;
}

}  // namespace decision
}  // namespace semaforr
