#ifndef SEMAFORR_DECISION_ARBITRATION_H
#define SEMAFORR_DECISION_ARBITRATION_H

#include <cstddef>
#include <map>
#include <vector>

#include <semaforr/core/FORRAction.hpp>

namespace semaforr {
namespace decision {

struct ActionArbitrationResult {
  bool selected = false;
  FORRAction action;
  double score = 0.0;
};

struct PlanArbitrationResult {
  bool selected = false;
  std::size_t index = 0;
  double cost = 0.0;
};

// Ties retain the first entry in FORRAction map order.
ActionArbitrationResult selectHighestScoringAction(
  const std::map<FORRAction, double>& scores);

// Ties retain the earliest candidate supplied by the planners.
PlanArbitrationResult selectLowestCostPlan(
  const std::vector<double>& costs);

}  // namespace decision
}  // namespace semaforr

#endif  // SEMAFORR_DECISION_ARBITRATION_H
