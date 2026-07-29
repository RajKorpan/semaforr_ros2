#ifndef SEMAFORR_DECISION_DECISION_TIER_H
#define SEMAFORR_DECISION_DECISION_TIER_H

#include <string>
#include <vector>

#include <semaforr/core/FORRAction.hpp>
#include <semaforr/core/Position.hpp>
#include <semaforr/decision/decision_result.hpp>

namespace semaforr {
namespace decision {

struct TierOneResult {
  bool decided = false;
  FORRAction action;
  std::string selected_policy;
  std::vector<Veto> vetoes;
};

class TierOneDecision {
public:
  virtual ~TierOneDecision() = default;
  virtual TierOneResult decide() = 0;
};

struct TierTwoResult {
  bool plan_selected = false;
  std::string chosen_planner;
  double computation_time_seconds = 0.0;
};

class TierTwoDecision {
public:
  virtual ~TierTwoDecision() = default;
  virtual TierTwoResult plan(Position current, bool select_next_task) = 0;
};

struct TierThreeResult {
  bool selected = false;
  FORRAction action;
  std::vector<AdvisorContribution> contributions;
};

class TierThreeDecision {
public:
  virtual ~TierThreeDecision() = default;
  virtual TierThreeResult decide() = 0;
};

}  // namespace decision
}  // namespace semaforr

#endif  // SEMAFORR_DECISION_DECISION_TIER_H
