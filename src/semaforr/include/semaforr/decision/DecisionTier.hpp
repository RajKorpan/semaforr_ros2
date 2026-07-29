#ifndef SEMAFORR_DECISION_DECISION_TIER_H
#define SEMAFORR_DECISION_DECISION_TIER_H

#include <string>

#include <semaforr/core/FORRAction.hpp>
#include <semaforr/core/Position.hpp>

namespace semaforr {
namespace decision {

struct TierOneResult {
  bool decided = false;
  FORRAction action;
  double decision_tier = 0.0;
  bool vetoes_recorded = false;
  std::string vetoed_actions;
};

class TierOneDecision {
public:
  virtual ~TierOneDecision() = default;
  virtual TierOneResult decide() = 0;
};

struct TierTwoResult {
  bool plan_selected = false;
  std::string chosen_planner;
  std::string planner_comments;
  double computation_time_seconds = 0.0;
};

class TierTwoDecision {
public:
  virtual ~TierTwoDecision() = default;
  virtual TierTwoResult plan(Position current, bool select_next_task) = 0;
};

struct TierThreeResult {
  FORRAction action;
  std::string advisors;
  std::string advisor_comments;
};

class TierThreeDecision {
public:
  virtual ~TierThreeDecision() = default;
  virtual TierThreeResult decide() = 0;
};

}  // namespace decision
}  // namespace semaforr

#endif  // SEMAFORR_DECISION_DECISION_TIER_H
