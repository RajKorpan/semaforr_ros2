/*
 * Tier-one decision implementation.
 */

#include <semaforr/decision/DecisionTier.hpp>
#include "DecisionTierFactory.h"

#include <semaforr/core/action_adapter.hpp>
#include <semaforr/core/FORRGeometry.hpp>
#include <semaforr/decision/Beliefs.hpp>
#include <semaforr/decision/Tier1Advisor.hpp>

#include <algorithm>
#include <set>
#include <utility>
#include <vector>

namespace semaforr {
namespace decision {
namespace {

void appendVetoes(
  TierOneResult& result,
  const std::vector<FORRAction>& actions,
  std::string rule,
  std::string explanation)
{
  for (const auto& action : actions) {
    result.vetoes.push_back({
      core::toDomainAction(action), rule, explanation});
  }
}

class DefaultTierOneDecision final : public TierOneDecision {
public:
  explicit DefaultTierOneDecision(TierOneDependencies dependencies)
    : dependencies_(dependencies) {}

  TierOneResult decide() override {
    TierOneResult result;
    Beliefs& beliefs = dependencies_.beliefs;
    Tier1Advisor& advisor = dependencies_.advisor;

    CartesianPoint current_position(
      beliefs.getAgentState()->getCurrentPosition().getX(),
      beliefs.getAgentState()->getCurrentPosition().getY());

    if (current_position.get_distance(
          beliefs.getAgentState()->getFarthestPoint()) <= 0.75) {
      beliefs.getAgentState()->setGetOutTriggered(false);
    }
    if (current_position.get_distance(
          beliefs.getAgentState()->getRepositionPoint()) <= 0.75 ||
        beliefs.getAgentState()->getRepositionCount() >= 20) {
      beliefs.getAgentState()->setRepositionTriggered(false);
      beliefs.getAgentState()->setRepositionCount(0);
    }

    if (advisor.advisorVictory(&result.action)) {
      result.selected_policy = "victory";
      result.decided = true;
    } else {
      advisor.advisorAvoidObstacles();

      std::vector<FORRAction> obstacle_vetoes;
      std::set<FORRAction>* vetoed_actions =
        beliefs.getAgentState()->getVetoedActions();
      for (const FORRAction& action : *vetoed_actions) {
        obstacle_vetoes.push_back(action);
      }

      advisor.advisorNotOpposite();
      std::vector<FORRAction> opposite_vetoes;
      vetoed_actions = beliefs.getAgentState()->getVetoedActions();
      for (const FORRAction& action : *vetoed_actions) {
        if (std::find(
              obstacle_vetoes.begin(), obstacle_vetoes.end(), action) ==
            obstacle_vetoes.end()) {
          opposite_vetoes.push_back(action);
        }
      }

      if (advisor.advisorEnforcer(&result.action)) {
        const std::string planner_name =
          beliefs.getAgentState()->getCurrentTask()->getPlannerName();
        if (planner_name == "skeleton" || planner_name == "hallwayskel") {
          const int creator = beliefs.getAgentState()
                                ->getCurrentTask()
                                ->getSkeletonWaypoint()
                                .getCreator();
          if (creator == 2) {
            result.selected_policy = "enforcer_exit";
          } else if (creator == 3) {
            result.selected_policy = "enforcer_passage";
          } else {
            result.selected_policy =
              advisor.getShortcut() ? "enforcer_shortcut" : "enforcer";
          }
        } else {
          result.selected_policy = "enforcer";
        }
        result.decided = true;
      }

      if (dependencies_.doorway_enabled && !result.decided &&
          advisor.advisorDoorway(&result.action)) {
        result.selected_policy = "doorway";
        result.decided = true;
      }
      if (dependencies_.behind_enabled && !result.decided &&
          advisor.advisorBehindYou(&result.action)) {
        result.selected_policy = "behind_you";
        result.decided = true;
      }
      if (dependencies_.get_out_enabled && !result.decided &&
          advisor.advisorGetOut(&result.action)) {
        result.selected_policy = "get_out";
        result.decided = true;
      }
      if (dependencies_.find_a_way_enabled && !result.decided &&
          (dependencies_.highway_finished > 1 ||
           dependencies_.frontier_finished > 1) &&
          advisor.advisorFindAWay(&result.action)) {
        result.selected_policy = "find_a_way";
        result.decided = true;
      }
      if (dependencies_.dont_go_back_enabled) {
        advisor.advisorDontGoBack();
      }

      std::vector<FORRAction> dont_go_back_vetoes;
      vetoed_actions = beliefs.getAgentState()->getVetoedActions();
      for (const FORRAction& action : *vetoed_actions) {
        if (std::find(
              obstacle_vetoes.begin(), obstacle_vetoes.end(), action) ==
              obstacle_vetoes.end() &&
            std::find(
              opposite_vetoes.begin(), opposite_vetoes.end(), action) ==
              opposite_vetoes.end()) {
          dont_go_back_vetoes.push_back(action);
        }
      }

      appendVetoes(
        result, obstacle_vetoes, "avoid_obstacles",
        "action exceeds currently visible obstacle clearance");
      appendVetoes(
        result, opposite_vetoes, "not_opposite",
        "action immediately reverses recent motion");
      appendVetoes(
        result, dont_go_back_vetoes, "dont_go_back",
        "action returns toward recently visited space");
    }

    advisor.resetShortcut();
    return result;
  }

private:
  TierOneDependencies dependencies_;
};

}  // namespace

std::unique_ptr<TierOneDecision> makeTierOneDecision(
  TierOneDependencies dependencies) {
  return std::make_unique<DefaultTierOneDecision>(dependencies);
}

}  // namespace decision
}  // namespace semaforr
