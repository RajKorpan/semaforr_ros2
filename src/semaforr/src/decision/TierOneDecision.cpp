/*
 * Tier-one decision implementation.
 */

#include <semaforr/decision/DecisionTier.h>
#include "DecisionTierFactory.h"

#include <semaforr/core/FORRGeometry.h>
#include <semaforr/decision/Beliefs.h>
#include <semaforr/decision/Tier1Advisor.h>

#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

namespace semaforr {
namespace decision {
namespace {

class DefaultTierOneDecision final : public TierOneDecision {
public:
  explicit DefaultTierOneDecision(TierOneDependencies dependencies)
    : dependencies_(dependencies) {}

  TierOneResult decide() override {
    TierOneResult result;
    Beliefs& beliefs = dependencies_.beliefs;
    Tier1Advisor& advisor = dependencies_.advisor;

    std::cout << "Tier 1 Decision Making" << std::endl;
    CartesianPoint current_position(
      beliefs.getAgentState()->getCurrentPosition().getX(),
      beliefs.getAgentState()->getCurrentPosition().getY());
    std::cout << "Inside tier 1 decision. Current Position: "
              << current_position.get_x() << " "
              << current_position.get_y() << std::endl;

    if (current_position.get_distance(
          beliefs.getAgentState()->getFarthestPoint()) <= 0.75) {
      std::cout << "if statement 1 triggered" << std::endl;
      beliefs.getAgentState()->setGetOutTriggered(false);
    }
    if (current_position.get_distance(
          beliefs.getAgentState()->getRepositionPoint()) <= 0.75 ||
        beliefs.getAgentState()->getRepositionCount() >= 20) {
      std::cout << "if statement 2 triggered" << std::endl;
      beliefs.getAgentState()->setRepositionTriggered(false);
      beliefs.getAgentState()->setRepositionCount(0);
    }

    if (advisor.advisorVictory(&result.action)) {
      std::cout << "if statement 3 triggered" << std::endl;
      result.decision_tier = 1.1;
      result.decided = true;
    } else {
      std::cout << "else statement triggered" << std::endl;
      advisor.advisorAvoidObstacles();
      std::cout << "Advisor AvoidObstacles vetoed actions" << std::endl;

      std::vector<FORRAction> obstacle_vetoes;
      std::set<FORRAction>* vetoed_actions =
        beliefs.getAgentState()->getVetoedActions();
      for (const FORRAction& action : *vetoed_actions) {
        obstacle_vetoes.push_back(action);
      }

      advisor.advisorNotOpposite();
      std::cout << "Advisor NotOpposite vetoed actions" << std::endl;
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
        std::cout << "if statement 1 inside else statement triggered"
                  << std::endl;
        const std::string planner_name =
          beliefs.getAgentState()->getCurrentTask()->getPlannerName();
        if (planner_name == "skeleton" || planner_name == "hallwayskel") {
          const int creator = beliefs.getAgentState()
                                ->getCurrentTask()
                                ->getSkeletonWaypoint()
                                .getCreator();
          if (creator == 2) {
            result.decision_tier = 1.5;
          } else if (creator == 3) {
            result.decision_tier = 1.6;
          } else {
            result.decision_tier = advisor.getShortcut() ? 1.21 : 1.2;
          }
        } else {
          result.decision_tier = 1.2;
        }
        result.decided = true;
      }

      if (dependencies_.doorway_enabled && !result.decided &&
          advisor.advisorDoorway(&result.action)) {
        result.decision_tier = 1.3;
        result.decided = true;
      }
      if (dependencies_.behind_enabled && !result.decided &&
          advisor.advisorBehindYou(&result.action)) {
        result.decision_tier = 1.4;
        result.decided = true;
      }
      if (dependencies_.get_out_enabled && !result.decided &&
          advisor.advisorGetOut(&result.action)) {
        result.decision_tier = 1.5;
        result.decided = true;
      }
      if (dependencies_.find_a_way_enabled && !result.decided &&
          (dependencies_.highway_finished > 1 ||
           dependencies_.frontier_finished > 1) &&
          advisor.advisorFindAWay(&result.action)) {
        result.decision_tier = 1.6;
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

      std::stringstream veto_list;
      for (const FORRAction& action : obstacle_vetoes) {
        veto_list << action.type << " " << action.parameter << " 1a;";
      }
      for (const FORRAction& action : opposite_vetoes) {
        veto_list << action.type << " " << action.parameter << " 1b;";
      }
      for (const FORRAction& action : dont_go_back_vetoes) {
        veto_list << action.type << " " << action.parameter << " 1c;";
      }
      result.vetoes_recorded = true;
      result.vetoed_actions = veto_list.str();
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
