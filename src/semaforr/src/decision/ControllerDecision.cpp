/*
 * Controller decision-pipeline orchestration.
 */

#include <semaforr/decision/Controller.hpp>

#include <iostream>

FORRAction Controller::FORRDecision() {
  std::cout << "In FORR decision" << std::endl;
  std::cout << "Created decision object" << std::endl;

  const semaforr::decision::TierOneResult tier_one =
    tierOneDecision->decide();
  if (tier_one.vetoes_recorded) {
    decisionStats.vetoedActions = tier_one.vetoed_actions;
  }

  FORRAction decision;
  if (tier_one.decided) {
    decision = tier_one.action;
    decisionStats.decisionTier = tier_one.decision_tier;
  } else {
    std::cout << "Decision to be made by t3!!" << std::endl;
    const semaforr::decision::TierThreeResult tier_three =
      tierThreeDecision->decide();
    decision = tier_three.action;
    decisionStats.decisionTier = 3;
    decisionStats.advisors = tier_three.advisors;
    decisionStats.advisorComments = tier_three.advisor_comments;
  }

  std::cout << "Exiting FORR decision with action: "
            << decision.type << " " << decision.parameter << std::endl;
  return decision;
}
