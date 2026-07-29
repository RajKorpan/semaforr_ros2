/*
 * Controller decision-pipeline orchestration.
 */

#include <semaforr/decision/Controller.hpp>

#include <semaforr/core/action_adapter.hpp>

semaforr::decision::DecisionResult Controller::FORRDecision() {
  const semaforr::decision::TierOneResult tier_one =
    tierOneDecision->decide();

  semaforr::decision::DecisionResult result;
  result.vetoes = tier_one.vetoes;
  if (tier_one.decided) {
    result.action = semaforr::core::toDomainAction(tier_one.action);
    result.source = semaforr::decision::DecisionSource::MandatoryRule;
    result.tier = semaforr::decision::DecisionTier::TierOne;
    result.selected_policy = tier_one.selected_policy;
  } else {
    const semaforr::decision::TierThreeResult tier_three =
      tierThreeDecision->decide();
    result.action = semaforr::core::toDomainAction(tier_three.action);
    result.source = tier_three.selected
      ? semaforr::decision::DecisionSource::TierThreeAdvisor
      : semaforr::decision::DecisionSource::SafeStop;
    result.tier = tier_three.selected
      ? semaforr::decision::DecisionTier::TierThree
      : semaforr::decision::DecisionTier::SafeStop;
    result.selected_policy =
      tier_three.selected ? "advisor_arbitration" : "no_advisor_score";
    result.contributions = tier_three.contributions;
  }
  return result;
}
