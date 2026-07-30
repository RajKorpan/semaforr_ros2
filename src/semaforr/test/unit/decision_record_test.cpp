#include <gtest/gtest.h>

#include <semaforr/decision/decision_result.hpp>

namespace {

using semaforr::decision::ActionOutcome;
using semaforr::decision::DecisionResult;
using semaforr::decision::DecisionSource;
using semaforr::decision::DecisionTier;

TEST(DecisionRecord, UsesNamedTierAndOutcomeValues) {
  DecisionResult result;
  result.sequence = 42U;
  result.source = DecisionSource::TierThreeAdvisor;
  result.tier = DecisionTier::TierThree;
  result.selected_policy = "advisor_arbitration";
  result.action_outcome = ActionOutcome::Completed;

  EXPECT_EQ(semaforr::decision::toString(result.tier), "tier_three");
  EXPECT_EQ(semaforr::decision::toString(result.source), "tier_three_advisor");
  EXPECT_EQ(semaforr::decision::toString(result.action_outcome), "completed");
}

TEST(DecisionRecord, DefaultsToAnExplainableSafeStop) {
  const DecisionResult result;
  EXPECT_EQ(result.source, DecisionSource::SafeStop);
  EXPECT_EQ(result.tier, DecisionTier::SafeStop);
  EXPECT_EQ(result.action_outcome, ActionOutcome::Pending);
  EXPECT_EQ(semaforr::decision::toString(result.tier), "safe_stop");
}

}  // namespace
