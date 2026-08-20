#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <semaforr/decision/decision_coordinator.hpp>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using semaforr::decision::ActionScore;
using semaforr::decision::Advisor;
using semaforr::decision::AdvisorEvaluation;
using semaforr::decision::DecisionContext;
using semaforr::decision::DecisionCoordinator;
using semaforr::decision::DecisionSource;
using semaforr::decision::DecisionTier;
using semaforr::decision::Decision;
using semaforr::decision::MandatoryRule;
using semaforr::decision::Veto;
using semaforr::decision::VetoRule;
using semaforr::domain::Action;
using semaforr::domain::ActionType;

class FixedAdvisor final : public Advisor {
 public:
  FixedAdvisor(std::string name, std::vector<ActionScore> scores,
               double weight = 1.0,
               semaforr::decision::ScoreNormalization normalization =
                   semaforr::decision::ScoreNormalization::None)
      : name_(std::move(name)), scores_(std::move(scores)), weight_(weight),
        normalization_(normalization) {}

  std::string_view name() const noexcept override { return name_; }
  semaforr::decision::AdvisorMetadata metadata() const override {
    return {{}, {}, false, normalization_, "fixed test scores"};
  }

  AdvisorEvaluation evaluate(const DecisionContext&,
                             std::span<const Action>) const override {
    return {true, scores_, weight_, "fixed test scores"};
  }

 private:
  std::string name_;
  std::vector<ActionScore> scores_;
  double weight_;
  semaforr::decision::ScoreNormalization normalization_;
};

class FixedVeto final : public VetoRule {
 public:
  explicit FixedVeto(Action action, std::string name = "veto_rule")
      : action_(action), name_(std::move(name)) {}

  std::string_view name() const noexcept override { return name_; }

  std::vector<Veto> evaluate(const DecisionContext&) const override {
    return {{action_, "test-veto", "blocked for test"}};
  }

 private:
  Action action_;
  std::string name_;
};

class FixedMandatory final : public MandatoryRule {
 public:
  FixedMandatory(std::string name, std::optional<Decision> decision)
      : name_(std::move(name)), decision_(std::move(decision)) {}

  std::string_view name() const noexcept override { return name_; }
  std::optional<Decision> evaluate(const DecisionContext&) const override {
    return decision_;
  }

 private:
  std::string name_;
  std::optional<Decision> decision_;
};

semaforr::domain::WorldModel world() { return {}; }

TEST(DecisionCoordinator, EmptyCandidateSetReturnsSafeStop) {
  auto model = world();
  DecisionCoordinator coordinator;
  const auto result = coordinator.decide(DecisionContext{model}, {});
  EXPECT_EQ(result.action, Action::pause());
  EXPECT_EQ(result.source, DecisionSource::SafeStop);
  EXPECT_EQ(result.tier, DecisionTier::SafeStop);
  EXPECT_EQ(result.selected_policy, "no_safe_candidate");
}

TEST(DecisionCoordinator, ASingleSurvivorIsSelectedBeforeTierThree) {
  auto model = world();
  const Action forward(ActionType::Forward, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  DecisionCoordinator coordinator;
  coordinator.addVetoRule(std::make_unique<FixedVeto>(forward));
  coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
      "would-score-vetoed", std::vector<ActionScore>{{forward, 100.0}}));

  const std::vector<Action> candidates{forward, left};
  const auto result = coordinator.decide(DecisionContext{model}, candidates);
  EXPECT_EQ(result.action, left);
  EXPECT_EQ(result.tier, DecisionTier::TierOne);
  EXPECT_EQ(result.selected_policy, "tier1:only_surviving_action");
  EXPECT_TRUE(result.contributions.empty());
  ASSERT_EQ(result.decision_cycle.size(), 2U);
  EXPECT_EQ(result.decision_cycle[0].component, "veto_rule");
  EXPECT_EQ(result.decision_cycle[1].component, "viable_action_set");
}

TEST(DecisionCoordinator, RejectsNonFiniteAdvice) {
  auto model = world();
  const Action forward(ActionType::Forward, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  DecisionCoordinator coordinator;
  coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
      "nan", std::vector<ActionScore>{
                 {forward, std::numeric_limits<double>::quiet_NaN()}}));

  const std::vector<Action> candidates{forward, left};
  EXPECT_THROW(coordinator.decide(DecisionContext{model}, candidates),
               std::domain_error);
}

TEST(DecisionCoordinator, SameSeedProducesSameTieChoiceAndDiagnostics) {
  auto model = world();
  const std::vector<Action> candidates{Action(ActionType::Forward, 1U),
                                       Action(ActionType::TurnLeft, 1U),
                                       Action(ActionType::TurnRight, 1U)};
  semaforr::decision::ArbitrationConfiguration configuration;
  configuration.random_seed = 42U;
  DecisionCoordinator first(configuration);
  DecisionCoordinator second(configuration);
  first.addAdvisor(std::make_unique<FixedAdvisor>(
      "tie",
      std::vector<ActionScore>{
          {candidates[0], 1.0}, {candidates[1], 1.0}, {candidates[2], 1.0}}));
  second.addAdvisor(std::make_unique<FixedAdvisor>(
      "tie",
      std::vector<ActionScore>{
          {candidates[0], 1.0}, {candidates[1], 1.0}, {candidates[2], 1.0}}));

  const auto first_result = first.decide(DecisionContext{model}, candidates);
  const auto second_result = second.decide(DecisionContext{model}, candidates);
  EXPECT_EQ(first_result.action, second_result.action);
  EXPECT_EQ(first_result.contributions, second_result.contributions);
  auto ordered_candidates = candidates;
  std::sort(ordered_candidates.begin(), ordered_candidates.end());
  EXPECT_EQ(first_result.tier_three_tie_candidates, ordered_candidates);
  EXPECT_TRUE(first_result.tier_three_random_selection_used);
  EXPECT_EQ(first_result.tier_three_random_selection_index,
            second_result.tier_three_random_selection_index);
}

TEST(DecisionCoordinator, CompatibilityCommentsAreZeroToTenAndUnweighted) {
  auto model = world();
  const Action forward(ActionType::Forward, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  semaforr::decision::ArbitrationConfiguration configuration;
  configuration.scoring_policy =
      semaforr::decision::TierThreeScoringPolicy::CompatibilityComments;
  configuration.tie_policy = semaforr::decision::TierThreeTiePolicy::Exact;
  DecisionCoordinator coordinator(configuration);
  coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
      "compatibility", std::vector<ActionScore>{{forward, -7.0}, {left, 3.0}},
      100.0));
  const std::vector<Action> candidates{forward, left};
  const auto result = coordinator.decideTierThree({model}, candidates);
  ASSERT_EQ(result.contributions.size(), 2U);
  EXPECT_EQ(result.tier_three_scoring_policy,
            "compatibility_comments_0_10_unweighted");
  for (const auto& contribution : result.contributions) {
    EXPECT_GE(contribution.normalized_score, 0.0);
    EXPECT_LE(contribution.normalized_score, 10.0);
    EXPECT_DOUBLE_EQ(contribution.weighted_score,
                     contribution.normalized_score);
    EXPECT_DOUBLE_EQ(contribution.weight, 100.0);
  }
  EXPECT_EQ(result.action, left);
}

TEST(DecisionCoordinator, AdaptedPolicyPreservesRawNormalizedAndWeightedScores) {
  auto model = world();
  const Action forward(ActionType::Forward, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  semaforr::decision::ArbitrationConfiguration configuration;
  DecisionCoordinator coordinator(configuration);
  coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
      "adapted", std::vector<ActionScore>{{forward, 2.0}, {left, 6.0}}, 2.5,
      semaforr::decision::ScoreNormalization::SignedUnit));
  const std::vector<Action> candidates{forward, left};
  const auto result = coordinator.decideTierThree({model}, candidates);
  ASSERT_EQ(result.contributions.size(), 2U);
  const auto selected = std::find_if(
      result.contributions.begin(), result.contributions.end(),
      [&](const auto& contribution) { return contribution.action == left; });
  ASSERT_NE(selected, result.contributions.end());
  EXPECT_DOUBLE_EQ(selected->raw_score, 6.0);
  EXPECT_DOUBLE_EQ(selected->normalized_score, 1.0);
  EXPECT_DOUBLE_EQ(selected->weighted_score, 2.5);
  EXPECT_DOUBLE_EQ(selected->final_total, 2.5);
  EXPECT_TRUE(selected->viable);
}

TEST(DecisionCoordinator, ExactAndToleranceTiePoliciesAreDistinct) {
  auto model = world();
  const Action forward(ActionType::Forward, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  const std::vector<Action> candidates{forward, left};
  auto make = [&](semaforr::decision::TierThreeTiePolicy policy) {
    semaforr::decision::ArbitrationConfiguration configuration;
    configuration.tie_policy = policy;
    configuration.tie_tolerance = 1.0e-6;
    DecisionCoordinator coordinator(configuration);
    coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
        "nearly_tied",
        std::vector<ActionScore>{{forward, 1.0}, {left, 1.0 - 1.0e-9}}));
    return coordinator;
  };
  auto exact = make(semaforr::decision::TierThreeTiePolicy::Exact);
  auto tolerance = make(semaforr::decision::TierThreeTiePolicy::Tolerance);
  const auto exact_result = exact.decideTierThree({model}, candidates);
  const auto tolerance_result = tolerance.decideTierThree({model}, candidates);
  EXPECT_EQ(exact_result.tier_three_tie_candidates.size(), 1U);
  EXPECT_EQ(tolerance_result.tier_three_tie_candidates.size(), 2U);
  EXPECT_FALSE(exact_result.tier_three_random_selection_used);
  EXPECT_TRUE(tolerance_result.tier_three_random_selection_used);
}

TEST(DecisionCoordinator, NoAdvisorUsesConfiguredFallback) {
  auto model = world();
  const Action left(ActionType::TurnLeft, 1U);
  semaforr::decision::ArbitrationConfiguration configuration;
  configuration.fallback = left;
  DecisionCoordinator coordinator(configuration);
  const std::vector<Action> candidates{left,
                                       Action(ActionType::TurnRight, 1U)};
  const auto result = coordinator.decide(DecisionContext{model}, candidates);
  EXPECT_EQ(result.action, left);
  EXPECT_EQ(result.source, DecisionSource::Fallback);
  EXPECT_EQ(result.tier, DecisionTier::Fallback);
  EXPECT_EQ(result.selected_policy, "configured_fallback");
}

TEST(DecisionCoordinator, MandateStopsBeforeVetoesAndTierThree) {
  auto model = world();
  const Action forward(ActionType::Forward, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  DecisionCoordinator coordinator;
  coordinator.addMandatoryRule(std::make_unique<FixedMandatory>(
      "Victory", Decision{forward, "Victory", "target visible"}));
  coordinator.addVetoRule(std::make_unique<FixedVeto>(forward));
  coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
      "would-prefer-left", std::vector<ActionScore>{{left, 10.0}}));

  const auto result = coordinator.decide(DecisionContext{model},
                                         std::vector<Action>{forward, left});
  EXPECT_EQ(result.action, forward);
  EXPECT_EQ(result.selected_policy, "mandatory_rule:Victory");
  ASSERT_EQ(result.decision_cycle.size(), 1U);
  EXPECT_EQ(result.decision_cycle.front().component, "Victory");
  EXPECT_EQ(result.decision_cycle.front().outcome, "mandated_action");
  EXPECT_EQ(result.decision_cycle.front().final_attribution,
            DecisionTier::TierOne);
}

TEST(DecisionCoordinator, StagedTierOnePreservesRegisteredSemanticOrder) {
  auto model = world();
  const Action forward(ActionType::Forward, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  const Action right(ActionType::TurnRight, 1U);
  const std::vector<Action> candidates{forward, left, right};
  DecisionCoordinator coordinator;
  coordinator.addMandatoryRule(
      std::make_unique<FixedMandatory>("Victory", std::nullopt));
  coordinator.addVetoRule(
      std::make_unique<FixedVeto>(Action::pause(), "AvoidObstacles"));
  coordinator.addVetoRule(
      std::make_unique<FixedVeto>(Action::pause(), "NotOpposite"));
  coordinator.addVetoRule(
      std::make_unique<FixedVeto>(forward, "Forward"));
  coordinator.addVetoRule(
      std::make_unique<FixedVeto>(left, "Precedent"));

  const auto early = coordinator.evaluateTierOneStage(
      DecisionContext{model}, candidates,
      semaforr::decision::TierOneStage::BeforeEnforcer);
  ASSERT_EQ(early.trace.size(), 3U);
  EXPECT_EQ(early.trace[0].component, "Victory");
  EXPECT_EQ(early.trace[1].component, "AvoidObstacles");
  EXPECT_EQ(early.trace[2].component, "NotOpposite");
  auto ordered_candidates = candidates;
  std::sort(ordered_candidates.begin(), ordered_candidates.end());
  EXPECT_EQ(early.survivors, ordered_candidates);

  const auto late = coordinator.evaluateTierOneStage(
      DecisionContext{model}, early.survivors,
      semaforr::decision::TierOneStage::AfterLowLevelExploration);
  ASSERT_EQ(late.trace.size(), 2U);
  EXPECT_EQ(late.trace[0].component, "Forward");
  EXPECT_EQ(late.trace[1].component, "Precedent");
  ASSERT_EQ(late.survivors.size(), 1U);
  EXPECT_EQ(late.survivors.front(), right);
}

TEST(DecisionCoordinator, TierThreeRunsOnlyAfterTierOneContinues) {
  auto model = world();
  const Action forward(ActionType::Forward, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  DecisionCoordinator coordinator;
  coordinator.addMandatoryRule(
      std::make_unique<FixedMandatory>("Victory", std::nullopt));
  coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
      "Greedy", std::vector<ActionScore>{{forward, 2.0}, {left, 1.0}}));

  const auto result = coordinator.decide(DecisionContext{model},
                                         std::vector<Action>{forward, left});
  EXPECT_EQ(result.action, forward);
  EXPECT_EQ(result.tier, DecisionTier::TierThree);
  ASSERT_EQ(result.decision_cycle.size(), 3U);
  EXPECT_EQ(result.decision_cycle[0].tier, "tier1");
  EXPECT_EQ(result.decision_cycle[0].outcome, "no_mandate_continue");
  EXPECT_EQ(result.decision_cycle[1].component, "Greedy");
  EXPECT_EQ(result.decision_cycle[2].outcome, "advisor_vote_selected");
}

TEST(DecisionCoordinator,
     CircumstanceWeightingIsEvidenceGatedAndPreservesBaseTotals) {
  auto model = world();
  model.mission = semaforr::domain::Mission(
      {{1U, {4.0, 0.0}}}, 100U);
  ASSERT_TRUE(model.mission.activate_next());
  semaforr::domain::LaserObservation laser;
  laser.angle_min = semaforr::domain::Angle(-0.2);
  laser.angle_increment = semaforr::domain::Angle(0.1);
  laser.minimum_range = semaforr::domain::Distance(0.1);
  laser.maximum_range = semaforr::domain::Distance(5.0);
  laser.ranges_m = {2.0, 2.0, 2.0, 2.0, 2.0};
  model.robot.laser = laser;
  semaforr::domain::SettingNormalizationConfiguration normalization;
  normalization.assignment_confidence_threshold = 0.5;
  const auto setting = semaforr::domain::normalizeSetting(laser, normalization);
  auto& circumstances = model.spatial.circumstances;
  circumstances.minimum_cluster_size = 1U;
  circumstances.minimum_case_evidence = 10U;
  circumstances.assignment_confidence_threshold = 0.5;
  circumstances.accuracy_threshold = 0.75;
  circumstances.clusters.push_back({7U, setting, 20U, 1.0});
  const auto key = semaforr::domain::circumstanceCaseKey(
      7U, model.robot.pose, model.mission.active()->target, circumstances);
  const Action forward(ActionType::Forward, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  semaforr::domain::CircumstanceCaseEvidence evidence;
  evidence.key = key;
  evidence.evidence = 20U;
  evidence.accuracy = 0.95;
  semaforr::domain::ActionCaseEvidence good;
  good.action = forward;
  good.executed = 10U;
  good.effective_evidence = 10.0;
  good.confidence = 0.95;
  semaforr::domain::ActionCaseEvidence poor;
  poor.action = left;
  poor.executed = 10U;
  poor.effective_evidence = 10.0;
  poor.confidence = 0.05;
  evidence.actions = {good, poor};
  circumstances.cases.push_back(evidence);

  semaforr::decision::ArbitrationConfiguration configuration;
  configuration.circumstance_weighting_enabled = true;
  configuration.circumstance_minimum_evidence = 10U;
  configuration.circumstance_minimum_action_evidence = 5U;
  configuration.circumstance_minimum_assignment_confidence = 0.5;
  configuration.circumstance_minimum_case_accuracy = 0.75;
  configuration.circumstance_maximum_influence = 0.5;
  configuration.tie_policy = semaforr::decision::TierThreeTiePolicy::Exact;
  DecisionCoordinator coordinator(configuration);
  coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
      "base", std::vector<ActionScore>{{forward, 1.0}, {left, 1.1}}));
  const auto result = coordinator.decideTierThree(
      {model}, std::vector<Action>{forward, left});
  EXPECT_EQ(result.action, forward);
  EXPECT_TRUE(result.circumstance_weighting_applied);
  EXPECT_TRUE(result.circumstance_weighting_changed_winner);
  ASSERT_EQ(result.tier_three_totals.size(), 2U);
  const auto adjusted = std::find_if(
      result.tier_three_totals.begin(), result.tier_three_totals.end(),
      [&](const auto& total) { return total.action == forward; });
  ASSERT_NE(adjusted, result.tier_three_totals.end());
  EXPECT_DOUBLE_EQ(adjusted->pre_circumstance_total, 1.0);
  EXPECT_GT(adjusted->circumstance_multiplier, 1.0);
  EXPECT_EQ(adjusted->circumstance_action_evidence, 10U);
}

}  // namespace
