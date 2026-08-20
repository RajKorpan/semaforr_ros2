#include <gtest/gtest.h>

#include <semaforr_msgs/msg/advisor_contribution.hpp>
#include <semaforr_msgs/msg/decision_veto.hpp>
#include <semaforr_msgs/msg/explanation_question.hpp>
#include <semaforr_msgs/msg/plan_candidate_diagnostic.hpp>
#include <semaforr_msgs/msg/plan_step_trace.hpp>
#include <semaforr_msgs/msg/tier_three_action_total.hpp>
#include <why/why_system.hpp>

#include <algorithm>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>

namespace {

using semaforr::why::ExplanationQuestion;
using semaforr::why::UnifiedWhySystem;
using semaforr_msgs::msg::DecisionAction;
using semaforr_msgs::msg::DecisionRecord;

std::map<std::string, std::string> goldenExplanations() {
  std::ifstream stream(std::string(WHY_TEST_SOURCE_DIR) +
                       "/test/fixtures/why_explanations.golden");
  if (!stream) throw std::runtime_error("cannot open Why golden fixture");
  std::map<std::string, std::string> result;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line.front() == '#') continue;
    const auto separator = line.find('=');
    if (separator == std::string::npos)
      throw std::runtime_error("malformed Why golden fixture line");
    result.emplace(line.substr(0U, separator), line.substr(separator + 1U));
  }
  return result;
}

DecisionAction action(std::uint8_t type, std::uint32_t magnitude = 0U) {
  DecisionAction result;
  result.type = type;
  result.magnitude_index = magnitude;
  return result;
}

DecisionRecord tierThreeRecord() {
  DecisionRecord record;
  record.decision_id = 41U;
  record.action_id = 47U;
  record.execution_id = 47U;
  record.selected_tier = DecisionRecord::TIER_THREE;
  record.selected_action = action(DecisionAction::TURN_LEFT, 1U);
  record.candidates = {record.selected_action,
                       action(DecisionAction::FORWARD, 1U),
                       action(DecisionAction::TURN_RIGHT, 1U)};
  record.viable_actions = record.candidates;
  record.action_lifecycle_status = "selected";
  record.tier_three_scoring_policy = "compatibility_unweighted";
  record.tier_three_tie_policy = "seeded_exact_tie";
  record.decision_gini_agreement = 0.8;
  record.decision_standardized_total = 1.4;
  record.decision_relative_support = 0.6;
  record.decision_confidence_category = "high";
  record.source_provenance = {"current_sensor_readings", "active_plan"};

  semaforr_msgs::msg::AdvisorContribution contribution;
  contribution.advisor = "Greedy";
  contribution.action = record.selected_action;
  contribution.raw_score = 8.0;
  contribution.normalized_score = 0.6;
  contribution.advisor_mean = 5.0;
  contribution.advisor_standard_deviation = 2.0;
  contribution.relative_support = 1.5;
  contribution.weight = 1.0;
  contribution.weighted_score = 8.0;
  contribution.viable = true;
  contribution.final_total = 14.0;
  record.advisor_contributions.push_back(contribution);

  semaforr_msgs::msg::TierThreeActionTotal selected_total;
  selected_total.action = record.selected_action;
  selected_total.total = 14.0;
  record.tier_three_action_totals.push_back(selected_total);
  semaforr_msgs::msg::TierThreeActionTotal alternative_total;
  alternative_total.action = action(DecisionAction::FORWARD, 1U);
  alternative_total.total = 9.0;
  record.tier_three_action_totals.push_back(alternative_total);
  return record;
}

semaforr_msgs::msg::PlanCandidateDiagnostic plan(
    std::uint64_t id, const std::string& planner, double total) {
  semaforr_msgs::msg::PlanCandidateDiagnostic result;
  result.plan_id = id;
  result.planner = planner;
  result.plan_family = planner == "HighwayPlan" ? "highway" : "grid";
  result.objectives = {"distance", "crowd_risk"};
  result.raw_costs = {12.0 + total, 2.0 + total};
  result.normalized_costs = {total, total / 2.0};
  result.summed_score = total;
  result.metadata.planner_name = planner;
  result.metadata.plan_type = planner == "HighwayPlan" ? "model-based"
                                                        : "grid-based";
  result.metadata.objective_name = planner == "HighwayPlan"
                                       ? "highway travel"
                                       : "metric distance";
  result.metadata.objective_description = planner == "HighwayPlan"
      ? "prefer travel through the learned highway network and its intersections"
      : "minimize metric or graph path cost";
  result.metadata.representation_dependencies =
      planner == "HighwayPlan"
          ? std::vector<std::string>{"highway_graph", "skeleton"}
          : std::vector<std::string>{"planning_traversability"};
  geometry_msgs::msg::Point first;
  first.x = 1.0;
  geometry_msgs::msg::Point second;
  second.x = 1.0;
  second.y = 2.0;
  result.geometry = {first, second};
  return result;
}

TEST(WhyTraceStore, RetrievesStableHistoricalIdentifiersAndUpsertsLifecycle) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  record.has_plan = true;
  record.active_plan_id = 12U;
  record.has_planning_episode = true;
  record.planning_episode_id = 7U;
  why.record(record);
  ASSERT_NE(why.traces().decision(41U), nullptr);
  ASSERT_NE(why.traces().action(47U), nullptr);
  ASSERT_NE(why.traces().plan(12U), nullptr);
  ASSERT_NE(why.traces().planningEpisode(7U), nullptr);
  record.action_lifecycle_status = "completed";
  why.record(record);
  EXPECT_EQ(why.traces().action(47U)->action_lifecycle_status, "completed");
}

TEST(WhyDecision, PreservesTierThreeEvidenceAndConfidence) {
  UnifiedWhySystem why;
  why.record(tierThreeRecord());
  ExplanationQuestion question;
  question.question_id = 1U;
  question.question_type = ExplanationQuestion::WHY_DECISION;
  const auto answer = why.answer(question);
  EXPECT_TRUE(answer.found);
  EXPECT_EQ(answer.decision_id, 41U);
  EXPECT_EQ(answer.primary_reasoning_source, "tier_three_voting");
  EXPECT_NE(answer.natural_language_response.find("raw 8"), std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("strong support"),
            std::string::npos);

  question.question_type = ExplanationQuestion::DECISION_CONFIDENCE;
  const auto confidence = why.answer(question);
  EXPECT_EQ(confidence.confidence_category, "high");
  EXPECT_DOUBLE_EQ(confidence.gini_agreement, 0.8);
  EXPECT_NE(confidence.natural_language_response.find("not a guarantee"),
            std::string::npos);
}

TEST(WhyDecision, ExplainsCircumstanceEvidenceWithoutCallingItSafety) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  record.circumstance_match_available = true;
  record.circumstance_id = 42U;
  record.circumstance_assignment_confidence = 0.97;
  record.circumstance_learning_mode = "adapted_threshold";
  record.circumstance_model_version = "circumstance_case_v2";
  record.circumstance_weighting_policy =
      "evidence_gated_laplace_confidence";
  record.circumstance_weighting_applied = true;
  record.circumstance_weighting_changed_winner = false;
  auto& selected = record.tier_three_action_totals.front();
  selected.pre_circumstance_total = 12.0;
  selected.circumstance_multiplier = 1.1;
  selected.post_circumstance_total = 13.2;
  selected.circumstance_action_evidence = 20U;
  selected.circumstance_action_confidence = 0.9;
  why.record(record);
  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::WHY_DECISION;
  const auto answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find("20 effective outcomes"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("did not change"),
            std::string::npos);
  EXPECT_EQ(answer.natural_language_response.find("unsafe"),
            std::string::npos);
}

class TierOneMandateExplanation
    : public ::testing::TestWithParam<std::pair<std::string, std::string>> {};

TEST_P(TierOneMandateExplanation, UsesComponentSpecificSemantics) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  record.selected_tier = DecisionRecord::TIER_ONE;
  record.selected_policy = GetParam().first;
  why.record(record);
  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::WHY_DECISION;
  const auto answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find(GetParam().second),
            std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(
    TierOneComponents, TierOneMandateExplanation,
    ::testing::Values(
        std::pair{"mandatory_rule:Victory", "target was directly reachable"},
        std::pair{"reactive:Thru", "tight opening"},
        std::pair{"reactive:Behind", "outside the current view"},
        std::pair{"reactive:Out", "confined area"},
        std::pair{"reactive:LLE", "lacked sufficient learned connectivity"},
        std::pair{"hle:PursueCandidate", "passage candidate"},
        std::pair{"mandatory_rule:AvoidObstacles", "observed clearance"},
        std::pair{"mandatory_rule:NotOpposite", "orientation-history"},
        std::pair{"mandatory_rule:Forward", "visited space"},
        std::pair{"mandatory_rule:Precedent", "circumstance evidence"}));

TEST(WhyDecision, DistinguishesGridAndModelEnforcer) {
  UnifiedWhySystem why;
  auto grid = tierThreeRecord();
  grid.decision_id = 50U;
  grid.action_id = 51U;
  grid.selected_tier = DecisionRecord::TIER_ONE;
  grid.selected_policy = "mandatory_rule:Enforcer:grid";
  grid.enforcer_mode = "grid";
  grid.active_plan_step = 3U;
  grid.has_operational_target = true;
  grid.operational_target.x = 2.0;
  grid.enforcer_reason = "farthest_reachable_waypoint";
  why.record(grid);
  ExplanationQuestion question;
  question.has_decision_id = true;
  question.decision_id = 50U;
  const auto grid_answer = why.answer(question);
  EXPECT_NE(grid_answer.natural_language_response.find("path index 3"),
            std::string::npos);
  EXPECT_NE(grid_answer.natural_language_response.find("waypoint (2"),
            std::string::npos);

  auto model = grid;
  model.decision_id = 52U;
  model.action_id = 53U;
  model.selected_policy = "mandatory_rule:Enforcer:model";
  model.enforcer_mode = "model";
  model.has_plan = true;
  model.active_plan_id = 12U;
  model.active_plan_step = 0U;
  model.planning_candidates = {plan(12U, "HighwayPlan", 0.4)};
  semaforr_msgs::msg::PlanStepTrace step;
  step.step_type = "intersection";
  step.primary_entity_id = 9U;
  model.planning_candidates.front().typed_steps.push_back(step);
  why.record(model);
  question.decision_id = 52U;
  const auto model_answer = why.answer(question);
  EXPECT_NE(model_answer.natural_language_response.find("intersection 9"),
            std::string::npos);
}

class LifecycleExplanation :
    public ::testing::TestWithParam<std::string> {};

TEST_P(LifecycleExplanation, NeverConflatesSelectionAndExecution) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  record.action_lifecycle_status = GetParam();
  why.record(record);
  ExplanationQuestion question;
  const auto answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find(GetParam()),
            std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(
    ActionLifecycle, LifecycleExplanation,
    ::testing::Values("selected", "commanded", "started", "completed",
                      "partial_movement", "cancelled", "timed_out",
                      "controller_failure", "safety_interrupted",
                      "goal_preempted"));

TEST(WhyCounterfactual, DistinguishesSafetyCognitiveAndLowerPreference) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  semaforr_msgs::msg::DecisionVeto veto;
  veto.action = action(DecisionAction::TURN_RIGHT, 1U);
  veto.rule = "NotOpposite";
  veto.reason_code = "opposes_recent_orientation";
  veto.rejection_kind = "cognitive";
  veto.explanation_category = "opposes recent orientation";
  record.vetoes.push_back(veto);
  why.record(record);

  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::WHY_NOT_ACTION;
  question.has_alternative_action = true;
  question.alternative_action = veto.action;
  auto answer = why.answer(question);
  EXPECT_EQ(answer.primary_reasoning_source, "NotOpposite");
  EXPECT_EQ(answer.structured_facts.at(1), "cognitive");
  EXPECT_EQ(answer.natural_language_response.find("unsafe"),
            std::string::npos);

  question.alternative_action = action(DecisionAction::FORWARD, 1U);
  answer = why.answer(question);
  EXPECT_EQ(answer.primary_reasoning_source, "lower_preference");
  EXPECT_NE(answer.natural_language_response.find("not classified as unsafe"),
            std::string::npos);
}

TEST(WhyCounterfactual, ReportsNotGeneratedHardSafetyTieAndPlanEnforcement) {
  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::WHY_NOT_ACTION;
  question.has_alternative_action = true;

  UnifiedWhySystem not_generated;
  auto record = tierThreeRecord();
  question.alternative_action = action(DecisionAction::FORWARD, 7U);
  not_generated.record(record);
  EXPECT_EQ(not_generated.answer(question).primary_reasoning_source,
            "candidate_generation");

  UnifiedWhySystem safety;
  semaforr_msgs::msg::DecisionVeto veto;
  veto.action = action(DecisionAction::FORWARD, 1U);
  veto.rule = "HardSafetyFilter";
  veto.reason_code = "collision_predicted";
  veto.rejection_kind = "safety";
  veto.explanation_category = "unsafe";
  record.vetoes = {veto};
  safety.record(record);
  question.alternative_action = veto.action;
  const auto safety_answer = safety.answer(question);
  EXPECT_EQ(safety_answer.structured_facts.at(1), "safety");
  EXPECT_NE(safety_answer.natural_language_response.find("unsafe"),
            std::string::npos);

  UnifiedWhySystem tied;
  record.vetoes.clear();
  record.tier_three_tie_candidates = {
      record.selected_action, action(DecisionAction::FORWARD, 1U)};
  tied.record(record);
  const auto tie_answer = tied.answer(question);
  EXPECT_EQ(tie_answer.primary_reasoning_source, "tie_breaking");

  UnifiedWhySystem enforced;
  record.tier_three_tie_candidates.clear();
  record.selected_tier = DecisionRecord::TIER_ONE;
  record.has_plan = true;
  record.active_plan_id = 12U;
  record.active_plan_step = 2U;
  enforced.record(record);
  const auto plan_answer = enforced.answer(question);
  EXPECT_EQ(plan_answer.primary_reasoning_source, "plan_enforcement");
  EXPECT_NE(plan_answer.natural_language_response.find("plan step 2"),
            std::string::npos);
}

TEST(WhyPlan, ExplainsSelectionAlternativesConfidenceAndTypedHighwayRoute) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  record.has_plan = true;
  record.active_plan_id = 12U;
  record.planning_candidates = {plan(12U, "HighwayPlan", 0.4),
                                plan(13U, "DistancePlan", 1.2)};
  semaforr_msgs::msg::PlanStepTrace entry;
  entry.step_id = 1U;
  entry.step_type = "highway_entry";
  entry.primary_entity_id = 4U;
  semaforr_msgs::msg::PlanStepTrace travel;
  travel.step_id = 2U;
  travel.step_type = "highway";
  travel.primary_entity_id = 4U;
  travel.secondary_entity_id = 9U;
  semaforr_msgs::msg::PlanStepTrace intersection;
  intersection.step_id = 3U;
  intersection.step_type = "intersection";
  intersection.primary_entity_id = 9U;
  semaforr_msgs::msg::PlanStepTrace exit;
  exit.step_id = 4U;
  exit.step_type = "highway_exit";
  exit.primary_entity_id = 8U;
  record.planning_candidates[0].typed_steps =
      {entry, travel, intersection, exit};
  why.record(record);

  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::WHY_PLAN;
  auto answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find("learned highway network"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("DistancePlan"),
            std::string::npos);

  question.question_type = ExplanationQuestion::ALTERNATIVE_PLAN;
  answer = why.answer(question);
  EXPECT_EQ(answer.plan_id, 13U);

  question.question_type = ExplanationQuestion::PLAN_CONFIDENCE;
  answer = why.answer(question);
  EXPECT_GT(answer.confidence_value, 0.0);

  question.question_type = ExplanationQuestion::ROUTE_DESCRIPTION;
  answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find("intersection 9"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("exit highway 8"),
            std::string::npos);
  EXPECT_FALSE(answer.exact_distances_m.empty());
  EXPECT_EQ(answer.exact_distances_m.size(),
            answer.direction_categories.size());
}

TEST(WhyHypothetical, UsesImmutableCallbackWithoutRecordingARealDecision) {
  UnifiedWhySystem why;
  int evaluations = 0;
  why.setHypotheticalEvaluator([&](const geometry_msgs::msg::Pose2D& pose) {
    ++evaluations;
    auto record = tierThreeRecord();
    record.robot_pose = pose;
    return record;
  });
  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::HYPOTHETICAL_POSE;
  question.has_hypothetical_pose = true;
  question.hypothetical_pose.x = -4.0;
  const auto first = why.answer(question);
  const auto second = why.answer(question);
  EXPECT_TRUE(first.hypothetical);
  EXPECT_TRUE(second.hypothetical);
  EXPECT_EQ(evaluations, 2);
  EXPECT_TRUE(why.traces().decisionIds().empty());
  EXPECT_EQ(first.natural_language_response, second.natural_language_response);
}

TEST(WhyPlanComparison, RefusesToInventUserRouteCostsWithoutEvaluator) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  record.has_plan = true;
  record.active_plan_id = 12U;
  record.planning_candidates = {plan(12U, "HighwayPlan", 0.4)};
  why.record(record);
  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::COMPARE_PLAN;
  geometry_msgs::msg::Point point;
  question.user_route.push_back(point);
  const auto unavailable = why.answer(question);
  EXPECT_FALSE(unavailable.found);
  EXPECT_EQ(unavailable.primary_reasoning_source,
            "route_evaluator_unavailable");

  why.setRouteEvaluator([](const auto&, const auto& objectives) {
    return std::vector<double>(objectives.size(), 3.0);
  });
  const auto evaluated = why.answer(question);
  EXPECT_TRUE(evaluated.found);
  EXPECT_EQ(evaluated.structured_facts.size(), 2U);
}

TEST(WhyGoldenFixtures, PreserveEveryPublicExplanationCategory) {
  const auto golden = goldenExplanations();
  ASSERT_EQ(golden.size(), 8U);
  UnifiedWhySystem why;

  auto record = tierThreeRecord();
  semaforr_msgs::msg::AdvisorContribution opposition =
      record.advisor_contributions.front();
  opposition.advisor = "GoAround";
  opposition.relative_support = -1.6;
  opposition.raw_score = 1.0;
  record.advisor_contributions.push_back(opposition);
  record.has_plan = true;
  record.active_plan_id = 12U;
  record.planning_candidates = {plan(12U, "HighwayPlan", 0.4),
                                plan(13U, "DistancePlan", 1.2)};
  why.record(record);

  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::WHY_DECISION;
  auto answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find(golden.at("tier3_support")),
            std::string::npos);
  EXPECT_NE(
      answer.natural_language_response.find(golden.at("tier3_opposition")),
      std::string::npos);

  question.question_type = ExplanationQuestion::DECISION_CONFIDENCE;
  answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find(golden.at("confidence")),
            std::string::npos);

  question.question_type = ExplanationQuestion::WHY_NOT_ACTION;
  question.has_alternative_action = true;
  question.alternative_action = action(DecisionAction::FORWARD, 1U);
  answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find(golden.at("counterfactual")),
            std::string::npos);

  question = ExplanationQuestion();
  question.question_type = ExplanationQuestion::COMPARE_PLAN;
  question.has_alternative_plan_id = true;
  question.alternative_plan_id = 13U;
  answer = why.answer(question);
  EXPECT_NE(
      answer.natural_language_response.find(golden.at("plan_comparison")),
      std::string::npos);

  question = ExplanationQuestion();
  question.question_type = ExplanationQuestion::ALTERNATIVE_PLAN;
  answer = why.answer(question);
  EXPECT_NE(
      answer.natural_language_response.find(golden.at("alternative_route")),
      std::string::npos);

  question.question_type = ExplanationQuestion::ROUTE_DESCRIPTION;
  answer = why.answer(question);
  EXPECT_NE(std::find(answer.direction_categories.begin(),
                      answer.direction_categories.end(),
                      golden.at("egocentric")),
            answer.direction_categories.end());

  UnifiedWhySystem tier_one;
  auto tier_one_record = tierThreeRecord();
  tier_one_record.selected_tier = DecisionRecord::TIER_ONE;
  tier_one_record.selected_policy = "mandatory_rule:Victory";
  tier_one.record(tier_one_record);
  question = ExplanationQuestion();
  question.question_type = ExplanationQuestion::WHY_DECISION;
  answer = tier_one.answer(question);
  EXPECT_NE(answer.natural_language_response.find(golden.at("tier1")),
            std::string::npos);
}

}  // namespace
