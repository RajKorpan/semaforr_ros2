#include <gtest/gtest.h>

#include <semaforr_msgs/msg/advisor_contribution.hpp>
#include <semaforr_msgs/msg/decision_veto.hpp>
#include <semaforr_msgs/msg/explanation_question.hpp>
#include <semaforr_msgs/msg/plan_candidate_diagnostic.hpp>
#include <semaforr_msgs/msg/plan_step_trace.hpp>
#include <semaforr_msgs/msg/tier_three_action_total.hpp>
#include <why/why_system.hpp>

#include <algorithm>
#include <cmath>
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
  record.decision_selected_comment_sum = 22.0;
  record.decision_advisor_count = 4U;
  record.decision_normalized_support_proportion = 0.55;
  record.decision_action_total_mean = 16.75;
  record.decision_action_total_standard_deviation = std::sqrt(188.75 / 3.0);
  record.decision_gamma = 0.495;
  record.decision_zeta =
      (22.0 - record.decision_action_total_mean) /
      record.decision_action_total_standard_deviation;
  record.decision_lambda =
      (0.5 - record.decision_gamma) * record.decision_zeta;
  record.decision_gini_agreement = record.decision_gamma;
  record.decision_standardized_total = record.decision_zeta;
  record.decision_relative_support = record.decision_lambda;
  record.decision_agreement_category = "my reasons conflict";
  record.decision_support_category = "don't really want";
  record.decision_confidence_category = "not";
  record.source_provenance = {"current_sensor_readings", "active_plan"};

  semaforr_msgs::msg::AdvisorContribution contribution;
  contribution.advisor = "Greedy";
  contribution.action = record.selected_action;
  contribution.raw_score = 8.0;
  contribution.normalized_score = 10.0;
  contribution.advisor_mean = 3.0;
  contribution.advisor_standard_deviation = std::sqrt(22.0);
  contribution.relative_support = 7.0 / std::sqrt(22.0);
  contribution.weight = 1.0;
  contribution.weighted_score = 8.0;
  contribution.viable = true;
  contribution.final_total = 14.0;
  record.advisor_contributions.push_back(contribution);

  semaforr_msgs::msg::TierThreeActionTotal selected_total;
  selected_total.action = record.selected_action;
  selected_total.total = 14.0;
  selected_total.chapter_five_comment_total = 22.0;
  record.tier_three_action_totals.push_back(selected_total);
  semaforr_msgs::msg::TierThreeActionTotal alternative_total;
  alternative_total.action = action(DecisionAction::FORWARD, 1U);
  alternative_total.total = 9.0;
  alternative_total.chapter_five_comment_total = 19.0;
  record.tier_three_action_totals.push_back(alternative_total);
  return record;
}

semaforr_msgs::msg::PlanCandidateDiagnostic plan(
    std::uint64_t id, const std::string& planner, double total) {
  semaforr_msgs::msg::PlanCandidateDiagnostic result;
  result.plan_id = id;
  result.planner = planner;
  result.plan_family = planner == "HighwayPlan" ? "highway" : "grid";
  result.objectives = {"distance", "highway_distance"};
  result.raw_costs = planner == "HighwayPlan"
                         ? std::vector<double>{14.0, -100.0}
                         : std::vector<double>{10.0, -50.0};
  result.normalized_costs = {total, total / 2.0};
  result.summed_score = total;
  result.metadata.planner_name = planner;
  result.metadata.plan_type = planner == "HighwayPlan" ? "model-based"
                                                        : "grid-based";
  result.metadata.objective_name = planner == "HighwayPlan"
                                       ? "highway_distance"
                                       : "distance";
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

TEST(WhyDecision, PreservesAttemptedActionOutcomeAndActualReachedPose) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  record.action_lifecycle_status = "partial_movement";
  record.has_execution_result = true;
  record.execution_start_pose.x = 1.0;
  record.execution_start_pose.y = 2.0;
  record.execution_final_pose.x = 1.35;
  record.execution_final_pose.y = 2.1;
  record.distance_achieved_m = 0.36;
  record.rotation_achieved_rad = 0.12;
  record.execution_cancellation_reason = "local safety stop";
  why.record(record);
  ExplanationQuestion question;
  question.question_id = 99U;
  question.question_type = ExplanationQuestion::WHY_DECISION;
  const auto answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find("turn left"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("partial_movement"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("actually reached (1.35, 2.1)"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("local safety stop"),
            std::string::npos);
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
  EXPECT_FALSE(answer.structured_facts.empty());
  EXPECT_NE(answer.structured_facts.front().find("raw=8"), std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("want to get close"),
            std::string::npos);

  question.question_type = ExplanationQuestion::DECISION_CONFIDENCE;
  const auto confidence = why.answer(question);
  EXPECT_EQ(confidence.confidence_category, "not");
  EXPECT_DOUBLE_EQ(confidence.gini_agreement, 0.495);
  EXPECT_NE(confidence.natural_language_response.find("gamma=0.495"),
            std::string::npos);
}

TEST(WhyDecision, ReportsReadyAndDegradedSocialEvidence) {
  UnifiedWhySystem why;
  auto ready = tierThreeRecord();
  ready.social_input_source = "social_context_tracked";
  ready.social_prediction_source = "gst";
  ready.social_input_status = "ready";
  ready.live_social_revision = 3U;
  ready.crowd_density_revision = 5U;
  ready.crowd_risk_revision = 6U;
  ready.crowd_flow_revision = 7U;
  ready.formation_evidence_participated = true;
  why.record(ready);
  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::WHY_DECISION;
  auto answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find(
                "Social evidence came from social_context_tracked with gst"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find(
                "Formation evidence participated"),
            std::string::npos);
  EXPECT_TRUE(std::any_of(answer.structured_facts.begin(),
                          answer.structured_facts.end(), [](const auto& fact) {
                            return fact ==
                                   "crowd_revisions=live:3,density:5,risk:6,flow:7";
                          }));

  auto degraded = ready;
  degraded.decision_id = 42U;
  degraded.social_input_status = "missing_or_stale";
  degraded.formation_evidence_participated = false;
  why.record(degraded);
  question.has_decision_id = true;
  question.decision_id = 42U;
  answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find(
                "Social input was degraded (missing_or_stale)"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find(
                "was not treated as current"),
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
  intersection.has_target = true;
  intersection.target.x = 1.0;
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
  EXPECT_EQ(answer.confidence_category, "really");
  EXPECT_EQ(answer.primary_reasoning_source, "chapter_five_table_5_10");

  question.question_type = ExplanationQuestion::ROUTE_DESCRIPTION;
  answer = why.answer(question);
  EXPECT_NE(answer.natural_language_response.find("turn left at an intersection"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("We will"),
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

TEST(WhyDecisionConfidence, UsesChapterFiveIntervalsAndZeroVariancePolicy) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  record.decision_gamma = 0.1;
  record.decision_zeta = 2.0;
  record.decision_lambda = 0.8;
  why.record(record);
  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::DECISION_CONFIDENCE;
  auto answer = why.answer(question);
  EXPECT_EQ(answer.confidence_category, "really");
  EXPECT_DOUBLE_EQ(answer.confidence_value, 0.8);
  EXPECT_NE(answer.natural_language_response.find("many reasons"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("really want"),
            std::string::npos);

  record.decision_id = 42U;
  record.decision_gamma = 0.5;
  record.decision_zeta = 0.0;
  record.decision_lambda = 0.0;
  record.decision_action_total_standard_deviation = 0.0;
  why.record(record);
  question.has_decision_id = true;
  question.decision_id = 42U;
  answer = why.answer(question);
  EXPECT_TRUE(std::isfinite(answer.confidence_value));
  EXPECT_EQ(answer.confidence_category, "not");
  EXPECT_NE(answer.natural_language_response.find("reasons conflict"),
            std::string::npos);
}

TEST(WhyDecisionExplanation, OmitsWeakAdvisorCommentsAndUsesRelativeBands) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  auto weak = record.advisor_contributions.front();
  weak.advisor = "Novelty";
  weak.relative_support = 0.75;
  auto opposition = weak;
  opposition.advisor = "GoAround";
  opposition.relative_support = -1.6;
  record.advisor_contributions.push_back(weak);
  record.advisor_contributions.push_back(opposition);
  why.record(record);
  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::WHY_DECISION;
  const auto answer = why.answer(question);
  EXPECT_EQ(answer.natural_language_response.find("go somewhere new"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("really don't want"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("turn towards this wall"),
            std::string::npos);
}

TEST(WhyRoute, UsesChapterFiveTurnsDistancesAndAlternativeTemplate) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  record.robot_pose.x = 0.0;
  record.robot_pose.y = 0.0;
  record.has_plan = true;
  record.active_plan_id = 12U;
  auto selected = plan(12U, "DistancePlan", 0.4);
  geometry_msgs::msg::Point east;
  east.x = 1.01;
  geometry_msgs::msg::Point north = east;
  north.y = 7.01;
  geometry_msgs::msg::Point west = north;
  west.x = -4.0;
  selected.geometry = {east, north, west};
  auto alternative = plan(13U, "DistancePlan", 1.2);
  alternative.geometry = {east};
  record.planning_candidates = {selected, alternative};
  why.record(record);

  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::ROUTE_DESCRIPTION;
  auto answer = why.answer(question);
  EXPECT_EQ(answer.natural_language_response,
            "We will go straight about 2 meters, turn left, go straight "
            "about 8 meters, turn left, and go straight about 6 meters to "
            "reach our target.");
  ASSERT_EQ(answer.direction_categories.size(), 5U);
  EXPECT_EQ(answer.direction_categories[1], "turn left");
  EXPECT_EQ(answer.distance_categories[0], "2 meters");

  question.question_type = ExplanationQuestion::ALTERNATIVE_PLAN;
  question.has_alternative_plan_id = true;
  question.alternative_plan_id = 13U;
  answer = why.answer(question);
  EXPECT_EQ(answer.natural_language_response,
            "We could go straight about 2 meters to reach our target.");
}

TEST(WhyRoute, CoversEveryTableFiveElevenDirectionAndWraparound) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  record.robot_pose.x = 0.0;
  record.robot_pose.y = 0.0;
  record.has_plan = true;
  record.active_plan_id = 20U;
  auto selected = plan(20U, "DistancePlan", 0.0);
  selected.geometry.clear();
  double x = 0.0;
  double y = 0.0;
  constexpr double pi = 3.14159265358979323846;
  for (const double angle :
       {0.0, pi / 4.0, pi / 2.0, 3.0 * pi / 4.0, pi,
        -3.0 * pi / 4.0, -pi / 2.0, -pi / 4.0, 0.0}) {
    geometry_msgs::msg::Point point;
    x += 0.5 * std::cos(angle);
    y += 0.5 * std::sin(angle);
    point.x = x;
    point.y = y;
    selected.geometry.push_back(point);
  }
  record.planning_candidates = {selected};
  why.record(record);
  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::ROUTE_DESCRIPTION;
  const auto answer = why.answer(question);
  ASSERT_EQ(answer.direction_categories.size(), 17U);
  for (std::size_t index = 1U; index < answer.direction_categories.size();
       index += 2U)
    EXPECT_EQ(answer.direction_categories[index], "turn left a little");
}

TEST(WhyRoute, CoversTableFiveThirteenDistanceBoundaries) {
  const std::vector<std::pair<double, std::string>> cases{
      {1.0, "1 meter"},       {1.01, "2 meters"},
      {2.0, "2 meters"},      {2.01, "4 meters"},
      {10.0, "10 meters"},    {10.01, "15 meters"},
      {15.0, "15 meters"},    {50.0, "50 meters"},
      {50.01, "60 meters"},   {110.0, "110 meters"},
      {110.01, "more than 110 meters"}};
  std::uint64_t decision_id = 100U;
  for (const auto& [distance, expected] : cases) {
    UnifiedWhySystem why;
    auto record = tierThreeRecord();
    record.decision_id = decision_id++;
    record.robot_pose.x = 0.0;
    record.robot_pose.y = 0.0;
    record.has_plan = true;
    record.active_plan_id = record.decision_id;
    auto selected = plan(record.active_plan_id, "DistancePlan", 0.0);
    geometry_msgs::msg::Point endpoint;
    endpoint.x = distance;
    selected.geometry = {endpoint};
    record.planning_candidates = {selected};
    why.record(record);
    ExplanationQuestion question;
    question.question_type = ExplanationQuestion::ROUTE_DESCRIPTION;
    const auto answer = why.answer(question);
    ASSERT_EQ(answer.distance_categories.size(), 1U);
    EXPECT_EQ(answer.distance_categories.front(), expected);
  }
}

TEST(WhyPlanConfidence, RequiresRecordedComparablePlanAndUsesTableFiveTen) {
  UnifiedWhySystem why;
  auto record = tierThreeRecord();
  record.has_plan = true;
  record.active_plan_id = 12U;
  record.planning_candidates = {plan(12U, "HighwayPlan", 0.4)};
  why.record(record);
  ExplanationQuestion question;
  question.question_type = ExplanationQuestion::PLAN_CONFIDENCE;
  auto answer = why.answer(question);
  EXPECT_FALSE(answer.found);
  EXPECT_EQ(answer.confidence_category, "unavailable");
  EXPECT_NE(answer.natural_language_response.find("did not fabricate"),
            std::string::npos);

  record.decision_id = 43U;
  record.planning_candidates.push_back(plan(13U, "DistancePlan", 1.2));
  why.record(record);
  question.has_decision_id = true;
  question.decision_id = 43U;
  answer = why.answer(question);
  EXPECT_TRUE(answer.found);
  EXPECT_EQ(answer.confidence_category, "really");
  EXPECT_NE(answer.structured_facts.back().find("really"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("better at following long hallways"),
            std::string::npos);
  EXPECT_NE(answer.natural_language_response.find("longer"),
            std::string::npos);

  record.decision_id = 44U;
  record.planning_candidates = {plan(12U, "HighwayPlan", 0.4),
                                plan(13U, "DistancePlan", 1.2)};
  record.planning_candidates[0].raw_costs = {12.0, -10.0};
  record.planning_candidates[1].raw_costs = {10.0, 0.0};
  why.record(record);
  question.decision_id = 44U;
  answer = why.answer(question);
  EXPECT_EQ(answer.confidence_category, "only somewhat");

  record.decision_id = 45U;
  record.planning_candidates[0].raw_costs = {20.0, -2.0};
  record.planning_candidates[1].raw_costs = {10.0, 0.0};
  why.record(record);
  question.decision_id = 45U;
  answer = why.answer(question);
  EXPECT_EQ(answer.confidence_category, "not");
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
