#include <why/why_system.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace semaforr::why {
namespace {

constexpr double kPi = 3.14159265358979323846;

using Action = semaforr_msgs::msg::DecisionAction;
using PlanCandidate = semaforr_msgs::msg::PlanCandidateDiagnostic;

bool sameAction(const Action& left, const Action& right) {
  return left.type == right.type &&
         left.magnitude_index == right.magnitude_index;
}

std::string actionText(const Action& action) {
  switch (action.type) {
    case Action::FORWARD: return "move forward";
    case Action::TURN_RIGHT: return "turn right";
    case Action::TURN_LEFT: return "turn left";
    case Action::PAUSE: return "pause";
    default: return "stop";
  }
}

std::string relativeMagnitude(double support) {
  const double magnitude = std::abs(support);
  const std::string strength = magnitude >= 1.5 ? "strong"
                               : magnitude >= 0.5 ? "moderate"
                               : magnitude > 1.0e-9 ? "weak"
                                                   : "neutral";
  if (strength == "neutral") return strength;
  return strength + (support > 0.0 ? " support" : " opposition");
}

const PlanCandidate* selectedPlan(const DecisionRecord& record) {
  if (!record.has_plan) return nullptr;
  const auto found = std::find_if(
      record.planning_candidates.begin(), record.planning_candidates.end(),
      [&](const auto& candidate) {
        return candidate.plan_id == record.active_plan_id;
      });
  return found == record.planning_candidates.end() ? nullptr : &*found;
}

double pathLength(const std::vector<geometry_msgs::msg::Point>& path) {
  double result = 0.0;
  for (std::size_t index = 1U; index < path.size(); ++index)
    result += std::hypot(path[index].x - path[index - 1U].x,
                         path[index].y - path[index - 1U].y);
  return result;
}

std::string directionCategory(double angle) {
  const double absolute = std::abs(angle);
  if (absolute <= 0.17) return "straight ahead";
  if (absolute >= 2.62) return "behind";
  const std::string side = angle > 0.0 ? "left" : "right";
  if (absolute <= 0.61) return "slightly " + side;
  if (absolute <= 1.92) return side;
  return "sharply " + side;
}

std::string distanceCategory(double distance) {
  if (distance < 0.75) return "very near";
  if (distance < 2.0) return "near";
  if (distance < 5.0) return "a moderate distance";
  if (distance < 10.0) return "far";
  return "very far";
}

std::uint8_t inferredQuestionType(const ExplanationQuestion& question) {
  if (question.natural_language_question.empty() &&
      question.question_type <= ExplanationQuestion::COMBINED)
    return question.question_type;
  std::string text = question.natural_language_question;
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char value) {
                   return static_cast<char>(std::tolower(value));
                 });
  if (text.find("another way") != std::string::npos)
    return ExplanationQuestion::ALTERNATIVE_PLAN;
  if (text.find("getting there") != std::string::npos ||
      text.find("how are we") != std::string::npos)
    return ExplanationQuestion::ROUTE_DESCRIPTION;
  if (text.find("plan") != std::string::npos &&
      text.find("sure") != std::string::npos)
    return ExplanationQuestion::PLAN_CONFIDENCE;
  if (text.find("better than") != std::string::npos)
    return ExplanationQuestion::COMPARE_PLAN;
  if (text.find("why not") != std::string::npos ||
      text.find("instead") != std::string::npos)
    return ExplanationQuestion::WHY_NOT_ACTION;
  if (text.find("if you were") != std::string::npos)
    return ExplanationQuestion::HYPOTHETICAL_POSE;
  if (text.find("why") != std::string::npos &&
      (text.find("route") != std::string::npos ||
       text.find("highway") != std::string::npos ||
       text.find("region") != std::string::npos))
    return ExplanationQuestion::COMBINED;
  if (text.find("plan") != std::string::npos)
    return ExplanationQuestion::WHY_PLAN;
  if (text.find("sure") != std::string::npos)
    return ExplanationQuestion::DECISION_CONFIDENCE;
  return ExplanationQuestion::WHY_DECISION;
}

std::string tierOnePhrase(const DecisionRecord& record) {
  const std::string& policy = record.selected_policy;
  if (policy.find("Victory") != std::string::npos)
    return "Victory selected it because the target was directly reachable";
  if (policy.find("Enforcer") != std::string::npos) {
    if (record.enforcer_mode == "grid") {
      std::ostringstream text;
      text << "GridPlanEnforcer selected it at path index "
           << record.active_plan_step;
      if (record.has_operational_target)
        text << " toward waypoint (" << record.operational_target.x << ", "
             << record.operational_target.y << ')';
      if (!record.enforcer_reason.empty())
        text << " using " << record.enforcer_reason;
      return text.str();
    }
    std::string step = "model step";
    if (const auto* plan = selectedPlan(record);
        plan && record.active_plan_step < plan->typed_steps.size()) {
      const auto& typed = plan->typed_steps[record.active_plan_step];
      step = typed.step_type + " " +
             std::to_string(typed.primary_entity_id);
    }
    return "ModelPlanEnforcer selected it to operationalize " + step +
           (record.enforcer_reason.empty()
                ? std::string{}
                : " using " + record.enforcer_reason);
  }
  if (policy.find("Thru") != std::string::npos)
    return "Thru retained control to pass through a tight opening";
  if (policy.find("Behind") != std::string::npos)
    return "Behind retained control to recover a target or waypoint outside the current view";
  if (policy.find("Out") != std::string::npos)
    return "Out retained control to escape the recently known confined area";
  if (policy.find("LLE") != std::string::npos)
    return "LLE retained control because target-directed planning lacked sufficient learned connectivity";
  if (policy.find("HLE") != std::string::npos ||
      policy.find("hle:") != std::string::npos)
    return "HLE selected it while pursuing the current passage candidate";
  if (policy.find("AvoidObstacles") != std::string::npos)
    return "AvoidObstacles selected it from motions with sufficient observed clearance";
  if (policy.find("NotOpposite") != std::string::npos)
    return "NotOpposite selected it after applying the execution-confirmed orientation-history constraint";
  if (policy.find("Forward") != std::string::npos)
    return "Forward selected it to avoid returning to footprint-sized visited space";
  if (policy.find("Precedent") != std::string::npos)
    return "Precedent selected it using the recorded high-confidence circumstance evidence";
  return "the recorded Tier-1 component mandated it";
}

}  // namespace

void ExplanationTraceStore::record(const DecisionRecord& record) {
  decisions_.insert_or_assign(record.decision_id, record);
  action_to_decision_[record.action_id] = record.decision_id;
  if (record.execution_id != 0U)
    execution_to_decision_[record.execution_id] = record.decision_id;
  if (record.has_task)
    task_to_decision_[record.task.task_index] = record.decision_id;
  if (record.has_plan) plan_to_decision_[record.active_plan_id] = record.decision_id;
  if (record.has_planning_episode)
    episode_to_decision_[record.planning_episode_id] = record.decision_id;
}

const DecisionRecord* ExplanationTraceStore::latestDecision() const noexcept {
  return decisions_.empty() ? nullptr : &decisions_.rbegin()->second;
}

const DecisionRecord* ExplanationTraceStore::decision(
    std::uint64_t id) const noexcept {
  const auto found = decisions_.find(id);
  return found == decisions_.end() ? nullptr : &found->second;
}

const DecisionRecord* ExplanationTraceStore::action(
    std::uint64_t id) const noexcept {
  const auto found = action_to_decision_.find(id);
  return found == action_to_decision_.end() ? nullptr : decision(found->second);
}

const DecisionRecord* ExplanationTraceStore::execution(
    std::uint64_t id) const noexcept {
  const auto found = execution_to_decision_.find(id);
  return found == execution_to_decision_.end() ? nullptr
                                                : decision(found->second);
}

const DecisionRecord* ExplanationTraceStore::task(
    std::uint64_t id) const noexcept {
  const auto found = task_to_decision_.find(id);
  return found == task_to_decision_.end() ? nullptr : decision(found->second);
}

const DecisionRecord* ExplanationTraceStore::currentTask() const noexcept {
  return task_to_decision_.empty() ? nullptr
                                  : decision(task_to_decision_.rbegin()->second);
}

const DecisionRecord* ExplanationTraceStore::plan(
    std::uint64_t id) const noexcept {
  const auto found = plan_to_decision_.find(id);
  return found == plan_to_decision_.end() ? nullptr : decision(found->second);
}

const DecisionRecord* ExplanationTraceStore::planningEpisode(
    std::uint64_t id) const noexcept {
  const auto found = episode_to_decision_.find(id);
  return found == episode_to_decision_.end() ? nullptr : decision(found->second);
}

std::vector<std::uint64_t> ExplanationTraceStore::decisionIds() const {
  std::vector<std::uint64_t> result;
  for (const auto& [id, record] : decisions_) {
    static_cast<void>(record);
    result.push_back(id);
  }
  return result;
}

const DecisionRecord* UnifiedWhySystem::resolveDecision(
    const ExplanationQuestion& question) const noexcept {
  if (question.has_decision_id) return traces_.decision(question.decision_id);
  if (question.has_action_id) return traces_.action(question.action_id);
  if (question.has_execution_id)
    return traces_.execution(question.execution_id);
  if (question.has_plan_id) return traces_.plan(question.plan_id);
  if (question.has_planning_episode_id)
    return traces_.planningEpisode(question.planning_episode_id);
  if (question.has_task_id) return traces_.task(question.task_id);
  return traces_.latestDecision();
}

ExplanationResponse UnifiedWhySystem::baseResponse(
    const ExplanationQuestion& question, const DecisionRecord* record) const {
  ExplanationResponse response;
  response.explanation_id = next_explanation_id_++;
  response.question_id = question.question_id;
  response.question_type = inferredQuestionType(question);
  response.found = record != nullptr;
  if (!record) return response;
  response.has_decision_id = true;
  response.decision_id = record->decision_id;
  response.has_action_id = true;
  response.action_id = record->action_id;
  response.has_execution_id = record->execution_id != 0U;
  response.execution_id = record->execution_id;
  response.has_task_id = record->has_task;
  response.task_id = record->has_task ? record->task.task_index : 0U;
  response.has_plan_id = record->has_plan;
  response.plan_id = record->active_plan_id;
  response.plan_revision = record->active_plan_revision;
  response.confidence_category = record->decision_confidence_category;
  response.gini_agreement = record->decision_gini_agreement;
  response.standardized_support = record->decision_standardized_total;
  response.relative_support = record->decision_relative_support;
  response.source_provenance = record->source_provenance;
  if (const auto* plan = selectedPlan(*record)) {
    response.referenced_model_revisions = plan->dependency_revisions;
    for (const auto& dependency : plan->metadata.representation_dependencies)
      if (std::find(response.source_provenance.begin(),
                    response.source_provenance.end(), dependency) ==
          response.source_provenance.end())
        response.source_provenance.push_back(dependency);
    if (plan->static_map_contributed &&
        std::find(response.source_provenance.begin(),
                  response.source_provenance.end(), "static_map") ==
            response.source_provenance.end())
      response.source_provenance.push_back("static_map");
  }
  return response;
}

ExplanationResponse UnifiedWhySystem::explainDecision(
    const ExplanationQuestion& question, const DecisionRecord& record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "decision";
  std::ostringstream text;
  text << "Action " << record.action_id << " was "
       << (record.action_lifecycle_status.empty() ? "selected"
                                                  : record.action_lifecycle_status)
       << ": " << actionText(record.selected_action) << ". ";
  if (record.selected_tier == DecisionRecord::TIER_ONE) {
    response.primary_reasoning_source = "tier_one";
    text << tierOnePhrase(record) << '.';
  } else if (record.selected_tier == DecisionRecord::TIER_THREE) {
    response.primary_reasoning_source = "tier_three_voting";
    text << "It had the strongest recorded Tier-3 total under "
         << record.tier_three_scoring_policy << ".";
    for (const auto& contribution : record.advisor_contributions) {
      if (!sameAction(contribution.action, record.selected_action)) continue;
      response.referenced_advisors.push_back(contribution.advisor);
      text << ' ' << contribution.advisor << " gave "
           << relativeMagnitude(contribution.relative_support)
           << " (raw " << contribution.raw_score << ", transformed "
           << contribution.normalized_score << ", mean "
           << contribution.advisor_mean << ", standard deviation "
           << contribution.advisor_standard_deviation << ", weight "
           << contribution.weight << ", contribution "
           << contribution.weighted_score << ").";
    }
    if (record.circumstance_match_available) {
      text << " The setting matched a " << record.circumstance_learning_mode
           << " circumstance with assignment confidence "
           << record.circumstance_assignment_confidence << ".";
      if (record.circumstance_weighting_applied) {
        const auto selected = std::find_if(
            record.tier_three_action_totals.begin(),
            record.tier_three_action_totals.end(), [&](const auto& total) {
              return sameAction(total.action, record.selected_action);
            });
        if (selected != record.tier_three_action_totals.end())
          text << " Historical execution evidence ("
               << selected->circumstance_action_evidence
               << " effective outcomes, confidence "
               << selected->circumstance_action_confidence
               << ") multiplied its base total "
               << selected->pre_circumstance_total << " by "
               << selected->circumstance_multiplier << " to produce "
               << selected->post_circumstance_total << ".";
        text << (record.circumstance_weighting_changed_winner
                     ? " This changed the Tier-3 winner."
                     : " This did not change the Tier-3 winner.");
      } else if (!record.circumstance_reason.empty()) {
        text << " Circumstance weighting remained neutral: "
             << record.circumstance_reason << ".";
      }
    }
  } else if (record.selected_tier == DecisionRecord::EXPLORATION) {
    response.primary_reasoning_source = "initial_exploration";
    text << "The exploration state machine selected it.";
  } else {
    response.primary_reasoning_source = record.selected_policy;
    text << "The recorded policy was " << record.selected_policy << '.';
  }
  if (record.has_plan) {
    response.referenced_planners.push_back(record.selected_planner);
    response.referenced_plan_steps.push_back(record.active_plan_step);
    text << " The active plan was " << record.active_plan_id << " revision "
         << record.active_plan_revision;
    if (record.has_operational_target)
      text << ", operationalized at (" << record.operational_target.x << ", "
           << record.operational_target.y << ')';
    text << '.';
    if (!record.plan_status.empty())
      text << " Its recorded status was " << record.plan_status << '.';
    for (const auto& event : record.plan_execution_events) {
      response.structured_facts.push_back("plan_event=" + event);
      text << " Plan event: " << event << '.';
    }
  }
  const auto precedent = std::find_if(
      record.decision_cycle.begin(), record.decision_cycle.end(),
      [](const auto& event) { return event.component == "Precedent"; });
  if (precedent != record.decision_cycle.end() &&
      precedent->reason_code.find("precedent:abstained:") == 0U)
    text << " Precedent abstained ("
         << precedent->reason_code.substr(
                std::string("precedent:abstained:").size())
         << "), so learned experience did not remove an action.";
  response.natural_language_response = text.str();
  return response;
}

ExplanationResponse UnifiedWhySystem::explainCounterfactual(
    const ExplanationQuestion& question, const DecisionRecord& record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "counterfactual_action";
  if (!question.has_alternative_action) {
    response.natural_language_response =
        "No alternative action was supplied for the comparison.";
    return response;
  }
  const auto& alternative = question.alternative_action;
  const bool generated = std::any_of(
      record.candidates.begin(), record.candidates.end(),
      [&](const auto& action) { return sameAction(action, alternative); });
  if (!generated) {
    response.primary_reasoning_source = "candidate_generation";
    response.natural_language_response = actionText(alternative) +
        " was not generated in that decision cycle.";
    return response;
  }
  const auto veto = std::find_if(
      record.vetoes.begin(), record.vetoes.end(),
      [&](const auto& item) { return sameAction(item.action, alternative); });
  if (veto != record.vetoes.end()) {
    response.primary_reasoning_source = veto->rule;
    response.structured_facts = {veto->reason_code, veto->rejection_kind,
                                 veto->explanation_category};
    response.natural_language_response = actionText(alternative) +
        " was removed by " + veto->rule + " as " +
        veto->explanation_category + ", not merely because it had a lower "
        "preference. The recorded reason was " + veto->reason_code + '.';
    return response;
  }
  const bool viable = std::any_of(
      record.viable_actions.begin(), record.viable_actions.end(),
      [&](const auto& action) { return sameAction(action, alternative); });
  if (!viable) {
    response.primary_reasoning_source = "not_viable";
    response.natural_language_response = actionText(alternative) +
        " was generated but was not in the recorded viable action set. No "
        "safety or cognitive cause was recorded, so I will not invent one.";
    return response;
  }
  const bool tied = std::any_of(
      record.tier_three_tie_candidates.begin(),
      record.tier_three_tie_candidates.end(),
      [&](const auto& action) { return sameAction(action, alternative); });
  if (tied) {
    response.primary_reasoning_source = "tie_breaking";
    response.natural_language_response = actionText(alternative) +
        " tied with the selected action; the recorded " +
        record.tier_three_tie_policy + " tie rule and seeded random choice "
        "selected " + actionText(record.selected_action) + '.';
    return response;
  }
  double alternative_total = 0.0, selected_total = 0.0;
  for (const auto& total : record.tier_three_action_totals) {
    if (sameAction(total.action, alternative)) alternative_total = total.total;
    if (sameAction(total.action, record.selected_action))
      selected_total = total.total;
  }
  response.primary_reasoning_source = record.has_plan &&
                                              record.selected_tier ==
                                                  DecisionRecord::TIER_ONE
                                          ? "plan_enforcement"
                                          : "lower_preference";
  response.structured_facts = {
      "alternative_total=" + std::to_string(alternative_total),
      "selected_total=" + std::to_string(selected_total)};
  std::ostringstream rationale;
  rationale << actionText(alternative) << " remained viable";
  if (response.primary_reasoning_source == "plan_enforcement") {
    rationale << " but was inconsistent with the action mandated for active "
                 "plan step "
              << record.active_plan_step;
  } else {
    rationale << " but had lower recorded support (" << alternative_total
              << " versus " << selected_total << ')';
  }
  rationale << ". It was not classified as unsafe.";
  const auto alternative_trace = std::find_if(
      record.tier_three_action_totals.begin(),
      record.tier_three_action_totals.end(), [&](const auto& total) {
        return sameAction(total.action, alternative);
      });
  if (alternative_trace != record.tier_three_action_totals.end() &&
      record.circumstance_weighting_applied)
    rationale << " Circumstance evidence changed its base support from "
              << alternative_trace->pre_circumstance_total << " by multiplier "
              << alternative_trace->circumstance_multiplier << " to "
              << alternative_trace->post_circumstance_total << ".";
  for (const auto& contribution : record.advisor_contributions) {
    if (!sameAction(contribution.action, alternative)) continue;
    response.referenced_advisors.push_back(contribution.advisor);
    response.structured_facts.push_back(
        contribution.advisor + ":raw=" +
        std::to_string(contribution.raw_score) + ",relative=" +
        std::to_string(contribution.relative_support) + ",contribution=" +
        std::to_string(contribution.weighted_score));
  }
  response.natural_language_response = rationale.str();
  return response;
}

ExplanationResponse UnifiedWhySystem::explainDecisionConfidence(
    const ExplanationQuestion& question, const DecisionRecord& record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "decision_confidence";
  response.primary_reasoning_source = "recorded_tier_three_confidence";
  response.confidence_value =
      0.45 * record.decision_gini_agreement +
      0.35 * std::clamp(record.decision_standardized_total / 2.0, 0.0, 1.0) +
      0.20 * std::clamp(record.decision_relative_support, 0.0, 1.0);
  std::ostringstream text;
  text << "Reasoning confidence was " << record.decision_confidence_category
       << ": advisor agreement=" << record.decision_gini_agreement
       << ", standardized winning support="
       << record.decision_standardized_total << ", relative support="
       << record.decision_relative_support
       << ". This is confidence in the reasoning outcome, not a guarantee "
          "that physical execution will succeed.";
  if (record.circumstance_match_available)
    text << " Circumstance assignment confidence was "
         << record.circumstance_assignment_confidence << " under model "
         << record.circumstance_model_version << " ("
         << record.circumstance_learning_mode << "). "
         << record.circumstance_reason << '.';
  response.natural_language_response = text.str();
  return response;
}

ExplanationResponse UnifiedWhySystem::explainPlan(
    const ExplanationQuestion& question, const DecisionRecord& record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "plan_selection";
  const auto* selected = selectedPlan(record);
  if (!selected) {
    response.found = false;
    response.natural_language_response =
        "No recorded planning candidate matches the requested plan.";
    return response;
  }
  response.primary_reasoning_source = "tier_two_selection";
  response.referenced_planners.push_back(selected->planner);
  response.referenced_model_revisions = selected->dependency_revisions;
  std::ostringstream text;
  text << "Plan " << selected->plan_id << " was generated by "
       << selected->planner << " to "
       << selected->metadata.objective_description << ". It is a "
       << selected->metadata.plan_type << " plan using ";
  for (std::size_t index = 0U;
       index < selected->metadata.representation_dependencies.size(); ++index) {
    if (index != 0U) text << ", ";
    text << selected->metadata.representation_dependencies[index];
  }
  text << ". Tier 2 compared " << record.planning_candidates.size()
       << " candidate plans; its final vote was " << selected->summed_score
       << ".";
  if (record.planning_tie_candidates.size() > 1U)
    text << " It tied with other planners and "
         << record.planning_tie_break_reason << ".";
  for (const auto& candidate : record.planning_candidates) {
    response.referenced_planners.push_back(candidate.planner);
    text << ' ' << candidate.planner << " total=" << candidate.summed_score
         << " [";
    for (std::size_t index = 0U; index < candidate.objectives.size(); ++index) {
      if (index != 0U) text << ", ";
      text << candidate.objectives[index] << " raw="
           << candidate.raw_costs[index] << " normalized="
           << candidate.normalized_costs[index];
    }
    text << "].";
  }
  response.natural_language_response = text.str();
  return response;
}

ExplanationResponse UnifiedWhySystem::explainAlternativePlan(
    const ExplanationQuestion& question, const DecisionRecord& record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "alternative_plan";
  const PlanCandidate* alternative = nullptr;
  for (const auto& candidate : record.planning_candidates) {
    if (candidate.plan_id == record.active_plan_id) continue;
    if (!alternative || candidate.summed_score < alternative->summed_score)
      alternative = &candidate;
  }
  if (!alternative) {
    response.found = false;
    response.natural_language_response =
        "No recorded alternative plan exists; I did not silently replan.";
    return response;
  }
  response.has_plan_id = true;
  response.plan_id = alternative->plan_id;
  response.primary_reasoning_source = "recorded_tier_two_alternative";
  response.referenced_planners.push_back(alternative->planner);
  response.referenced_model_revisions = alternative->dependency_revisions;
  response.route = alternative->geometry;
  response.natural_language_response =
      "The next recorded alternative is plan " +
      std::to_string(alternative->plan_id) + " from " +
      alternative->planner + ". It optimizes " +
      alternative->metadata.objective_description + " and received total " +
      std::to_string(alternative->summed_score) +
      ", compared with the selected plan's lower winning total.";
  return response;
}

ExplanationResponse UnifiedWhySystem::explainPlanConfidence(
    const ExplanationQuestion& question, const DecisionRecord& record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "plan_confidence";
  response.primary_reasoning_source = "tier_two_vote_distribution";
  const auto* selected = selectedPlan(record);
  if (!selected) {
    response.found = false;
    response.natural_language_response = "No selected plan trace is available.";
    return response;
  }
  std::vector<double> votes;
  for (const auto& candidate : record.planning_candidates)
    votes.push_back(candidate.summed_score);
  const double mean = std::accumulate(votes.begin(), votes.end(), 0.0) /
                      static_cast<double>(votes.size());
  double variance = 0.0;
  for (const double vote : votes) variance += (vote - mean) * (vote - mean);
  variance /= static_cast<double>(votes.size());
  const double deviation = std::sqrt(variance);
  double runner_up = std::numeric_limits<double>::infinity();
  for (const auto& candidate : record.planning_candidates)
    if (candidate.plan_id != selected->plan_id)
      runner_up = std::min(runner_up, candidate.summed_score);
  const double separation = std::isfinite(runner_up)
                                ? runner_up - selected->summed_score
                                : 0.0;
  const double standardized = deviation <= 1.0e-12
                                  ? 0.0
                                  : separation / deviation;
  std::size_t selected_objective_wins = 0U;
  for (std::size_t objective = 0U;
       objective < selected->normalized_costs.size(); ++objective) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& candidate : record.planning_candidates)
      if (objective < candidate.normalized_costs.size())
        best = std::min(best, candidate.normalized_costs[objective]);
    if (std::abs(selected->normalized_costs[objective] - best) <= 1.0e-12)
      ++selected_objective_wins;
  }
  const double objective_agreement = selected->normalized_costs.empty()
      ? 0.0
      : static_cast<double>(selected_objective_wins) /
            static_cast<double>(selected->normalized_costs.size());
  const double relative = !std::isfinite(runner_up)
      ? 0.0
      : std::clamp(separation /
                       std::max(1.0, std::abs(runner_up) +
                                         std::abs(selected->summed_score)),
                   0.0, 1.0);
  const double confidence = std::clamp(
      0.4 * objective_agreement +
          0.35 * std::clamp(standardized / 2.0, 0.0, 1.0) +
          0.25 * relative,
      0.0, 1.0);
  response.confidence_value = confidence;
  response.gini_agreement = objective_agreement;
  response.standardized_support = standardized;
  response.relative_support = relative;
  response.confidence_category = confidence >= 0.75 ? "high"
                                 : confidence >= 0.45 ? "moderate"
                                                      : "low";
  response.natural_language_response =
      "Plan-selection confidence was " + response.confidence_category +
      " from the recorded range-voting distribution: winner separation=" +
      std::to_string(separation) + ", standardized separation=" +
      std::to_string(standardized) + ", objective agreement=" +
      std::to_string(objective_agreement) +
      ". This does not claim that the map, learned models, future obstacles, "
      "or local execution are certain.";
  return response;
}

ExplanationResponse UnifiedWhySystem::explainRoute(
    const ExplanationQuestion& question, const DecisionRecord& record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "route_description";
  const auto* selected = selectedPlan(record);
  if (!selected) {
    response.found = false;
    response.natural_language_response = "No selected route is recorded.";
    return response;
  }
  response.route = selected->geometry;
  response.primary_reasoning_source = selected->metadata.plan_type;
  response.referenced_planners.push_back(selected->planner);
  std::ostringstream text;
  text << "The " << selected->metadata.plan_type << " route from "
       << selected->planner << " will ";
  if (!selected->typed_steps.empty()) {
    for (std::size_t index = 0U; index < selected->typed_steps.size(); ++index) {
      const auto& step = selected->typed_steps[index];
      response.referenced_plan_steps.push_back(step.step_id);
      if (index != 0U) text << ", then ";
      if (step.step_type == "highway_entry")
        text << "enter highway " << step.primary_entity_id;
      else if (step.step_type == "highway")
        text << "continue on highway " << step.primary_entity_id
             << " to intersection " << step.secondary_entity_id;
      else if (step.step_type == "intersection")
        text << "pass through intersection " << step.primary_entity_id;
      else if (step.step_type == "highway_exit")
        text << "exit highway " << step.primary_entity_id;
      else if (step.step_type == "skeleton_transition")
        text << "follow a supporting subtrail from region "
             << step.primary_entity_id << " to region "
             << step.secondary_entity_id;
      else if (step.step_type == "region")
        text << "move into region " << step.primary_entity_id;
      else if (step.step_type == "subtrail")
        text << "follow learned subtrail " << step.primary_entity_id;
      else
        text << "approach the " << step.step_type;
    }
  } else {
    text << "follow simplified geometric segments";
  }
  text << ".";
  double heading = record.robot_pose.theta;
  geometry_msgs::msg::Point previous;
  previous.x = record.robot_pose.x;
  previous.y = record.robot_pose.y;
  std::vector<geometry_msgs::msg::Point> simplified;
  std::optional<double> segment_bearing;
  geometry_msgs::msg::Point segment_start = previous;
  geometry_msgs::msg::Point segment_end = previous;
  for (const auto& point : selected->geometry) {
    const double dx = point.x - segment_end.x;
    const double dy = point.y - segment_end.y;
    if (std::hypot(dx, dy) <= 1.0e-6) continue;
    const double bearing = std::atan2(point.y - segment_start.y,
                                      point.x - segment_start.x);
    if (segment_bearing &&
        std::abs(std::remainder(bearing - *segment_bearing,
                                2.0 * kPi)) > 0.17) {
      simplified.push_back(segment_end);
      segment_start = segment_end;
      segment_bearing = std::atan2(point.y - segment_start.y,
                                   point.x - segment_start.x);
    } else if (!segment_bearing) {
      segment_bearing = bearing;
    }
    segment_end = point;
  }
  if (segment_bearing) simplified.push_back(segment_end);
  if (!simplified.empty()) text << " Geometrically, ";
  for (std::size_t index = 0U; index < simplified.size(); ++index) {
    const auto& point = simplified[index];
    const double dx = point.x - previous.x;
    const double dy = point.y - previous.y;
    const double distance = std::hypot(dx, dy);
    if (distance <= 1.0e-6) continue;
    const double bearing = std::atan2(dy, dx);
    const double turn = std::remainder(bearing - heading, 2.0 * kPi);
    response.exact_distances_m.push_back(distance);
    response.exact_turn_angles_rad.push_back(turn);
    response.direction_categories.push_back(directionCategory(turn));
    response.distance_categories.push_back(distanceCategory(distance));
    if (index != 0U) text << ", then ";
    text << response.direction_categories.back() << " for "
         << response.distance_categories.back() << " (" << distance
         << " m after a " << turn << " rad turn)";
    heading = bearing;
    previous = point;
  }
  if (!simplified.empty()) text << '.';
  response.natural_language_response = text.str();
  return response;
}

ExplanationResponse UnifiedWhySystem::comparePlan(
    const ExplanationQuestion& question, const DecisionRecord& record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "plan_comparison";
  const auto* selected = selectedPlan(record);
  if (!selected) {
    response.found = false;
    response.natural_language_response = "No selected plan is recorded.";
    return response;
  }
  const PlanCandidate* alternative = nullptr;
  if (question.has_alternative_plan_id) {
    for (const auto& candidate : record.planning_candidates)
      if (candidate.plan_id == question.alternative_plan_id)
        alternative = &candidate;
  }
  if (alternative) {
    response.primary_reasoning_source = "recorded_plan_cost_matrix";
    const auto count = std::min(
        {selected->objectives.size(), selected->raw_costs.size(),
         alternative->raw_costs.size()});
    for (std::size_t index = 0U; index < count; ++index) {
      response.structured_facts.push_back(
          selected->objectives[index] + ":robot=" +
          std::to_string(selected->raw_costs[index]) + ",alternative=" +
          std::to_string(alternative->raw_costs[index]));
    }
    response.natural_language_response =
        "The selected " + selected->planner + " plan scored " +
        std::to_string(selected->summed_score) + " versus " +
        std::to_string(alternative->summed_score) + " for " +
        alternative->planner + ". The lower total won; the objective arrays "
        "in the response trace preserve the tradeoffs rather than claiming "
        "unqualified superiority.";
    response.referenced_planners = {selected->planner, alternative->planner};
    return response;
  }
  if (!question.user_route.empty()) {
    if (!route_evaluator_) {
      response.found = false;
      response.primary_reasoning_source = "route_evaluator_unavailable";
      response.natural_language_response =
          "A user route was supplied, but no immutable objective evaluator is "
          "available. I will not invent crowd or learned-model costs after "
          "the fact.";
      return response;
    }
    const auto costs = route_evaluator_(question.user_route,
                                        selected->objectives);
    response.primary_reasoning_source = "immutable_route_evaluator";
    for (std::size_t index = 0U; index < costs.size(); ++index)
      response.structured_facts.push_back(
          selected->objectives.at(index) + "=" + std::to_string(costs[index]));
    response.natural_language_response =
        "The supplied route was evaluated under the same recorded objectives. "
        "The structured facts report each tradeoff; the selected route length "
        "was " + std::to_string(pathLength(selected->geometry)) + " metres.";
    return response;
  }
  response.found = false;
  response.natural_language_response =
      "Supply a recorded alternative plan ID or a user route for comparison.";
  return response;
}

ExplanationResponse UnifiedWhySystem::explainHypothetical(
    const ExplanationQuestion& question) const {
  auto response = baseResponse(question, nullptr);
  response.hypothetical = true;
  response.explanation_category = "hypothetical_pose";
  if (!question.has_hypothetical_pose || !hypothetical_evaluator_) {
    response.found = false;
    response.natural_language_response =
        "Hypothetical evaluation requires a pose and an immutable evaluator.";
    return response;
  }
  const auto hypothetical = hypothetical_evaluator_(question.hypothetical_pose);
  response = explainDecision(question, hypothetical);
  response.hypothetical = true;
  response.explanation_category = "hypothetical_pose";
  response.primary_reasoning_source = "immutable_world_snapshot";
  response.natural_language_response =
      "Hypothetically, without changing navigation state, " +
      response.natural_language_response;
  return response;
}

ExplanationResponse UnifiedWhySystem::answer(
    const ExplanationQuestion& question) const {
  const auto type = inferredQuestionType(question);
  if (type == ExplanationQuestion::HYPOTHETICAL_POSE)
    return explainHypothetical(question);
  const auto* record = resolveDecision(question);
  if (!record) {
    auto response = baseResponse(question, nullptr);
    response.natural_language_response =
        "No matching historical decision or plan trace was found.";
    return response;
  }
  switch (type) {
    case ExplanationQuestion::WHY_DECISION:
      return explainDecision(question, *record);
    case ExplanationQuestion::DECISION_CONFIDENCE:
      return explainDecisionConfidence(question, *record);
    case ExplanationQuestion::WHY_NOT_ACTION:
      return explainCounterfactual(question, *record);
    case ExplanationQuestion::WHY_PLAN:
      return explainPlan(question, *record);
    case ExplanationQuestion::COMPARE_PLAN:
      return comparePlan(question, *record);
    case ExplanationQuestion::ALTERNATIVE_PLAN:
      return explainAlternativePlan(question, *record);
    case ExplanationQuestion::PLAN_CONFIDENCE:
      return explainPlanConfidence(question, *record);
    case ExplanationQuestion::ROUTE_DESCRIPTION:
      return explainRoute(question, *record);
    case ExplanationQuestion::COMBINED: {
      auto decision = explainDecision(question, *record);
      const auto route = explainRoute(question, *record);
      if (route.found) {
        decision.explanation_category = "combined_action_and_plan";
        decision.natural_language_response += " " +
                                              route.natural_language_response;
        decision.referenced_plan_steps = route.referenced_plan_steps;
      }
      return decision;
    }
    default: break;
  }
  return explainDecision(question, *record);
}

}  // namespace semaforr::why
