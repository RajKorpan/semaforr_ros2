#include <algorithm>
#include <cmath>
#include <map>
#include <semaforr/decision/decision_coordinator.hpp>
#include <set>
#include <stdexcept>
#include <tuple>

namespace semaforr::decision {
namespace {

using Action = domain::Action;

bool vetoLess(const Veto& left, const Veto& right) {
  return std::tie(left.action, left.rule, left.explanation) <
         std::tie(right.action, right.rule, right.explanation);
}

bool contributionLess(const AdvisorContribution& left,
                      const AdvisorContribution& right) {
  return std::tie(left.advisor, left.action, left.explanation) <
         std::tie(right.advisor, right.action, right.explanation);
}

std::vector<double> transformScores(
    const AdvisorEvaluation& evaluation, ScoreNormalization normalization,
    TierThreeScoringPolicy policy) {
  std::vector<double> transformed;
  transformed.reserve(evaluation.scores.size());
  if (evaluation.scores.empty()) return transformed;
  const auto [minimum, maximum] = std::minmax_element(
      evaluation.scores.begin(), evaluation.scores.end(),
      [](const auto& left, const auto& right) {
        return left.raw_score < right.raw_score;
      });
  const double span = maximum->raw_score - minimum->raw_score;
  for (const auto& score : evaluation.scores) {
    if (policy == TierThreeScoringPolicy::CompatibilityComments) {
      transformed.push_back(span <= 1.0e-12
                                ? 5.0
                                : 10.0 * (score.raw_score -
                                          minimum->raw_score) /
                                      span);
      continue;
    }
    if (normalization == ScoreNormalization::None) {
      transformed.push_back(score.raw_score);
      continue;
    }
    const double unit = span <= 1.0e-12
                            ? 0.5
                            : (score.raw_score - minimum->raw_score) / span;
    transformed.push_back(normalization == ScoreNormalization::SignedUnit
                              ? (span <= 1.0e-12 ? 0.0 : 2.0 * unit - 1.0)
                              : unit);
  }
  return transformed;
}

}  // namespace

DecisionCoordinator::DecisionCoordinator(ArbitrationConfiguration configuration)
    : configuration_(std::move(configuration)),
      random_(configuration_.random_seed) {
  if (!std::isfinite(configuration_.tie_tolerance) ||
      configuration_.tie_tolerance < 0.0) {
    throw std::invalid_argument("tie tolerance must be finite and nonnegative");
  }
  if (!std::isfinite(configuration_.unscored_baseline)) {
    throw std::invalid_argument("unscored baseline must be finite");
  }
}

std::optional<DecisionResult> DecisionCoordinator::mandatoryDecision(
    const DecisionContext& context,
    std::span<const Action> candidates) const {
  for (const auto& rule : mandatory_rules_) {
    if (auto decision = rule->evaluate(context)) {
      if (std::find(candidates.begin(), candidates.end(), decision->action) ==
          candidates.end())
        continue;
      DecisionResult result;
      result.action = decision->action;
      result.source = DecisionSource::MandatoryRule;
      result.tier = DecisionTier::TierOne;
      result.selected_policy = "mandatory_rule:" + decision->rule;
      return result;
    }
  }
  return std::nullopt;
}

TierOnePass DecisionCoordinator::evaluateTierOne(
    const DecisionContext& context,
    std::span<const Action> candidates) const {
  TierOnePass pass;
  pass.survivors.assign(candidates.begin(), candidates.end());
  std::sort(pass.survivors.begin(), pass.survivors.end());
  pass.survivors.erase(
      std::unique(pass.survivors.begin(), pass.survivors.end()),
      pass.survivors.end());
  for (const auto& rule : mandatory_rules_) {
    DecisionCycleEvent event;
    event.tier = "tier1";
    event.component = std::string(rule->name());
    event.input_actions = pass.survivors;
    if (auto decision = rule->evaluate(context)) {
      if (std::binary_search(pass.survivors.begin(), pass.survivors.end(),
                             decision->action)) {
        event.mandate = decision->action;
        event.outcome = "mandated_action";
        event.reason_code = decision->explanation;
        event.final_attribution = DecisionTier::TierOne;
        event.order = pass.trace.size() + 1U;
        pass.trace.push_back(std::move(event));
        DecisionResult result;
        result.action = decision->action;
        result.source = DecisionSource::MandatoryRule;
        result.tier = DecisionTier::TierOne;
        result.selected_policy = "mandatory_rule:" + decision->rule;
        result.decision_cycle = pass.trace;
        pass.decision = std::move(result);
        return pass;
      }
      event.outcome = "mandate_not_viable_continue";
    } else {
      event.outcome = "no_mandate_continue";
    }
    event.order = pass.trace.size() + 1U;
    pass.trace.push_back(std::move(event));
  }

  for (const auto& rule : veto_rules_) {
    DecisionCycleEvent event;
    event.tier = "tier1";
    event.component = std::string(rule->name());
    event.input_actions = pass.survivors;
    event.vetoes = rule->evaluate(context);
    pass.vetoes.insert(pass.vetoes.end(), event.vetoes.begin(),
                       event.vetoes.end());
    std::set<Action> vetoed;
    for (const auto& veto : event.vetoes) vetoed.insert(veto.action);
    std::erase_if(pass.survivors,
                  [&](const auto& action) { return vetoed.contains(action); });
    event.outcome = event.vetoes.empty() ? "no_veto_continue"
                                         : "vetoes_applied_continue";
    event.order = pass.trace.size() + 1U;
    pass.trace.push_back(std::move(event));
  }
  std::sort(pass.vetoes.begin(), pass.vetoes.end(), vetoLess);

  if (pass.survivors.empty()) {
    DecisionResult result;
    result.action = Action::pause();
    result.source = DecisionSource::SafeStop;
    result.tier = DecisionTier::SafeStop;
    result.selected_policy = "no_safe_candidate";
    result.vetoes = pass.vetoes;
    result.decision_cycle = pass.trace;
    result.decision_cycle.push_back(
        {result.decision_cycle.size() + 1U, "tier1", "viable_action_set", {},
         std::nullopt, {}, "no_survivor_safe_stop", false,
         DecisionTier::SafeStop});
    pass.decision = std::move(result);
  } else if (pass.survivors.size() == 1U) {
    DecisionResult result;
    result.action = pass.survivors.front();
    result.source = DecisionSource::MandatoryRule;
    result.tier = DecisionTier::TierOne;
    result.selected_policy = "tier1:only_surviving_action";
    result.vetoes = pass.vetoes;
    result.decision_cycle = pass.trace;
    result.decision_cycle.push_back(
        {result.decision_cycle.size() + 1U, "tier1", "viable_action_set",
         pass.survivors, pass.survivors.front(), {},
         "single_survivor_selected", false, DecisionTier::TierOne,
         "tier1:only_surviving_action"});
    pass.decision = std::move(result);
  }
  return pass;
}

void DecisionCoordinator::addMandatoryRule(
    std::unique_ptr<MandatoryRule> rule) {
  if (!rule) {
    throw std::invalid_argument("mandatory rule must not be null");
  }
  mandatory_rules_.push_back(std::move(rule));
}

void DecisionCoordinator::addVetoRule(std::unique_ptr<VetoRule> rule) {
  if (!rule) {
    throw std::invalid_argument("veto rule must not be null");
  }
  veto_rules_.push_back(std::move(rule));
}

void DecisionCoordinator::addAdvisor(std::unique_ptr<Advisor> advisor) {
  if (!advisor) {
    throw std::invalid_argument("advisor must not be null");
  }
  advisors_.push_back(std::move(advisor));
}

DecisionResult DecisionCoordinator::decideTierThree(
    const DecisionContext& context, std::span<const Action> candidates) {
  DecisionResult result;
  result.tier_three_scoring_policy =
      configuration_.scoring_policy ==
              TierThreeScoringPolicy::CompatibilityComments
          ? "compatibility_comments_0_10_unweighted"
          : "weighted_normalized";
  result.tier_three_tie_policy =
      configuration_.tie_policy == TierThreeTiePolicy::Exact ? "exact"
                                                              : "tolerance";
  result.tier_three_tie_tolerance = configuration_.tie_tolerance;
  result.tier_three_random_seed = configuration_.random_seed;
  if (context.active_plan_objective) {
    result.operational_target = context.active_plan_objective->target;
    result.active_plan_step = context.active_plan_objective->step_index;
    result.plan_id = context.active_plan_objective->plan_id;
  }
  std::vector<Action> survivors(candidates.begin(), candidates.end());
  std::sort(survivors.begin(), survivors.end());
  survivors.erase(std::unique(survivors.begin(), survivors.end()),
                  survivors.end());
  if (survivors.empty()) {
    result.action = Action::pause();
    result.source = DecisionSource::SafeStop;
    result.tier = DecisionTier::SafeStop;
    result.selected_policy = "no_safe_candidate";
    result.decision_cycle.push_back(
        {1U, "tier3", "viable_action_set", {}, std::nullopt, {},
         "no_survivor_safe_stop", false, DecisionTier::SafeStop});
    return result;
  }

  std::map<Action, double> totals;
  std::set<Action> scored;
  if (configuration_.unscored_policy != UnscoredActionPolicy::Exclude) {
    const double initial =
        configuration_.unscored_policy == UnscoredActionPolicy::Baseline
            ? configuration_.unscored_baseline
            : 0.0;
    for (const auto& action : survivors) {
      totals.emplace(action, initial);
    }
  }

  for (const auto& advisor : advisors_) {
    const AdvisorEvaluation evaluation = advisor->evaluate(context, survivors);
    const auto metadata = advisor->metadata();
    result.decision_cycle.push_back(
        {result.decision_cycle.size() + 1U, "tier3",
         std::string(advisor->name()), survivors, std::nullopt, {},
         evaluation.participated ? "advisor_scored_continue"
                                : "advisor_not_applicable_continue",
         false, std::nullopt});
    if (!evaluation.participated) {
      continue;
    }
    if (!std::isfinite(evaluation.weight)) {
      throw std::domain_error("advisor '" + std::string(advisor->name()) +
                              "' returned a non-finite weight");
    }
    const auto transformed = transformScores(
        evaluation, metadata.normalization, configuration_.scoring_policy);
    for (std::size_t index = 0U; index < evaluation.scores.size(); ++index) {
      const auto& score = evaluation.scores[index];
      if (!std::binary_search(survivors.begin(), survivors.end(),
                              score.action)) {
        throw std::domain_error("advisor '" + std::string(advisor->name()) +
                                "' scored an unavailable or vetoed action");
      }
      if (!std::isfinite(score.raw_score)) {
        throw std::domain_error("advisor '" + std::string(advisor->name()) +
                                "' returned a non-finite score");
      }
      const double applied_weight =
          configuration_.scoring_policy ==
                  TierThreeScoringPolicy::CompatibilityComments
              ? 1.0
              : evaluation.weight;
      const double weighted = transformed[index] * applied_weight;
      if (!std::isfinite(weighted)) {
        throw std::domain_error("weighted advisor contribution is non-finite");
      }
      totals[score.action] += weighted;
      scored.insert(score.action);
      AdvisorContribution contribution;
      contribution.advisor = std::string(advisor->name());
      contribution.action = score.action;
      contribution.raw_score = score.raw_score;
      contribution.normalized_score = transformed[index];
      contribution.weight = evaluation.weight;
      contribution.weighted_score = weighted;
      contribution.viable = true;
      contribution.explanation = evaluation.explanation;
      contribution.model_revision_used = evaluation.model_revision_used;
      result.contributions.push_back(std::move(contribution));
    }
  }
  std::sort(result.contributions.begin(), result.contributions.end(),
            contributionLess);

  if (scored.empty()) {
    if (configuration_.fallback &&
        std::binary_search(survivors.begin(), survivors.end(),
                           *configuration_.fallback)) {
      result.action = *configuration_.fallback;
      result.source = DecisionSource::Fallback;
      result.tier = DecisionTier::Fallback;
      result.selected_policy = "configured_fallback";
    } else {
      result.action = Action::pause();
      result.source = DecisionSource::SafeStop;
      result.tier = DecisionTier::SafeStop;
      result.selected_policy = "no_advisor_score";
    }
    result.decision_cycle.push_back(
        {result.decision_cycle.size() + 1U, "tier3", "tier3_fallback",
         survivors, result.action, {}, result.selected_policy, false,
         result.tier});
    return result;
  }

  if (configuration_.unscored_policy == UnscoredActionPolicy::Exclude) {
    for (auto iterator = totals.begin(); iterator != totals.end();) {
      if (!scored.contains(iterator->first)) {
        iterator = totals.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }

  for (const auto& action : survivors) {
    const auto total = totals.find(action);
    result.tier_three_totals.push_back(
        {action, total == totals.end() ? 0.0 : total->second, true,
         scored.contains(action)});
  }
  for (auto& contribution : result.contributions)
    contribution.final_total = totals.at(contribution.action);

  const auto maximum =
      std::max_element(totals.begin(), totals.end(),
                       [](const auto& left, const auto& right) {
                         return left.second < right.second;
                       })
          ->second;
  std::vector<Action> tied;
  for (const auto& [action, total] : totals) {
    const bool is_tied =
        configuration_.tie_policy == TierThreeTiePolicy::Exact
            ? total == maximum
            : std::abs(total - maximum) <= configuration_.tie_tolerance;
    if (is_tied) {
      tied.push_back(action);
    }
  }
  result.tier_three_tie_candidates = tied;
  std::size_t selected_index = 0U;
  if (tied.size() > 1U) {
    std::uniform_int_distribution<std::size_t> choose(0U, tied.size() - 1U);
    selected_index = choose(random_);
    result.tier_three_random_selection_used = true;
    result.tier_three_random_selection_index = selected_index;
  }
  result.action = tied[selected_index];
  result.source = DecisionSource::TierThreeAdvisor;
  result.tier = DecisionTier::TierThree;
  result.selected_policy = "advisor_arbitration";
  result.decision_cycle.push_back(
      {result.decision_cycle.size() + 1U, "tier3", "advisor_arbitration",
       survivors, result.action, {}, "advisor_vote_selected", false,
       DecisionTier::TierThree});
  return result;
}

DecisionResult DecisionCoordinator::decide(
    const DecisionContext& context, std::span<const Action> candidates) {
  auto tier_one = evaluateTierOne(context, candidates);
  if (tier_one.decision) return *tier_one.decision;
  auto result = decideTierThree(context, tier_one.survivors);
  result.vetoes = std::move(tier_one.vetoes);
  result.decision_cycle.insert(result.decision_cycle.begin(),
                               tier_one.trace.begin(), tier_one.trace.end());
  for (std::size_t index = 0U; index < result.decision_cycle.size(); ++index)
    result.decision_cycle[index].order = index + 1U;
  return result;
}

}  // namespace semaforr::decision
