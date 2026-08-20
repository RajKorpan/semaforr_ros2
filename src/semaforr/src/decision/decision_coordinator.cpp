#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
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
    if (normalization == ScoreNormalization::SignedUnit) {
      transformed.push_back(span <= 1.0e-12 ? 0.0 : 2.0 * unit - 1.0);
    } else if (normalization == ScoreNormalization::TenPoint) {
      transformed.push_back(span <= 1.0e-12 ? 5.0 : 10.0 * unit);
    } else {
      transformed.push_back(unit);
    }
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
    const DecisionContext rule_context{
        context.world, context.action_space, candidates,
        context.active_plan_objective};
    if (auto decision = rule->evaluate(rule_context)) {
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
  auto pass = evaluateTierOneStage(context, candidates, TierOneStage::All);
  if (pass.decision) return pass;

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

TierOnePass DecisionCoordinator::evaluateTierOneStage(
    const DecisionContext& context, std::span<const Action> candidates,
    TierOneStage stage) const {
  TierOnePass pass;
  pass.survivors.assign(candidates.begin(), candidates.end());
  std::sort(pass.survivors.begin(), pass.survivors.end());
  pass.survivors.erase(
      std::unique(pass.survivors.begin(), pass.survivors.end()),
      pass.survivors.end());
  const auto component_stage = [](std::string_view name) {
    return name == "Forward" || name == "Precedent"
               ? TierOneStage::AfterLowLevelExploration
               : TierOneStage::BeforeEnforcer;
  };
  for (const auto& registered : tier_one_order_) {
    const std::string_view name =
        registered.kind == RegisteredRuleKind::Mandatory
            ? mandatory_rules_[registered.index]->name()
            : veto_rules_[registered.index]->name();
    if (stage != TierOneStage::All && component_stage(name) != stage) continue;

    DecisionCycleEvent event;
    event.tier = "tier1";
    event.component = std::string(name);
    event.input_actions = pass.survivors;
    const DecisionContext rule_context{
        context.world, context.action_space, pass.survivors,
        context.active_plan_objective};
    if (registered.kind == RegisteredRuleKind::Mandatory) {
      const auto& rule = mandatory_rules_[registered.index];
      if (auto decision = rule->evaluate(rule_context)) {
        if (std::binary_search(pass.survivors.begin(), pass.survivors.end(),
                               decision->action)) {
          event.mandate = decision->action;
          event.outcome = "mandated_action";
          event.reason_code = decision->explanation;
          event.final_attribution = DecisionTier::TierOne;
          event.remaining_actions = pass.survivors;
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
    } else {
      const auto& rule = veto_rules_[registered.index];
      event.vetoes = rule->evaluate(rule_context);
      event.reason_code = rule->lastReason();
      pass.vetoes.insert(pass.vetoes.end(), event.vetoes.begin(),
                         event.vetoes.end());
      std::set<Action> vetoed;
      for (const auto& veto : event.vetoes) vetoed.insert(veto.action);
      std::erase_if(
          pass.survivors,
          [&](const auto& action) { return vetoed.contains(action); });
      event.outcome = event.vetoes.empty() ? "no_veto_continue"
                                           : "vetoes_applied_continue";
    }
    event.remaining_actions = pass.survivors;
    event.order = pass.trace.size() + 1U;
    pass.trace.push_back(std::move(event));
  }
  std::sort(pass.vetoes.begin(), pass.vetoes.end(), vetoLess);
  return pass;
}

void DecisionCoordinator::addMandatoryRule(
    std::unique_ptr<MandatoryRule> rule) {
  if (!rule) {
    throw std::invalid_argument("mandatory rule must not be null");
  }
  mandatory_rules_.push_back(std::move(rule));
  tier_one_order_.push_back(
      {RegisteredRuleKind::Mandatory, mandatory_rules_.size() - 1U});
}

void DecisionCoordinator::addVetoRule(std::unique_ptr<VetoRule> rule) {
  if (!rule) {
    throw std::invalid_argument("veto rule must not be null");
  }
  veto_rules_.push_back(std::move(rule));
  tier_one_order_.push_back(
      {RegisteredRuleKind::Veto, veto_rules_.size() - 1U});
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
    const double advisor_mean =
        transformed.empty()
            ? 0.0
            : std::accumulate(transformed.begin(), transformed.end(), 0.0) /
                  static_cast<double>(transformed.size());
    double advisor_variance = 0.0;
    for (const double value : transformed)
      advisor_variance += (value - advisor_mean) * (value - advisor_mean);
    if (!transformed.empty())
      advisor_variance /= static_cast<double>(transformed.size());
    const double advisor_standard_deviation = std::sqrt(advisor_variance);
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
      contribution.advisor_mean = advisor_mean;
      contribution.advisor_standard_deviation = advisor_standard_deviation;
      contribution.relative_support =
          advisor_standard_deviation <= 1.0e-12
              ? 0.0
              : (transformed[index] - advisor_mean) /
                    advisor_standard_deviation;
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

  const std::map<Action, double> pre_circumstance_totals = totals;
  result.circumstance_weighting_policy =
      configuration_.circumstance_weighting_enabled
          ? "evidence_gated_laplace_confidence"
          : "disabled";
  const auto base_winner = [&]() -> std::optional<Action> {
    if (pre_circumstance_totals.empty()) return std::nullopt;
    return std::max_element(
               pre_circumstance_totals.begin(),
               pre_circumstance_totals.end(),
               [](const auto& left, const auto& right) {
                 return left.second < right.second;
               })
        ->first;
  }();
  std::map<Action, std::pair<double, const domain::ActionCaseEvidence*>>
      circumstance_adjustments;
  const auto& circumstance_model = context.world.spatial.circumstances;
  result.circumstance_learning_mode =
      std::string(domain::toString(circumstance_model.learning_mode));
  result.circumstance_model_version = circumstance_model.model_version;
  result.circumstance_classifier_version =
      circumstance_model.classifier_version;
  if (configuration_.circumstance_weighting_enabled &&
      context.world.robot.laser && context.world.mission.active() &&
      !circumstance_model.clusters.empty()) {
    domain::SettingNormalizationConfiguration normalization;
    normalization.resolution_m =
        circumstance_model.clusters.front().centroid.resolution_m;
    normalization.radius_m =
        circumstance_model.clusters.front().centroid.radius_m;
    normalization.assignment_confidence_threshold =
        circumstance_model.assignment_confidence_threshold;
    normalization.similarity_l1_threshold =
        circumstance_model.similarity_l1_threshold;
    normalization.distance_bin_base_m =
        circumstance_model.distance_bin_base_m;
    normalization.angle_bin_count = circumstance_model.angle_bin_count;
    const auto setting = domain::normalizeSetting(
        *context.world.robot.laser, normalization);
    const auto match = domain::matchCircumstance(circumstance_model, setting);
    if (match) {
      result.circumstance_match_available = true;
      result.circumstance_id = match->id;
      result.circumstance_assignment_confidence = match->confidence;
      const auto key = domain::circumstanceCaseKey(
          match->id, context.world.robot.pose,
          context.world.mission.active()->target, circumstance_model);
      const auto case_evidence = std::find_if(
          circumstance_model.cases.begin(), circumstance_model.cases.end(),
          [&](const auto& item) { return item.key == key; });
      const bool case_is_reliable =
          match->confidence >=
              configuration_.circumstance_minimum_assignment_confidence &&
          case_evidence != circumstance_model.cases.end() &&
          case_evidence->evidence >=
              configuration_.circumstance_minimum_evidence &&
          case_evidence->accuracy >=
              configuration_.circumstance_minimum_case_accuracy;
      if (case_is_reliable) {
        for (auto& [action, total] : totals) {
          const auto* action_evidence =
              domain::findActionEvidence(*case_evidence, action);
          double multiplier = 1.0;
          if (action_evidence &&
              action_evidence->effective_evidence >= static_cast<double>(
                  configuration_.circumstance_minimum_action_evidence)) {
            const double evidence_blend = std::min(
                1.0, action_evidence->effective_evidence /
                         (2.0 * static_cast<double>(
                                    configuration_
                                        .circumstance_minimum_action_evidence)));
            multiplier = 1.0 +
                         configuration_.circumstance_maximum_influence *
                             evidence_blend *
                             (2.0 * action_evidence->confidence - 1.0);
          }
          circumstance_adjustments[action] = {multiplier, action_evidence};
          total *= multiplier;
          result.circumstance_weighting_applied |=
              std::abs(multiplier - 1.0) > 1.0e-12;
        }
        result.circumstance_reason =
            result.circumstance_weighting_applied
                ? "reliable circumstance case adjusted Tier-3 totals"
                : "case matched but action evidence remained sparse or neutral";
      } else {
        result.circumstance_reason =
            "neutral multiplier: circumstance or case evidence was insufficient";
      }
    } else {
      result.circumstance_reason =
          "neutral multiplier: current setting did not match a circumstance";
    }
  } else if (!configuration_.circumstance_weighting_enabled) {
    result.circumstance_reason = "circumstance Tier-3 weighting disabled";
  }
  const auto adjusted_winner = totals.empty()
                                   ? std::optional<Action>{}
                                   : std::optional<Action>{
                                         std::max_element(
                                             totals.begin(), totals.end(),
                                             [](const auto& left,
                                                const auto& right) {
                                               return left.second < right.second;
                                             })
                                             ->first};
  result.circumstance_weighting_changed_winner =
      base_winner && adjusted_winner && *base_winner != *adjusted_winner;

  for (const auto& action : survivors) {
    const auto total = totals.find(action);
    TierThreeActionTotal trace{
        action, total == totals.end() ? 0.0 : total->second, true,
        scored.contains(action)};
    const auto before = pre_circumstance_totals.find(action);
    trace.pre_circumstance_total =
        before == pre_circumstance_totals.end() ? 0.0 : before->second;
    trace.post_circumstance_total = trace.total;
    if (const auto adjustment = circumstance_adjustments.find(action);
        adjustment != circumstance_adjustments.end()) {
      trace.circumstance_multiplier = adjustment->second.first;
      if (adjustment->second.second) {
        trace.circumstance_action_evidence = static_cast<std::size_t>(
            adjustment->second.second->effective_evidence);
        trace.circumstance_action_confidence =
            adjustment->second.second->confidence;
      }
    }
    result.tier_three_totals.push_back(std::move(trace));
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
  const double totals_mean =
      std::accumulate(totals.begin(), totals.end(), 0.0,
                      [](double sum, const auto& item) {
                        return sum + item.second;
                      }) /
      static_cast<double>(totals.size());
  double totals_variance = 0.0;
  for (const auto& [action, total] : totals) {
    static_cast<void>(action);
    totals_variance += (total - totals_mean) * (total - totals_mean);
  }
  totals_variance /= static_cast<double>(totals.size());
  const double totals_deviation = std::sqrt(totals_variance);
  result.decision_confidence.standardized_total =
      totals_deviation <= 1.0e-12
          ? 0.0
          : (maximum - totals_mean) / totals_deviation;
  std::map<Action, double> preferred_counts;
  std::map<std::string, std::pair<double, Action>> advisor_preferences;
  for (const auto& contribution : result.contributions) {
    const auto found = advisor_preferences.find(contribution.advisor);
    if (found == advisor_preferences.end() ||
        contribution.normalized_score > found->second.first)
      advisor_preferences.insert_or_assign(
          contribution.advisor,
          std::pair{contribution.normalized_score, contribution.action});
  }
  for (const auto& [advisor, preference] : advisor_preferences) {
    static_cast<void>(advisor);
    preferred_counts[preference.second] += 1.0;
  }
  const double participating_advisors =
      static_cast<double>(advisor_preferences.size());
  double gini = 1.0;
  if (participating_advisors > 0.0) {
    gini = 1.0;
    for (const auto& [action, count] : preferred_counts) {
      static_cast<void>(action);
      const double probability = count / participating_advisors;
      gini -= probability * probability;
    }
  }
  const double maximum_gini = totals.size() <= 1U
                                  ? 1.0
                                  : 1.0 - 1.0 /
                                              static_cast<double>(totals.size());
  result.decision_confidence.gini_agreement =
      std::clamp(1.0 - gini / maximum_gini, 0.0, 1.0);
  const auto selected_total = totals.at(result.action);
  double runner_up = -std::numeric_limits<double>::infinity();
  for (const auto& [action, total] : totals)
    if (action != result.action) runner_up = std::max(runner_up, total);
  result.decision_confidence.relative_support =
      !std::isfinite(runner_up)
          ? 1.0
          : (selected_total - runner_up) /
                std::max(1.0, std::abs(selected_total));
  const double confidence_score =
      0.45 * result.decision_confidence.gini_agreement +
      0.35 * std::clamp(result.decision_confidence.standardized_total / 2.0,
                        0.0, 1.0) +
      0.20 * std::clamp(result.decision_confidence.relative_support, 0.0, 1.0);
  result.decision_confidence.category =
      confidence_score >= 0.75 ? "high"
      : confidence_score >= 0.45 ? "moderate"
                                 : "low";
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
