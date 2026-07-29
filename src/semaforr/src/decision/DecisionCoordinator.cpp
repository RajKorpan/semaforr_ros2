#include <semaforr/decision/decision_coordinator.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

namespace semaforr::decision {
namespace {

using Action = domain::Action;

bool vetoLess(const Veto& left, const Veto& right)
{
  return std::tie(left.action, left.rule, left.explanation) <
    std::tie(right.action, right.rule, right.explanation);
}

bool contributionLess(
  const AdvisorContribution& left,
  const AdvisorContribution& right)
{
  return std::tie(left.advisor, left.action, left.explanation) <
    std::tie(right.advisor, right.action, right.explanation);
}

}  // namespace

DecisionCoordinator::DecisionCoordinator(
  ArbitrationConfiguration configuration)
  : configuration_(std::move(configuration)),
    random_(configuration_.random_seed)
{
  if (!std::isfinite(configuration_.tie_tolerance) ||
      configuration_.tie_tolerance < 0.0) {
    throw std::invalid_argument("tie tolerance must be finite and nonnegative");
  }
  if (!std::isfinite(configuration_.unscored_baseline)) {
    throw std::invalid_argument("unscored baseline must be finite");
  }
}

void DecisionCoordinator::addMandatoryRule(std::unique_ptr<MandatoryRule> rule)
{
  if (!rule) {
    throw std::invalid_argument("mandatory rule must not be null");
  }
  mandatory_rules_.push_back(std::move(rule));
}

void DecisionCoordinator::addVetoRule(std::unique_ptr<VetoRule> rule)
{
  if (!rule) {
    throw std::invalid_argument("veto rule must not be null");
  }
  veto_rules_.push_back(std::move(rule));
}

void DecisionCoordinator::addAdvisor(std::unique_ptr<Advisor> advisor)
{
  if (!advisor) {
    throw std::invalid_argument("advisor must not be null");
  }
  advisors_.push_back(std::move(advisor));
}

DecisionResult DecisionCoordinator::decide(
  const DecisionContext& context,
  std::span<const Action> candidates)
{
  for (const auto& rule : mandatory_rules_) {
    if (auto decision = rule->evaluate(context)) {
      DecisionResult result;
      result.action = decision->action;
      result.source = DecisionSource::MandatoryRule;
      result.tier = DecisionTier::TierOne;
      result.selected_policy = "mandatory_rule";
      return result;
    }
  }

  DecisionResult result;
  for (const auto& rule : veto_rules_) {
    auto vetoes = rule->evaluate(context);
    result.vetoes.insert(
      result.vetoes.end(),
      std::make_move_iterator(vetoes.begin()),
      std::make_move_iterator(vetoes.end()));
  }
  std::sort(result.vetoes.begin(), result.vetoes.end(), vetoLess);

  std::set<Action> vetoed;
  for (const auto& veto : result.vetoes) {
    vetoed.insert(veto.action);
  }

  std::vector<Action> survivors;
  for (const Action& candidate : candidates) {
    if (!vetoed.contains(candidate)) {
      survivors.push_back(candidate);
    }
  }
  std::sort(survivors.begin(), survivors.end());
  survivors.erase(std::unique(survivors.begin(), survivors.end()), survivors.end());
  if (survivors.empty()) {
    result.action = Action::pause();
    result.source = DecisionSource::SafeStop;
    result.tier = DecisionTier::SafeStop;
    result.selected_policy = "no_safe_candidate";
    return result;
  }

  std::map<Action, double> totals;
  std::set<Action> scored;
  if (configuration_.unscored_policy != UnscoredActionPolicy::Exclude) {
    const double initial =
      configuration_.unscored_policy == UnscoredActionPolicy::Baseline
      ? configuration_.unscored_baseline : 0.0;
    for (const auto& action : survivors) {
      totals.emplace(action, initial);
    }
  }

  for (const auto& advisor : advisors_) {
    const AdvisorEvaluation evaluation = advisor->evaluate(context, survivors);
    if (!evaluation.participated) {
      continue;
    }
    if (!std::isfinite(evaluation.weight)) {
      throw std::domain_error(
        "advisor '" + std::string(advisor->name()) + "' returned a non-finite weight");
    }
    for (const auto& score : evaluation.scores) {
      if (!std::binary_search(survivors.begin(), survivors.end(), score.action)) {
        throw std::domain_error(
          "advisor '" + std::string(advisor->name()) +
          "' scored an unavailable or vetoed action");
      }
      if (!std::isfinite(score.raw_score)) {
        throw std::domain_error(
          "advisor '" + std::string(advisor->name()) +
          "' returned a non-finite score");
      }
      const double weighted = score.raw_score * evaluation.weight;
      if (!std::isfinite(weighted)) {
        throw std::domain_error("weighted advisor contribution is non-finite");
      }
      totals[score.action] += weighted;
      scored.insert(score.action);
      result.contributions.push_back({
        std::string(advisor->name()),
        score.action,
        score.raw_score,
        evaluation.weight,
        weighted,
        evaluation.explanation});
    }
  }
  std::sort(
    result.contributions.begin(),
    result.contributions.end(),
    contributionLess);

  if (scored.empty()) {
    if (configuration_.fallback &&
        std::binary_search(
          survivors.begin(), survivors.end(), *configuration_.fallback)) {
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

  const auto maximum = std::max_element(
    totals.begin(), totals.end(),
    [](const auto& left, const auto& right) {
      return left.second < right.second;
    })->second;
  std::vector<Action> tied;
  for (const auto& [action, total] : totals) {
    if (std::abs(total - maximum) <= configuration_.tie_tolerance) {
      tied.push_back(action);
    }
  }
  std::uniform_int_distribution<std::size_t> choose(0U, tied.size() - 1U);
  result.action = tied[choose(random_)];
  result.source = DecisionSource::TierThreeAdvisor;
  result.tier = DecisionTier::TierThree;
  result.selected_policy = "advisor_arbitration";
  return result;
}

}  // namespace semaforr::decision
