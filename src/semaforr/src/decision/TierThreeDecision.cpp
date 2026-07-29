/*
 * Tier-three decision implementation.
 */

#include <semaforr/decision/DecisionTier.hpp>
#include <semaforr/decision/Arbitration.hpp>
#include "DecisionTierFactory.h"

#include <semaforr/core/action_adapter.hpp>
#include <semaforr/decision/Tier3Advisor.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#include <utility>

namespace semaforr {
namespace decision {
namespace {

class DefaultTierThreeDecision final : public TierThreeDecision {
public:
  explicit DefaultTierThreeDecision(TierThreeDependencies dependencies)
    : dependencies_(dependencies) {}

  TierThreeResult decide() override {
    TierThreeResult result;
    std::map<FORRAction, double> comments;
    std::map<FORRAction, double> all_comments;

    // Retain the baseline lookup performed by the legacy tier-three pipeline.
    double rotation_baseline = 0.0;
    double linear_baseline = 0.0;
    for (const auto& owned_advisor : dependencies_.advisors) {
      Tier3Advisor* advisor = owned_advisor.get();
      if (advisor->get_name() == "RotationBaseLine") {
        rotation_baseline = advisor->get_weight();
      }
      if (advisor->get_name() == "BaseLine") {
        linear_baseline = advisor->get_weight();
      }
    }
    (void)rotation_baseline;
    (void)linear_baseline;

    for (const auto& owned_advisor : dependencies_.advisors) {
      Tier3Advisor* advisor = owned_advisor.get();
      advisor->set_commenting();
      if (!advisor->is_active() || !advisor->is_commenting()) {
        continue;
      }

      comments = advisor->allAdvice();

      for (const auto& comment : comments) {
        const double weight = advisor->get_weight();
        const double contribution = comment.second * weight;
        if (!std::isfinite(contribution)) {
          continue;
        }
        result.contributions.push_back({
          advisor->get_name(),
          core::toDomainAction(comment.first),
          comment.second,
          weight,
          contribution,
          "legacy tier-three advisor score"});

        const auto existing = all_comments.find(comment.first);
        if (existing == all_comments.end()) {
          all_comments[comment.first] = contribution;
        } else {
          const double aggregate = existing->second + contribution;
          if (std::isfinite(aggregate)) {
            existing->second = aggregate;
          }
        }
      }
    }

    const ActionArbitrationResult selection =
      selectHighestScoringAction(all_comments);
    result.selected = selection.selected;
    if (!selection.selected) {
      result.action = FORRAction(PAUSE, 0);
    } else {
      result.action = selection.action;
    }
    std::sort(
      result.contributions.begin(), result.contributions.end(),
      [](const AdvisorContribution& left,
         const AdvisorContribution& right) {
        return std::tie(left.advisor, left.action) <
          std::tie(right.advisor, right.action);
      });

    return result;
  }

private:
  TierThreeDependencies dependencies_;
};

}  // namespace

std::unique_ptr<TierThreeDecision> makeTierThreeDecision(
  TierThreeDependencies dependencies) {
  return std::make_unique<DefaultTierThreeDecision>(dependencies);
}

}  // namespace decision
}  // namespace semaforr
