/*
 * Tier-three decision implementation.
 */

#include <semaforr/decision/DecisionTier.h>
#include <semaforr/decision/Arbitration.h>
#include "DecisionTierFactory.h"

#include <semaforr/decision/Tier3Advisor.h>

#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
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

    std::stringstream advisors_list;
    std::stringstream advisor_comments_list;
    std::cout << "processing advisors::" << std::endl;
    for (const auto& owned_advisor : dependencies_.advisors) {
      Tier3Advisor* advisor = owned_advisor.get();
      std::cout << advisor->get_name() << std::endl;
      advisor->set_commenting();
      if (!advisor->is_active() || !advisor->is_commenting()) {
        advisors_list << advisor->get_name() << " "
                      << advisor->get_weight() << " "
                      << advisor->is_active() << " "
                      << advisor->is_commenting() << ";";
        continue;
      }

      advisors_list << advisor->get_name() << " "
                    << advisor->get_weight() << " "
                    << advisor->is_active() << " "
                    << advisor->is_commenting() << ";";

      std::cout << "Before commenting " << std::endl;
      comments = advisor->allAdvice();
      std::cout << "after commenting " << std::endl;

      for (const auto& comment : comments) {
        const float weight = advisor->get_weight();
        advisor_comments_list << advisor->get_name() << " "
                              << comment.first.type << " "
                              << comment.first.parameter << " "
                              << comment.second << ";";

        std::cout << "Start of score aggregation" << std::endl;
        const double contribution = comment.second * weight;
        if (!std::isfinite(contribution)) {
          continue;
        }

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

    std::cout << "After score aggregation" << std::endl;
    for (const auto& comment : all_comments) {
      const double action_weight = 1.0;
      std::cout << "Values are : " << comment.first.type << " "
                << comment.first.parameter << " with value: "
                << comment.second << " and weight: "
                << action_weight << std::endl;
    }

    const ActionArbitrationResult selection =
      selectHighestScoringAction(all_comments);
    std::size_t tied_decisions = 0;
    for (const auto& comment : all_comments) {
      if (selection.selected && std::isfinite(comment.second) &&
          comment.second == selection.score) {
        ++tied_decisions;
      }
    }

    std::cout << "Max vote strength "
              << (selection.selected ? selection.score : 0.0) << std::endl;
    std::cout << "There are " << tied_decisions
              << " decisions that got the highest grade " << std::endl;
    if (!selection.selected) {
      result.action = FORRAction(PAUSE, 0);
    } else {
      result.action = selection.action;
    }

    result.advisors = advisors_list.str();
    result.advisor_comments = advisor_comments_list.str();
    std::cout << " advisors = " << result.advisors
              << "\nadvisorComments = " << result.advisor_comments
              << std::endl;
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
