/*
 * Tier-three decision implementation.
 */

#include <semaforr/decision/DecisionTier.h>
#include "DecisionTierFactory.h"

#include <semaforr/decision/Tier3Advisor.h>

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <map>
#include <sstream>
#include <utility>
#include <vector>

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
    std::vector<FORRAction> best_decisions;

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
        if (all_comments.find(comment.first) == all_comments.end()) {
          all_comments[comment.first] = comment.second * weight;
        } else {
          all_comments[comment.first] += comment.second * weight;
        }
      }
    }

    std::cout << "After score aggregation" << std::endl;
    double max_advice_strength = -1000.0;
    double max_weight = 1.0;
    for (const auto& comment : all_comments) {
      const double action_weight = 1.0;
      std::cout << "Values are : " << comment.first.type << " "
                << comment.first.parameter << " with value: "
                << comment.second << " and weight: "
                << action_weight << std::endl;
      if (action_weight * comment.second > max_advice_strength) {
        max_advice_strength = action_weight * comment.second;
        max_weight = action_weight;
      }
    }
    std::cout << "Max vote strength " << max_advice_strength << std::endl;

    for (const auto& comment : all_comments) {
      if (max_weight * comment.second == max_advice_strength) {
        best_decisions.push_back(comment.first);
      }
    }

    std::cout << "There are " << best_decisions.size()
              << " decisions that got the highest grade " << std::endl;
    if (best_decisions.empty()) {
      result.action = FORRAction(PAUSE, 0);
    } else {
      std::srand(std::time(nullptr));
      const int random_number =
        std::rand() % static_cast<int>(best_decisions.size());
      result.action = best_decisions.at(random_number);
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
