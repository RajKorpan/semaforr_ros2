#ifndef SEMAFORR_DECISION_LEARNED_CROWD_ADVISOR_HPP
#define SEMAFORR_DECISION_LEARNED_CROWD_ADVISOR_HPP

#include <chrono>
#include <semaforr/decision/advisor.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::decision {

enum class LearnedCrowdObjective {
  AvoidDensity,
  AvoidEncounterRisk,
  PreferFollowingFlow
};

struct LearnedCrowdAdvisorConfiguration {
  LearnedCrowdObjective objective{LearnedCrowdObjective::AvoidDensity};
  std::vector<double> move_distances_m;
  std::vector<double> rotation_angles_rad;
  double weight{1.0};
  double minimum_cell_confidence{0.0};
  std::string advisor_name{"learned_crowd"};
  std::chrono::nanoseconds maximum_live_age{
      std::chrono::milliseconds(750)};
  double minimum_live_confidence{0.25};
};

class LearnedCrowdAdvisor final : public Advisor {
 public:
  explicit LearnedCrowdAdvisor(LearnedCrowdAdvisorConfiguration configuration);

  std::string_view name() const noexcept override {
    return configuration_.advisor_name;
  }
  std::vector<std::string_view> dependencies() const override {
    switch (configuration_.objective) {
      case LearnedCrowdObjective::AvoidDensity:
        return {"learned_crowd_density"};
      case LearnedCrowdObjective::AvoidEncounterRisk:
        return {"live_or_learned_crowd_risk"};
      case LearnedCrowdObjective::PreferFollowingFlow:
        return {"learned_crowd_flow"};
    }
    return {};
  }
  AdvisorMetadata metadata() const override {
    std::string_view rationale;
    switch (configuration_.objective) {
      case LearnedCrowdObjective::AvoidDensity:
        rationale = "prefer actions entering lower learned crowd density";
        break;
      case LearnedCrowdObjective::AvoidEncounterRisk:
        rationale = "avoid learned encounters and fresh predicted collisions";
        break;
      case LearnedCrowdObjective::PreferFollowingFlow:
        rationale = "align travel with the learned local crowd flow";
        break;
    }
    return {dependencies(),
            {domain::ActionType::Pause, domain::ActionType::Forward,
             domain::ActionType::TurnLeft, domain::ActionType::TurnRight},
            true, ScoreNormalization::SignedUnit, rationale};
  }

  AdvisorEvaluation evaluate(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const override;

 private:
  std::pair<domain::Point2D, domain::Angle> expected(
      const domain::WorldModel& world, const domain::Action& action) const;

  LearnedCrowdAdvisorConfiguration configuration_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_LEARNED_CROWD_ADVISOR_HPP
