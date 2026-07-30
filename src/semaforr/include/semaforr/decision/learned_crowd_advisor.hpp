#ifndef SEMAFORR_DECISION_LEARNED_CROWD_ADVISOR_HPP
#define SEMAFORR_DECISION_LEARNED_CROWD_ADVISOR_HPP

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
};

class LearnedCrowdAdvisor final : public Advisor {
 public:
  explicit LearnedCrowdAdvisor(LearnedCrowdAdvisorConfiguration configuration);

  std::string_view name() const noexcept override {
    return configuration_.advisor_name;
  }
  std::vector<std::string_view> dependencies() const override {
    return {"learned_crowd_field"};
  }
  AdvisorMetadata metadata() const override {
    return {dependencies(),
            {domain::ActionType::Pause, domain::ActionType::Forward,
             domain::ActionType::TurnLeft, domain::ActionType::TurnRight},
            false, ScoreNormalization::None,
            "learned crowd density, risk, or flow preference"};
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
