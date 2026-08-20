#ifndef SEMAFORR_DECISION_NAVIGATION_ADVISOR_HPP
#define SEMAFORR_DECISION_NAVIGATION_ADVISOR_HPP

#include <semaforr/decision/advisor.hpp>
#include <semaforr/domain/world_model.hpp>
#include <string>
#include <string_view>

namespace semaforr::decision {

enum class NavigationAdvisorObjective { GoalProgress, Clearance, Exploration };

enum class ActionSelection { All, Linear, Rotation };

struct NavigationAdvisorConfiguration {
  std::string name;
  NavigationAdvisorObjective objective{
      NavigationAdvisorObjective::GoalProgress};
  ActionSelection selection{ActionSelection::All};
  domain::ActionSpace action_space{{0.1}, {0.1}};
  double weight{1.0};
};

class NavigationAdvisor final : public Advisor {
 public:
  explicit NavigationAdvisor(NavigationAdvisorConfiguration configuration);

  std::string_view name() const noexcept override {
    return configuration_.name;
  }

  AdvisorEvaluation evaluate(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const override;
  AdvisorMetadata metadata() const override {
    return {{},
            {domain::ActionType::Pause, domain::ActionType::Forward,
             domain::ActionType::TurnLeft, domain::ActionType::TurnRight},
            configuration_.objective !=
                NavigationAdvisorObjective::GoalProgress,
            ScoreNormalization::TenPoint,
            "local navigation objective"};
  }

 private:
  bool accepts(const domain::Action& action) const noexcept;
  double score(const DecisionContext& context,
               const domain::Action& action) const;

  NavigationAdvisorConfiguration configuration_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_NAVIGATION_ADVISOR_HPP
