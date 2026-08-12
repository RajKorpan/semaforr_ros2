#ifndef SEMAFORR_DECISION_ADVISORS_HEURISTIC_ADVISOR_HPP
#define SEMAFORR_DECISION_ADVISORS_HEURISTIC_ADVISOR_HPP

#include <semaforr/decision/advisor.hpp>
#include <semaforr/domain/world_model.hpp>
#include <string>

namespace semaforr::decision {

enum class HeuristicObjective {
  BigStep, ElbowRoom, Novelty, GoAround, Greedy, Curiosity, Enfilade,
  VisualScan, Convey, Enter, Exit, Trailer, Unlikely, Access, Crossroads,
  Follow, LeastAngle, SpatialLearner, Stay
};

struct HeuristicAdvisorConfiguration {
  std::string name;
  HeuristicObjective objective;
  domain::ActionSpace action_space{{0.1}, {0.1}};
  double weight = 1.0;
};

class HeuristicAdvisor final : public Advisor {
 public:
  explicit HeuristicAdvisor(HeuristicAdvisorConfiguration);
  std::string_view name() const noexcept override { return configuration_.name; }
  std::vector<std::string_view> dependencies() const override;
  AdvisorMetadata metadata() const override;
  AdvisorEvaluation evaluate(
      const DecisionContext&,
      std::span<const domain::Action> candidates) const override;

 private:
  bool accepts(domain::ActionType) const noexcept;
  bool applicable(const DecisionContext&) const;
  double score(const DecisionContext&, const domain::Action&) const;
  HeuristicAdvisorConfiguration configuration_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_ADVISORS_HEURISTIC_ADVISOR_HPP
