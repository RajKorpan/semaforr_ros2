#ifndef SEMAFORR_DECISION_DISSERTATION_ADVISOR_HPP
#define SEMAFORR_DECISION_DISSERTATION_ADVISOR_HPP

#include <semaforr/decision/advisor.hpp>
#include <semaforr/domain/world_model.hpp>
#include <string>

namespace semaforr::decision {

enum class DissertationAdvisorObjective {
  BigStep, ElbowRoom, Novelty, GoAround, Greedy, Curiosity, Enfilade,
  VisualScan, Convey, Enter, Exit, Trailer, Unlikely, Access, Crossroads,
  Follow, LeastAngle, SpatialLearner, Stay
};

struct DissertationAdvisorConfiguration {
  std::string name;
  DissertationAdvisorObjective objective;
  domain::ActionSpace action_space{{0.1}, {0.1}};
  double weight = 1.0;
};

class DissertationAdvisor final : public Advisor {
 public:
  explicit DissertationAdvisor(DissertationAdvisorConfiguration);
  std::string_view name() const noexcept override { return configuration_.name; }
  std::vector<std::string_view> dependencies() const override;
  AdvisorMetadata metadata() const override;
  AdvisorEvaluation evaluate(
      const DecisionContext&,
      std::span<const domain::Action> candidates) const override;

 private:
  bool accepts(domain::ActionType) const noexcept;
  double score(const domain::WorldModel&, const domain::Action&) const;
  DissertationAdvisorConfiguration configuration_;
};

}  // namespace semaforr::decision

#endif
