#ifndef SEMAFORR_DECISION_DECISION_COORDINATOR_HPP
#define SEMAFORR_DECISION_DECISION_COORDINATOR_HPP

#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <semaforr/decision/advisor.hpp>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/decision/rules.hpp>
#include <span>
#include <vector>

namespace semaforr::decision {

enum class UnscoredActionPolicy { Exclude, Zero, Baseline };

struct ArbitrationConfiguration {
  double tie_tolerance{1.0e-9};
  UnscoredActionPolicy unscored_policy{UnscoredActionPolicy::Exclude};
  double unscored_baseline{0.0};
  std::optional<domain::Action> fallback;
  std::uint32_t random_seed{0U};
};

class DecisionCoordinator {
 public:
  explicit DecisionCoordinator(ArbitrationConfiguration configuration = {});

  void addMandatoryRule(std::unique_ptr<MandatoryRule> rule);
  void addVetoRule(std::unique_ptr<VetoRule> rule);
  void addAdvisor(std::unique_ptr<Advisor> advisor);

  DecisionResult decide(const DecisionContext& context,
                        std::span<const domain::Action> candidates);
  std::optional<DecisionResult> mandatoryDecision(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const;

 private:
  ArbitrationConfiguration configuration_;
  std::mt19937 random_;
  std::vector<std::unique_ptr<MandatoryRule>> mandatory_rules_;
  std::vector<std::unique_ptr<VetoRule>> veto_rules_;
  std::vector<std::unique_ptr<Advisor>> advisors_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_DECISION_COORDINATOR_HPP
