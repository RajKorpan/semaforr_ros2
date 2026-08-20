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
enum class TierThreeScoringPolicy { CompatibilityComments, WeightedNormalized };
enum class TierThreeTiePolicy { Exact, Tolerance };

struct ArbitrationConfiguration {
  double tie_tolerance{1.0e-9};
  UnscoredActionPolicy unscored_policy{UnscoredActionPolicy::Exclude};
  double unscored_baseline{0.0};
  std::optional<domain::Action> fallback;
  std::uint32_t random_seed{0U};
  TierThreeScoringPolicy scoring_policy{
      TierThreeScoringPolicy::WeightedNormalized};
  TierThreeTiePolicy tie_policy{TierThreeTiePolicy::Tolerance};
  bool circumstance_weighting_enabled{false};
  std::size_t circumstance_minimum_evidence{10U};
  std::size_t circumstance_minimum_action_evidence{5U};
  double circumstance_minimum_assignment_confidence{0.95};
  double circumstance_minimum_case_accuracy{0.75};
  double circumstance_maximum_influence{0.5};
};

struct TierOnePass {
  std::optional<DecisionResult> decision;
  std::vector<domain::Action> survivors;
  std::vector<Veto> vetoes;
  std::vector<DecisionCycleEvent> trace;
};

// The rules owned by DecisionCoordinator surround the Tier-1 components that
// are stateful engine subsystems (Enforcer, reactive planners, and LLE).
// Keeping these stages explicit prevents C++ interface type (mandatory versus
// veto) from changing the cognitive order.
enum class TierOneStage { BeforeEnforcer, AfterLowLevelExploration, All };

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
  TierOnePass evaluateTierOne(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const;
  TierOnePass evaluateTierOneStage(
      const DecisionContext& context,
      std::span<const domain::Action> candidates,
      TierOneStage stage) const;
  DecisionResult decideTierThree(
      const DecisionContext& context,
      std::span<const domain::Action> candidates);

 private:
  enum class RegisteredRuleKind { Mandatory, Veto };
  struct RegisteredRule {
    RegisteredRuleKind kind;
    std::size_t index;
  };

  ArbitrationConfiguration configuration_;
  std::mt19937 random_;
  std::vector<std::unique_ptr<MandatoryRule>> mandatory_rules_;
  std::vector<std::unique_ptr<VetoRule>> veto_rules_;
  std::vector<RegisteredRule> tier_one_order_;
  std::vector<std::unique_ptr<Advisor>> advisors_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_DECISION_COORDINATOR_HPP
