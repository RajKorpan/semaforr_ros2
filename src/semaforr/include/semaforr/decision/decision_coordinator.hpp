/**
 * @file decision_coordinator.hpp
 * @brief Decision coordinator responsibilities.
 *
 * @details This file defines decision coordinator behavior for tiered decision
 * making and action arbitration. It centers on `UnscoredActionPolicy`,
 * `TierThreeScoringPolicy`, `TierThreeTiePolicy`,
 * `ArbitrationConfiguration`, `TierOnePass`, `TierOneStage`,
 * `DecisionCoordinator`, `RegisteredRuleKind`. Its package-relative
 * location is `include/semaforr/decision/decision_coordinator.hpp`.
 */
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

/**
 * @brief Enumerates the supported unscored action policy values used by
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class UnscoredActionPolicy { Exclude, Zero, Baseline };
/**
 * @brief Enumerates the supported tier three scoring policy values used by
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class TierThreeScoringPolicy { CompatibilityComments, WeightedNormalized };
/**
 * @brief Enumerates the supported tier three tie policy values used by this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class TierThreeTiePolicy { Exact, Tolerance };

/**
 * @brief Encapsulates arbitration configuration state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Encapsulates tier one pass state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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
/**
 * @brief Enumerates the supported tier one stage values used by this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class TierOneStage { BeforeEnforcer, AfterLowLevelExploration, All };

/**
 * @brief Encapsulates decision coordinator state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class DecisionCoordinator {
 public:
  /**
   * @brief Performs the decision coordinator operation for this subsystem.
   *
   * Arguments:
   * - @p configuration: Supplies configuration input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit DecisionCoordinator(ArbitrationConfiguration configuration = {});

  /**
   * @brief Performs the add mandatory rule operation for this subsystem.
   *
   * Arguments:
   * - @p rule: Supplies rule input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void addMandatoryRule(std::unique_ptr<MandatoryRule> rule);
  /**
   * @brief Performs the add veto rule operation for this subsystem.
   *
   * Arguments:
   * - @p rule: Supplies rule input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void addVetoRule(std::unique_ptr<VetoRule> rule);
  /**
   * @brief Performs the add advisor operation for this subsystem.
   *
   * Arguments:
   * - @p advisor: Supplies advisor input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void addAdvisor(std::unique_ptr<Advisor> advisor);

  /**
   * @brief Selects package content for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - `DecisionResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  DecisionResult decide(const DecisionContext& context,
                        std::span<const domain::Action> candidates);
  /**
   * @brief Performs the mandatory decision operation for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - `std::optional<DecisionResult>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<DecisionResult> mandatoryDecision(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const;
  /**
   * @brief Evaluates tier one for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - `TierOnePass` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  TierOnePass evaluateTierOne(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const;
  /**
   * @brief Evaluates tier one stage for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   * - @p stage: Supplies stage input to the operation.
   *
   * Returns:
   * - `TierOnePass` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  TierOnePass evaluateTierOneStage(
      const DecisionContext& context,
      std::span<const domain::Action> candidates,
      TierOneStage stage) const;
  /**
   * @brief Selects tier three for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - `DecisionResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  DecisionResult decideTierThree(
      const DecisionContext& context,
      std::span<const domain::Action> candidates);

 private:
  /**
   * @brief Enumerates the supported registered rule kind values used by
   * this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - Not applicable to this declaration.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  enum class RegisteredRuleKind { Mandatory, Veto };
  /**
   * @brief Encapsulates registered rule state and behavior for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - Not applicable to this declaration.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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
