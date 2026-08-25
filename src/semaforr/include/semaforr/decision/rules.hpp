/**
 * @file rules.hpp
 * @brief Rules responsibilities.
 *
 * @details This file defines rules behavior for tiered decision making and action
 * arbitration. It centers on `Decision`, `MandatoryRule`, `VetoRule`,
 * `PlanOperationalizer`, `ReplanningRequest`, `ReplanningTrigger`. Its
 * package-relative location is `include/semaforr/decision/rules.hpp`.
 */
#ifndef SEMAFORR_DECISION_RULES_HPP
#define SEMAFORR_DECISION_RULES_HPP

#include <optional>
#include <semaforr/decision/context.hpp>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::decision {

/**
 * @brief Encapsulates decision state and behavior for this subsystem.
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
struct Decision {
  domain::Action action;
  std::string rule;
  std::string explanation;
};

/**
 * @brief Encapsulates mandatory rule state and behavior for this subsystem.
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
class MandatoryRule {
 public:
  /**
   * @brief Performs the mandatory rule operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual ~MandatoryRule() = default;
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::string_view name() const noexcept { return "mandatory_rule"; }
  /**
   * @brief Performs the dependencies operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<std::string_view>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::vector<std::string_view> dependencies() const { return {}; }
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   *
   * Returns:
   * - `std::optional<Decision>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::optional<Decision> evaluate(
      const DecisionContext& context) const = 0;
};

/**
 * @brief Encapsulates veto rule state and behavior for this subsystem.
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
class VetoRule {
 public:
  /**
   * @brief Performs the veto rule operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual ~VetoRule() = default;
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::string_view name() const noexcept { return "veto_rule"; }
  /**
   * @brief Performs the dependencies operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<std::string_view>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::vector<std::string_view> dependencies() const { return {}; }
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   *
   * Returns:
   * - `std::vector<Veto>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::vector<Veto> evaluate(const DecisionContext& context) const = 0;
  /**
   * @brief Performs the last reason operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::string lastReason() const { return {}; }
};

/**
 * @brief Encapsulates plan operationalizer state and behavior for this
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
class PlanOperationalizer {
 public:
  /**
   * @brief Constructs operationalizer for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual ~PlanOperationalizer() = default;
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::string_view name() const noexcept = 0;
  /**
   * @brief Performs the operationalize operation for this subsystem.
   *
   * Arguments:
   * - @p HierarchicalPlan: Supplies hierarchical plan input to the
   * operation.
   *
   * Returns:
   * - `std::vector<domain::Point2D>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::vector<domain::Point2D> operationalize(
      const planning::HierarchicalPlan&) const = 0;
  /**
   * @brief Performs the operationalize next operation for this subsystem.
   *
   * Arguments:
   * - @p HierarchicalPlan: Supplies hierarchical plan input to the
   * operation.
   * - @p SpatialModel: Supplies spatial model input to the operation.
   * - @p Pose2D: Supplies pose2 d input to the operation.
   * - @p Distance: Supplies distance input to the operation.
   *
   * Returns:
   * - `std::optional<domain::Point2D>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::optional<domain::Point2D> operationalizeNext(
      planning::HierarchicalPlan&, const domain::SpatialModel&,
      const domain::Pose2D&, domain::Distance) const = 0;
};

/**
 * @brief Encapsulates replanning request state and behavior for this
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
struct ReplanningRequest {
  bool requested = false;
  std::string reason;
};

/**
 * @brief Encapsulates replanning trigger state and behavior for this
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
class ReplanningTrigger {
 public:
  /**
   * @brief Performs the replanning trigger operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual ~ReplanningTrigger() = default;
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::string_view name() const noexcept = 0;
  /**
   * @brief Evaluates replan for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `ReplanningRequest` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual ReplanningRequest evaluateReplan(const DecisionContext&) const = 0;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_RULES_HPP
