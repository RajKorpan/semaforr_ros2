/**
 * @file navigation_advisor.hpp
 * @brief Navigation advisor responsibilities.
 *
 * @details This file defines navigation advisor behavior for tiered decision making
 * and action arbitration. It centers on `NavigationAdvisorObjective`,
 * `ActionSelection`, `NavigationAdvisorConfiguration`,
 * `NavigationAdvisor`. Its package-relative location is
 * `include/semaforr/decision/advisors/navigation_advisor.hpp`.
 */
#ifndef SEMAFORR_DECISION_NAVIGATION_ADVISOR_HPP
#define SEMAFORR_DECISION_NAVIGATION_ADVISOR_HPP

#include <semaforr/decision/advisor.hpp>
#include <semaforr/domain/world_model.hpp>
#include <string>
#include <string_view>

namespace semaforr::decision {

/**
 * @brief Enumerates the supported navigation advisor objective values used
 * by this subsystem.
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
enum class NavigationAdvisorObjective { GoalProgress, Clearance, Exploration };

/**
 * @brief Enumerates the supported action selection values used by this
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
enum class ActionSelection { All, Linear, Rotation };

/**
 * @brief Encapsulates navigation advisor configuration state and behavior
 * for this subsystem.
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
struct NavigationAdvisorConfiguration {
  std::string name;
  NavigationAdvisorObjective objective{
      NavigationAdvisorObjective::GoalProgress};
  ActionSelection selection{ActionSelection::All};
  domain::ActionSpace action_space{{0.1}, {0.1}};
  double weight{1.0};
};

/**
 * @brief Encapsulates navigation advisor state and behavior for this
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
class NavigationAdvisor final : public Advisor {
 public:
  /**
   * @brief Performs the navigation advisor operation for this subsystem.
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
  explicit NavigationAdvisor(NavigationAdvisorConfiguration configuration);

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
  std::string_view name() const noexcept override {
    return configuration_.name;
  }

  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - `AdvisorEvaluation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  AdvisorEvaluation evaluate(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const override;
  /**
   * @brief Performs the metadata operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `AdvisorMetadata` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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
  /**
   * @brief Performs the accepts operation for this subsystem.
   *
   * Arguments:
   * - @p action: Supplies action input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool accepts(const domain::Action& action) const noexcept;
  /**
   * @brief Performs the score operation for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   * - @p action: Supplies action input to the operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double score(const DecisionContext& context,
               const domain::Action& action) const;

  NavigationAdvisorConfiguration configuration_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_NAVIGATION_ADVISOR_HPP
