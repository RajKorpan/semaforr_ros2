/**
 * @file heuristic_advisor.hpp
 * @brief Heuristic advisor responsibilities.
 *
 * @details This file defines heuristic advisor behavior for tiered decision making
 * and action arbitration. It centers on `HeuristicObjective`,
 * `HeuristicAdvisorConfiguration`, `HeuristicAdvisor`. Its
 * package-relative location is
 * `include/semaforr/decision/advisors/heuristic_advisor.hpp`.
 */
#ifndef SEMAFORR_DECISION_ADVISORS_HEURISTIC_ADVISOR_HPP
#define SEMAFORR_DECISION_ADVISORS_HEURISTIC_ADVISOR_HPP

#include <semaforr/decision/advisor.hpp>
#include <semaforr/domain/world_model.hpp>
#include <string>

namespace semaforr::decision {

/**
 * @brief Enumerates the supported heuristic objective values used by this
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
enum class HeuristicObjective {
  BigStep, ElbowRoom, Novelty, GoAround, Greedy, Curiosity, Enfilade,
  VisualScan, Convey, Enter, Exit, Trailer, Unlikely, Access, Crossroads,
  Follow, LeastAngle, SpatialLearner, Stay
};

/**
 * @brief Encapsulates heuristic advisor configuration state and behavior
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
struct HeuristicAdvisorConfiguration {
  std::string name;
  HeuristicObjective objective;
  domain::ActionSpace action_space{{0.1}, {0.1}};
  double weight = 1.0;
};

/**
 * @brief Encapsulates heuristic advisor state and behavior for this
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
class HeuristicAdvisor final : public Advisor {
 public:
  /**
   * @brief Performs the heuristic advisor operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit HeuristicAdvisor(HeuristicAdvisorConfiguration);
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
  std::string_view name() const noexcept override { return configuration_.name; }
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
  std::vector<std::string_view> dependencies() const override;
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
  AdvisorMetadata metadata() const override;
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - `AdvisorEvaluation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  AdvisorEvaluation evaluate(
      const DecisionContext&,
      std::span<const domain::Action> candidates) const override;

 private:
  /**
   * @brief Performs the accepts operation for this subsystem.
   *
   * Arguments:
   * - @p ActionType: Supplies action type input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool accepts(domain::ActionType) const noexcept;
  /**
   * @brief Performs the applicable operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool applicable(const DecisionContext&) const;
  /**
   * @brief Performs the score operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   * - @p Action: Supplies action input to the operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double score(const DecisionContext&, const domain::Action&) const;
  HeuristicAdvisorConfiguration configuration_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_ADVISORS_HEURISTIC_ADVISOR_HPP
