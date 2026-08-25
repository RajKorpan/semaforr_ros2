/**
 * @file advisor.hpp
 * @brief Advisor responsibilities.
 *
 * @details This file defines advisor behavior for tiered decision making and action
 * arbitration. It centers on `ScoreNormalization`, `AdvisorMetadata`,
 * `ActionScore`, `AdvisorEvaluation`, `Advisor`. Its package-relative
 * location is `include/semaforr/decision/advisor.hpp`.
 */
#ifndef SEMAFORR_DECISION_ADVISOR_HPP
#define SEMAFORR_DECISION_ADVISOR_HPP

#include <semaforr/decision/context.hpp>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::decision {

/**
 * @brief Enumerates the supported score normalization values used by this
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
enum class ScoreNormalization { None, UnitInterval, SignedUnit, TenPoint };

/**
 * @brief Encapsulates advisor metadata state and behavior for this
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
struct AdvisorMetadata {
  std::vector<std::string_view> required_representations;
  std::vector<domain::ActionType> scored_action_types;
  bool participates_without_target = false;
  ScoreNormalization normalization = ScoreNormalization::None;
  std::string_view rationale;
};

/**
 * @brief Encapsulates action score state and behavior for this subsystem.
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
struct ActionScore {
  domain::Action action;
  double raw_score{0.0};
};

/**
 * @brief Encapsulates advisor evaluation state and behavior for this
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
struct AdvisorEvaluation {
  bool participated{false};
  std::vector<ActionScore> scores;
  double weight{1.0};
  std::string explanation;
  std::size_t model_revision_used = 0U;
};

/**
 * @brief Encapsulates advisor state and behavior for this subsystem.
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
class Advisor {
 public:
  /**
   * @brief Performs the advisor operation for this subsystem.
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
  virtual ~Advisor() = default;
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
  virtual AdvisorMetadata metadata() const {
    return {dependencies(),
            {domain::ActionType::Pause, domain::ActionType::Forward,
             domain::ActionType::TurnLeft, domain::ActionType::TurnRight},
            false, ScoreNormalization::SignedUnit, "advisor score"};
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
  virtual AdvisorEvaluation evaluate(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const = 0;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_ADVISOR_HPP
