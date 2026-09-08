/**
 * @file learned_crowd_advisor.hpp
 * @brief Learned crowd advisor responsibilities.
 *
 * @details This file defines learned crowd advisor behavior for tiered decision
 * making and action arbitration. It centers on `LearnedCrowdObjective`,
 * `LearnedCrowdAdvisorConfiguration`, `LearnedCrowdAdvisor`. Its
 * package-relative location is
 * `include/semaforr/decision/advisors/social/learned_crowd_advisor.hpp`.
 */
#ifndef SEMAFORR_DECISION_LEARNED_CROWD_ADVISOR_HPP
#define SEMAFORR_DECISION_LEARNED_CROWD_ADVISOR_HPP

#include <chrono>
#include <semaforr/decision/advisor.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::decision {

/**
 * @brief Enumerates the supported learned crowd objective values used by
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
enum class LearnedCrowdObjective {
  AvoidDensity,
  AvoidEncounterRisk,
  PreferFollowingFlow
};

/**
 * @brief Encapsulates learned crowd advisor configuration state and
 * behavior for this subsystem.
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
struct LearnedCrowdAdvisorConfiguration {
  LearnedCrowdObjective objective{LearnedCrowdObjective::AvoidDensity};
  std::vector<double> move_distances_m;
  std::vector<double> rotation_angles_rad;
  double weight{1.0};
  double minimum_cell_confidence{0.0};
  std::string advisor_name{"learned_crowd"};
  std::chrono::nanoseconds maximum_live_age{
      /**
       * @brief Performs the milliseconds operation for this subsystem.
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
      std::chrono::milliseconds(750)};
  double minimum_live_confidence{0.25};
};

/**
 * @brief Encapsulates learned crowd advisor state and behavior for this
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
class LearnedCrowdAdvisor final : public Advisor {
 public:
  /**
   * @brief Performs the learned crowd advisor operation for this subsystem.
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
  explicit LearnedCrowdAdvisor(LearnedCrowdAdvisorConfiguration configuration);

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
    return configuration_.advisor_name;
  }
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
  std::vector<std::string_view> dependencies() const override {
    switch (configuration_.objective) {
      case LearnedCrowdObjective::AvoidDensity:
        return {"learned_crowd_density"};
      case LearnedCrowdObjective::AvoidEncounterRisk:
        return {"live_or_learned_crowd_risk"};
      case LearnedCrowdObjective::PreferFollowingFlow:
        return {"learned_crowd_flow"};
    }
    return {};
  }
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
    std::string_view rationale;
    switch (configuration_.objective) {
      case LearnedCrowdObjective::AvoidDensity:
        rationale = "prefer actions entering lower learned crowd density";
        break;
      case LearnedCrowdObjective::AvoidEncounterRisk:
        rationale = "avoid learned encounters and fresh predicted collisions";
        break;
      case LearnedCrowdObjective::PreferFollowingFlow:
        rationale = "align travel with the learned local crowd flow";
        break;
    }
    return {dependencies(),
            {domain::ActionType::Pause, domain::ActionType::Forward,
             domain::ActionType::TurnLeft, domain::ActionType::TurnRight},
            true,
            ScoreNormalization::TenPoint,
            rationale};
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

 private:
  /**
   * @brief Performs the expected operation for this subsystem.
   *
   * Arguments:
   * - @p world: Supplies world input to the operation.
   * - @p action: Supplies action input to the operation.
   *
   * Returns:
   * - `std::pair<domain::Point2D, domain::Angle>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::pair<domain::Point2D, domain::Angle> expected(
      const domain::WorldModel& world, const domain::Action& action) const;

  LearnedCrowdAdvisorConfiguration configuration_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_LEARNED_CROWD_ADVISOR_HPP
