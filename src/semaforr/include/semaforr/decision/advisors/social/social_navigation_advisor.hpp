/**
 * @file social_navigation_advisor.hpp
 * @brief Social navigation advisor responsibilities.
 *
 * @details This file defines social navigation advisor behavior for tiered decision
 * making and action arbitration. It centers on
 * `SocialAdvisorConfiguration`, `SocialNavigationAdvisor`. Its
 * package-relative location is
 * `include/semaforr/decision/advisors/social/social_navigation_advisor.hpp`.
 */
#ifndef SEMAFORR_DECISION_SOCIAL_NAVIGATION_ADVISOR_HPP
#define SEMAFORR_DECISION_SOCIAL_NAVIGATION_ADVISOR_HPP

#include <chrono>
#include <semaforr/decision/advisor.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::decision {

/**
 * @brief Encapsulates social advisor configuration state and behavior for
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
struct SocialAdvisorConfiguration {
  std::vector<double> move_distances_m;
  std::vector<double> rotation_angles_rad;
  /**
   * @brief Performs the milliseconds operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `std::chrono::nanoseconds maximum_age{` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::chrono::nanoseconds maximum_age{std::chrono::milliseconds(750)};
  double minimum_confidence{0.25};
  double prediction_horizon_s{2.0};
  double personal_space_m{1.2};
  double collision_distance_m{0.65};
  double weight{1.0};
  std::string advisor_name{"social_navigation"};
};

/**
 * @brief Encapsulates social navigation advisor state and behavior for this
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
class SocialNavigationAdvisor final : public Advisor {
 public:
  /**
   * @brief Performs the social navigation advisor operation for this
   * subsystem.
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
  explicit SocialNavigationAdvisor(SocialAdvisorConfiguration configuration);

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
    return {"live_crowd_observations"};
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
    return {dependencies(),
            {domain::ActionType::Pause, domain::ActionType::Forward,
             domain::ActionType::TurnLeft, domain::ActionType::TurnRight},
            true, ScoreNormalization::TenPoint,
            "predictive personal-space and collision preference"};
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
   * @brief Performs the score operation for this subsystem.
   *
   * Arguments:
   * - @p world: Supplies world input to the operation.
   * - @p crowd: Supplies crowd input to the operation.
   * - @p action: Supplies action input to the operation.
   *
   * Returns:
   * - `double` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  double score(const domain::WorldModel& world,
               const domain::CrowdObservation& crowd,
               const domain::Action& action) const;

  SocialAdvisorConfiguration configuration_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_SOCIAL_NAVIGATION_ADVISOR_HPP
