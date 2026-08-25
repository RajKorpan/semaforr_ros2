/**
 * @file obstacle_veto_rule.hpp
 * @brief Obstacle veto rule responsibilities.
 *
 * @details This file defines obstacle veto rule behavior for tiered decision making
 * and action arbitration. It centers on `ObstacleVetoRule`. Its
 * package-relative location is
 * `include/semaforr/decision/obstacle_veto_rule.hpp`.
 */
#ifndef SEMAFORR_DECISION_OBSTACLE_VETO_RULE_HPP
#define SEMAFORR_DECISION_OBSTACLE_VETO_RULE_HPP

#include <semaforr/decision/rules.hpp>
#include <vector>

namespace semaforr::decision {

/**
 * @brief Encapsulates obstacle veto rule state and behavior for this
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
class ObstacleVetoRule final : public VetoRule {
 public:
  /**
   * @brief Performs the obstacle veto rule operation for this subsystem.
   *
   * Arguments:
   * - @p move_distances_m: Supplies move distances m input to the
   * operation.
   * - @p robot_radius_m: Supplies robot radius m input to the operation.
   * - @p obstacle_buffer_m: Supplies obstacle buffer m input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ObstacleVetoRule(std::vector<double> move_distances_m, double robot_radius_m,
                   double obstacle_buffer_m);
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
    return "AvoidObstacles";
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
    return {"laser", "robot_footprint"};
  }

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
  std::vector<Veto> evaluate(const DecisionContext& context) const override;

 private:
  std::vector<double> move_distances_m_;
  double clearance_m_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_OBSTACLE_VETO_RULE_HPP
