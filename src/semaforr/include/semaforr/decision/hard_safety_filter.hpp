/**
 * @file hard_safety_filter.hpp
 * @brief Hard safety filter responsibilities.
 *
 * @details This file defines hard safety filter behavior for tiered decision making
 * and action arbitration. It centers on `SafetyFilterResult`,
 * `HardSafetyFilter`. Its package-relative location is
 * `include/semaforr/decision/hard_safety_filter.hpp`.
 */
#ifndef SEMAFORR_DECISION_HARD_SAFETY_FILTER_HPP
#define SEMAFORR_DECISION_HARD_SAFETY_FILTER_HPP

#include <chrono>
#include <cmath>
#include <semaforr/decision/obstacle_veto_rule.hpp>
#include <semaforr/domain/world_model.hpp>
#include <span>
#include <stdexcept>
#include <vector>

namespace semaforr::decision {

/**
 * @brief Encapsulates safety filter result state and behavior for this
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
struct SafetyFilterResult {
  std::vector<domain::Action> safe_actions;
  std::vector<Veto> vetoes;
};

/**
 * @brief Encapsulates hard safety filter state and behavior for this
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
class HardSafetyFilter {
 public:
  /**
   * @brief Performs the hard safety filter operation for this subsystem.
   *
   * Arguments:
   * - @p move_distances_m: Supplies move distances m input to the
   * operation.
   * - @p rotation_angles_rad: Supplies rotation angles rad input to the
   * operation.
   * - @p robot_radius_m: Supplies robot radius m input to the operation.
   * - @p obstacle_buffer_m: Supplies obstacle buffer m input to the
   * operation.
   * - @p sensor_freshness_timeout_s: Supplies sensor freshness timeout s
   * input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  HardSafetyFilter(std::vector<double> move_distances_m,
                   std::vector<double> rotation_angles_rad,
                   double robot_radius_m, double obstacle_buffer_m,
                   double sensor_freshness_timeout_s = 0.5)
      : action_space_(move_distances_m, std::move(rotation_angles_rad)),
        obstacle_filter_(std::move(move_distances_m), robot_radius_m,
                         obstacle_buffer_m),
        sensor_freshness_timeout_(
            std::chrono::duration<double>(sensor_freshness_timeout_s)) {
    if (!std::isfinite(sensor_freshness_timeout_s) ||
        sensor_freshness_timeout_s <= 0.0)
      throw std::invalid_argument(
          "hard safety sensor freshness timeout must be finite and positive");
  }
  /**
   * @brief Performs the filter operation for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - `SafetyFilterResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  SafetyFilterResult filter(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const;

 private:
  domain::ActionSpace action_space_;
  ObstacleVetoRule obstacle_filter_;
  std::chrono::duration<double> sensor_freshness_timeout_;
};

}  // namespace semaforr::decision

#endif
