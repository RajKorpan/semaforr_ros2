/**
 * @file obstacle_veto_rule.cpp
 * @brief Obstacle veto rule responsibilities.
 *
 * @details This file implements obstacle veto rule behavior for tiered decision
 * making and action arbitration. It records the declarations, settings,
 * fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/decision/obstacle_veto_rule.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/decision/obstacle_veto_rule.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace semaforr::decision {

/**
 * @brief Performs the obstacle veto rule operation for this subsystem.
 *
 * Arguments:
 * - @p move_distances_m: Supplies move distances m input to the operation.
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
ObstacleVetoRule::ObstacleVetoRule(std::vector<double> move_distances_m,
                                   double robot_radius_m,
                                   double obstacle_buffer_m)
    : move_distances_m_(std::move(move_distances_m)),
      clearance_m_(robot_radius_m + obstacle_buffer_m) {
  if (move_distances_m_.empty() ||
      !std::is_sorted(move_distances_m_.begin(), move_distances_m_.end()) ||
      !std::all_of(
          move_distances_m_.begin(), move_distances_m_.end(),
          [](double value) { return std::isfinite(value) && value > 0.0; })) {
    throw std::invalid_argument(
        "obstacle veto move distances must be finite, positive, and sorted");
  }
  if (!std::isfinite(robot_radius_m) || robot_radius_m < 0.0 ||
      !std::isfinite(obstacle_buffer_m) || obstacle_buffer_m < 0.0) {
    throw std::invalid_argument(
        "obstacle veto clearance values must be finite and nonnegative");
  }
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
std::vector<Veto> ObstacleVetoRule::evaluate(
    const DecisionContext& context) const {
  const auto& laser = context.world.robot.laser;
  if (!laser) {
    return {};
  }
  laser->validate();

  double nearest_longitudinal_m = std::numeric_limits<double>::infinity();
  double angle = laser->angle_min.radians();
  for (const double range_m : laser->ranges_m) {
    if (std::isfinite(range_m)) {
      const double longitudinal_m = range_m * std::cos(angle);
      const double lateral_m = std::abs(range_m * std::sin(angle));
      if (longitudinal_m > 0.0 && lateral_m <= clearance_m_) {
        nearest_longitudinal_m =
            std::min(nearest_longitudinal_m, longitudinal_m);
      }
    }
    angle += laser->angle_increment.radians();
  }

  std::vector<Veto> vetoes;
  for (std::size_t index = 0U; index < move_distances_m_.size(); ++index) {
    if (move_distances_m_[index] + clearance_m_ >= nearest_longitudinal_m) {
      vetoes.push_back(
          {domain::Action(domain::ActionType::Forward, index + 1U),
           "AvoidObstacles", "avoid_obstacles:forward_corridor_obstructed",
           RejectionKind::Cognitive, VetoCategory::ObstacleConflict});
    }
  }
  return vetoes;
}

}  // namespace semaforr::decision
