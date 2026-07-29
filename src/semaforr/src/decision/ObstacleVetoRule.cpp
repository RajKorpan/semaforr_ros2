#include <semaforr/decision/obstacle_veto_rule.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace semaforr::decision {

ObstacleVetoRule::ObstacleVetoRule(
  std::vector<double> move_distances_m,
  double robot_radius_m,
  double obstacle_buffer_m)
  : move_distances_m_(std::move(move_distances_m)),
    clearance_m_(robot_radius_m + obstacle_buffer_m)
{
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

std::vector<Veto> ObstacleVetoRule::evaluate(
  const DecisionContext& context) const
{
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
      vetoes.push_back({
        domain::Action(domain::ActionType::Forward, index + 1U),
        "avoid_obstacles",
        "forward corridor is obstructed"});
    }
  }
  return vetoes;
}

}  // namespace semaforr::decision
