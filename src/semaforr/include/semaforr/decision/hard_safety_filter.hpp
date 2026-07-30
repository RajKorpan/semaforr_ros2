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

struct SafetyFilterResult {
  std::vector<domain::Action> safe_actions;
  std::vector<Veto> vetoes;
};

class HardSafetyFilter {
 public:
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
