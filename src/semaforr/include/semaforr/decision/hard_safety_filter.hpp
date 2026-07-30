#ifndef SEMAFORR_DECISION_HARD_SAFETY_FILTER_HPP
#define SEMAFORR_DECISION_HARD_SAFETY_FILTER_HPP

#include <semaforr/decision/obstacle_veto_rule.hpp>
#include <span>
#include <vector>

namespace semaforr::decision {

struct SafetyFilterResult {
  std::vector<domain::Action> safe_actions;
  std::vector<Veto> vetoes;
};

class HardSafetyFilter {
 public:
  HardSafetyFilter(std::vector<double> move_distances_m,
                   double robot_radius_m, double obstacle_buffer_m)
      : obstacle_filter_(std::move(move_distances_m), robot_radius_m,
                         obstacle_buffer_m) {}
  SafetyFilterResult filter(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const;

 private:
  ObstacleVetoRule obstacle_filter_;
};

}  // namespace semaforr::decision

#endif
