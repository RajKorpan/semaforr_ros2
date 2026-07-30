#ifndef SEMAFORR_DECISION_OBSTACLE_VETO_RULE_HPP
#define SEMAFORR_DECISION_OBSTACLE_VETO_RULE_HPP

#include <semaforr/decision/rules.hpp>
#include <vector>

namespace semaforr::decision {

class ObstacleVetoRule final : public VetoRule {
 public:
  ObstacleVetoRule(std::vector<double> move_distances_m, double robot_radius_m,
                   double obstacle_buffer_m);

  std::vector<Veto> evaluate(const DecisionContext& context) const override;

 private:
  std::vector<double> move_distances_m_;
  double clearance_m_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_OBSTACLE_VETO_RULE_HPP
