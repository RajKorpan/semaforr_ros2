#ifndef SEMAFORR_DECISION_OBSTACLE_VETO_RULE_HPP
#define SEMAFORR_DECISION_OBSTACLE_VETO_RULE_HPP

#include <semaforr/decision/rules.hpp>
#include <vector>

namespace semaforr::decision {

class ObstacleVetoRule final : public VetoRule {
 public:
  ObstacleVetoRule(std::vector<double> move_distances_m, double robot_radius_m,
                   double obstacle_buffer_m);
  std::string_view name() const noexcept override {
    return "AvoidObstacles";
  }
  std::vector<std::string_view> dependencies() const override {
    return {"laser", "robot_footprint"};
  }

  std::vector<Veto> evaluate(const DecisionContext& context) const override;

 private:
  std::vector<double> move_distances_m_;
  double clearance_m_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_OBSTACLE_VETO_RULE_HPP
