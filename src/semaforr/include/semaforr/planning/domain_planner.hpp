#ifndef SEMAFORR_PLANNING_DOMAIN_PLANNER_HPP
#define SEMAFORR_PLANNING_DOMAIN_PLANNER_HPP

#include <semaforr/planning/planner.hpp>
#include <string>

namespace semaforr::planning {

using PlannerObjective = PlanObjective;

ObjectiveCosts evaluatePathObjectives(const PlanningRequest& request,
                                      const std::vector<domain::Point2D>& path);

class DomainPlanner final : public Planner {
 public:
  DomainPlanner(std::string name, PlannerObjective objective);
  PlanResult plan(const PlanningRequest& request) override;
  std::string_view name() const noexcept override { return name_; }
  PlanObjective objective() const noexcept override { return objective_; }

 private:
  std::string name_;
  PlannerObjective objective_;
};

}  // namespace semaforr::planning
#endif
