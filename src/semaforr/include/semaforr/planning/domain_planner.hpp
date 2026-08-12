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
  DomainPlanner(std::string name, PlannerObjective objective,
                OccupancySourceMode source_mode);
  PlanResult plan(const PlanningRequest& request) override;
  std::string_view name() const noexcept override { return name_; }
  PlanObjective objective() const noexcept override { return objective_; }
  PlanFamily planFamily() const noexcept override { return PlanFamily::Grid; }
  std::vector<domain::ModelDependency> dependencies(
      const PlanningRequest& request) const override;

 private:
  std::string name_;
  PlannerObjective objective_;
  OccupancySourceMode source_mode_ = OccupancySourceMode::StaticMapWithSensors;
};

}  // namespace semaforr::planning
#endif
