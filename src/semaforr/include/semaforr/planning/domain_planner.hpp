#ifndef SEMAFORR_PLANNING_DOMAIN_PLANNER_HPP
#define SEMAFORR_PLANNING_DOMAIN_PLANNER_HPP

#include <semaforr/planning/astar.hpp>
#include <semaforr/planning/planner.hpp>
#include <string>
#include <string_view>

namespace semaforr::planning {

enum class PlannerObjective {
  Distance,
  SkeletonDistance,
  CrowdDensity,
  EncounterRisk,
  FlowAlignment
};

class DomainPlanner final : public Planner {
 public:
  DomainPlanner(std::string name, PlannerObjective objective);

  PlanResult plan(const PlanningRequest& request) override;
  std::string_view name() const noexcept override { return name_; }

 private:
  PlanResult direct(const PlanningRequest& request) const;
  double socialPenalty(const PlanningRequest& request, domain::Point2D from,
                       domain::Point2D to) const;

  std::string name_;
  PlannerObjective objective_;
  AStar astar_;
};

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_DOMAIN_PLANNER_HPP
