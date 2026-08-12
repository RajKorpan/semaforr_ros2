#ifndef SEMAFORR_PLANNING_HIERARCHICAL_PLAN_HPP
#define SEMAFORR_PLANNING_HIERARCHICAL_PLAN_HPP

#include <semaforr/planning/planner.hpp>
#include <string_view>

namespace semaforr::planning {

class SkeletonPlan final : public Planner {
 public:
  PlanResult plan(const PlanningRequest& request) override;
  std::string_view name() const noexcept override { return "skeleton_plan"; }
  PlanFamily planFamily() const noexcept override { return PlanFamily::Model; }
  PlanObjective objective() const noexcept override {
    return PlanObjective::SkeletonDistance;
  }
  std::vector<domain::ModelDependency> dependencies(
      const PlanningRequest&) const override;
};

class HighwayPlan final : public Planner {
 public:
  PlanResult plan(const PlanningRequest& request) override;
  std::string_view name() const noexcept override { return "highway_plan"; }
  PlanFamily planFamily() const noexcept override { return PlanFamily::Model; }
  PlanObjective objective() const noexcept override {
    return PlanObjective::HighwayDistance;
  }
  std::vector<domain::ModelDependency> dependencies(
      const PlanningRequest&) const override;
};

}  // namespace semaforr::planning

#endif
