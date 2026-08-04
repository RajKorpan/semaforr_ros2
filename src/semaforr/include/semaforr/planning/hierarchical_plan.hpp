#ifndef SEMAFORR_PLANNING_HIERARCHICAL_PLAN_HPP
#define SEMAFORR_PLANNING_HIERARCHICAL_PLAN_HPP

#include <semaforr/planning/planner.hpp>
#include <string_view>

namespace semaforr::planning {

class SkeletonPlan final : public Planner {
 public:
  PlanResult plan(const PlanningRequest& request) override;
  std::string_view name() const noexcept override { return "skeleton_plan"; }
  PlanObjective objective() const noexcept override {
    return PlanObjective::SkeletonDistance;
  }
};

class HighwayPlan final : public Planner {
 public:
  PlanResult plan(const PlanningRequest& request) override;
  std::string_view name() const noexcept override { return "highway_plan"; }
  PlanObjective objective() const noexcept override {
    return PlanObjective::HighwayDistance;
  }
};

}  // namespace semaforr::planning

#endif
