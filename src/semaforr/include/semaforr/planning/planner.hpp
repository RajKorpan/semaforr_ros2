#ifndef SEMAFORR_PLANNING_PLANNER_HPP
#define SEMAFORR_PLANNING_PLANNER_HPP

#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/world_model.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace semaforr::planning {

enum class PlanStepKind {
  ApproachNetwork,
  TraverseSkeleton,
  EnterHighway,
  FollowHighway,
  CrossIntersection,
  LeaveNetwork,
  ReachGoal
};

struct PlanStep {
  PlanStepKind kind = PlanStepKind::ReachGoal;
  domain::Point2D target;
  std::optional<std::size_t> network_node;
};

struct HierarchicalPlan {
  std::vector<PlanStep> steps;
  std::size_t spatial_revision = 0U;
  std::string strategy;

  bool empty() const noexcept { return steps.empty(); }
};

enum class PlanStatus { Success, NoPath, InvalidRequest, PlannerUnavailable };

struct PlanningRequest {
  domain::Pose2D start;
  domain::Point2D goal;
  const domain::SpatialModel* spatial_model{nullptr};
  const domain::CrowdModel* crowd_model{nullptr};
};

struct PlanResult {
  PlanStatus status{PlanStatus::PlannerUnavailable};
  std::vector<domain::Point2D> path;
  double cost_m{0.0};
  std::string explanation;
  std::optional<HierarchicalPlan> hierarchical;

  PlanResult() = default;
  PlanResult(PlanStatus plan_status, std::vector<domain::Point2D> plan_path,
             double plan_cost_m, std::string plan_explanation,
             std::optional<HierarchicalPlan> hierarchical_plan = std::nullopt)
      : status(plan_status),
        path(std::move(plan_path)),
        cost_m(plan_cost_m),
        explanation(std::move(plan_explanation)),
        hierarchical(std::move(hierarchical_plan)) {}

  bool succeeded() const noexcept { return status == PlanStatus::Success; }
};

class Planner {
 public:
  virtual ~Planner() = default;
  virtual PlanResult plan(const PlanningRequest& request) = 0;
  virtual std::string_view name() const noexcept = 0;
};

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_PLANNER_HPP
