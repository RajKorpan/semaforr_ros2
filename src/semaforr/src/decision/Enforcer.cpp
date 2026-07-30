#include <semaforr/decision/enforcer.hpp>

namespace semaforr::decision {

std::vector<domain::Point2D> Enforcer::operationalize(
    const planning::HierarchicalPlan& plan) const {
  std::vector<domain::Point2D> waypoints;
  waypoints.reserve(plan.steps.size());
  for (const auto& step : plan.steps) {
    if (waypoints.empty() ||
        domain::distance(waypoints.back(), step.target).meters() >
            domain::geometry_tolerance_m)
      waypoints.push_back(step.target);
  }
  return waypoints;
}

std::size_t Enforcer::activeStep(const planning::HierarchicalPlan& plan,
                                 const domain::Pose2D& pose,
                                 domain::Distance tolerance) const noexcept {
  std::size_t step = 0U;
  while (step < plan.steps.size() &&
         domain::distance(pose.position, plan.steps[step].target).meters() <=
             tolerance.meters() + domain::geometry_tolerance_m)
    ++step;
  return step;
}

}  // namespace semaforr::decision
