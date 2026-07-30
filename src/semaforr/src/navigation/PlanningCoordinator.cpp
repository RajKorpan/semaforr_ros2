#include <algorithm>
#include <cmath>
#include <semaforr/planning/planning_coordinator.hpp>
#include <stdexcept>

namespace semaforr::planning {

void PlanningCoordinator::registerPlanner(std::unique_ptr<Planner> planner) {
  if (!planner) {
    throw std::invalid_argument("planner must not be null");
  }
  const std::string name(planner->name());
  if (name.empty()) {
    throw std::invalid_argument("planner name must not be empty");
  }
  if (std::any_of(
          planners_.begin(), planners_.end(),
          [&name](const auto& current) { return current->name() == name; })) {
    throw std::invalid_argument("planner '" + name + "' is already registered");
  }
  planners_.push_back(std::move(planner));
}

std::optional<SelectedPlan> PlanningCoordinator::selectPlan(
    const PlanningRequest& request) {
  const std::size_t revision =
      request.spatial_model ? request.spatial_model->revision : 0U;
  if (cached_ && cached_->start.position == request.start.position &&
      cached_->start.heading == request.start.heading &&
      cached_->goal == request.goal &&
      cached_->spatial_revision == revision) {
    ++cache_hits_;
    return cached_->selected;
  }
  std::optional<SelectedPlan> selected;
  for (const auto& planner : planners_) {
    PlanResult candidate = planner->plan(request);
    if (!candidate.succeeded()) {
      continue;
    }
    if (!std::isfinite(candidate.cost_m) || candidate.cost_m < 0.0) {
      throw std::domain_error("planner '" + std::string(planner->name()) +
                              "' returned an invalid path cost");
    }
    const std::string name(planner->name());
    if (!selected || candidate.cost_m < selected->result.cost_m ||
        (candidate.cost_m == selected->result.cost_m &&
         name < selected->planner)) {
      selected = SelectedPlan{std::move(candidate), name};
    }
  }
  if (selected)
    cached_ = CacheEntry{request.start, request.goal, revision, *selected};
  return selected;
}

}  // namespace semaforr::planning
