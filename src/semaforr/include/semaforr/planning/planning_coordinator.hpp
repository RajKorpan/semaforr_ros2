#ifndef SEMAFORR_PLANNING_PLANNING_COORDINATOR_HPP
#define SEMAFORR_PLANNING_PLANNING_COORDINATOR_HPP

#include <memory>
#include <optional>
#include <semaforr/planning/planner.hpp>
#include <string>
#include <vector>

namespace semaforr::planning {

struct SelectedPlan {
  PlanResult result;
  std::string planner;
};

class PlanningCoordinator {
 public:
  void registerPlanner(std::unique_ptr<Planner> planner);
  std::optional<SelectedPlan> selectPlan(const PlanningRequest& request);
  std::size_t plannerCount() const noexcept { return planners_.size(); }

 private:
  std::vector<std::unique_ptr<Planner>> planners_;
};

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_PLANNING_COORDINATOR_HPP
