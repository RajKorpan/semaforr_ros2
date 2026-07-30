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
  std::size_t cacheHits() const noexcept { return cache_hits_; }
  void clearCache() noexcept { cached_.reset(); }

 private:
  struct CacheEntry {
    domain::Pose2D start;
    domain::Point2D goal;
    std::size_t spatial_revision = 0U;
    SelectedPlan selected;
  };

  std::vector<std::unique_ptr<Planner>> planners_;
  std::optional<CacheEntry> cached_;
  std::size_t cache_hits_ = 0U;
};

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_PLANNING_COORDINATOR_HPP
