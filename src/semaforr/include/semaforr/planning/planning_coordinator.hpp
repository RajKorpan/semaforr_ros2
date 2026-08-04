#ifndef SEMAFORR_PLANNING_PLANNING_COORDINATOR_HPP
#define SEMAFORR_PLANNING_PLANNING_COORDINATOR_HPP

#include <memory>
#include <optional>
#include <semaforr/planning/planner.hpp>
#include <string>
#include <vector>

namespace semaforr::planning {

enum class PlanSelectionPolicy {
  Single,
  MinimumNormalizedCost,
  RangeVote,
  ParetoThenVote,
  ShortestValid
};
PlanSelectionPolicy planSelectionPolicyFromString(std::string_view value);
std::string_view toString(PlanSelectionPolicy value) noexcept;

struct SelectedPlan {
  PlanResult result;
  std::string planner;
  PlanSelectionPolicy policy = PlanSelectionPolicy::RangeVote;
  double normalized_vote = 0.0;
};

class PlanningCoordinator {
 public:
  explicit PlanningCoordinator(
      PlanSelectionPolicy policy = PlanSelectionPolicy::RangeVote)
      : policy_(policy) {}
  void registerPlanner(std::unique_ptr<Planner> planner);
  std::optional<SelectedPlan> selectPlan(const PlanningRequest& request);
  std::size_t plannerCount() const noexcept { return planners_.size(); }
  std::size_t cacheHits() const noexcept { return cache_hits_; }
  void clearCache() noexcept { cache_.clear(); }
  void setSelectionPolicy(PlanSelectionPolicy value) noexcept {
    policy_ = value;
  }

 private:
  struct CacheEntry {
    std::string planner;
    long long start_x = 0, start_y = 0, goal_x = 0, goal_y = 0;
    std::size_t spatial_revision = 0U;
    std::uint64_t crowd_revision = 0U;
    PlanResult result;
  };
  std::vector<std::unique_ptr<Planner>> planners_;
  std::vector<CacheEntry> cache_;
  PlanSelectionPolicy policy_;
  std::size_t cache_hits_ = 0U;
};
}  // namespace semaforr::planning
#endif
