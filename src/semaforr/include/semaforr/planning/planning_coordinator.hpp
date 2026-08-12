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
  struct CandidateEvidence {
    PlanId plan_id = 0U;
    std::string planner;
    PlanFamily family = PlanFamily::Grid;
    ObjectiveCosts raw_costs;
    ObjectiveCosts normalized_costs;
    double summed_score = 0.0;
    bool tied_for_best = false;
  };
  struct SelectionEvidence {
    std::vector<CandidateEvidence> candidates;
    std::vector<std::string> tie_candidates;
    std::string tie_break_reason;
    std::uint64_t random_seed = 0U;
  };
  PlanResult result;
  std::string planner;
  PlanSelectionPolicy policy = PlanSelectionPolicy::RangeVote;
  double normalized_vote = 0.0;
  SelectionEvidence evidence;
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
  void setSelectionPolicy(PlanSelectionPolicy value) noexcept;
  domain::Revision configurationRevision() const noexcept {
    return configuration_revision_;
  }

 private:
  struct CacheEntry {
    std::string planner;
    long long start_x = 0, start_y = 0, goal_x = 0, goal_y = 0;
    std::optional<domain::TaskId> task_id;
    domain::DependencyRevisions dependency_revisions;
    PlanResult result;
  };
  std::vector<std::unique_ptr<Planner>> planners_;
  std::vector<CacheEntry> cache_;
  PlanSelectionPolicy policy_;
  std::size_t cache_hits_ = 0U;
  domain::Revision configuration_revision_ = 1U;
  PlanId next_plan_id_ = 1U;
};
}  // namespace semaforr::planning
#endif
