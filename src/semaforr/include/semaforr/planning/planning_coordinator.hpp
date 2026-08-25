/**
 * @file planning_coordinator.hpp
 * @brief Planning coordinator responsibilities.
 *
 * @details This file defines planning coordinator behavior for path planning and
 * hierarchical plan construction. It centers on `PlanSelectionPolicy`,
 * `SelectedPlan`, `CandidateEvidence`, `SelectionEvidence`,
 * `PlanningCoordinator`, `CacheEntry`. Its package-relative location is
 * `include/semaforr/planning/planning_coordinator.hpp`.
 */
#ifndef SEMAFORR_PLANNING_PLANNING_COORDINATOR_HPP
#define SEMAFORR_PLANNING_PLANNING_COORDINATOR_HPP

#include <memory>
#include <optional>
#include <random>
#include <semaforr/planning/planner.hpp>
#include <string>
#include <vector>

namespace semaforr::planning {

using PlanningEpisodeId = std::uint64_t;

/**
 * @brief Enumerates the supported plan selection policy values used by this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class PlanSelectionPolicy {
  Single,
  MinimumNormalizedCost,
  RangeVote,
  ParetoThenVote,
  ShortestValid
};
/**
 * @brief Constructs selection policy from string for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `PlanSelectionPolicy` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PlanSelectionPolicy planSelectionPolicyFromString(std::string_view value);
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(PlanSelectionPolicy value) noexcept;

/**
 * @brief Encapsulates selected plan state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct SelectedPlan {
  /**
   * @brief Encapsulates candidate evidence state and behavior for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - Not applicable to this declaration.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  struct CandidateEvidence {
    PlanId plan_id = 0U;
    std::string planner;
    PlanFamily family = PlanFamily::Grid;
    ObjectiveCosts raw_costs;
    ObjectiveCosts normalized_costs;
    double summed_score = 0.0;
    bool tied_for_best = false;
    PlannerMetadata metadata;
    std::vector<domain::Point2D> geometry;
    std::vector<PlanStep> typed_steps;
    domain::DependencyRevisions dependency_revisions;
    domain::Revision planner_configuration_revision = 0U;
    PlanningOperatingMode operating_mode{PlanningOperatingMode::Mapless};
    bool static_map_contributed{false};
  };
  /**
   * @brief Encapsulates selection evidence state and behavior for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - Not applicable to this declaration.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  struct SelectionEvidence {
    PlanningEpisodeId planning_episode_id = 0U;
    std::optional<domain::TaskId> task_id;
    domain::Pose2D start;
    domain::Point2D target;
    PlanSelectionPolicy policy{PlanSelectionPolicy::RangeVote};
    PlanId selected_plan_id = 0U;
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

/**
 * @brief Encapsulates planning coordinator state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class PlanningCoordinator {
 public:
  /**
   * @brief Constructs ning coordinator for this subsystem.
   *
   * Arguments:
   * - @p policy: Supplies policy input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit PlanningCoordinator(
      PlanSelectionPolicy policy = PlanSelectionPolicy::RangeVote)
      : policy_(policy) {}
  /**
   * @brief Registers planner for this subsystem.
   *
   * Arguments:
   * - @p planner: Supplies planner input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void registerPlanner(std::unique_ptr<Planner> planner);
  /**
   * @brief Performs the select plan operation for this subsystem.
   *
   * Arguments:
   * - @p request: Supplies request input to the operation.
   *
   * Returns:
   * - `std::optional<SelectedPlan>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<SelectedPlan> selectPlan(const PlanningRequest& request);
  /**
   * @brief Constructs ner count for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t plannerCount() const noexcept { return planners_.size(); }
  /**
   * @brief Performs the cache hits operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t cacheHits() const noexcept { return cache_hits_; }
  /**
   * @brief Clears cache for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void clearCache() noexcept { cache_.clear(); }
  /**
   * @brief Sets selection policy for this subsystem.
   *
   * Arguments:
   * - @p value: Supplies value input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void setSelectionPolicy(PlanSelectionPolicy value) noexcept;
  /**
   * @brief Sets tie policy for this subsystem.
   *
   * Arguments:
   * - @p seeded_exact_ties: Supplies seeded exact ties input to the
   * operation.
   * - @p random_seed: Supplies random seed input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void setTiePolicy(bool seeded_exact_ties, std::uint64_t random_seed) noexcept;
  /**
   * @brief Performs the configuration revision operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `domain::Revision` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Revision configurationRevision() const noexcept {
    return configuration_revision_;
  }

 private:
  /**
   * @brief Encapsulates cache entry state and behavior for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - Not applicable to this declaration.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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
  PlanningEpisodeId next_episode_id_ = 1U;
  bool seeded_exact_ties_{false};
  std::uint64_t random_seed_{0U};
  std::mt19937_64 random_{0U};
};
}  // namespace semaforr::planning
#endif
