/**
 * @file planning_coordinator.cpp
 * @brief Planning coordinator responsibilities.
 *
 * @details This file implements planning coordinator behavior for path planning and
 * hierarchical plan construction. It centers on `Candidate`. Its
 * package-relative location is `src/planning/planning_coordinator.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/planning_coordinator.hpp>
#include <stdexcept>

namespace semaforr::planning {
namespace {
/**
 * @brief Performs the surrogate operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 * - @p resolution: Supplies resolution input to the operation.
 *
 * Returns:
 * - `long long` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
long long surrogate(double value, double resolution) {
  return std::llround(value / std::max(.05, resolution));
}
/**
 * @brief Performs the dominates operation for this subsystem.
 *
 * Arguments:
 * - @p a: Supplies a input to the operation.
 * - @p b: Supplies b input to the operation.
 * - @p objectives: Supplies objectives input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool dominates(const PlanResult& a, const PlanResult& b,
               const std::vector<PlanObjective>& objectives) {
  bool strict = false;
  for (auto objective : objectives) {
    const double av = a.objective_costs.at(objective),
                 bv = b.objective_costs.at(objective);
    if (av > bv) return false;
    if (av < bv) strict = true;
  }
  return strict;
}
}  // namespace

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
PlanSelectionPolicy planSelectionPolicyFromString(std::string_view value) {
  if (value == "single") return PlanSelectionPolicy::Single;
  if (value == "minimum_normalized_cost")
    return PlanSelectionPolicy::MinimumNormalizedCost;
  if (value == "range_vote") return PlanSelectionPolicy::RangeVote;
  if (value == "pareto_then_vote") return PlanSelectionPolicy::ParetoThenVote;
  if (value == "shortest_valid") return PlanSelectionPolicy::ShortestValid;
  throw std::invalid_argument("unknown Tier-2 selection policy '" +
                              std::string(value) + "'");
}
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
std::string_view toString(PlanSelectionPolicy value) noexcept {
  switch (value) {
    case PlanSelectionPolicy::Single:
      return "single";
    case PlanSelectionPolicy::MinimumNormalizedCost:
      return "minimum_normalized_cost";
    case PlanSelectionPolicy::RangeVote:
      return "range_vote";
    case PlanSelectionPolicy::ParetoThenVote:
      return "pareto_then_vote";
    case PlanSelectionPolicy::ShortestValid:
      return "shortest_valid";
  }
  return "range_vote";
}

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
void PlanningCoordinator::registerPlanner(std::unique_ptr<Planner> planner) {
  if (!planner) throw std::invalid_argument("planner must not be null");
  const std::string name(planner->name());
  if (name.empty())
    throw std::invalid_argument("planner name must not be empty");
  if (std::any_of(planners_.begin(), planners_.end(),
                  [&](const auto& p) { return p->name() == name; }))
    throw std::invalid_argument("planner '" + name + "' is already registered");
  if (!planner->metadata().valid())
    throw std::invalid_argument("planner '" + name +
                                "' has incomplete explanation metadata");
  planners_.push_back(std::move(planner));
  ++configuration_revision_;
}

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
void PlanningCoordinator::setSelectionPolicy(PlanSelectionPolicy value) noexcept {
  if (policy_ == value) return;
  policy_ = value;
  ++configuration_revision_;
}

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
void PlanningCoordinator::setTiePolicy(bool seeded_exact_ties,
                                       std::uint64_t random_seed) noexcept {
  if (seeded_exact_ties_ == seeded_exact_ties && random_seed_ == random_seed)
    return;
  seeded_exact_ties_ = seeded_exact_ties;
  random_seed_ = random_seed;
  random_.seed(random_seed_);
  ++configuration_revision_;
}

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
std::optional<SelectedPlan> PlanningCoordinator::selectPlan(
    const PlanningRequest& request) {
  PlanningRequest effective = request;
  effective.planner_configuration_revision = configuration_revision_;
  const double resolution =
      request.static_map && request.static_map->occupancyAvailable()
          ? request.static_map->occupancy.geometry.resolution_m
          : request.spatial_model &&
                    request.spatial_model->sensed_occupancy.valid()
                ? request.spatial_model->sensed_occupancy.geometry.resolution_m
                : .25;
  const long long sx = surrogate(request.start.position.x_m, resolution),
                  sy = surrogate(request.start.position.y_m, resolution),
                  gx = surrogate(request.goal.x_m, resolution),
                  gy = surrogate(request.goal.y_m, resolution);
  struct Candidate {
    PlanResult result;
    std::string planner;
    PlannerMetadata metadata;
    double vote = 0.0;
    ObjectiveCosts normalized;
  };
  std::vector<Candidate> candidates;
  std::vector<PlanObjective> objectives;
  for (const auto& planner : planners_) {
    PlanResult result;
    const std::string name(planner->name());
    auto declared = planner->dependencies(effective);
    declared.push_back(domain::ModelDependency::PlannerConfiguration);
    std::sort(declared.begin(), declared.end(), [](auto a, auto b) {
      return static_cast<int>(a) < static_cast<int>(b);
    });
    declared.erase(std::unique(declared.begin(), declared.end()),
                   declared.end());
    domain::DependencyRevisions dependencies;
    for (const auto dependency : declared)
      dependencies[dependency] = currentRevision(effective, dependency);
    auto found =
        std::find_if(cache_.begin(), cache_.end(), [&](const CacheEntry& e) {
          if (e.planner != name || e.start_x != sx || e.start_y != sy ||
              e.goal_x != gx || e.goal_y != gy ||
              e.task_id != effective.task_id ||
              e.dependency_revisions != dependencies)
            return false;
          const domain::Distance tolerance(
              std::max(0.05, resolution * 0.5));
          return stalePlanReasons(e.result, effective, tolerance, tolerance)
              .empty();
        });
    if (found != cache_.end()) {
      result = found->result;
      ++cache_hits_;
    } else {
      result = planner->plan(effective);
      result.family = planner->planFamily();
      if (result.succeeded() && result.plan_id == 0U)
        result.plan_id = next_plan_id_++;
      attachDependencySnapshot(result, effective, declared);
      cache_.erase(
          std::remove_if(cache_.begin(), cache_.end(),
                         [&](const CacheEntry& e) {
                           return e.planner == name && e.start_x == sx &&
                                  e.start_y == sy && e.goal_x == gx &&
                                  e.goal_y == gy &&
                                  e.task_id == effective.task_id;
                         }),
          cache_.end());
      cache_.push_back({name, sx, sy, gx, gy, effective.task_id, dependencies,
                        result});
    }
    if (!result.succeeded()) continue;
    if (!std::isfinite(result.cost_m) || result.cost_m < 0.0)
      throw std::domain_error("planner '" + name +
                              "' returned an invalid path cost");
    result.primary_objective = planner->objective();
    result.objective_costs = evaluatePathObjectives(request, result.path);
    objectives.push_back(planner->objective());
    auto metadata = planner->metadata();
    metadata.representation_dependencies.clear();
    for (const auto dependency : declared)
      if (dependency != domain::ModelDependency::PlannerConfiguration)
        metadata.representation_dependencies.push_back(
            std::string(domain::toString(dependency)));
    candidates.push_back(
        {std::move(result), name, std::move(metadata), 0.0, {}});
  }
  if (candidates.empty()) return std::nullopt;
  std::sort(objectives.begin(), objectives.end(), [](auto a, auto b) {
    return static_cast<int>(a) < static_cast<int>(b);
  });
  objectives.erase(std::unique(objectives.begin(), objectives.end()),
                   objectives.end());
  if (policy_ == PlanSelectionPolicy::ParetoThenVote) {
    std::vector<Candidate> frontier;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      bool dominated = false;
      for (std::size_t j = 0; j < candidates.size(); ++j)
        if (i != j &&
            dominates(candidates[j].result, candidates[i].result, objectives)) {
          dominated = true;
          break;
        }
      if (!dominated) frontier.push_back(candidates[i]);
    }
    candidates = std::move(frontier);
  }
  const auto selected = [&](std::size_t winner) {
    SelectedPlan::SelectionEvidence evidence;
    evidence.planning_episode_id = next_episode_id_++;
    evidence.task_id = effective.task_id;
    evidence.start = effective.start;
    evidence.target = effective.goal;
    evidence.policy = policy_;
    evidence.selected_plan_id = candidates[winner].result.plan_id;
    const double best_score = candidates[winner].vote;
    for (const auto& candidate : candidates) {
      const bool tied = std::abs(candidate.vote - best_score) <= 1e-12;
      evidence.candidates.push_back(
          {candidate.result.plan_id, candidate.planner,
           candidate.result.family, candidate.result.objective_costs,
           candidate.normalized, candidate.vote, tied, candidate.metadata,
           candidate.result.path,
           candidate.result.hierarchical
               ? candidate.result.hierarchical->steps
               : std::vector<PlanStep>{},
           candidate.result.dependency_revisions,
           candidate.result.planner_configuration_revision,
           candidate.result.operating_mode,
           candidate.result.static_map_contributed});
      if (tied) evidence.tie_candidates.push_back(candidate.planner);
    }
    evidence.tie_break_reason =
        evidence.tie_candidates.size() > 1U
            ? (seeded_exact_ties_ ? "seeded_exact_planner_tie"
                                  : "lexicographically_smallest_planner_name")
            : "unique_lowest_combined_score";
    evidence.random_seed = random_seed_;
    return SelectedPlan{candidates[winner].result, candidates[winner].planner,
                        policy_, candidates[winner].vote,
                        std::move(evidence)};
  };
  if (policy_ == PlanSelectionPolicy::Single) {
    return selected(0U);
  }
  if (policy_ == PlanSelectionPolicy::ShortestValid) {
    auto best = std::min_element(
        candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
          const double ac =
                           a.result.objective_costs.at(PlanObjective::Distance),
                       bc =
                           b.result.objective_costs.at(PlanObjective::Distance);
          return ac != bc ? ac < bc : a.planner < b.planner;
        });
    const double best_distance =
        best->result.objective_costs.at(PlanObjective::Distance);
    std::vector<std::size_t> tied;
    for (std::size_t index = 0U; index < candidates.size(); ++index) {
      candidates[index].vote = candidates[index].result.objective_costs.at(
          PlanObjective::Distance);
      if (candidates[index].vote == best_distance) tied.push_back(index);
    }
    const std::size_t winner = seeded_exact_ties_ && tied.size() > 1U
                                   ? tied[std::uniform_int_distribution<std::size_t>(
                                              0U, tied.size() - 1U)(random_)]
                                   : static_cast<std::size_t>(best - candidates.begin());
    return selected(winner);
  }
  for (auto objective : objectives) {
    double low = std::numeric_limits<double>::infinity(),
           high = -std::numeric_limits<double>::infinity();
    for (const auto& c : candidates) {
      const double value = c.result.objective_costs.at(objective);
      low = std::min(low, value);
      high = std::max(high, value);
    }
    for (auto& c : candidates) {
      const double normalized =
          (high - low) <= 1e-12
              ? 0.0
              : 10.0 * (c.result.objective_costs.at(objective) - low) /
                    (high - low);
      c.normalized[objective] = normalized;
      c.vote += normalized;
    }
  }
  if (policy_ == PlanSelectionPolicy::MinimumNormalizedCost)
    for (auto& c : candidates)
      c.vote /=
          static_cast<double>(std::max<std::size_t>(1, objectives.size()));
  const auto best = std::min_element(
      candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
        return a.vote != b.vote ? a.vote < b.vote : a.planner < b.planner;
      });
  std::vector<std::size_t> tied;
  for (std::size_t index = 0U; index < candidates.size(); ++index)
    if (candidates[index].vote == best->vote) tied.push_back(index);
  const std::size_t winner = seeded_exact_ties_ && tied.size() > 1U
                                 ? tied[std::uniform_int_distribution<std::size_t>(
                                            0U, tied.size() - 1U)(random_)]
                                 : static_cast<std::size_t>(best - candidates.begin());
  return selected(winner);
}
}  // namespace semaforr::planning
