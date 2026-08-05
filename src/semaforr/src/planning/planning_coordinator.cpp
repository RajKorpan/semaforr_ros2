#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/planning_coordinator.hpp>
#include <stdexcept>

namespace semaforr::planning {
namespace {
long long surrogate(double value, double resolution) {
  return std::llround(value / std::max(.05, resolution));
}
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

void PlanningCoordinator::registerPlanner(std::unique_ptr<Planner> planner) {
  if (!planner) throw std::invalid_argument("planner must not be null");
  const std::string name(planner->name());
  if (name.empty())
    throw std::invalid_argument("planner name must not be empty");
  if (std::any_of(planners_.begin(), planners_.end(),
                  [&](const auto& p) { return p->name() == name; }))
    throw std::invalid_argument("planner '" + name + "' is already registered");
  planners_.push_back(std::move(planner));
}

std::optional<SelectedPlan> PlanningCoordinator::selectPlan(
    const PlanningRequest& request) {
  const double resolution = request.spatial_model
                                ? request.spatial_model->known_grid.resolution_m
                                : .25;
  const long long sx = surrogate(request.start.position.x_m, resolution),
                  sy = surrogate(request.start.position.y_m, resolution),
                  gx = surrogate(request.goal.x_m, resolution),
                  gy = surrogate(request.goal.y_m, resolution);
  const std::size_t spatial =
      request.spatial_model ? request.spatial_model->revision : 0U;
  const std::size_t static_map =
      request.static_map ? request.static_map->revision : 0U;
  const std::uint64_t crowd =
      request.crowd_model ? request.crowd_model->learned().version : 0U;
  struct Candidate {
    PlanResult result;
    std::string planner;
    double vote = 0.0;
  };
  std::vector<Candidate> candidates;
  std::vector<PlanObjective> objectives;
  for (const auto& planner : planners_) {
    PlanResult result;
    const std::string name(planner->name());
    const auto objective = planner->objective();
    const bool social = objective == PlanObjective::CrowdDensity ||
                        objective == PlanObjective::EncounterRisk ||
                        objective == PlanObjective::FlowOpposition;
    const std::uint64_t relevant_crowd = social ? crowd : 0U;
    const std::size_t relevant_spatial =
        objective == PlanObjective::HighwayDistance && request.spatial_model
            ? spatial ^ (request.spatial_model->highways.revision + 0x9e3779b9U)
            : spatial ^ (static_map + 0x85ebca6bU);
    auto found =
        std::find_if(cache_.begin(), cache_.end(), [&](const CacheEntry& e) {
          return e.planner == name && e.start_x == sx && e.start_y == sy &&
                 e.goal_x == gx && e.goal_y == gy &&
                 e.spatial_revision == relevant_spatial &&
                 e.crowd_revision == relevant_crowd;
        });
    if (found != cache_.end()) {
      result = found->result;
      ++cache_hits_;
    } else {
      result = planner->plan(request);
      cache_.erase(
          std::remove_if(cache_.begin(), cache_.end(),
                         [&](const CacheEntry& e) {
                           return e.planner == name &&
                                  (e.spatial_revision != relevant_spatial ||
                                   e.crowd_revision != relevant_crowd);
                         }),
          cache_.end());
      cache_.push_back(
          {name, sx, sy, gx, gy, relevant_spatial, relevant_crowd, result});
    }
    if (!result.succeeded()) continue;
    if (!std::isfinite(result.cost_m) || result.cost_m < 0.0)
      throw std::domain_error("planner '" + name +
                              "' returned an invalid path cost");
    result.primary_objective = planner->objective();
    result.objective_costs = evaluatePathObjectives(request, result.path);
    objectives.push_back(planner->objective());
    candidates.push_back({std::move(result), name, 0.0});
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
  if (policy_ == PlanSelectionPolicy::Single) {
    const auto& c = candidates.front();
    return SelectedPlan{c.result, c.planner, policy_, 0.0};
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
    return SelectedPlan{
        best->result, best->planner, policy_,
        best->result.objective_costs.at(PlanObjective::Distance)};
  }
  for (auto objective : objectives) {
    double low = std::numeric_limits<double>::infinity(),
           high = -std::numeric_limits<double>::infinity();
    for (const auto& c : candidates) {
      const double value = c.result.objective_costs.at(objective);
      low = std::min(low, value);
      high = std::max(high, value);
    }
    for (auto& c : candidates)
      c.vote += (high - low) <= 1e-12
                    ? 0.0
                    : 10.0 * (c.result.objective_costs.at(objective) - low) /
                          (high - low);
  }
  if (policy_ == PlanSelectionPolicy::MinimumNormalizedCost)
    for (auto& c : candidates)
      c.vote /=
          static_cast<double>(std::max<std::size_t>(1, objectives.size()));
  const auto best = std::min_element(
      candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
        return a.vote != b.vote ? a.vote < b.vote : a.planner < b.planner;
      });
  return SelectedPlan{best->result, best->planner, policy_, best->vote};
}
}  // namespace semaforr::planning
