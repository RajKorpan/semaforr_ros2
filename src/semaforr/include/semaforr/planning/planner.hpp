#ifndef SEMAFORR_PLANNING_PLANNER_HPP
#define SEMAFORR_PLANNING_PLANNER_HPP

#include <map>
#include <optional>
#include <semaforr/domain/highway.hpp>
#include <semaforr/domain/world_model.hpp>
#include <semaforr/planning/traversability.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace semaforr::planning {

enum class PlanObjective {
  Distance,
  CrowdDensity,
  EncounterRisk,
  FlowOpposition,
  RegionPreference,
  HallwayPreference,
  TrailPreference,
  ConveyorPreference,
  SkeletonDistance,
  HighwayDistance
};

using ObjectiveCosts = std::map<PlanObjective, double>;

struct WaypointStep {
  domain::Point2D target;
};
struct SubtrailStep {
  std::vector<domain::Point2D> waypoints;
  std::optional<domain::TrailId> trail_id;
  std::size_t cursor = 0U;
};
struct RegionStep {
  std::size_t region_id = 0U;
  domain::Point2D center;
};
struct HighwayStep {
  domain::HighwayId highway_id = 0U;
  domain::IntersectionId from = 0U;
  domain::IntersectionId to = 0U;
  std::vector<domain::Point2D> fallback_subtrail;
};
struct IntersectionStep {
  domain::IntersectionId intersection_id = 0U;
  domain::Point2D centroid;
};

using PlanStep = std::variant<WaypointStep, SubtrailStep, RegionStep,
                              HighwayStep, IntersectionStep>;

enum class PlanValidity { Valid, Stale, Invalid, Complete };

struct HierarchicalPlan {
  std::string planner;
  PlanObjective objective = PlanObjective::Distance;
  std::vector<PlanStep> steps;
  ObjectiveCosts estimated_objective_costs;
  std::map<std::string, std::size_t> source_model_revisions;
  std::size_t cursor = 0U;
  std::string provenance;
  PlanValidity validity = PlanValidity::Valid;
  std::vector<std::string> diagnostics;

  bool empty() const noexcept { return steps.empty(); }
  bool exhausted() const noexcept { return cursor >= steps.size(); }
};

std::optional<domain::Point2D> stepTarget(const PlanStep& step) noexcept;
std::string_view toString(PlanObjective objective) noexcept;

enum class PlanStatus { Success, NoPath, InvalidRequest, PlannerUnavailable };

struct PlanningRequest {
  domain::Pose2D start;
  domain::Point2D goal;
  const domain::SpatialModel* spatial_model{nullptr};
  const domain::CrowdModel* crowd_model{nullptr};
  const domain::StaticMap* static_map{nullptr};
  TraversabilityConfiguration traversability;

  PlanningRequest() = default;
  PlanningRequest(domain::Pose2D request_start, domain::Point2D request_goal,
                  const domain::SpatialModel* request_spatial = nullptr,
                  const domain::CrowdModel* request_crowd = nullptr,
                  const domain::StaticMap* request_map = nullptr,
                  TraversabilityConfiguration traversal_configuration = {})
      : start(request_start),
        goal(request_goal),
        spatial_model(request_spatial),
        crowd_model(request_crowd),
        static_map(request_map),
        traversability(std::move(traversal_configuration)) {}
};

struct PlanResult {
  PlanStatus status{PlanStatus::PlannerUnavailable};
  std::vector<domain::Point2D> path;
  double cost_m{0.0};
  std::string explanation;
  std::optional<HierarchicalPlan> hierarchical;
  PlanObjective primary_objective = PlanObjective::Distance;
  ObjectiveCosts objective_costs;

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
  virtual PlanObjective objective() const noexcept {
    return PlanObjective::Distance;
  }
};

}  // namespace semaforr::planning
#endif
