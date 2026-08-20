#include <semaforr/planning/planner.hpp>
#include <sstream>

namespace semaforr::planning {

std::string_view toString(PlanFamily family) noexcept {
  return family == PlanFamily::Grid ? "grid" : "model";
}

std::string_view toString(PlanValidity validity) noexcept {
  switch (validity) {
    case PlanValidity::Valid: return "valid";
    case PlanValidity::Stale: return "stale";
    case PlanValidity::Invalid: return "invalid";
    case PlanValidity::Complete: return "complete";
  }
  return "invalid";
}

std::string_view toString(PlanningOperatingMode mode) noexcept {
  return mode == PlanningOperatingMode::MapEnabled ? "map_enabled"
                                                   : "mapless";
}

std::optional<domain::Point2D> stepTarget(const PlanStep& step) noexcept {
  return std::visit(
      [](const auto& value) -> std::optional<domain::Point2D> {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, WaypointStep>) {
          return value.target;
        } else if constexpr (std::is_same_v<T, RegionStep>) {
          return value.center;
        } else if constexpr (std::is_same_v<T, VisibilityConnectionStep>) {
          return value.to;
        } else if constexpr (std::is_same_v<T, IntersectionStep>) {
          return value.centroid;
        } else if constexpr (std::is_same_v<T, HighwayEntryStep>) {
          return value.entry;
        } else if constexpr (std::is_same_v<T, HighwayExitStep>) {
          return value.exit;
        } else if constexpr (std::is_same_v<T, FinalTargetStep>) {
          return value.target;
        } else if constexpr (std::is_same_v<T, SkeletonTransitionStep>) {
          if (value.supporting_subtrail.empty()) return std::nullopt;
          // The transition is complete at its destination. Returning the
          // first marker would let active-step advancement skip an entire
          // edge whenever that marker is the current region center.
          return value.supporting_subtrail.back();
        } else if constexpr (std::is_same_v<T, SubtrailStep>) {
          if (value.waypoints.empty()) return std::nullopt;
          return value
              .waypoints[std::min(value.cursor, value.waypoints.size() - 1U)];
        } else {
          return std::nullopt;
        }
      },
      step);
}

std::string_view toString(PlanObjective objective) noexcept {
  switch (objective) {
    case PlanObjective::Distance:
      return "distance";
    case PlanObjective::CrowdDensity:
      return "crowd_density";
    case PlanObjective::EncounterRisk:
      return "encounter_risk";
    case PlanObjective::FlowOpposition:
      return "flow_opposition";
    case PlanObjective::RegionPreference:
      return "region";
    case PlanObjective::HallwayPreference:
      return "hallway";
    case PlanObjective::TrailPreference:
      return "trail";
    case PlanObjective::ConveyorPreference:
      return "conveyor";
    case PlanObjective::SkeletonDistance:
      return "skeleton_distance";
    case PlanObjective::HighwayDistance:
      return "highway_distance";
  }
  return "distance";
}

std::string_view objectiveDescription(PlanObjective objective) noexcept {
  switch (objective) {
    case PlanObjective::Distance:
      return "minimize metric or graph path cost";
    case PlanObjective::CrowdDensity:
      return "prefer routes through areas with lower estimated crowd density";
    case PlanObjective::EncounterRisk:
      return "prefer routes with lower estimated crowd-related navigation risk";
    case PlanObjective::FlowOpposition:
      return "prefer routes aligned with favorable crowd movement patterns";
    case PlanObjective::RegionPreference:
      return "prefer learned regions and their entrance and doorway affordances";
    case PlanObjective::HallwayPreference:
      return "prefer travel through learned hallway structures";
    case PlanObjective::TrailPreference:
      return "prefer previously learned successful trails";
    case PlanObjective::ConveyorPreference:
      return "prefer areas associated with repeated successful traversal";
    case PlanObjective::SkeletonDistance:
      return "navigate the learned region-connectivity skeleton and supporting subtrails";
    case PlanObjective::HighwayDistance:
      return "prefer the learned highway network and its intersections";
  }
  return "unknown planning objective";
}

PlannerMetadata Planner::metadata() const {
  return {std::string(name()), planFamily(), objective(),
          std::string(toString(objective())),
          std::string(objectiveDescription(objective())), {}, false,
          planFamily() == PlanFamily::Model};
}

domain::Revision currentRevision(const PlanningRequest& request,
                                 domain::ModelDependency dependency) noexcept {
  using D = domain::ModelDependency;
  switch (dependency) {
    case D::StaticMapGeometry:
      return request.static_map ? request.static_map->geometry_revision : 0U;
    case D::StaticOccupancy:
      return request.static_map ? request.static_map->occupancy_revision : 0U;
    case D::CrowdDensity:
    case D::CrowdRisk:
    case D::CrowdFlow:
      return request.crowd_model ? request.crowd_model->revisionOf(dependency)
                                 : 0U;
    case D::PlannerConfiguration:
      return request.planner_configuration_revision;
    default:
      return request.spatial_model
                 ? request.spatial_model->revisionOf(dependency)
                 : 0U;
  }
}

std::vector<std::string> stalePlanReasons(
    const PlanResult& plan, const PlanningRequest& request,
    domain::Distance start_tolerance, domain::Distance target_tolerance,
    bool execution_invalidated) {
  auto reasons = dependencyChangeReasons(plan.dependency_revisions, request);
  if (plan.task_id != request.task_id) reasons.push_back("task_changed");
  if (domain::distance(plan.planned_start.position, request.start.position)
          .meters() > start_tolerance.meters())
    reasons.push_back("start_moved_beyond_tolerance");
  if (domain::distance(plan.planned_goal, request.goal).meters() >
      target_tolerance.meters())
    reasons.push_back("target_moved_beyond_tolerance");
  if (execution_invalidated)
    reasons.push_back("execution_invalidated_remaining_route");
  return reasons;
}

std::vector<std::string> dependencyChangeReasons(
    const domain::DependencyRevisions& consumed,
    const PlanningRequest& request) {
  std::vector<std::string> reasons;
  for (const auto& [dependency, revision] : consumed) {
    const auto current = currentRevision(request, dependency);
    if (current != revision) {
      std::ostringstream reason;
      reason << "dependency_changed:" << domain::toString(dependency) << ':'
             << revision << "->" << current;
      reasons.push_back(reason.str());
    }
  }
  return reasons;
}

void attachDependencySnapshot(
    PlanResult& plan, const PlanningRequest& request,
    std::vector<domain::ModelDependency> dependencies) {
  dependencies.push_back(domain::ModelDependency::PlannerConfiguration);
  std::sort(dependencies.begin(), dependencies.end(), [](auto a, auto b) {
    return static_cast<int>(a) < static_cast<int>(b);
  });
  dependencies.erase(std::unique(dependencies.begin(), dependencies.end()),
                     dependencies.end());
  plan.dependency_revisions.clear();
  for (const auto dependency : dependencies)
    plan.dependency_revisions[dependency] =
        currentRevision(request, dependency);
  plan.planned_start = request.start;
  plan.planned_goal = request.goal;
  plan.task_id = request.task_id;
  plan.planner_configuration_revision =
      request.planner_configuration_revision;
  plan.operating_mode = request.static_map
                            ? PlanningOperatingMode::MapEnabled
                            : PlanningOperatingMode::Mapless;
  plan.static_map_contributed = request.static_map != nullptr;
  if (plan.created_at == std::chrono::steady_clock::time_point{})
    plan.created_at = std::chrono::steady_clock::now();
  if (plan.hierarchical) {
    plan.hierarchical->id = plan.plan_id;
    plan.hierarchical->family = plan.family;
    plan.hierarchical->dependency_revisions = plan.dependency_revisions;
    plan.hierarchical->planned_start = request.start;
    plan.hierarchical->planned_goal = request.goal;
    plan.hierarchical->task_id = request.task_id;
    plan.hierarchical->planner_configuration_revision =
        request.planner_configuration_revision;
    plan.hierarchical->created_at = plan.created_at;
    plan.hierarchical->operating_mode = plan.operating_mode;
    plan.hierarchical->static_map_contributed = plan.static_map_contributed;
    plan.hierarchical->geometric_path = plan.path;
  }
}

}  // namespace semaforr::planning
