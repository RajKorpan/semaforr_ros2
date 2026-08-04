#include <semaforr/planning/planner.hpp>

namespace semaforr::planning {

std::optional<domain::Point2D> stepTarget(const PlanStep& step) noexcept {
  return std::visit(
      [](const auto& value) -> std::optional<domain::Point2D> {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, WaypointStep>) {
          return value.target;
        } else if constexpr (std::is_same_v<T, RegionStep>) {
          return value.center;
        } else if constexpr (std::is_same_v<T, IntersectionStep>) {
          return value.centroid;
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

}  // namespace semaforr::planning
