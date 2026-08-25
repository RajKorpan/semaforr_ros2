/**
 * @file model_revision.cpp
 * @brief Model revision responsibilities.
 *
 * @details This file implements model revision behavior for ROS-independent domain
 * state and value types. It records the declarations, settings, fixtures,
 * or guidance needed by that responsibility. Its package-relative location
 * is `src/domain/model_revision.cpp`.
 */
#include <semaforr/domain/model_revision.hpp>

namespace semaforr::domain {

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p dependency: Supplies dependency input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(ModelDependency dependency) noexcept {
  switch (dependency) {
    case ModelDependency::StaticMapGeometry: return "static_map_geometry";
    case ModelDependency::StaticOccupancy: return "static_occupancy";
    case ModelDependency::SensedOccupancy: return "sensed_occupancy";
    case ModelDependency::Familiarity: return "familiarity";
    case ModelDependency::Inclusion: return "inclusion";
    case ModelDependency::Trails: return "trails";
    case ModelDependency::Conveyors: return "conveyors";
    case ModelDependency::Regions: return "regions";
    case ModelDependency::DoorsAndExits: return "doors_and_exits";
    case ModelDependency::Hallways: return "hallways";
    case ModelDependency::Barriers: return "barriers";
    case ModelDependency::Skeleton: return "skeleton";
    case ModelDependency::Highways: return "highways";
    case ModelDependency::HighwayGraph: return "highway_graph";
    case ModelDependency::Circumstances: return "circumstances";
    case ModelDependency::LiveCrowdObservation:
      return "live_crowd_observation";
    case ModelDependency::CrowdDensity: return "crowd_density";
    case ModelDependency::CrowdRisk: return "crowd_risk";
    case ModelDependency::CrowdFlow: return "crowd_flow";
    case ModelDependency::PlannerConfiguration:
      return "planner_configuration";
    case ModelDependency::VisibilityGeometry: return "visibility_geometry";
  }
  return "unknown";
}

}  // namespace semaforr::domain
