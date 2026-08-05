#ifndef SEMAFORR_PLANNING_TRAVERSABILITY_HPP
#define SEMAFORR_PLANNING_TRAVERSABILITY_HPP

#include <optional>
#include <semaforr/domain/grid_layers.hpp>
#include <semaforr/domain/static_map.hpp>
#include <string>

namespace semaforr::planning {

enum class OccupancySourceMode {
  StaticMapWithSensors,
  SensorDerivedPartial,
  LearnedFreespaceWithOptionalOccupancy
};

struct TraversabilityConfiguration {
  domain::UnknownSpacePolicy unknown_policy =
      domain::UnknownSpacePolicy::Prohibited;
  domain::UnknownSpacePolicy sensor_unknown_policy =
      domain::UnknownSpacePolicy::Prohibited;
  double robot_radius_m = 0.22;
  double safety_clearance_m = 0.08;
  double localization_uncertainty_m = 0.05;
  double turning_footprint_margin_m = 0.0;
  double dynamic_obstacle_margin_m = 0.10;
  float unknown_cost_multiplier = 8.0F;
  std::optional<domain::Point2D> current_sensor_origin;
  double current_sensor_range_m = 0.0;

  TraversabilityConfiguration() = default;
  TraversabilityConfiguration(
      domain::UnknownSpacePolicy map_policy,
      domain::UnknownSpacePolicy partial_sensor_policy, double robot_radius,
      double safety_clearance, double localization_uncertainty,
      double turning_margin, double dynamic_margin, float unknown_cost)
      : unknown_policy(map_policy),
        sensor_unknown_policy(partial_sensor_policy),
        robot_radius_m(robot_radius),
        safety_clearance_m(safety_clearance),
        localization_uncertainty_m(localization_uncertainty),
        turning_footprint_margin_m(turning_margin),
        dynamic_obstacle_margin_m(dynamic_margin),
        unknown_cost_multiplier(unknown_cost) {}
};

struct TraversabilityBuildResult {
  domain::TraversabilityGrid grid;
  std::string diagnostic;
  std::size_t static_conflicts = 0U;
  std::size_t traversable_cells = 0U;
  std::size_t occupied_cells = 0U;
  std::size_t inflated_cells = 0U;
};

TraversabilityBuildResult deriveTraversability(
    OccupancySourceMode mode, const domain::StaticMap* static_map,
    const domain::SensedOccupancyGrid* sensed,
    const TraversabilityConfiguration& configuration = {});

const char* toString(domain::UnknownSpacePolicy policy) noexcept;

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_TRAVERSABILITY_HPP
