/**
 * @file traversability.hpp
 * @brief Traversability responsibilities.
 *
 * @details This file defines traversability behavior for path planning and
 * hierarchical plan construction. It centers on `OccupancySourceMode`,
 * `TraversabilityConfiguration`, `TraversabilityBuildResult`. Its
 * package-relative location is
 * `include/semaforr/planning/traversability.hpp`.
 */
#ifndef SEMAFORR_PLANNING_TRAVERSABILITY_HPP
#define SEMAFORR_PLANNING_TRAVERSABILITY_HPP

#include <optional>
#include <semaforr/domain/grid_layers.hpp>
#include <semaforr/domain/static_map.hpp>
#include <string>

namespace semaforr::planning {

/**
 * @brief Enumerates the supported occupancy source mode values used by this
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
enum class OccupancySourceMode {
  StaticMapWithSensors,
  SensorDerivedPartial,
  // Prefer static-map occupancy and otherwise require incrementally sensed
  // occupancy. Familiarity and learned graphs are never substituted.
  StaticOrSensorDerived
};

/**
 * @brief Encapsulates traversability configuration state and behavior for
 * this subsystem.
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

  /**
   * @brief Performs the traversability configuration operation for this
   * subsystem.
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
  TraversabilityConfiguration() = default;
  /**
   * @brief Performs the traversability configuration operation for this
   * subsystem.
   *
   * Arguments:
   * - @p map_policy: Supplies map policy input to the operation.
   * - @p partial_sensor_policy: Supplies partial sensor policy input to the
   * operation.
   * - @p robot_radius: Supplies robot radius input to the operation.
   * - @p safety_clearance: Supplies safety clearance input to the
   * operation.
   * - @p localization_uncertainty: Supplies localization uncertainty input
   * to the operation.
   * - @p turning_margin: Supplies turning margin input to the operation.
   * - @p dynamic_margin: Supplies dynamic margin input to the operation.
   * - @p unknown_cost: Supplies unknown cost input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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

/**
 * @brief Encapsulates traversability build result state and behavior for
 * this subsystem.
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
struct TraversabilityBuildResult {
  domain::TraversabilityGrid grid;
  std::string diagnostic;
  std::size_t static_conflicts = 0U;
  std::size_t traversable_cells = 0U;
  std::size_t occupied_cells = 0U;
  std::size_t inflated_cells = 0U;
};

/**
 * @brief Performs the derive traversability operation for this subsystem.
 *
 * Arguments:
 * - @p mode: Supplies mode input to the operation.
 * - @p static_map: Supplies static map input to the operation.
 * - @p sensed: Supplies sensed input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `TraversabilityBuildResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
TraversabilityBuildResult deriveTraversability(
    OccupancySourceMode mode, const domain::StaticMap* static_map,
    const domain::SensedOccupancyGrid* sensed,
    const TraversabilityConfiguration& configuration = {});

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p policy: Supplies policy input to the operation.
 *
 * Returns:
 * - `const char*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const char* toString(domain::UnknownSpacePolicy policy) noexcept;

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_TRAVERSABILITY_HPP
