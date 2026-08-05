#include <algorithm>
#include <cmath>
#include <semaforr/planning/traversability.hpp>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace semaforr::planning {
namespace {

std::optional<domain::SensedOccupancyCell> sensedAt(
    const domain::SensedOccupancyGrid* sensed, domain::Point2D point) {
  if (!sensed || !sensed->valid()) return std::nullopt;
  const auto index = sensed->geometry.index(point);
  if (!index) return std::nullopt;
  return sensed->cells[*index];
}

bool permitUnknown(domain::UnknownSpacePolicy policy, domain::Point2D point,
                   const TraversabilityConfiguration& configuration) {
  if (policy == domain::UnknownSpacePolicy::HighCost ||
      policy == domain::UnknownSpacePolicy::ExplorationOnly)
    return true;
  return policy == domain::UnknownSpacePolicy::WithinSensorRange &&
         configuration.current_sensor_origin &&
         configuration.current_sensor_range_m > 0.0 &&
         domain::distance(*configuration.current_sensor_origin, point)
                 .meters() <= configuration.current_sensor_range_m;
}

}  // namespace

TraversabilityBuildResult deriveTraversability(
    OccupancySourceMode mode, const domain::StaticMap* static_map,
    const domain::SensedOccupancyGrid* sensed,
    const TraversabilityConfiguration& configuration) {
  if (configuration.robot_radius_m < 0.0 ||
      configuration.safety_clearance_m < 0.0 ||
      configuration.localization_uncertainty_m < 0.0 ||
      configuration.turning_footprint_margin_m < 0.0 ||
      configuration.dynamic_obstacle_margin_m < 0.0 ||
      configuration.unknown_cost_multiplier < 1.0F)
    throw std::invalid_argument("traversability margins and unknown cost are invalid");

  TraversabilityBuildResult result;
  domain::GridExtent extent;
  const domain::StaticOccupancyGrid* prior = nullptr;
  if (mode == OccupancySourceMode::StaticMapWithSensors) {
    if (!static_map || !static_map->occupancyAvailable()) {
      result.diagnostic = "static-map occupancy is unavailable";
      return result;
    }
    prior = &static_map->occupancy;
    extent = {prior->columns, prior->rows, prior->resolution_m, prior->origin};
    result.grid.complete_prior_bounds = true;
    result.grid.source_static_revision = static_map->revision;
  } else {
    if (!sensed || !sensed->valid() || sensed->observedCellCount() == 0U) {
      result.diagnostic = "sensor-derived occupancy is not sufficiently defined";
      return result;
    }
    extent = sensed->geometry;
  }
  result.grid.geometry = extent;
  result.grid.unknown_policy = configuration.unknown_policy;
  result.grid.source_sensed_revision = sensed ? sensed->revision : 0U;
  result.grid.cells.resize(extent.columns * extent.rows);
  std::vector<bool> occupied(result.grid.cells.size(), false);
  std::vector<bool> dynamic(result.grid.cells.size(), false);

  for (std::size_t index = 0U; index < result.grid.cells.size(); ++index) {
    auto& output = result.grid.cells[index];
    const auto point = extent.center(index);
    const auto sensor = sensedAt(sensed, point);
    const auto static_state = prior ? prior->cells[index]
                                    : domain::StaticOccupancyState::StaticUnknown;
    if (prior && static_state == domain::StaticOccupancyState::StaticOccupied) {
      output = {domain::TraversabilityState::NonTraversable, 1.0F,
                domain::OccupancyEvidenceSource::StaticMap};
      occupied[index] = true;
      ++result.occupied_cells;
      if (sensor && sensor->state == domain::SensedOccupancyState::ObservedFree)
        ++result.static_conflicts;
      continue;
    }
    if (sensor &&
        sensor->state == domain::SensedOccupancyState::ObservedOccupied) {
      output = {domain::TraversabilityState::NonTraversable, 1.0F,
                sensor->source};
      occupied[index] = true;
      dynamic[index] = sensor->dynamic;
      ++result.occupied_cells;
      continue;
    }
    const bool known_free =
        (prior && static_state == domain::StaticOccupancyState::StaticFree) ||
        (sensor && sensor->state == domain::SensedOccupancyState::ObservedFree);
    if (known_free) {
      auto source = prior ? domain::OccupancyEvidenceSource::StaticMap
                          : domain::OccupancyEvidenceSource::None;
      if (sensor && sensor->state == domain::SensedOccupancyState::ObservedFree)
        source = source | sensor->source;
      output = {domain::TraversabilityState::Traversable, 1.0F, source};
      ++result.traversable_cells;
    } else if (permitUnknown(configuration.unknown_policy, point,
                             configuration)) {
      output = {domain::TraversabilityState::UnknownPermitted,
                configuration.unknown_cost_multiplier,
                domain::OccupancyEvidenceSource::UnknownSpacePolicy};
      ++result.traversable_cells;
    } else {
      output = {domain::TraversabilityState::UnknownProhibited, 1.0F,
                domain::OccupancyEvidenceSource::UnknownSpacePolicy};
    }
  }

  const double base_margin = configuration.robot_radius_m +
                             configuration.safety_clearance_m +
                             configuration.localization_uncertainty_m +
                             configuration.turning_footprint_margin_m;
  for (std::size_t source = 0U; source < occupied.size(); ++source) {
    if (!occupied[source]) continue;
    const double margin = base_margin +
                          (dynamic[source]
                               ? configuration.dynamic_obstacle_margin_m
                               : 0.0);
    const int radius = static_cast<int>(std::ceil(margin / extent.resolution_m));
    const int source_row = static_cast<int>(source / extent.columns);
    const int source_column = static_cast<int>(source % extent.columns);
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::hypot(static_cast<double>(dx), static_cast<double>(dy)) *
                extent.resolution_m >
            margin)
          continue;
        const int row = source_row + dy, column = source_column + dx;
        if (row < 0 || column < 0 || row >= static_cast<int>(extent.rows) ||
            column >= static_cast<int>(extent.columns))
          continue;
        const auto index = static_cast<std::size_t>(row) * extent.columns +
                           static_cast<std::size_t>(column);
        if (occupied[index]) continue;
        if (result.grid.cells[index].permitsTraversal()) {
          --result.traversable_cells;
          ++result.inflated_cells;
        }
        result.grid.cells[index] = {
            domain::TraversabilityState::InflatedObstacle, 1.0F,
            result.grid.cells[index].provenance |
                domain::OccupancyEvidenceSource::Inflation};
      }
    }
  }
  std::ostringstream diagnostic;
  diagnostic << (mode == OccupancySourceMode::StaticMapWithSensors
                     ? "static map+sensed"
                     : "partial sensor-derived")
             << " traversability; unknown_policy="
             << toString(configuration.unknown_policy)
             << "; traversable=" << result.traversable_cells
             << "; occupied=" << result.occupied_cells
             << "; inflated=" << result.inflated_cells
             << "; static_sensor_conflicts=" << result.static_conflicts;
  result.diagnostic = diagnostic.str();
  return result;
}

const char* toString(domain::UnknownSpacePolicy policy) noexcept {
  switch (policy) {
    case domain::UnknownSpacePolicy::Prohibited:
      return "prohibited";
    case domain::UnknownSpacePolicy::HighCost:
      return "high_cost";
    case domain::UnknownSpacePolicy::WithinSensorRange:
      return "within_sensor_range";
    case domain::UnknownSpacePolicy::ExplorationOnly:
      return "exploration_only";
  }
  return "unknown";
}

}  // namespace semaforr::planning
