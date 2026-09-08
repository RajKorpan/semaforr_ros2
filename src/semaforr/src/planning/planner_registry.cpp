/**
 * @file planner_registry.cpp
 * @brief Planner registry responsibilities.
 *
 * @details This file implements planner registry behavior for path planning and
 * hierarchical plan construction. It records the declarations, settings,
 * fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/planning/planner_registry.cpp`.
 */
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/planner_registry.hpp>
#include <stdexcept>

namespace semaforr::planning {
/**
 * @brief Performs the add operation for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 * - @p model: Supplies model input to the operation.
 * - @p factory: Supplies factory input to the operation.
 * - @p map_requirement: Supplies map requirement input to the operation.
 * - @p occupancy_requirement: Supplies occupancy requirement input to the
 * operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void PlannerRegistry::add(std::string name, PlannerInputModel model,
                          Factory factory, StaticMapRequirement map_requirement,
                          OccupancyRequirement occupancy_requirement) {
  if (name.empty() || !factory)
    throw std::invalid_argument(
        "planner registration requires a name and factory");
  if (!entries_
           .emplace(std::move(name),
                    Entry{model, std::move(factory), map_requirement,
                          occupancy_requirement})
           .second)
    throw std::invalid_argument("planner is already registered");
}
/**
 * @brief Performs the occupancy requirement operation for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 *
 * Returns:
 * - `OccupancyRequirement` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
OccupancyRequirement PlannerRegistry::occupancyRequirement(
    const std::string& name) const {
  const auto found = entries_.find(name);
  if (found == entries_.end())
    throw std::invalid_argument("unknown planner '" + name + "'");
  return found->second.occupancy_requirement;
}
/**
 * @brief Performs the declaration operation for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 * - @p request: Supplies request input to the operation.
 *
 * Returns:
 * - `PlannerDeclaration` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PlannerDeclaration PlannerRegistry::declaration(
    const std::string& name, const PlanningRequest& request) const {
  const auto found = entries_.find(name);
  if (found == entries_.end())
    throw std::invalid_argument("unknown planner '" + name + "'");
  auto planner = found->second.factory();
  auto metadata = planner->metadata();
  metadata.name = name;
  metadata.requires_static_map =
      found->second.map_requirement == StaticMapRequirement::Required;
  metadata.supports_mapless_operation =
      found->second.map_requirement != StaticMapRequirement::Required;
  metadata.representation_dependencies.clear();
  for (const auto dependency : planner->dependencies(request))
    metadata.representation_dependencies.push_back(
        std::string(domain::toString(dependency)));
  if (!metadata.valid())
    throw std::logic_error("planner '" + name +
                           "' has incomplete explanation metadata");
  return {name,
          found->second.model,
          planner->planFamily(),
          found->second.map_requirement,
          found->second.occupancy_requirement,
          found->second.occupancy_requirement ==
                  OccupancyRequirement::SensedPartial ||
              found->second.occupancy_requirement ==
                  OccupancyRequirement::StaticOrSensedPartial,
          planner->objective(),
          planner->dependencies(request),
          std::move(metadata)};
}
/**
 * @brief Performs the map requirement operation for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 *
 * Returns:
 * - `StaticMapRequirement` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
StaticMapRequirement PlannerRegistry::mapRequirement(
    const std::string& name) const {
  const auto found = entries_.find(name);
  if (found == entries_.end())
    throw std::invalid_argument("unknown planner '" + name + "'");
  return found->second.map_requirement;
}
/**
 * @brief Creates package content for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 *
 * Returns:
 * - `std::unique_ptr<Planner>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::unique_ptr<Planner> PlannerRegistry::create(
    const std::string& name) const {
  const auto found = entries_.find(name);
  if (found == entries_.end())
    throw std::invalid_argument("unknown planner '" + name + "'");
  return found->second.factory();
}
/**
 * @brief Performs the input model operation for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 *
 * Returns:
 * - `PlannerInputModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PlannerInputModel PlannerRegistry::inputModel(const std::string& name) const {
  const auto found = entries_.find(name);
  if (found == entries_.end())
    throw std::invalid_argument("unknown planner '" + name + "'");
  return found->second.model;
}
/**
 * @brief Performs the names operation for this subsystem.
 *
 * Arguments:
 * - @p model: Supplies model input to the operation.
 *
 * Returns:
 * - `std::vector<std::string>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<std::string> PlannerRegistry::names(PlannerInputModel model) const {
  std::vector<std::string> result;
  for (const auto& [name, entry] : entries_)
    if (entry.model == model) result.push_back(name);
  return result;
}
/**
 * @brief Performs the default planner registry operation for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `PlannerRegistry` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PlannerRegistry defaultPlannerRegistry() {
  PlannerRegistry registry;
  const auto domain = [&](std::string name, PlannerInputModel input,
                          PlanObjective objective) {
    registry.add(
        name, input,
        [name, objective] {
          return std::make_unique<DomainPlanner>(name, objective);
        },
        StaticMapRequirement::Required, OccupancyRequirement::StaticMap);
  };
  domain("distance", PlannerInputModel::Grid, PlanObjective::Distance);
  registry.add(
      "sensor_distance", PlannerInputModel::Grid,
      [] {
        return std::make_unique<DomainPlanner>(
            "sensor_distance", PlanObjective::Distance,
            OccupancySourceMode::SensorDerivedPartial);
      },
      StaticMapRequirement::Independent, OccupancyRequirement::SensedPartial);
  domain("density", PlannerInputModel::Grid, PlanObjective::CrowdDensity);
  domain("risk", PlannerInputModel::Grid, PlanObjective::EncounterRisk);
  domain("flow", PlannerInputModel::Grid, PlanObjective::FlowOpposition);
  const auto learned = [&](std::string name, PlanObjective objective) {
    registry.add(
        name, PlannerInputModel::AffordanceModifiedGrid,
        [name, objective] {
          return std::make_unique<DomainPlanner>(
              name, objective, OccupancySourceMode::StaticOrSensorDerived);
        },
        StaticMapRequirement::Optional,
        OccupancyRequirement::StaticOrSensedPartial);
  };
  learned("region", PlanObjective::RegionPreference);
  learned("hallway", PlanObjective::HallwayPreference);
  learned("trail", PlanObjective::TrailPreference);
  learned("conveyor", PlanObjective::ConveyorPreference);
  registry.add("skeleton", PlannerInputModel::Freespace,
               [] { return std::make_unique<SkeletonPlan>(); });
  registry.add("highway", PlannerInputModel::Freespace,
               [] { return std::make_unique<HighwayPlan>(); });
  return registry;
}
}  // namespace semaforr::planning
