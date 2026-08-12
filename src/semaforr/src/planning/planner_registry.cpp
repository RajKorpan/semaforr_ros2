#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/planner_registry.hpp>
#include <stdexcept>

namespace semaforr::planning {
void PlannerRegistry::add(std::string name, PlannerInputModel model,
                          Factory factory,
                          StaticMapRequirement map_requirement,
                          OccupancyRequirement occupancy_requirement) {
  if (name.empty() || !factory)
    throw std::invalid_argument(
        "planner registration requires a name and factory");
  if (!entries_.emplace(std::move(name),
                        Entry{model, std::move(factory), map_requirement,
                              occupancy_requirement})
           .second)
    throw std::invalid_argument("planner is already registered");
}
OccupancyRequirement PlannerRegistry::occupancyRequirement(
    const std::string& name) const {
  const auto found = entries_.find(name);
  if (found == entries_.end())
    throw std::invalid_argument("unknown planner '" + name + "'");
  return found->second.occupancy_requirement;
}
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
          planner->objective(), planner->dependencies(request),
          std::move(metadata)};
}
StaticMapRequirement PlannerRegistry::mapRequirement(
    const std::string& name) const {
  const auto found = entries_.find(name);
  if (found == entries_.end())
    throw std::invalid_argument("unknown planner '" + name + "'");
  return found->second.map_requirement;
}
std::unique_ptr<Planner> PlannerRegistry::create(
    const std::string& name) const {
  const auto found = entries_.find(name);
  if (found == entries_.end())
    throw std::invalid_argument("unknown planner '" + name + "'");
  return found->second.factory();
}
PlannerInputModel PlannerRegistry::inputModel(const std::string& name) const {
  const auto found = entries_.find(name);
  if (found == entries_.end())
    throw std::invalid_argument("unknown planner '" + name + "'");
  return found->second.model;
}
std::vector<std::string> PlannerRegistry::names(PlannerInputModel model) const {
  std::vector<std::string> result;
  for (const auto& [name, entry] : entries_)
    if (entry.model == model) result.push_back(name);
  return result;
}
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
              name, objective,
              OccupancySourceMode::StaticOrSensorDerived);
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
