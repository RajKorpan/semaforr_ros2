#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/planner_registry.hpp>
#include <stdexcept>

namespace semaforr::planning {
void PlannerRegistry::add(std::string name, PlannerInputModel model,
                          Factory factory) {
  if (name.empty() || !factory)
    throw std::invalid_argument(
        "planner registration requires a name and factory");
  if (!entries_.emplace(std::move(name), Entry{model, std::move(factory)})
           .second)
    throw std::invalid_argument("planner is already registered");
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
    registry.add(name, input, [name, objective] {
      return std::make_unique<DomainPlanner>(name, objective);
    });
  };
  domain("distance", PlannerInputModel::Grid, PlanObjective::Distance);
  domain("density", PlannerInputModel::Grid, PlanObjective::CrowdDensity);
  domain("risk", PlannerInputModel::Grid, PlanObjective::EncounterRisk);
  domain("flow", PlannerInputModel::Grid, PlanObjective::FlowOpposition);
  domain("region", PlannerInputModel::AffordanceModifiedGrid,
         PlanObjective::RegionPreference);
  domain("hallway", PlannerInputModel::AffordanceModifiedGrid,
         PlanObjective::HallwayPreference);
  domain("trail", PlannerInputModel::AffordanceModifiedGrid,
         PlanObjective::TrailPreference);
  domain("conveyor", PlannerInputModel::AffordanceModifiedGrid,
         PlanObjective::ConveyorPreference);
  registry.add("skeleton", PlannerInputModel::Freespace,
               [] { return std::make_unique<SkeletonPlan>(); });
  registry.add("highway", PlannerInputModel::Freespace,
               [] { return std::make_unique<HighwayPlan>(); });
  return registry;
}
}  // namespace semaforr::planning
