#ifndef SEMAFORR_PLANNING_PLANNER_REGISTRY_HPP
#define SEMAFORR_PLANNING_PLANNER_REGISTRY_HPP

#include <functional>
#include <map>
#include <memory>
#include <semaforr/planning/planner.hpp>
#include <string>
#include <vector>

namespace semaforr::planning {
enum class PlannerInputModel { Grid, AffordanceModifiedGrid, Freespace };
enum class StaticMapRequirement { Required, Optional, Independent };
enum class OccupancyRequirement {
  None,
  StaticMap,
  SensedPartial,
  StaticOrSensedPartial
};
struct PlannerDeclaration {
  std::string name;
  PlannerInputModel input_model{PlannerInputModel::Grid};
  PlanFamily plan_family{PlanFamily::Grid};
  StaticMapRequirement static_map{StaticMapRequirement::Independent};
  OccupancyRequirement occupancy{OccupancyRequirement::None};
  bool supports_partial_sensor_occupancy{false};
  PlanObjective objective{PlanObjective::Distance};
  std::vector<domain::ModelDependency> revision_dependencies;
  PlannerMetadata explanation_metadata;
};
class PlannerRegistry {
 public:
  using Factory = std::function<std::unique_ptr<Planner>()>;
  void add(std::string name, PlannerInputModel model, Factory factory,
           StaticMapRequirement map_requirement =
               StaticMapRequirement::Independent,
           OccupancyRequirement occupancy_requirement =
               OccupancyRequirement::None);
  std::unique_ptr<Planner> create(const std::string& name) const;
  PlannerInputModel inputModel(const std::string& name) const;
  StaticMapRequirement mapRequirement(const std::string& name) const;
  OccupancyRequirement occupancyRequirement(const std::string& name) const;
  PlannerDeclaration declaration(const std::string& name,
                                 const PlanningRequest& request) const;
  std::vector<std::string> names(PlannerInputModel model) const;

 private:
  struct Entry {
    PlannerInputModel model;
    Factory factory;
    StaticMapRequirement map_requirement;
    OccupancyRequirement occupancy_requirement;
  };
  std::map<std::string, Entry> entries_;
};
PlannerRegistry defaultPlannerRegistry();
}  // namespace semaforr::planning
#endif
