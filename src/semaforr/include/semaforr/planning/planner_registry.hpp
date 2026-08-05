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
class PlannerRegistry {
 public:
  using Factory = std::function<std::unique_ptr<Planner>()>;
  void add(std::string name, PlannerInputModel model, Factory factory,
           StaticMapRequirement map_requirement =
               StaticMapRequirement::Independent);
  std::unique_ptr<Planner> create(const std::string& name) const;
  PlannerInputModel inputModel(const std::string& name) const;
  StaticMapRequirement mapRequirement(const std::string& name) const;
  std::vector<std::string> names(PlannerInputModel model) const;

 private:
  struct Entry {
    PlannerInputModel model;
    Factory factory;
    StaticMapRequirement map_requirement;
  };
  std::map<std::string, Entry> entries_;
};
PlannerRegistry defaultPlannerRegistry();
}  // namespace semaforr::planning
#endif
