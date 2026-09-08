/**
 * @file planner_registry.hpp
 * @brief Planner registry responsibilities.
 *
 * @details This file defines planner registry behavior for path planning and
 * hierarchical plan construction. It centers on `PlannerInputModel`,
 * `StaticMapRequirement`, `OccupancyRequirement`, `PlannerDeclaration`,
 * `PlannerRegistry`, `Entry`. Its package-relative location is
 * `include/semaforr/planning/planner_registry.hpp`.
 */
#ifndef SEMAFORR_PLANNING_PLANNER_REGISTRY_HPP
#define SEMAFORR_PLANNING_PLANNER_REGISTRY_HPP

#include <functional>
#include <map>
#include <memory>
#include <semaforr/planning/planner.hpp>
#include <string>
#include <vector>

namespace semaforr::planning {
/**
 * @brief Enumerates the supported planner input model values used by this
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
enum class PlannerInputModel { Grid, AffordanceModifiedGrid, Freespace };
/**
 * @brief Enumerates the supported static map requirement values used by
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
enum class StaticMapRequirement { Required, Optional, Independent };
/**
 * @brief Enumerates the supported occupancy requirement values used by this
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
enum class OccupancyRequirement {
  None,
  StaticMap,
  SensedPartial,
  StaticOrSensedPartial
};
/**
 * @brief Encapsulates planner declaration state and behavior for this
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
/**
 * @brief Encapsulates planner registry state and behavior for this
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
class PlannerRegistry {
 public:
  using Factory = std::function<std::unique_ptr<Planner>()>;
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
  void add(
      std::string name, PlannerInputModel model, Factory factory,
      StaticMapRequirement map_requirement = StaticMapRequirement::Independent,
      OccupancyRequirement occupancy_requirement = OccupancyRequirement::None);
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
  std::unique_ptr<Planner> create(const std::string& name) const;
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
  PlannerInputModel inputModel(const std::string& name) const;
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
  StaticMapRequirement mapRequirement(const std::string& name) const;
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
  OccupancyRequirement occupancyRequirement(const std::string& name) const;
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
  PlannerDeclaration declaration(const std::string& name,
                                 const PlanningRequest& request) const;
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
  std::vector<std::string> names(PlannerInputModel model) const;

 private:
  /**
   * @brief Encapsulates entry state and behavior for this subsystem.
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
  struct Entry {
    PlannerInputModel model;
    Factory factory;
    StaticMapRequirement map_requirement;
    OccupancyRequirement occupancy_requirement;
  };
  std::map<std::string, Entry> entries_;
};
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
PlannerRegistry defaultPlannerRegistry();
}  // namespace semaforr::planning
#endif
