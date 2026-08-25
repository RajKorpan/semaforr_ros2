/**
 * @file domain_planner.hpp
 * @brief Domain planner responsibilities.
 *
 * @details This file defines domain planner behavior for path planning and
 * hierarchical plan construction. It centers on `DomainPlanner`. Its
 * package-relative location is
 * `include/semaforr/planning/domain_planner.hpp`.
 */
#ifndef SEMAFORR_PLANNING_DOMAIN_PLANNER_HPP
#define SEMAFORR_PLANNING_DOMAIN_PLANNER_HPP

#include <semaforr/planning/planner.hpp>
#include <string>

namespace semaforr::planning {

using PlannerObjective = PlanObjective;

/**
 * @brief Evaluates path objectives for this subsystem.
 *
 * Arguments:
 * - @p request: Supplies request input to the operation.
 * - @p path: Supplies path input to the operation.
 *
 * Returns:
 * - `ObjectiveCosts` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
ObjectiveCosts evaluatePathObjectives(const PlanningRequest& request,
                                      const std::vector<domain::Point2D>& path);

/**
 * @brief Encapsulates domain planner state and behavior for this subsystem.
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
class DomainPlanner final : public Planner {
 public:
  /**
   * @brief Performs the domain planner operation for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   * - @p objective: Supplies objective input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  DomainPlanner(std::string name, PlannerObjective objective);
  /**
   * @brief Performs the domain planner operation for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   * - @p objective: Supplies objective input to the operation.
   * - @p source_mode: Supplies source mode input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  DomainPlanner(std::string name, PlannerObjective objective,
                OccupancySourceMode source_mode);
  /**
   * @brief Constructs package content for this subsystem.
   *
   * Arguments:
   * - @p request: Supplies request input to the operation.
   *
   * Returns:
   * - `PlanResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PlanResult plan(const PlanningRequest& request) override;
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string_view name() const noexcept override { return name_; }
  /**
   * @brief Performs the objective operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `PlanObjective` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PlanObjective objective() const noexcept override { return objective_; }
  /**
   * @brief Constructs family for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `PlanFamily` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PlanFamily planFamily() const noexcept override { return PlanFamily::Grid; }
  /**
   * @brief Performs the metadata operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `PlannerMetadata` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PlannerMetadata metadata() const override;
  /**
   * @brief Performs the dependencies operation for this subsystem.
   *
   * Arguments:
   * - @p request: Supplies request input to the operation.
   *
   * Returns:
   * - `std::vector<domain::ModelDependency>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<domain::ModelDependency> dependencies(
      const PlanningRequest& request) const override;

 private:
  std::string name_;
  PlannerObjective objective_;
  OccupancySourceMode source_mode_ = OccupancySourceMode::StaticMapWithSensors;
};

}  // namespace semaforr::planning
#endif
