/**
 * @file hierarchical_plan.hpp
 * @brief Hierarchical plan responsibilities.
 *
 * @details This file defines hierarchical plan behavior for path planning and
 * hierarchical plan construction. It centers on `SkeletonPlan`,
 * `HighwayPlan`. Its package-relative location is
 * `include/semaforr/planning/hierarchical_plan.hpp`.
 */
#ifndef SEMAFORR_PLANNING_HIERARCHICAL_PLAN_HPP
#define SEMAFORR_PLANNING_HIERARCHICAL_PLAN_HPP

#include <semaforr/planning/planner.hpp>
#include <string_view>

namespace semaforr::planning {

/**
 * @brief Encapsulates skeleton plan state and behavior for this subsystem.
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
class SkeletonPlan final : public Planner {
 public:
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
  std::string_view name() const noexcept override { return "skeleton_plan"; }
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
  PlanFamily planFamily() const noexcept override { return PlanFamily::Model; }
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
  PlanObjective objective() const noexcept override {
    return PlanObjective::SkeletonDistance;
  }
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
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `std::vector<domain::ModelDependency>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<domain::ModelDependency> dependencies(
      const PlanningRequest&) const override;
};

/**
 * @brief Encapsulates highway plan state and behavior for this subsystem.
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
class HighwayPlan final : public Planner {
 public:
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
  std::string_view name() const noexcept override { return "highway_plan"; }
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
  PlanFamily planFamily() const noexcept override { return PlanFamily::Model; }
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
  PlanObjective objective() const noexcept override {
    return PlanObjective::HighwayDistance;
  }
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
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `std::vector<domain::ModelDependency>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<domain::ModelDependency> dependencies(
      const PlanningRequest&) const override;
};

}  // namespace semaforr::planning

#endif
