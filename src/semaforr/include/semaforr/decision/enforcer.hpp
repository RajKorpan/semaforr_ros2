/**
 * @file enforcer.hpp
 * @brief Enforcer responsibilities.
 *
 * @details This file defines enforcer behavior for tiered decision making and
 * action arbitration. It centers on `EnforcerMode`, `EnforcementStatus`,
 * `LocalActionPrediction`, `PlanEnforcementContext`,
 * `PlanEnforcementResult`, `LocalActionEvaluator`, `GridPlanEnforcer`,
 * `ModelPlanEnforcer`. Its package-relative location is
 * `include/semaforr/decision/enforcer.hpp`.
 */
#ifndef SEMAFORR_DECISION_ENFORCER_HPP
#define SEMAFORR_DECISION_ENFORCER_HPP

#include <cstddef>
#include <semaforr/decision/rules.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/traversability.hpp>
#include <span>
#include <string>
#include <vector>

namespace semaforr::decision {

/**
 * @brief Enumerates the supported enforcer mode values used by this
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
enum class EnforcerMode { Grid, Model };
/**
 * @brief Enumerates the supported enforcement status values used by this
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
enum class EnforcementStatus {
  Mandated,
  Complete,
  CannotOperationalize,
  Invalid,
  Stale
};

/**
 * @brief Encapsulates local action prediction state and behavior for this
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
struct LocalActionPrediction {
  domain::Action action;
  domain::Pose2D predicted_pose;
  double progress_m = 0.0;
  bool selected = false;
};

/**
 * @brief Encapsulates plan enforcement context state and behavior for this
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
struct PlanEnforcementContext {
  const domain::SpatialModel& spatial;
  const domain::CrowdModel* crowd = nullptr;
  const domain::StaticMap* static_map = nullptr;
  const domain::Pose2D& pose;
  const domain::ActionSpace& action_space;
  std::span<const domain::Action> viable_actions;
  planning::TraversabilityConfiguration traversability;
  domain::Distance tolerance{0.5};
  std::optional<domain::TaskId> task_id;
};

/**
 * @brief Encapsulates plan enforcement result state and behavior for this
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
struct PlanEnforcementResult {
  EnforcementStatus status{EnforcementStatus::CannotOperationalize};
  EnforcerMode mode{EnforcerMode::Grid};
  std::optional<domain::Action> action;
  std::optional<domain::Point2D> operational_target;
  std::size_t step_index = 0U;
  std::size_t skipped_elements = 0U;
  double lookahead_m = 0.0;
  std::string step_type;
  std::string supporting_id;
  std::string reason_code;
  std::string validation_evidence;
  std::optional<std::string> shortcut;
  std::optional<std::string> repair;
  domain::DependencyRevisions dependency_revisions;
  std::vector<LocalActionPrediction> candidates;
};

/**
 * @brief Encapsulates local action evaluator state and behavior for this
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
class LocalActionEvaluator {
 public:
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p result: Supplies result input to the operation.
   * - @p context: Supplies context input to the operation.
   *
   * Returns:
   * - `PlanEnforcementResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PlanEnforcementResult evaluate(PlanEnforcementResult result,
                                 const PlanEnforcementContext& context) const;
};

/**
 * @brief Encapsulates grid plan enforcer state and behavior for this
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
class GridPlanEnforcer {
 public:
  /**
   * @brief Performs the enforce operation for this subsystem.
   *
   * Arguments:
   * - @p plan: Supplies plan input to the operation.
   * - @p context: Supplies context input to the operation.
   *
   * Returns:
   * - `PlanEnforcementResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PlanEnforcementResult enforce(planning::HierarchicalPlan& plan,
                                const PlanEnforcementContext& context) const;
};

/**
 * @brief Encapsulates model plan enforcer state and behavior for this
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
class ModelPlanEnforcer {
 public:
  /**
   * @brief Performs the enforce operation for this subsystem.
   *
   * Arguments:
   * - @p plan: Supplies plan input to the operation.
   * - @p context: Supplies context input to the operation.
   *
   * Returns:
   * - `PlanEnforcementResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PlanEnforcementResult enforce(planning::HierarchicalPlan& plan,
                                const PlanEnforcementContext& context) const;
};

/**
 * @brief Encapsulates enforcer state and behavior for this subsystem.
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
class Enforcer final : public PlanOperationalizer {
 public:
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
  std::string_view name() const noexcept override { return "enforcer"; }
  /**
   * @brief Performs the operationalize operation for this subsystem.
   *
   * Arguments:
   * - @p plan: Supplies plan input to the operation.
   *
   * Returns:
   * - `std::vector<domain::Point2D>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<domain::Point2D> operationalize(
      const planning::HierarchicalPlan& plan) const override;
  /**
   * @brief Performs the active step operation for this subsystem.
   *
   * Arguments:
   * - @p plan: Supplies plan input to the operation.
   * - @p pose: Supplies pose input to the operation.
   * - @p tolerance: Supplies tolerance input to the operation.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t activeStep(const planning::HierarchicalPlan& plan,
                         const domain::Pose2D& pose,
                         domain::Distance tolerance) const noexcept;
  /**
   * @brief Performs the operationalize next operation for this subsystem.
   *
   * Arguments:
   * - @p plan: Supplies plan input to the operation.
   * - @p spatial: Supplies spatial input to the operation.
   * - @p pose: Supplies pose input to the operation.
   * - @p tolerance: Supplies tolerance input to the operation.
   *
   * Returns:
   * - `std::optional<domain::Point2D>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<domain::Point2D> operationalizeNext(
      planning::HierarchicalPlan& plan, const domain::SpatialModel& spatial,
      const domain::Pose2D& pose, domain::Distance tolerance) const override;
  /**
   * @brief Performs the enforce operation for this subsystem.
   *
   * Arguments:
   * - @p plan: Supplies plan input to the operation.
   * - @p context: Supplies context input to the operation.
   *
   * Returns:
   * - `PlanEnforcementResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PlanEnforcementResult enforce(planning::HierarchicalPlan& plan,
                                const PlanEnforcementContext& context) const;

 private:
  GridPlanEnforcer grid_;
  ModelPlanEnforcer model_;
};

}  // namespace semaforr::decision

#endif
