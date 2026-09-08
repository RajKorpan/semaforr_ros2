/**
 * @file decision_result.hpp
 * @brief Decision result responsibilities.
 *
 * @details This file defines decision result behavior for tiered decision
 * making and action arbitration. It centers on `DecisionSource`,
 * `DecisionTier`, `ActionOutcome`, `RejectionKind`, `VetoCategory`, `Veto`,
 * `DecisionCycleEvent`, `AdvisorContribution`. Its package-relative
 * location is `include/semaforr/decision/decision_result.hpp`.
 */
#ifndef SEMAFORR_DECISION_DECISION_RESULT_HPP
#define SEMAFORR_DECISION_DECISION_RESULT_HPP

#include <cstdint>
#include <optional>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/action_execution.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/navigation/navigation_phase.hpp>
#include <semaforr/planning/planner.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace semaforr::decision {

/**
 * @brief Enumerates the supported decision source values used by this
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
enum class DecisionSource {
  MandatoryRule,
  TierThreeAdvisor,
  Planner,
  Exploration,
  Fallback,
  SafeStop
};

/**
 * @brief Enumerates the supported decision tier values used by this
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
enum class DecisionTier {
  TierOne,
  TierTwo,
  TierThree,
  Exploration,
  Fallback,
  SafeStop
};

/**
 * @brief Enumerates the supported action outcome values used by this
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
enum class ActionOutcome {
  Pending,
  Completed,
  TimedOut,
  OdometryReset,
  ClockReset,
  Cancelled,
  SensorLost,
  Shutdown,
  PartialMovement,
  NoMovement,
  SafetyInterrupted,
  ControllerRejected,
  ControllerFailure,
  GoalPreempted,
  NavigationModeTransition
};

/**
 * @brief Enumerates the supported rejection kind values used by this
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
enum class RejectionKind { Safety, Cognitive, NotViable };
/**
 * @brief Enumerates the supported veto category values used by this
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
enum class VetoCategory {
  Unsafe,
  ObstacleConflict,
  OpposesRecentOrientation,
  IneffectivePrecedent,
  ReturnsToVisitedSpace,
  ActivePlanConflict,
  NoUsefulProgress,
  NotViable,
  ReactiveControl,
  ExplorationPreference,
  CaseBasedPrecedent,
  PlanEnforcement,
  InvalidNavigationState
};

/**
 * @brief Encapsulates veto state and behavior for this subsystem.
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
struct Veto {
  /**
   * @brief Performs the pause operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `domain::Action action{` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Action action{domain::Action::pause()};
  std::string rule;
  std::string explanation;
  std::string reason_code;
  RejectionKind rejection_kind{RejectionKind::Cognitive};
  VetoCategory category{VetoCategory::InvalidNavigationState};

  /**
   * @brief Performs the veto operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Veto() = default;
  /**
   * @brief Performs the veto operation for this subsystem.
   *
   * Arguments:
   * - @p vetoed_action: Supplies vetoed action input to the operation.
   * - @p vetoing_rule: Supplies vetoing rule input to the operation.
   * - @p detail: Supplies detail input to the operation.
   * - @p kind: Supplies kind input to the operation.
   * - @p semantic: Supplies semantic input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Veto(domain::Action vetoed_action, std::string vetoing_rule,
       std::string detail, RejectionKind kind = RejectionKind::Cognitive,
       VetoCategory semantic = VetoCategory::InvalidNavigationState)
      : action(vetoed_action),
        rule(std::move(vetoing_rule)),
        explanation(detail),
        reason_code(std::move(detail)),
        rejection_kind(kind),
        category(semantic) {}

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const Veto&) const = default;
};

/**
 * @brief Encapsulates decision cycle event state and behavior for this
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
struct DecisionCycleEvent {
  std::size_t order{0U};
  std::string tier;
  std::string component;
  std::vector<domain::Action> input_actions;
  std::optional<domain::Action> mandate;
  std::vector<Veto> vetoes;
  std::vector<domain::Action> remaining_actions;
  std::string outcome;
  bool returned_to_earlier_tier{false};
  std::optional<DecisionTier> final_attribution;
  std::string reason_code;

  /**
   * @brief Performs the decision cycle event operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  DecisionCycleEvent() = default;
  /**
   * @brief Performs the decision cycle event operation for this subsystem.
   *
   * Arguments:
   * - @p event_order: Supplies event order input to the operation.
   * - @p event_tier: Supplies event tier input to the operation.
   * - @p event_component: Supplies event component input to the operation.
   * - @p inputs: Supplies inputs input to the operation.
   * - @p event_mandate: Supplies event mandate input to the operation.
   * - @p event_vetoes: Supplies event vetoes input to the operation.
   * - @p event_outcome: Supplies event outcome input to the operation.
   * - @p returned: Supplies returned input to the operation.
   * - @p attribution: Supplies attribution input to the operation.
   * - @p event_reason_code: Supplies event reason code input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  DecisionCycleEvent(std::size_t event_order, std::string event_tier,
                     std::string event_component,
                     std::vector<domain::Action> inputs,
                     std::optional<domain::Action> event_mandate,
                     std::vector<Veto> event_vetoes,
                     std::string event_outcome = {}, bool returned = false,
                     std::optional<DecisionTier> attribution = std::nullopt,
                     std::string event_reason_code = {})
      : order(event_order),
        tier(std::move(event_tier)),
        component(std::move(event_component)),
        input_actions(std::move(inputs)),
        mandate(event_mandate),
        vetoes(std::move(event_vetoes)),
        outcome(std::move(event_outcome)),
        returned_to_earlier_tier(returned),
        final_attribution(attribution),
        reason_code(std::move(event_reason_code)) {}

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const DecisionCycleEvent&) const = default;
};

/**
 * @brief Encapsulates advisor contribution state and behavior for this
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
struct AdvisorContribution {
  std::string advisor;
  /**
   * @brief Performs the pause operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `domain::Action action{` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Action action{domain::Action::pause()};
  double raw_score{0.0};
  double normalized_score{0.0};
  double advisor_mean{0.0};
  double advisor_standard_deviation{0.0};
  double relative_support{0.0};
  double weight{1.0};
  double weighted_score{0.0};
  bool viable{true};
  double final_total{0.0};
  std::string explanation;
  std::size_t model_revision_used{0U};

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const AdvisorContribution&) const = default;
};

/**
 * @brief Encapsulates predicted action result state and behavior for this
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
struct PredictedActionResult {
  /**
   * @brief Performs the pause operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `domain::Action action{` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Action action{domain::Action::pause()};
  domain::Pose2D predicted_pose;
  bool viable{false};
  std::string evidence_source;
};

/**
 * @brief Encapsulates decision confidence state and behavior for this
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
struct DecisionConfidence {
  double selected_comment_sum{0.0};
  std::size_t advisor_count{0U};
  double normalized_support_proportion{0.0};
  double action_total_mean{0.0};
  double action_total_standard_deviation{0.0};
  double gamma{0.0};
  double zeta{0.0};
  double lambda{0.0};
  std::string agreement_category{"not_available"};
  std::string support_category{"not_available"};
  double gini_agreement{0.0};
  double standardized_total{0.0};
  double relative_support{0.0};
  std::string category{"not_available"};
};

/**
 * @brief Encapsulates tier three action total state and behavior for this
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
struct TierThreeActionTotal {
  /**
   * @brief Performs the pause operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `domain::Action action{` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Action action{domain::Action::pause()};
  double total{0.0};
  bool viable{true};
  bool scored{false};
  double pre_circumstance_total{0.0};
  double circumstance_multiplier{1.0};
  double post_circumstance_total{0.0};
  double chapter_five_comment_total{0.0};
  std::size_t circumstance_action_evidence{0U};
  double circumstance_action_confidence{0.0};

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const TierThreeActionTotal&) const = default;
};

/**
 * @brief Encapsulates task diagnostic state and behavior for this
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
struct TaskDiagnostic {
  std::uint64_t task_index{0U};
  std::uint64_t decision_count{0U};
  domain::Point2D target;
  std::optional<domain::Point2D> waypoint;

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const TaskDiagnostic&) const = default;
};

/**
 * @brief Encapsulates plan candidate diagnostic state and behavior for this
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
struct PlanCandidateDiagnostic {
  planning::PlanId plan_id = 0U;
  std::string planner;
  planning::PlanFamily family = planning::PlanFamily::Grid;
  planning::ObjectiveCosts raw_costs;
  planning::ObjectiveCosts normalized_costs;
  double summed_score = 0.0;
  bool tied_for_best = false;
  planning::PlannerMetadata metadata;
  std::vector<domain::Point2D> geometry;
  std::vector<planning::PlanStep> typed_steps;
  domain::DependencyRevisions dependency_revisions;
  domain::Revision planner_configuration_revision = 0U;
  planning::PlanningOperatingMode operating_mode{
      planning::PlanningOperatingMode::Mapless};
  bool static_map_contributed{false};
  domain::Revision live_social_revision{0U};
  domain::Revision crowd_density_revision{0U};
  domain::Revision crowd_risk_revision{0U};
  domain::Revision crowd_flow_revision{0U};
  std::string social_input_source{"none"};
  std::string social_prediction_source{"none"};
  std::string social_input_status{"unavailable"};
  bool formation_evidence_participated{false};
};

/**
 * @brief Encapsulates decision result state and behavior for this
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
struct DecisionResult {
  std::uint64_t sequence{0U};
  domain::DecisionId decision_id{0U};
  domain::ActionId action_id{0U};
  domain::ActionId execution_id{0U};
  domain::Pose2D robot_pose;
  navigation::NavigationPhase navigation_phase{
      navigation::NavigationPhase::TargetNavigation};
  std::string configuration_fingerprint;
  std::vector<std::string> component_manifest;
  std::vector<std::string> phase_events;
  std::optional<TaskDiagnostic> task;
  std::vector<domain::Action> candidates;
  std::vector<domain::Action> viable_actions;
  std::vector<PredictedActionResult> predicted_actions;
  /**
   * @brief Performs the pause operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `domain::Action action{` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Action action{domain::Action::pause()};
  DecisionSource source{DecisionSource::SafeStop};
  DecisionTier tier{DecisionTier::SafeStop};
  std::string selected_policy;
  std::vector<Veto> vetoes;
  std::vector<AdvisorContribution> contributions;
  std::vector<TierThreeActionTotal> tier_three_totals;
  bool circumstance_match_available{false};
  std::uint64_t circumstance_id{0U};
  double circumstance_assignment_confidence{0.0};
  std::string circumstance_learning_mode;
  std::string circumstance_model_version;
  std::string circumstance_classifier_version;
  std::string circumstance_weighting_policy{"disabled"};
  bool circumstance_weighting_applied{false};
  bool circumstance_weighting_changed_winner{false};
  std::string circumstance_reason;
  DecisionConfidence decision_confidence;
  std::string tier_three_scoring_policy;
  std::string tier_three_tie_policy;
  double tier_three_tie_tolerance{0.0};
  std::uint32_t tier_three_random_seed{0U};
  std::vector<domain::Action> tier_three_tie_candidates;
  bool tier_three_random_selection_used{false};
  std::optional<std::size_t> tier_three_random_selection_index;
  std::vector<DecisionCycleEvent> decision_cycle;
  std::optional<std::string> planner;
  std::optional<planning::PlanId> plan_id;
  domain::Revision plan_revision{0U};
  std::optional<planning::PlanFamily> plan_family;
  std::optional<std::string> enforcer_mode;
  std::optional<std::size_t> active_plan_step;
  std::optional<domain::Point2D> operational_target;
  std::string enforcer_reason;
  std::string plan_status;
  std::vector<std::string> plan_execution_events;
  std::vector<PlanCandidateDiagnostic> planning_candidates;
  std::optional<std::uint64_t> planning_episode_id;
  std::vector<std::string> planning_tie_candidates;
  std::string planning_tie_break_reason;
  double decision_latency_s{0.0};
  double planning_latency_s{0.0};
  double model_update_cost_s{0.0};
  std::uint64_t allocation_count{0U};
  std::uint64_t allocation_bytes{0U};
  std::uint64_t covered_cells{0U};
  ActionOutcome action_outcome{ActionOutcome::Pending};
  std::string action_lifecycle_status{"selected"};
  std::optional<domain::ActionExecutionResult> execution_result;
  double action_duration_s{0.0};
  double action_progress{0.0};
  double action_target{0.0};
  std::string outcome_detail;
  std::vector<std::string> source_provenance;
  domain::Revision live_social_revision{0U};
  domain::Revision crowd_density_revision{0U};
  domain::Revision crowd_risk_revision{0U};
  domain::Revision crowd_flow_revision{0U};
  std::string social_input_source{"none"};
  std::string social_prediction_source{"none"};
  std::string social_input_status{"unavailable"};
  bool formation_evidence_available{false};
  bool formation_evidence_participated{false};
};

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p source: Supplies source input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(DecisionSource source) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p tier: Supplies tier input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(DecisionTier tier) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p outcome: Supplies outcome input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(ActionOutcome outcome) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p kind: Supplies kind input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(RejectionKind kind) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p category: Supplies category input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(VetoCategory category) noexcept;

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_DECISION_RESULT_HPP
