/**
 * @file navigation_engine.hpp
 * @brief Navigation engine responsibilities.
 *
 * @details This file defines navigation engine behavior for tiered decision
 * making and action arbitration. It centers on `NavigationEngine`,
 * `PendingExecution`. Its package-relative location is
 * `include/semaforr/decision/navigation_engine.hpp`.
 */
#ifndef SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP
#define SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/decision/enforcer.hpp>
#include <semaforr/decision/hard_safety_filter.hpp>
#include <semaforr/decision/mission_manager.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/domain/world_model.hpp>
#include <semaforr/exploration/exploration_coordinator.hpp>
#include <semaforr/navigation/navigation_phase.hpp>
#include <semaforr/planning/planning_coordinator.hpp>
#include <semaforr/planning/reactive_planner.hpp>
#include <semaforr/social/crowd_field_learner.hpp>
#include <semaforr/spatial/spatial_learning_coordinator.hpp>
#include <span>
#include <string>
#include <vector>

namespace semaforr::decision {

/**
 * @brief Encapsulates navigation engine state and behavior for this
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
class NavigationEngine {
 public:
  /**
   * @brief Performs the navigation engine operation for this subsystem.
   *
   * Arguments:
   * - @p world: Supplies world input to the operation.
   * - @p action_space: Supplies action space input to the operation.
   * - @p decisions: Supplies decisions input to the operation.
   * - @p mission: Supplies mission input to the operation.
   * - @p planning: Supplies planning input to the operation.
   * - @p learning: Supplies learning input to the operation.
   * - @p crowd_learning: Supplies crowd learning input to the operation.
   * - @p goal_tolerance: Supplies goal tolerance input to the operation.
   * - @p hard_safety: Supplies hard safety input to the operation.
   * - @p phases: Supplies phases input to the operation.
   * - @p configuration_fingerprint: Supplies configuration fingerprint
   * input to the operation.
   * - @p component_manifest: Supplies component manifest input to the
   * operation.
   * - @p reactive_planners: Supplies reactive planners input to the
   * operation.
   * - @p low_level_exploration_enabled: Supplies low level exploration
   * enabled input to the operation.
   * - @p enforcer_enabled: Supplies enforcer enabled input to the
   * operation.
   * - @p hle_configuration: Supplies hle configuration input to the
   * operation.
   * - @p low_level_explorer: Supplies low level explorer input to the
   * operation.
   * - @p plan_operationalizer: Supplies plan operationalizer input to the
   * operation.
   * - @p traversability: Supplies traversability input to the operation.
   * - @p maximum_planning_attempts_per_task: Supplies maximum planning
   * attempts per task input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  NavigationEngine(
      domain::WorldModel& world, const domain::ActionSpace& action_space,
      DecisionCoordinator& decisions, MissionManager& mission,
      planning::PlanningCoordinator& planning,
      spatial::SpatialLearningCoordinator& learning,
      social::CrowdFieldLearner* crowd_learning = nullptr,
      domain::Distance goal_tolerance = domain::Distance(0.5),
      HardSafetyFilter* hard_safety = nullptr,
      navigation::NavigationPhaseCoordinator* phases = nullptr,
      std::string configuration_fingerprint = {},
      std::vector<std::string> component_manifest = {},
      std::vector<std::unique_ptr<planning::ReactivePlanner>>
          reactive_planners = {},
      bool low_level_exploration_enabled = true, bool enforcer_enabled = true,
      exploration::HighLevelExplorationConfiguration hle_configuration = {},
      std::unique_ptr<planning::ReactivePlanner> low_level_explorer = nullptr,
      std::unique_ptr<PlanOperationalizer> plan_operationalizer = nullptr,
      planning::TraversabilityConfiguration traversability = {},
      std::size_t maximum_planning_attempts_per_task = 3U);

  /**
   * @brief Processes package content for this subsystem.
   *
   * Arguments:
   * - @p observation: Supplies observation input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void observe(const domain::RobotObservation& observation);
  /**
   * @brief Selects package content for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `DecisionResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  DecisionResult decide();
  /**
   * @brief Selects package content for this subsystem.
   *
   * Arguments:
   * - @p observation: Supplies observation input to the operation.
   *
   * Returns:
   * - `DecisionResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  DecisionResult decide(const domain::RobotObservation& observation);
  /**
   * @brief Performs the on action started operation for this subsystem.
   *
   * Arguments:
   * - @p event: Supplies event input to the operation.
   *
   * Returns:
   * - `domain::FeedbackDisposition` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::FeedbackDisposition onActionStarted(
      const domain::ActionStartedEvent& event);
  /**
   * @brief Performs the on action progress operation for this subsystem.
   *
   * Arguments:
   * - @p event: Supplies event input to the operation.
   *
   * Returns:
   * - `domain::FeedbackDisposition` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::FeedbackDisposition onActionProgress(
      const domain::ActionProgressEvent& event);
  /**
   * @brief Performs the on action completed operation for this subsystem.
   *
   * Arguments:
   * - @p result: Supplies result input to the operation.
   *
   * Returns:
   * - `domain::FeedbackDisposition` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::FeedbackDisposition onActionCompleted(
      domain::ActionExecutionResult result);
  /**
   * @brief Performs the on action failed operation for this subsystem.
   *
   * Arguments:
   * - @p result: Supplies result input to the operation.
   *
   * Returns:
   * - `domain::FeedbackDisposition` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::FeedbackDisposition onActionFailed(
      domain::ActionExecutionResult result);
  /**
   * @brief Performs the on action cancelled operation for this subsystem.
   *
   * Arguments:
   * - @p result: Supplies result input to the operation.
   *
   * Returns:
   * - `domain::FeedbackDisposition` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::FeedbackDisposition onActionCancelled(
      domain::ActionExecutionResult result);
  /**
   * @brief Performs the on controller restart operation for this subsystem.
   *
   * Arguments:
   * - @p when: Supplies when input to the operation.
   * - @p pose: Supplies pose input to the operation.
   *
   * Returns:
   * - `domain::FeedbackDisposition` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::FeedbackDisposition onControllerRestart(
      domain::ExecutionTimestamp when, const domain::Pose2D& pose);
  /**
   * @brief Performs the pending action operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const domain::SelectedActionRecord*` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const domain::SelectedActionRecord* pendingAction() const noexcept;
  /**
   * @brief Performs the execution diagnostics operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<std::string>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<std::string>& executionDiagnostics() const noexcept;
  /**
   * @brief Performs the decision trace operation for this subsystem.
   *
   * Arguments:
   * - @p id: Supplies id input to the operation.
   *
   * Returns:
   * - `const DecisionResult*` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const DecisionResult* decisionTrace(domain::DecisionId id) const noexcept;
  /**
   * @brief Performs the action trace operation for this subsystem.
   *
   * Arguments:
   * - @p id: Supplies id input to the operation.
   *
   * Returns:
   * - `const DecisionResult*` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const DecisionResult* actionTrace(domain::ActionId id) const noexcept;
  /**
   * @brief Performs the latest decision trace operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const DecisionResult*` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const DecisionResult* latestDecisionTrace() const noexcept;
  /**
   * @brief Performs the mission complete operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool missionComplete() noexcept;
  /**
   * @brief Performs the phase operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `navigation::NavigationPhase` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  navigation::NavigationPhase phase() const noexcept;

 private:
  /**
   * @brief Performs the candidates operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<domain::Action>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<domain::Action> candidates() const;
  /**
   * @brief Performs the prepare plan operation for this subsystem.
   *
   * Arguments:
   * - @p step: Supplies step input to the operation.
   *
   * Returns:
   * - `std::optional<std::string>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<std::string> preparePlan(MissionStep step);
  /**
   * @brief Performs the finish initial exploration operation for this
   * subsystem.
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
  void finishInitialExploration();
  /**
   * @brief Registers selection for this subsystem.
   *
   * Arguments:
   * - @p result: Supplies result input to the operation.
   * - @p episode: Supplies episode input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void registerSelection(DecisionResult& result,
                         spatial::NavigationEpisode episode);
  /**
   * @brief Performs the accept terminal operation for this subsystem.
   *
   * Arguments:
   * - @p result: Supplies result input to the operation.
   *
   * Returns:
   * - `domain::FeedbackDisposition` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::FeedbackDisposition acceptTerminal(
      domain::ActionExecutionResult result);
  /**
   * @brief Performs the terminal seen operation for this subsystem.
   *
   * Arguments:
   * - @p action_id: Supplies action id input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool terminalSeen(domain::ActionId action_id) const noexcept;
  /**
   * @brief Performs the enforcer action operation for this subsystem.
   *
   * Arguments:
   * - @p viable_actions: Supplies viable actions input to the operation.
   *
   * Returns:
   * - `std::optional<domain::Action>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<domain::Action> enforcerAction(
      std::span<const domain::Action> viable_actions) const;
  /**
   * @brief Performs the enforce active plan operation for this subsystem.
   *
   * Arguments:
   * - @p viable_actions: Supplies viable actions input to the operation.
   *
   * Returns:
   * - `PlanEnforcementResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PlanEnforcementResult enforceActivePlan(
      std::span<const domain::Action> viable_actions);
  /**
   * @brief Performs the append cycle diagnostics operation for this
   * subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void appendCycleDiagnostics(DecisionResult&) const;
  /**
   * @brief Performs the retain decision trace operation for this subsystem.
   *
   * Arguments:
   * - @p result: Supplies result input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void retainDecisionTrace(const DecisionResult& result);

  /**
   * @brief Encapsulates pending execution state and behavior for this
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
  struct PendingExecution {
    domain::SelectedActionRecord selection;
    spatial::NavigationEpisode episode;
    bool started{false};
    std::optional<domain::ActionStartedEvent> start;
    std::optional<domain::ActionProgressEvent> progress;
  };

  domain::WorldModel& world_;
  const domain::ActionSpace& action_space_;
  DecisionCoordinator& decisions_;
  MissionManager& mission_;
  planning::PlanningCoordinator& planning_;
  spatial::SpatialLearningCoordinator& learning_;
  social::CrowdFieldLearner* crowd_learning_;
  HardSafetyFilter* hard_safety_;
  navigation::NavigationPhaseCoordinator owned_phases_;
  navigation::NavigationPhaseCoordinator* phases_;
  std::string configuration_fingerprint_;
  std::vector<std::string> component_manifest_;
  exploration::ExplorationCoordinator exploration_;
  std::unique_ptr<PlanOperationalizer> enforcer_;
  planning::ReactivePlannerCoordinator reactive_;
  std::unique_ptr<planning::ReactivePlanner> lle_;
  bool low_level_exploration_enabled_;
  bool enforcer_enabled_;
  planning::TraversabilityConfiguration traversability_;
  domain::Distance goal_tolerance_;
  std::optional<domain::RobotObservation> observation_;
  std::optional<planning::HierarchicalPlan> active_hierarchy_;
  std::optional<planning::SelectedPlan::SelectionEvidence>
      active_selection_evidence_;
  std::optional<domain::TaskId> hierarchy_task_;
  std::vector<std::string> pending_phase_events_;
  std::uint64_t decision_sequence_{0U};
  domain::ActionId action_sequence_{0U};
  domain::PathId path_sequence_{0U};
  std::optional<PendingExecution> pending_execution_;
  std::deque<domain::ActionId> terminal_action_ids_;
  std::vector<std::string> execution_diagnostics_;
  std::map<domain::DecisionId, DecisionResult> explanation_history_;
  std::map<domain::ActionId, domain::DecisionId> action_to_decision_;
  bool finalize_initial_exploration_after_action_{false};
  std::size_t maximum_planning_attempts_per_task_{3U};
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP
