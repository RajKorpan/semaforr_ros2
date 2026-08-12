#ifndef SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP
#define SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP

#include <cstdint>
#include <deque>
#include <memory>
#include <map>
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

class NavigationEngine {
 public:
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

  void observe(const domain::RobotObservation& observation);
  DecisionResult decide();
  DecisionResult decide(const domain::RobotObservation& observation);
  domain::FeedbackDisposition onActionStarted(
      const domain::ActionStartedEvent& event);
  domain::FeedbackDisposition onActionProgress(
      const domain::ActionProgressEvent& event);
  domain::FeedbackDisposition onActionCompleted(
      domain::ActionExecutionResult result);
  domain::FeedbackDisposition onActionFailed(
      domain::ActionExecutionResult result);
  domain::FeedbackDisposition onActionCancelled(
      domain::ActionExecutionResult result);
  domain::FeedbackDisposition onControllerRestart(
      domain::ExecutionTimestamp when, const domain::Pose2D& pose);
  const domain::SelectedActionRecord* pendingAction() const noexcept;
  const std::vector<std::string>& executionDiagnostics() const noexcept;
  const DecisionResult* decisionTrace(domain::DecisionId id) const noexcept;
  const DecisionResult* actionTrace(domain::ActionId id) const noexcept;
  const DecisionResult* latestDecisionTrace() const noexcept;
  bool missionComplete() noexcept;
  navigation::NavigationPhase phase() const noexcept;

 private:
  std::vector<domain::Action> candidates() const;
  std::optional<std::string> preparePlan(MissionStep step);
  void finishInitialExploration();
  void registerSelection(DecisionResult& result,
                         spatial::NavigationEpisode episode);
  domain::FeedbackDisposition acceptTerminal(
      domain::ActionExecutionResult result);
  bool terminalSeen(domain::ActionId action_id) const noexcept;
  std::optional<domain::Action> enforcerAction(
      std::span<const domain::Action> viable_actions) const;
  PlanEnforcementResult enforceActivePlan(
      std::span<const domain::Action> viable_actions);
  void appendCycleDiagnostics(DecisionResult&) const;
  void retainDecisionTrace(const DecisionResult& result);

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
