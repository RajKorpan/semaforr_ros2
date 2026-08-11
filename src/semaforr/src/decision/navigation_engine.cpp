#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <semaforr/decision/navigation_engine.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <semaforr/spatial/coverage.hpp>
#include <limits>
#include <stdexcept>
#include <utility>

namespace semaforr::decision {

NavigationEngine::NavigationEngine(
    domain::WorldModel& world, const domain::ActionSpace& action_space,
    DecisionCoordinator& decisions, MissionManager& mission,
    planning::PlanningCoordinator& planning,
    spatial::SpatialLearningCoordinator& learning,
    social::CrowdFieldLearner* crowd_learning, domain::Distance goal_tolerance,
    HardSafetyFilter* hard_safety,
    navigation::NavigationPhaseCoordinator* phases,
    std::string configuration_fingerprint,
    std::vector<std::string> component_manifest,
    std::vector<std::unique_ptr<planning::ReactivePlanner>> reactive_planners,
    bool low_level_exploration_enabled, bool enforcer_enabled,
    exploration::HighLevelExplorationConfiguration hle_configuration,
    std::unique_ptr<planning::ReactivePlanner> low_level_explorer,
    std::unique_ptr<PlanOperationalizer> plan_operationalizer,
    planning::TraversabilityConfiguration traversability)
    : world_(world),
      action_space_(action_space),
      decisions_(decisions),
      mission_(mission),
      planning_(planning),
      learning_(learning),
      crowd_learning_(crowd_learning),
      hard_safety_(hard_safety),
      phases_(phases ? phases : &owned_phases_),
      configuration_fingerprint_(std::move(configuration_fingerprint)),
      component_manifest_(std::move(component_manifest)),
      exploration_(std::move(hle_configuration)),
      enforcer_(plan_operationalizer ? std::move(plan_operationalizer)
                                     : std::make_unique<Enforcer>()),
      reactive_(std::move(reactive_planners)),
      lle_(low_level_explorer ? std::move(low_level_explorer)
                              : std::make_unique<planning::LowLevelExplorer>()),
      low_level_exploration_enabled_(low_level_exploration_enabled),
      enforcer_enabled_(enforcer_enabled),
      traversability_(std::move(traversability)),
      goal_tolerance_(goal_tolerance) {
  exploration_.setModelFinalizer([this] {
    learning_.finalizeInitialExploration();
    learning_.applyTo(world_.spatial);
  });
}

std::vector<domain::Action> NavigationEngine::candidates() const {
  std::vector<domain::Action> actions;
  actions.reserve(1U + action_space_.move_distances_m().size() +
                  2U * action_space_.rotation_angles_rad().size());
  actions.push_back(domain::Action::pause());
  for (std::size_t index = 1U; index <= action_space_.move_distances_m().size();
       ++index) {
    actions.emplace_back(domain::ActionType::Forward, index);
  }
  for (std::size_t index = 1U;
       index <= action_space_.rotation_angles_rad().size(); ++index) {
    actions.emplace_back(domain::ActionType::TurnRight, index);
    actions.emplace_back(domain::ActionType::TurnLeft, index);
  }
  return actions;
}

void NavigationEngine::observe(const domain::RobotObservation& observation) {
  observation.laser.validate();
  observation_ = observation;
  auto phase_update = phases_->observe(observation, world_);
  pending_phase_events_.insert(pending_phase_events_.end(),
                               phase_update.events.begin(),
                               phase_update.events.end());
  if (world_.recovery.confined &&
      !world_.navigation_history.entries().empty() &&
      domain::distance(observation.pose.position,
                       world_.navigation_history.entries().back().pose.position)
              .meters() > 0.1)
    world_.recovery.confined = false;
  world_.robot.pose = observation.pose;
  world_.robot.laser = observation.laser;
  world_.robot.observed_at = observation.observed_at;
  world_.observation_history.record(
      {observation.pose, observation.observed_at});
  spatial::NavigationEpisode sensor_episode;
  sensor_episode.sequence = world_.observation_history.entries().size();
  sensor_episode.observation = observation;
  sensor_episode.active_task = world_.mission.active()
                                   ? std::optional<domain::TaskId>(
                                         world_.mission.active()->id)
                                   : std::nullopt;
  sensor_episode.initial_exploration =
      phases_->phase() == navigation::NavigationPhase::InitialExploration;
  sensor_episode.active_target =
      world_.mission.active()
          ? std::optional<domain::Point2D>(world_.mission.active()->target)
          : std::nullopt;
  learning_.observeSensor(std::move(sensor_episode));
  learning_.applyTo(world_.spatial);
  if (observation.crowd) {
    world_.crowd.update(*observation.crowd);
    if (crowd_learning_ &&
        crowd_learning_->observe(observation.pose, observation.laser,
                                 *observation.crowd)) {
      world_.crowd.setLearned(crowd_learning_->snapshot());
    }
  } else {
    world_.crowd.clearCurrent();
  }

  const bool had_active_task = world_.mission.active().has_value();
  if (world_.mission.active() &&
      domain::goalReached(observation.pose, world_.mission.active()->target,
                          goal_tolerance_)) {
    if (mission_.completeActiveTask())
      pending_phase_events_.push_back("target_completed");
  } else {
    mission_.advanceWaypoint(observation.pose, goal_tolerance_);
  }
  if (had_active_task && !world_.mission.active()) {
    active_hierarchy_.reset();
    hierarchy_task_.reset();
    learning_.finalizeTarget();
    learning_.applyTo(world_.spatial);
  }
}

std::optional<std::string> NavigationEngine::preparePlan(MissionStep step) {
  if (!world_.mission.active()) {
    return std::nullopt;
  }
  if (step == MissionStep::Ready && !world_.mission.active()->plan.empty()) {
    return std::nullopt;
  }
  if (enforcer_enabled_ && active_hierarchy_ && hierarchy_task_ &&
      *hierarchy_task_ == world_.mission.active()->id) {
    const auto next = enforcer_->operationalizeNext(
        *active_hierarchy_, world_.spatial, world_.robot.pose, goal_tolerance_);
    if (next) {
      mission_.installPlan({*next});
      mission_.advanceWaypoint(world_.robot.pose, goal_tolerance_);
      return active_hierarchy_->planner;
    }
    active_hierarchy_.reset();
    hierarchy_task_.reset();
  }
  auto traversal = traversability_;
  traversal.current_sensor_origin = world_.robot.pose.position;
  traversal.current_sensor_range_m =
      world_.robot.laser ? world_.robot.laser->maximum_range.meters() : 0.0;
  const auto selected = planning_.selectPlan(
      {world_.robot.pose, world_.mission.active()->target, &world_.spatial,
       &world_.crowd, world_.static_map, traversal});
  if (!selected) {
    mission_.installPlan({world_.mission.active()->target});
    return std::nullopt;
  }
  if (enforcer_enabled_ && selected->result.hierarchical) {
    active_hierarchy_ = *selected->result.hierarchical;
    hierarchy_task_ = world_.mission.active()->id;
    const auto next = enforcer_->operationalizeNext(
        *active_hierarchy_, world_.spatial, world_.robot.pose, goal_tolerance_);
    mission_.installPlan(next ? std::vector<domain::Point2D>{*next}
                              : selected->result.path);
  } else {
    mission_.installPlan(selected->result.path);
  }
  mission_.advanceWaypoint(world_.robot.pose, goal_tolerance_);
  return selected->planner;
}

void NavigationEngine::finishInitialExploration() {
  world_.spatial.unfinished_hle_candidates.clear();
  for (const auto& candidate : exploration_.unfinishedCandidates()) {
    const double heading = candidate.heading.radians();
    const double distance = candidate.clearance.meters();
    world_.spatial.unfinished_hle_candidates.push_back(
        {candidate.id,
         candidate.start,
         {candidate.start.x_m + distance * std::cos(heading),
          candidate.start.y_m + distance * std::sin(heading)}});
  }
  exploration_.finish();
}

DecisionResult NavigationEngine::decide() {
  const auto decision_started = std::chrono::steady_clock::now();
  const auto finalize_measurements = [&](DecisionResult& result) {
    result.decision_latency_s = std::chrono::duration<double>(
                                    std::chrono::steady_clock::now() -
                                    decision_started)
                                    .count();
    result.covered_cells = static_cast<std::uint64_t>(
        spatial::representedCoverageCells(world_.spatial));
  };
  if (!observation_) {
    throw std::logic_error("navigation decision requires an observation");
  }
  if (pending_execution_) {
    execution_diagnostics_.push_back(
        "missing_terminal_feedback:action=" +
        std::to_string(pending_execution_->selection.action_id));
    throw std::logic_error(
        "cannot select another action before terminal execution feedback");
  }
  if (decision_sequence_ == std::numeric_limits<std::uint64_t>::max())
    throw std::overflow_error("decision identifier space exhausted");
  navigation::PhaseDecision dispatch = phases_->next(world_);
  if (dispatch.phase == navigation::NavigationPhase::InitialExploration &&
      phases_->explorationTimeLimitReached()) {
    finishInitialExploration();
    phases_->completeInitialExploration();
    auto completed = phases_->takeEvents();
    pending_phase_events_.insert(pending_phase_events_.end(), completed.begin(),
                                 completed.end());
    dispatch = phases_->next(world_);
  }
  if (dispatch.phase == navigation::NavigationPhase::MissionComplete) {
    DecisionResult result;
    result.sequence = ++decision_sequence_;
    result.robot_pose = world_.robot.pose;
    result.navigation_phase = dispatch.phase;
    result.configuration_fingerprint = configuration_fingerprint_;
    result.component_manifest = component_manifest_;
    result.phase_events.swap(pending_phase_events_);
    result.action = domain::Action::pause();
    result.source = DecisionSource::SafeStop;
    result.tier = DecisionTier::SafeStop;
    result.selected_policy = "mission_complete_safe_stop";
    finalize_measurements(result);
    return result;
  }
  if (dispatch.phase == navigation::NavigationPhase::InitialExploration) {
    exploration::ExplorationUpdate exploration =
        exploration_.decide(*observation_, action_space_);
    DecisionResult result;
    result.sequence = ++decision_sequence_;
    result.robot_pose = world_.robot.pose;
    result.navigation_phase = phases_->phase();
    result.configuration_fingerprint = configuration_fingerprint_;
    result.component_manifest = component_manifest_;
    result.phase_events.swap(pending_phase_events_);
    result.action = exploration.decision.action;
    if (hard_safety_) {
      const std::array<domain::Action, 1U> exploration_candidate{result.action};
      auto filtered =
          hard_safety_->filter(DecisionContext{world_}, exploration_candidate);
      result.vetoes = std::move(filtered.vetoes);
      if (filtered.safe_actions.empty())
        result.action = domain::Action::pause();
    }
    result.source = DecisionSource::Exploration;
    result.tier = DecisionTier::Exploration;
    result.selected_policy =
        "hle:" + std::string(exploration::toString(exploration.decision.state));
    result.phase_events.insert(result.phase_events.end(),
                               exploration.events.begin(),
                               exploration.events.end());
    const auto model_update_started = std::chrono::steady_clock::now();
    spatial::NavigationEpisode episode;
    episode.observation = *observation_;
    episode.selected_action = result.action;
    episode.initial_exploration = true;
    episode.viable_actions = {result.action};
    episode.move_distances_m = action_space_.move_distances_m();
    episode.rotation_angles_rad = action_space_.rotation_angles_rad();
    registerSelection(result, std::move(episode));
    if (phases_->explorationBudgetReached()) {
      finalize_initial_exploration_after_action_ = true;
    }
    result.model_update_cost_s = std::chrono::duration<double>(
                                     std::chrono::steady_clock::now() -
                                     model_update_started)
                                     .count();
    finalize_measurements(result);
    return result;
  }
  const std::size_t skipped_before = world_.mission.skipped().size();
  const MissionStep mission_step = mission_.prepareDecision();
  if (world_.mission.skipped().size() > skipped_before)
    pending_phase_events_.push_back("target_skipped");
  if (mission_step == MissionStep::Complete) {
    phases_->completeMission();
    DecisionResult result;
    result.navigation_phase = phases_->phase();
    result.configuration_fingerprint = configuration_fingerprint_;
    result.component_manifest = component_manifest_;
    result.phase_events.swap(pending_phase_events_);
    finalize_measurements(result);
    return result;
  }
  const planning::ReactiveResult lle =
      low_level_exploration_enabled_ ? lle_->evaluate({world_, action_space_})
                                     : planning::ReactiveResult{};
  if (lle.status == planning::ReactiveStatus::RequestReplan &&
      world_.mission.active()) {
    mission_.clearPlan();
    planning_.clearCache();
    world_.recovery.confined = true;
  }
  const auto planning_started = std::chrono::steady_clock::now();
  const std::optional<std::string> selected_planner =
      lle.status == planning::ReactiveStatus::Action
          ? std::nullopt
          : preparePlan(mission_step);
  const double planning_latency_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    planning_started)
          .count();
  const auto available = candidates();
  std::vector<domain::Action> decision_candidates = available;
  std::vector<Veto> hard_vetoes;
  if (hard_safety_) {
    auto filtered = hard_safety_->filter(DecisionContext{world_}, available);
    decision_candidates = std::move(filtered.safe_actions);
    hard_vetoes = std::move(filtered.vetoes);
  }
  const planning::ReactiveResult reactive =
      reactive_.evaluate({world_, action_space_});
  DecisionResult result;
  if (lle.status == planning::ReactiveStatus::Action && lle.action &&
      std::find(decision_candidates.begin(), decision_candidates.end(),
                *lle.action) != decision_candidates.end()) {
    result.action = *lle.action;
    result.source = DecisionSource::MandatoryRule;
    result.tier = DecisionTier::TierOne;
    result.selected_policy = "reactive:LLE";
  } else if (reactive.status == planning::ReactiveStatus::Action &&
             reactive.action &&
             std::find(decision_candidates.begin(), decision_candidates.end(),
                       *reactive.action) != decision_candidates.end()) {
    result.action = *reactive.action;
    result.source = DecisionSource::MandatoryRule;
    result.tier = DecisionTier::TierOne;
    result.selected_policy = "reactive:" + reactive.planner;
  } else {
    result = decisions_.decide(DecisionContext{world_}, decision_candidates);
  }
  result.vetoes.insert(result.vetoes.begin(),
                       std::make_move_iterator(hard_vetoes.begin()),
                       std::make_move_iterator(hard_vetoes.end()));
  result.sequence = ++decision_sequence_;
  result.robot_pose = world_.robot.pose;
  result.navigation_phase = phases_->phase();
  result.configuration_fingerprint = configuration_fingerprint_;
  result.component_manifest = component_manifest_;
  result.phase_events.swap(pending_phase_events_);
  result.candidates = decision_candidates;
  result.planner = selected_planner;
  result.planning_latency_s = planning_latency_s;
  if (world_.mission.active()) {
    result.task = TaskDiagnostic{
        static_cast<std::uint64_t>(world_.mission.active()->id),
        static_cast<std::uint64_t>(world_.mission.decisions_for_active() + 1U),
        world_.mission.active()->target, world_.mission.active()->waypoint()};
  }
  const std::optional<domain::TaskId> active_task =
      world_.mission.active()
          ? std::optional<domain::TaskId>(world_.mission.active()->id)
          : std::nullopt;
  const auto model_update_started = std::chrono::steady_clock::now();
  spatial::NavigationEpisode episode;
  episode.observation = *observation_;
  episode.selected_action = result.action;
  episode.active_task = active_task;
  episode.task_started = mission_step == MissionStep::ActivatedTask ||
                         mission_step == MissionStep::SkippedTask;
  episode.active_target =
      world_.mission.active()
          ? std::optional<domain::Point2D>(world_.mission.active()->target)
          : std::nullopt;
  episode.viable_actions = decision_candidates;
  episode.move_distances_m = action_space_.move_distances_m();
  episode.rotation_angles_rad = action_space_.rotation_angles_rad();
  registerSelection(result, std::move(episode));
  result.model_update_cost_s = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() -
                                   model_update_started)
                                   .count();
  mission_.recordDecision();
  (void)planning_;
  finalize_measurements(result);
  return result;
}

DecisionResult NavigationEngine::decide(
    const domain::RobotObservation& observation) {
  observe(observation);
  return decide();
}

bool NavigationEngine::missionComplete() noexcept {
  if (mission_.complete()) phases_->completeMission();
  return phases_->phase() == navigation::NavigationPhase::MissionComplete;
}

navigation::NavigationPhase NavigationEngine::phase() const noexcept {
  return phases_->phase();
}

void NavigationEngine::registerSelection(DecisionResult& result,
                                         spatial::NavigationEpisode episode) {
  if (action_sequence_ == std::numeric_limits<domain::ActionId>::max())
    throw std::overflow_error("action identifier space exhausted");
  result.decision_id = result.sequence;
  result.action_id = ++action_sequence_;
  domain::SelectedActionRecord selection;
  selection.decision_id = result.decision_id;
  selection.action_id = result.action_id;
  selection.task_id = episode.active_task;
  selection.selected_at = std::chrono::steady_clock::now();
  selection.expected_start = world_.robot.pose;
  selection.action = result.action;
  selection.selected_tier = std::string(toString(result.tier));
  selection.provenance = result.selected_policy;
  if (result.planner) selection.provenance += ":" + *result.planner;
  const std::size_t magnitude = result.action.magnitude_index();
  if (magnitude > 0U && result.action.type() == domain::ActionType::Forward &&
      magnitude <= action_space_.move_distances_m().size())
    selection.intended_distance_m =
        action_space_.move_distances_m()[magnitude - 1U];
  if (magnitude > 0U &&
      (result.action.type() == domain::ActionType::TurnLeft ||
       result.action.type() == domain::ActionType::TurnRight) &&
      magnitude <= action_space_.rotation_angles_rad().size())
    selection.intended_rotation_rad =
        action_space_.rotation_angles_rad()[magnitude - 1U];
  episode.sequence = static_cast<std::size_t>(selection.decision_id);
  episode.selection = selection;
  episode.event = spatial::LearningEvent::DecisionSelected;
  world_.decision_history.record(selection);
  learning_.observeDecision(episode);
  pending_execution_ = PendingExecution{selection, std::move(episode), false,
                                        std::nullopt, std::nullopt};
}

bool NavigationEngine::terminalSeen(domain::ActionId action_id) const noexcept {
  return std::find(terminal_action_ids_.begin(), terminal_action_ids_.end(),
                   action_id) != terminal_action_ids_.end();
}

const domain::SelectedActionRecord* NavigationEngine::pendingAction() const
    noexcept {
  return pending_execution_ ? &pending_execution_->selection : nullptr;
}

const std::vector<std::string>& NavigationEngine::executionDiagnostics() const
    noexcept {
  return execution_diagnostics_;
}

domain::FeedbackDisposition NavigationEngine::onActionStarted(
    const domain::ActionStartedEvent& event) {
  if (terminalSeen(event.action_id)) return domain::FeedbackDisposition::Duplicate;
  if (!pending_execution_) return domain::FeedbackDisposition::UnknownAction;
  if (event.action_id != pending_execution_->selection.action_id)
    return event.decision_id != pending_execution_->selection.decision_id
               ? domain::FeedbackDisposition::StaleDecision
               : domain::FeedbackDisposition::UnknownAction;
  if (event.decision_id != pending_execution_->selection.decision_id)
    return domain::FeedbackDisposition::StaleDecision;
  if (pending_execution_->started)
    return domain::FeedbackDisposition::AlreadyStarted;
  pending_execution_->started = true;
  pending_execution_->start = event;
  world_.command_history.record(event);
  auto episode = pending_execution_->episode;
  learning_.observeActionStarted(std::move(episode));
  return domain::FeedbackDisposition::Accepted;
}

domain::FeedbackDisposition NavigationEngine::onActionProgress(
    const domain::ActionProgressEvent& event) {
  if (terminalSeen(event.action_id)) return domain::FeedbackDisposition::Duplicate;
  if (!pending_execution_) return domain::FeedbackDisposition::UnknownAction;
  if (event.action_id != pending_execution_->selection.action_id)
    return domain::FeedbackDisposition::UnknownAction;
  if (event.decision_id != pending_execution_->selection.decision_id)
    return domain::FeedbackDisposition::StaleDecision;
  if (!pending_execution_->started)
    return domain::FeedbackDisposition::NotStarted;
  pending_execution_->progress = event;
  auto episode = pending_execution_->episode;
  learning_.observeActionProgress(std::move(episode));
  return domain::FeedbackDisposition::Accepted;
}

domain::FeedbackDisposition NavigationEngine::acceptTerminal(
    domain::ActionExecutionResult result) {
  if (terminalSeen(result.action_id)) return domain::FeedbackDisposition::Duplicate;
  if (!pending_execution_) return domain::FeedbackDisposition::UnknownAction;
  if (result.action_id != pending_execution_->selection.action_id)
    return domain::FeedbackDisposition::UnknownAction;
  if (result.decision_id != pending_execution_->selection.decision_id)
    return domain::FeedbackDisposition::StaleDecision;
  if (result.task_id != pending_execution_->selection.task_id)
    return domain::FeedbackDisposition::TaskMismatch;
  if (!pending_execution_->started && result.successful())
    return domain::FeedbackDisposition::NotStarted;

  if (pending_execution_->start) {
    result.started_at = pending_execution_->start->started_at;
    result.start_pose = pending_execution_->start->start_pose;
  } else {
    result.start_pose = pending_execution_->selection.expected_start;
  }
  world_.execution_history.record(result);
  domain::NavigationHistoryEntry history{
      result.final_pose, pending_execution_->episode.observation.laser,
      pending_execution_->selection.action, result.task_id};
  history.decision_id = result.decision_id;
  history.action_id = result.action_id;
  history.execution_status = result.status;
  history.distance_achieved_m = result.distance_achieved_m;
  history.rotation_achieved_rad = result.rotation_achieved_rad;
  world_.navigation_history.record(history);
  if (result.successful()) world_.completed_path_history.record(history);

  auto episode = pending_execution_->episode;
  episode.observation.pose = result.final_pose;
  episode.execution_result = result;
  const auto model_update_started = std::chrono::steady_clock::now();
  learning_.observeActionTerminal(std::move(episode));
  learning_.applyTo(world_.spatial);
  if (finalize_initial_exploration_after_action_) {
    finishInitialExploration();
    phases_->completeInitialExploration();
    auto completed = phases_->takeEvents();
    pending_phase_events_.insert(pending_phase_events_.end(), completed.begin(),
                                 completed.end());
    finalize_initial_exploration_after_action_ = false;
  }
  execution_diagnostics_.push_back(
      "terminal_feedback:action=" + std::to_string(result.action_id) +
      ",status=" + std::string(domain::toString(result.status)) +
      ",learning_s=" +
      std::to_string(std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - model_update_started)
                         .count()));
  terminal_action_ids_.push_back(result.action_id);
  if (terminal_action_ids_.size() > 4096U) terminal_action_ids_.pop_front();
  pending_execution_.reset();
  return domain::FeedbackDisposition::Accepted;
}

domain::FeedbackDisposition NavigationEngine::onActionCompleted(
    domain::ActionExecutionResult result) {
  result.status = domain::ExecutionCompletionStatus::Succeeded;
  return acceptTerminal(std::move(result));
}

domain::FeedbackDisposition NavigationEngine::onActionFailed(
    domain::ActionExecutionResult result) {
  if (result.status == domain::ExecutionCompletionStatus::Succeeded)
    result.status = domain::ExecutionCompletionStatus::ControllerFailure;
  return acceptTerminal(std::move(result));
}

domain::FeedbackDisposition NavigationEngine::onActionCancelled(
    domain::ActionExecutionResult result) {
  if (result.status == domain::ExecutionCompletionStatus::Succeeded)
    result.status = domain::ExecutionCompletionStatus::Cancelled;
  return acceptTerminal(std::move(result));
}

domain::FeedbackDisposition NavigationEngine::onControllerRestart(
    domain::ExecutionTimestamp when, const domain::Pose2D& pose) {
  if (!pending_execution_) return domain::FeedbackDisposition::UnknownAction;
  domain::ActionExecutionResult result;
  result.decision_id = pending_execution_->selection.decision_id;
  result.action_id = pending_execution_->selection.action_id;
  result.task_id = pending_execution_->selection.task_id;
  result.finished_at = when;
  result.final_pose = pose;
  result.status = domain::ExecutionCompletionStatus::ControllerFailure;
  result.controller_failure = true;
  result.cancellation_reason = "controller_restart";
  return acceptTerminal(std::move(result));
}

}  // namespace semaforr::decision
