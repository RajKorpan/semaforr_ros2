#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <semaforr/decision/navigation_engine.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <semaforr/spatial/coverage.hpp>
#include <limits>
#include <sstream>
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
    planning::TraversabilityConfiguration traversability,
    std::size_t maximum_planning_attempts_per_task)
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
      goal_tolerance_(goal_tolerance),
      maximum_planning_attempts_per_task_(
          maximum_planning_attempts_per_task) {
  if (maximum_planning_attempts_per_task_ == 0U)
    throw std::invalid_argument(
        "maximum planning attempts per task must be positive");
  exploration_.setModelFinalizer([this] {
    learning_.finalizeInitialExploration();
    learning_.applyTo(world_.spatial);
    world_.synchronizeMutationJournal();
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

std::optional<domain::Action> NavigationEngine::enforcerAction(
    std::span<const domain::Action> viable_actions) const {
  if (!world_.mission.active()) return std::nullopt;
  const auto waypoint = world_.mission.active()->waypoint();
  if (!waypoint) return std::nullopt;
  const auto& pose = world_.robot.pose;
  const double heading = std::atan2(waypoint->y_m - pose.position.y_m,
                                    waypoint->x_m - pose.position.x_m);
  const double error = domain::Angle::normalize(
      heading - pose.heading.radians());
  domain::Action desired = domain::Action::pause();
  if (std::abs(error) > 0.2) {
    const auto& turns = action_space_.rotation_angles_rad();
    if (turns.empty()) return std::nullopt;
    const auto found = std::lower_bound(turns.begin(), turns.end(),
                                        std::abs(error));
    const std::size_t magnitude =
        found == turns.end()
            ? turns.size()
            : static_cast<std::size_t>(found - turns.begin()) + 1U;
    desired = domain::Action(error < 0.0 ? domain::ActionType::TurnRight
                                         : domain::ActionType::TurnLeft,
                             magnitude);
  } else {
    const auto& moves = action_space_.move_distances_m();
    if (moves.empty()) return std::nullopt;
    const double remaining = domain::distance(pose.position, *waypoint).meters();
    const auto found = std::upper_bound(moves.begin(), moves.end(), remaining);
    const std::size_t magnitude =
        found == moves.begin()
            ? 1U
            : static_cast<std::size_t>(found - moves.begin());
    desired = domain::Action(domain::ActionType::Forward, magnitude);
  }
  if (std::find(viable_actions.begin(), viable_actions.end(), desired) !=
      viable_actions.end())
    return desired;
  for (std::size_t magnitude = desired.magnitude_index(); magnitude > 1U;
       --magnitude) {
    const domain::Action shorter(desired.type(), magnitude - 1U);
    if (std::find(viable_actions.begin(), viable_actions.end(), shorter) !=
        viable_actions.end())
      return shorter;
  }
  return std::nullopt;
}

PlanEnforcementResult NavigationEngine::enforceActivePlan(
    std::span<const domain::Action> viable_actions) {
  if (!active_hierarchy_) {
    PlanEnforcementResult missing;
    missing.reason_code = "enforcer:no_active_tier2_plan";
    return missing;
  }
  auto traversal = traversability_;
  traversal.current_sensor_origin = world_.robot.pose.position;
  traversal.current_sensor_range_m =
      world_.robot.laser ? world_.robot.laser->maximum_range.meters() : 0.0;
  PlanEnforcementContext context{world_.spatial,
                                 &world_.crowd,
                                 world_.static_map,
                                 world_.robot.pose,
                                 action_space_,
                                 viable_actions,
                                 traversal,
                                 goal_tolerance_,
                                 world_.mission.active()
                                     ? std::optional<domain::TaskId>(
                                           world_.mission.active()->id)
                                     : std::nullopt};
  if (auto* typed = dynamic_cast<Enforcer*>(enforcer_.get()))
    return typed->enforce(*active_hierarchy_, context);

  PlanEnforcementResult result;
  result.mode = active_hierarchy_->family == planning::PlanFamily::Grid
                    ? EnforcerMode::Grid
                    : EnforcerMode::Model;
  result.operational_target = enforcer_->operationalizeNext(
      *active_hierarchy_, world_.spatial, world_.robot.pose, goal_tolerance_);
  result.step_index = active_hierarchy_->cursor;
  result.step_type = "custom_operationalizer";
  result.validation_evidence = "custom_plan_operationalizer";
  return LocalActionEvaluator{}.evaluate(std::move(result), context);
}

void NavigationEngine::appendCycleDiagnostics(DecisionResult& result) const {
  for (std::size_t index = 0U; index < result.decision_cycle.size(); ++index) {
    auto& event = result.decision_cycle[index];
    event.order = index + 1U;
    std::ostringstream diagnostic;
    diagnostic << "decision_cycle:" << event.order << ":tier=" << event.tier
               << ",component=" << event.component
               << ",inputs=" << event.input_actions.size()
               << ",vetoes=" << event.vetoes.size()
               << ",mandate=" << (event.mandate ? "true" : "false")
               << ",outcome=" << event.outcome
               << ",reason=" << event.reason_code
               << ",return="
               << (event.returned_to_earlier_tier ? "true" : "false");
    result.phase_events.push_back(diagnostic.str());
  }
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
  world_.synchronizeMutationJournal();
  if (observation.crowd) {
    world_.crowd.update(*observation.crowd);
    if (crowd_learning_ &&
        crowd_learning_->observe(observation.pose, observation.laser,
                                 *observation.crowd)) {
      world_.crowd.setLearned(crowd_learning_->snapshot());
      world_.synchronizeMutationJournal();
    }
  } else {
    world_.crowd.clearCurrent();
  }

  const bool had_active_task = world_.mission.active().has_value();
  if (world_.mission.active() &&
      domain::goalReached(observation.pose, world_.mission.active()->target,
                          goal_tolerance_)) {
    world_.recovery.planning_attempted = false;
    world_.recovery.plan_available = false;
    world_.recovery.completed_plan_failed_target = false;
    world_.recovery.tier_two_attempts = 0U;
    world_.recovery.consecutive_immediate_plan_failures = 0U;
    world_.recovery.plan_abandoned = false;
    if (mission_.completeActiveTask()) {
      world_.path_history.finish(true, false, observation.observed_at);
      pending_phase_events_.push_back("target_completed");
    }
  } else {
    const bool advanced = mission_.advanceWaypoint(observation.pose,
                                                    goal_tolerance_);
    if (advanced && world_.mission.active() &&
        !world_.mission.active()->waypoint() &&
        !world_.mission.active()->plan.empty()) {
      world_.recovery.plan_available = false;
      world_.recovery.completed_plan_failed_target = true;
    }
  }
  if (had_active_task && !world_.mission.active()) {
    active_hierarchy_.reset();
    active_selection_evidence_.reset();
    hierarchy_task_.reset();
    learning_.finalizeTarget();
    learning_.applyTo(world_.spatial);
    world_.synchronizeMutationJournal();
  }
}

std::optional<std::string> NavigationEngine::preparePlan(MissionStep step) {
  if (!world_.mission.active()) {
    return std::nullopt;
  }
  if (step == MissionStep::Ready && world_.mission.active()->waypoint()) {
    world_.recovery.plan_available = true;
    return std::nullopt;
  }
  if (world_.recovery.completed_plan_failed_target) return std::nullopt;
  if (enforcer_enabled_ && active_hierarchy_ && hierarchy_task_ &&
      *hierarchy_task_ == world_.mission.active()->id) {
    planning::PlanningRequest validation{
        world_.robot.pose, world_.mission.active()->target, &world_.spatial,
        &world_.crowd, world_.static_map, traversability_,
        world_.mission.active()->id, planning_.configurationRevision()};
    auto stale = planning::dependencyChangeReasons(
        active_hierarchy_->dependency_revisions, validation);
    if (active_hierarchy_->task_id != validation.task_id)
      stale.push_back("task_changed");
    if (domain::distance(active_hierarchy_->planned_goal, validation.goal)
            .meters() > goal_tolerance_.meters())
      stale.push_back("target_moved_beyond_tolerance");
    if (stale.empty()) {
      world_.recovery.planning_attempted = true;
      world_.recovery.plan_available = true;
      return active_hierarchy_->planner;
    } else {
      active_hierarchy_->validity = planning::PlanValidity::Stale;
      active_hierarchy_->diagnostics.insert(
          active_hierarchy_->diagnostics.end(), stale.begin(), stale.end());
    }
    active_hierarchy_.reset();
    active_selection_evidence_.reset();
    hierarchy_task_.reset();
  }
  auto traversal = traversability_;
  traversal.current_sensor_origin = world_.robot.pose.position;
  traversal.current_sensor_range_m =
      world_.robot.laser ? world_.robot.laser->maximum_range.meters() : 0.0;
  const auto selected = planning_.selectPlan(
      {world_.robot.pose, world_.mission.active()->target, &world_.spatial,
       &world_.crowd, world_.static_map, traversal,
       world_.mission.active()->id});
  if (!selected) {
    mission_.clearPlan();
    world_.recovery.planning_attempted = true;
    world_.recovery.plan_available = false;
    return std::nullopt;
  }
  if (enforcer_enabled_ && selected->result.hierarchical) {
    active_hierarchy_ = *selected->result.hierarchical;
    active_selection_evidence_ = selected->evidence;
    hierarchy_task_ = world_.mission.active()->id;
    mission_.installPlan(selected->result.path);
  } else {
    active_hierarchy_.reset();
    active_selection_evidence_.reset();
    mission_.installPlan(selected->result.path);
  }
  mission_.advanceWaypoint(world_.robot.pose, goal_tolerance_);
  world_.recovery.planning_attempted = true;
  world_.recovery.plan_available =
      world_.mission.active()->waypoint().has_value();
  world_.recovery.completed_plan_failed_target = false;
  return selected->planner;
}

void NavigationEngine::finishInitialExploration() {
  world_.spatial.unfinished_hle_candidates.clear();
  for (const auto& candidate : exploration_.unfinishedCandidates()) {
    world_.spatial.unfinished_hle_candidates.push_back(
        {candidate.id, candidate.start, candidate.endpoint});
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
  if (mission_step == MissionStep::ActivatedTask ||
      mission_step == MissionStep::SkippedTask) {
    world_.recovery.planning_attempted = false;
    world_.recovery.plan_available = false;
    world_.recovery.completed_plan_failed_target = false;
    world_.recovery.tier_two_attempts = 0U;
    world_.recovery.consecutive_immediate_plan_failures = 0U;
    world_.recovery.plan_abandoned = false;
  }
  if (world_.mission.skipped().size() > skipped_before) {
    world_.path_history.finish(false, true, std::chrono::steady_clock::now());
    learning_.finalizeTarget();
    learning_.applyTo(world_.spatial);
    world_.synchronizeMutationJournal();
    pending_phase_events_.push_back("target_skipped");
  }
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
  const auto available = candidates();
  std::vector<domain::Action> decision_candidates = available;
  std::vector<Veto> hard_vetoes;
  std::vector<DecisionCycleEvent> cycle;
  if (hard_safety_) {
    auto filtered = hard_safety_->filter(DecisionContext{world_}, available);
    decision_candidates = std::move(filtered.safe_actions);
    hard_vetoes = std::move(filtered.vetoes);
    cycle.push_back({0U, "safety", "HardSafetyFilter", available,
                     std::nullopt, hard_vetoes,
                     hard_vetoes.empty() ? "no_veto_continue"
                                         : "safety_vetoes_applied_continue"});
  }
  auto tier_one =
      decisions_.evaluateTierOne(DecisionContext{world_}, decision_candidates);
  cycle.insert(cycle.end(), tier_one.trace.begin(), tier_one.trace.end());
  std::vector<Veto> cognitive_vetoes = tier_one.vetoes;
  std::vector<domain::Action> viable = tier_one.survivors;
  DecisionResult result;
  bool decided = false;
  if (tier_one.decision) {
    result = *tier_one.decision;
    decided = true;
    if (result.selected_policy == "mandatory_rule:Victory") {
      reactive_.cancelAll(planning::InterruptionReason::TargetSensed);
      lle_->cancel(planning::InterruptionReason::TargetSensed);
    }
  }

  std::optional<std::string> selected_planner;
  double planning_latency_s = 0.0;
  const auto* active_lle =
      dynamic_cast<const planning::LowLevelExplorer*>(lle_.get());
  const bool lle_in_progress =
      active_lle &&
      active_lle->state() !=
          planning::LowLevelExplorationState::DetectMissingGuidance &&
      active_lle->state() != planning::LowLevelExplorationState::Complete;
  if (!decided && world_.mission.active() &&
      !world_.mission.active()->waypoint()) {
    if (world_.recovery.plan_abandoned) {
      cycle.push_back(
          {0U, "tier2", "PlanningCoordinator", viable, std::nullopt, {},
           "planning_abandoned_lle_eligible"});
    } else if (world_.recovery.completed_plan_failed_target) {
      cycle.push_back(
          {0U, "tier2", "PlanningCoordinator", viable, std::nullopt, {},
           "completed_plan_failed_lle_eligible"});
    } else if (lle_in_progress) {
      cycle.push_back({0U, "tier2", "PlanningCoordinator", viable,
                       std::nullopt, {},
                       "active_lle_retains_control_no_repeat_plan"});
    } else {
      const auto planning_started = std::chrono::steady_clock::now();
      ++world_.recovery.tier_two_attempts;
      selected_planner = preparePlan(mission_step);
      planning_latency_s =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        planning_started)
              .count();
      const bool plan_available =
          world_.mission.active()->waypoint().has_value();
      if (plan_available) {
        world_.recovery.consecutive_immediate_plan_failures = 0U;
      } else {
        ++world_.recovery.consecutive_immediate_plan_failures;
        world_.recovery.plan_abandoned =
            world_.recovery.consecutive_immediate_plan_failures >=
            maximum_planning_attempts_per_task_;
      }
      cycle.push_back(
          {0U, "tier2",
           selected_planner.value_or("PlanningCoordinator"), viable,
           std::nullopt, {},
           plan_available
               ? "plan_created_return_to_tier1:attempt=" +
                     std::to_string(world_.recovery.tier_two_attempts)
               : (world_.recovery.plan_abandoned
                      ? "immediate_plan_failure_abandoned_lle_eligible:attempt=" +
                            std::to_string(world_.recovery.tier_two_attempts)
                      : "immediate_plan_failure_return_to_tier1:attempt=" +
                            std::to_string(world_.recovery.tier_two_attempts)),
           true});
      auto returned = decisions_.evaluateTierOne(DecisionContext{world_},
                                                 decision_candidates);
      cycle.insert(cycle.end(), returned.trace.begin(), returned.trace.end());
      cognitive_vetoes = returned.vetoes;
      viable = returned.survivors;
      if (returned.decision) {
        result = *returned.decision;
        decided = true;
      }
    }
  }

  if (!decided && enforcer_enabled_ && world_.mission.active() &&
      active_hierarchy_) {
    auto enforcement = enforceActivePlan(viable);
    if (enforcement.status == EnforcementStatus::Stale ||
        enforcement.status == EnforcementStatus::Invalid) {
      cycle.push_back({0U, "tier1", "Enforcer", viable, std::nullopt, {},
                       "plan_invalidated_return_to_tier2", true,
                       std::nullopt, enforcement.reason_code});
      mission_.clearPlan();
      active_hierarchy_.reset();
      active_selection_evidence_.reset();
      hierarchy_task_.reset();
      world_.recovery.planning_attempted = false;
      world_.recovery.plan_available = false;
      const auto replanner = preparePlan(MissionStep::Ready);
      if (replanner && active_hierarchy_) {
        selected_planner = replanner;
        enforcement = enforceActivePlan(viable);
      }
    }
    const auto action = enforcement.action;
    const std::string mode = enforcement.mode == EnforcerMode::Grid
                                 ? "GridPlanEnforcer"
                                 : "ModelPlanEnforcer";
    cycle.push_back({0U, "tier1", mode, viable, action, {},
                     action ? "plan_action_selected"
                            : "plan_not_operationalizable_continue",
                     false,
                     action ? std::optional<DecisionTier>(DecisionTier::TierOne)
                            : std::nullopt,
                     enforcement.reason_code});
    if (action) {
      result.action = *action;
      result.source = DecisionSource::MandatoryRule;
      result.tier = DecisionTier::TierOne;
      result.selected_policy = "mandatory_rule:Enforcer:" +
                               std::string(enforcement.mode == EnforcerMode::Grid
                                               ? "grid"
                                               : "model");
      result.enforcer_mode = enforcement.mode == EnforcerMode::Grid
                                 ? "grid"
                                 : "model";
      result.active_plan_step = enforcement.step_index;
      result.operational_target = enforcement.operational_target;
      result.enforcer_reason = enforcement.reason_code;
      decided = true;
    }
  } else if (!decided && enforcer_enabled_ && world_.mission.active() &&
             world_.mission.active()->waypoint()) {
    // Backward-compatible path for externally supplied custom planners that
    // have not yet adopted the explicit plan schema. All built-in planners
    // publish a typed grid or model plan and use the modes above.
    const auto action = enforcerAction(viable);
    cycle.push_back({0U, "tier1", "Enforcer", viable, action, {},
                     action ? "legacy_custom_plan_action_selected"
                            : "legacy_custom_plan_not_operationalizable",
                     false,
                     action ? std::optional<DecisionTier>(DecisionTier::TierOne)
                            : std::nullopt,
                     action ? "enforcer:custom_grid_waypoint_progress"
                            : "enforcer:custom_grid_waypoint_unavailable"});
    if (action) {
      result.action = *action;
      result.source = DecisionSource::MandatoryRule;
      result.tier = DecisionTier::TierOne;
      result.selected_policy = "mandatory_rule:Enforcer";
      result.enforcer_mode = "grid";
      result.operational_target = world_.mission.active()->waypoint();
      result.enforcer_reason = "enforcer:custom_grid_waypoint_progress";
      decided = true;
    }
  }

  if (!decided) {
    auto reactive = reactive_.evaluateDetailed(
        {world_, action_space_, viable}, viable);
    cycle.insert(cycle.end(), reactive.trace.begin(), reactive.trace.end());
    if (reactive.result.status == planning::ReactiveStatus::InstallPlan &&
        !reactive.result.prepend_waypoints.empty() && world_.mission.active()) {
      mission_.prependPlan(std::move(reactive.result.prepend_waypoints));
      mission_.advanceWaypoint(world_.robot.pose, goal_tolerance_);
      world_.recovery.confined = false;
      world_.recovery.plan_available =
          world_.mission.active()->waypoint().has_value();
      if (!cycle.empty()) {
        cycle.back().outcome =
            "reverse_subtrail_installed_return_to_enforcer";
        cycle.back().returned_to_earlier_tier = true;
      }
      const auto enforced = enforcerAction(viable);
      cycle.push_back(
          {0U, "tier1", "Enforcer", viable, enforced, {},
           enforced ? "recovery_plan_action_selected"
                    : "recovery_plan_has_no_viable_action_continue",
           false,
           enforced ? std::optional<DecisionTier>(DecisionTier::TierOne)
                    : std::nullopt,
           enforced ? "enforcer:operationalized_out_reverse_subtrail"
                    : "enforcer:out_reverse_subtrail_not_operationalizable"});
      if (enforced) {
        result.action = *enforced;
        result.source = DecisionSource::MandatoryRule;
        result.tier = DecisionTier::TierOne;
        result.selected_policy = "mandatory_rule:Enforcer:out_reverse_subtrail";
        decided = true;
      }
    } else if (reactive.result.status == planning::ReactiveStatus::Action &&
        reactive.result.action &&
        std::find(viable.begin(), viable.end(), *reactive.result.action) !=
            viable.end()) {
      result.action = *reactive.result.action;
      result.source = DecisionSource::MandatoryRule;
      result.tier = DecisionTier::TierOne;
      result.selected_policy = "reactive:" + reactive.result.planner;
      decided = true;
    }
  }

  const bool lle_eligible = world_.recovery.plan_abandoned ||
                            world_.recovery.completed_plan_failed_target ||
                            lle_in_progress;
  if (!decided && low_level_exploration_enabled_ && lle_eligible) {
    const auto lle = lle_->evaluate({world_, action_space_});
    const bool lle_action_viable =
        lle.status != planning::ReactiveStatus::Action || !lle.action ||
        std::find(viable.begin(), viable.end(), *lle.action) != viable.end();
    DecisionCycleEvent event{0U, "tier1", "LLE", viable, lle.action, {}};
    event.reason_code = lle.explanation;
    event.outcome = !lle_action_viable
                        ? "reactive_action_not_viable_continue"
                    : lle.status == planning::ReactiveStatus::Action
                        ? "reactive_action_selected"
                    : lle.status == planning::ReactiveStatus::RequestReplan
                        ? "replan_requested_continue"
                        : "not_applicable_continue";
    if (lle.status == planning::ReactiveStatus::Action && lle_action_viable)
      event.final_attribution = DecisionTier::TierOne;
    cycle.push_back(std::move(event));
    if (lle.status == planning::ReactiveStatus::RequestReplan &&
        world_.mission.active()) {
      mission_.clearPlan();
      planning_.clearCache();
      world_.recovery.confined = true;
      world_.recovery.planning_attempted = false;
      world_.recovery.plan_available = false;
      world_.recovery.completed_plan_failed_target = false;
      world_.recovery.tier_two_attempts = 0U;
      world_.recovery.consecutive_immediate_plan_failures = 0U;
      world_.recovery.plan_abandoned = false;
    } else if (lle.status == planning::ReactiveStatus::Action && lle.action &&
               std::find(viable.begin(), viable.end(), *lle.action) !=
                   viable.end()) {
      result.action = *lle.action;
      result.source = DecisionSource::MandatoryRule;
      result.tier = DecisionTier::TierOne;
      result.selected_policy = "reactive:LLE";
      if (const auto* explorer =
              dynamic_cast<const planning::LowLevelExplorer*>(lle_.get()))
        result.selected_policy += ":" + explorer->lastTriggerReasonCode();
      decided = true;
    }
  }

  if (!decided) {
    result = decisions_.decideTierThree(DecisionContext{world_}, viable);
    cycle.insert(cycle.end(), result.decision_cycle.begin(),
                 result.decision_cycle.end());
  }
  result.decision_cycle = std::move(cycle);
  result.vetoes = std::move(cognitive_vetoes);
  result.vetoes.insert(result.vetoes.begin(),
                       std::make_move_iterator(hard_vetoes.begin()),
                       std::make_move_iterator(hard_vetoes.end()));
  result.sequence = ++decision_sequence_;
  result.robot_pose = world_.robot.pose;
  result.navigation_phase = phases_->phase();
  result.configuration_fingerprint = configuration_fingerprint_;
  result.component_manifest = component_manifest_;
  result.phase_events.swap(pending_phase_events_);
  appendCycleDiagnostics(result);
  result.candidates = decision_candidates;
  result.planner = selected_planner;
  if (active_hierarchy_) {
    if (!result.planner) result.planner = active_hierarchy_->planner;
    result.plan_id = active_hierarchy_->id;
    result.plan_family = active_hierarchy_->family;
  }
  if (active_selection_evidence_) {
    result.planning_tie_candidates =
        active_selection_evidence_->tie_candidates;
    result.planning_tie_break_reason =
        active_selection_evidence_->tie_break_reason;
    for (const auto& candidate : active_selection_evidence_->candidates)
      result.planning_candidates.push_back(
          {candidate.plan_id, candidate.planner, candidate.family,
           candidate.raw_costs, candidate.normalized_costs,
           candidate.summed_score, candidate.tied_for_best});
  }
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
  history.observation_pose = pending_execution_->episode.observation.pose;
  history.decision_id = result.decision_id;
  history.action_id = result.action_id;
  history.execution_status = result.status;
  history.distance_achieved_m = result.distance_achieved_m;
  history.rotation_achieved_rad = result.rotation_achieved_rad;
  world_.navigation_history.record(history);
  if (result.successful()) world_.completed_path_history.record(history);
  if (result.task_id) {
    domain::PathDecisionPoint path_point;
    path_point.selection = pending_execution_->selection;
    path_point.execution = result;
    path_point.decision_observation = pending_execution_->episode.observation;
    if (pending_execution_->started)
      path_point.executed_action = pending_execution_->selection.action;
    path_point.target = pending_execution_->episode.active_target;
    path_point.task_started = pending_execution_->episode.task_started;
    path_point.interrupted = !result.successful();
    if (!world_.path_history.active()) {
      world_.path_history.begin(++path_sequence_, result.task_id,
                                pending_execution_->episode.active_target);
    }
    world_.path_history.record(std::move(path_point));
  }
  if (!result.successful() && active_hierarchy_) {
    active_hierarchy_->validity = planning::PlanValidity::Stale;
    active_hierarchy_->diagnostics.push_back(
        "execution_invalidated_remaining_route:" +
        std::string(domain::toString(result.status)));
  }

  auto episode = pending_execution_->episode;
  episode.execution_result = result;
  episode.target_reached =
      episode.active_target &&
      domain::distance(result.final_pose.position, *episode.active_target)
              .meters() <= goal_tolerance_.meters();
  episode.task_finished = episode.target_reached;
  const auto model_update_started = std::chrono::steady_clock::now();
  learning_.observeActionTerminal(std::move(episode));
  learning_.applyTo(world_.spatial);
  world_.synchronizeMutationJournal();
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
