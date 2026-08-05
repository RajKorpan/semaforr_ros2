#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <semaforr/decision/navigation_engine.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <semaforr/spatial/coverage.hpp>
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
    world_.navigation_history.record(
        {observation_->pose, observation_->laser, result.action, std::nullopt});
    const auto model_update_started = std::chrono::steady_clock::now();
    learning_.observe({world_.navigation_history.entries().size(),
                       *observation_, result.action, std::nullopt, false, false,
                       true, true, std::nullopt, {},
                       action_space_.move_distances_m(),
                       action_space_.rotation_angles_rad()});
    learning_.applyTo(world_.spatial);
    if (phases_->explorationBudgetReached()) {
      finishInitialExploration();
      phases_->completeInitialExploration();
      auto completed = phases_->takeEvents();
      result.phase_events.insert(result.phase_events.end(), completed.begin(),
                                 completed.end());
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
  world_.navigation_history.record(
      {observation_->pose, observation_->laser, result.action, active_task});
  const auto model_update_started = std::chrono::steady_clock::now();
  learning_.observe({world_.navigation_history.entries().size(), *observation_,
                     result.action, active_task,
                     mission_step == MissionStep::ActivatedTask ||
                         mission_step == MissionStep::SkippedTask,
                     false, false, true,
                     world_.mission.active()
                         ? std::optional<domain::Point2D>(
                               world_.mission.active()->target)
                         : std::nullopt,
                     decision_candidates, action_space_.move_distances_m(),
                     action_space_.rotation_angles_rad()});
  learning_.applyTo(world_.spatial);
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

}  // namespace semaforr::decision
