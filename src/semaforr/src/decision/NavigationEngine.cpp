#include <semaforr/decision/navigation_engine.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <stdexcept>
#include <utility>

namespace semaforr::decision {

NavigationEngine::NavigationEngine(
    domain::WorldModel& world, const domain::ActionSpace& action_space,
    DecisionCoordinator& decisions, MissionManager& mission,
    planning::PlanningCoordinator& planning,
    spatial::SpatialLearningCoordinator& learning,
    social::CrowdFieldLearner* crowd_learning, domain::Distance goal_tolerance)
    : world_(world),
      action_space_(action_space),
      decisions_(decisions),
      mission_(mission),
      planning_(planning),
      learning_(learning),
      crowd_learning_(crowd_learning),
      goal_tolerance_(goal_tolerance) {}

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
  world_.robot.pose = observation.pose;
  world_.robot.laser = observation.laser;
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

  if (world_.mission.active() &&
      domain::goalReached(observation.pose, world_.mission.active()->target,
                          goal_tolerance_)) {
    mission_.completeActiveTask();
  } else {
    mission_.advanceWaypoint(observation.pose, goal_tolerance_);
  }
}

std::optional<std::string> NavigationEngine::preparePlan(MissionStep step) {
  if (!world_.mission.active()) {
    return std::nullopt;
  }
  if (step == MissionStep::Ready && !world_.mission.active()->plan.empty()) {
    return std::nullopt;
  }
  const auto selected =
      planning_.selectPlan({world_.robot.pose, world_.mission.active()->target,
                            &world_.spatial, &world_.crowd});
  if (!selected) {
    mission_.installPlan({world_.mission.active()->target});
    return std::nullopt;
  }
  mission_.installPlan(selected->result.path);
  mission_.advanceWaypoint(world_.robot.pose, goal_tolerance_);
  return selected->planner;
}

DecisionResult NavigationEngine::decide() {
  if (!observation_) {
    throw std::logic_error("navigation decision requires an observation");
  }
  const MissionStep mission_step = mission_.prepareDecision();
  if (mission_step == MissionStep::Complete) {
    return {};
  }
  const std::optional<std::string> selected_planner = preparePlan(mission_step);
  const auto available = candidates();
  DecisionResult result = decisions_.decide(DecisionContext{world_}, available);
  result.sequence = ++decision_sequence_;
  result.robot_pose = world_.robot.pose;
  result.candidates = available;
  result.planner = selected_planner;
  if (world_.mission.active()) {
    result.task = TaskDiagnostic{
        static_cast<std::uint64_t>(world_.mission.active()->id),
        static_cast<std::uint64_t>(world_.mission.decisions_for_active() + 1U),
        world_.mission.active()->target, world_.mission.active()->waypoint()};
  }
  world_.navigation_history.record(
      {observation_->pose, observation_->laser, result.action});
  const std::optional<domain::TaskId> active_task =
      world_.mission.active()
          ? std::optional<domain::TaskId>(world_.mission.active()->id)
          : std::nullopt;
  learning_.observe({world_.navigation_history.entries().size(), *observation_,
                     result.action, active_task,
                     mission_step == MissionStep::ActivatedTask ||
                         mission_step == MissionStep::SkippedTask,
                     false});
  learning_.applyTo(world_.spatial);
  mission_.recordDecision();
  (void)planning_;
  return result;
}

DecisionResult NavigationEngine::decide(
    const domain::RobotObservation& observation) {
  observe(observation);
  return decide();
}

bool NavigationEngine::missionComplete() const noexcept {
  return mission_.complete();
}

}  // namespace semaforr::decision
