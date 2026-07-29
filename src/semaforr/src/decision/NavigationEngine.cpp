#include <semaforr/decision/navigation_engine.hpp>

namespace semaforr::decision {

NavigationEngine::NavigationEngine(
  domain::WorldModel& world,
  const domain::ActionSpace& action_space,
  DecisionCoordinator& decisions,
  MissionManager& mission,
  planning::PlanningCoordinator& planning,
  spatial::SpatialLearningCoordinator& learning)
  : world_(world),
    action_space_(action_space),
    decisions_(decisions),
    mission_(mission),
    planning_(planning),
    learning_(learning)
{
}

std::vector<domain::Action> NavigationEngine::candidates() const
{
  std::vector<domain::Action> actions;
  actions.reserve(
    1U + action_space_.move_distances_m().size() +
    2U * action_space_.rotation_angles_rad().size());
  actions.push_back(domain::Action::pause());
  for (std::size_t index = 1U;
       index <= action_space_.move_distances_m().size(); ++index) {
    actions.emplace_back(domain::ActionType::Forward, index);
  }
  for (std::size_t index = 1U;
       index <= action_space_.rotation_angles_rad().size(); ++index) {
    actions.emplace_back(domain::ActionType::TurnRight, index);
    actions.emplace_back(domain::ActionType::TurnLeft, index);
  }
  return actions;
}

DecisionResult NavigationEngine::decide(
  const domain::RobotObservation& observation)
{
  observation.laser.validate();
  world_.robot.pose = observation.pose;
  world_.robot.laser = observation.laser;
  world_.robot.crowd = observation.crowd;
  world_.crowd.current = observation.crowd;
  world_.crowd.history.push_back(observation.crowd);

  const MissionStep mission_step = mission_.prepareDecision();
  if (mission_step == MissionStep::Complete) {
    return {};
  }
  const auto available = candidates();
  DecisionResult result =
    decisions_.decide(DecisionContext{world_}, available);
  world_.navigation_history.record({
    observation.pose, observation.laser, result.action});
  const std::optional<domain::TaskId> active_task =
    world_.mission.active()
    ? std::optional<domain::TaskId>(world_.mission.active()->id)
    : std::nullopt;
  learning_.observe({
    world_.navigation_history.entries().size(),
    observation,
    result.action,
    active_task,
    mission_step == MissionStep::ActivatedTask ||
      mission_step == MissionStep::SkippedTask,
    false});
  learning_.applyTo(world_.spatial);
  mission_.recordDecision();
  (void)planning_;
  return result;
}

}  // namespace semaforr::decision
