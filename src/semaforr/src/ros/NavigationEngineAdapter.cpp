#include <semaforr/ros/navigation_engine_adapter.hpp>

#include <cmath>
#include <memory>
#include <utility>

#include <semaforr/decision/Controller.hpp>

namespace semaforr::ros {

class NavigationEngineAdapter::Impl {
public:
  explicit Impl(config::Configuration configuration)
    : controller(std::make_unique<Controller>(std::move(configuration)))
  {
  }

  std::unique_ptr<Controller> controller;
};

NavigationEngineAdapter::NavigationEngineAdapter(
  config::Configuration configuration)
  : impl_(std::make_unique<Impl>(std::move(configuration)))
{
}

NavigationEngineAdapter::~NavigationEngineAdapter() = default;
NavigationEngineAdapter::NavigationEngineAdapter(
  NavigationEngineAdapter&&) noexcept = default;
NavigationEngineAdapter& NavigationEngineAdapter::operator=(
  NavigationEngineAdapter&&) noexcept = default;

void NavigationEngineAdapter::observe(
  const SynchronizedSensors& sensors,
  const domain::PoseArray& crowd,
  const domain::PoseArray& crowd_history)
{
  impl_->controller->updateState(
    sensors.pose, sensors.scan, crowd, crowd_history);
}

bool NavigationEngineAdapter::missionComplete()
{
  return impl_->controller->isMissionComplete();
}

decision::DecisionResult NavigationEngineAdapter::decide()
{
  return impl_->controller->decide();
}

ActionExecutionRequest NavigationEngineAdapter::executionRequest(
  const domain::Action& action) const
{
  ActionExecutionRequest request{action, 0.0, 0.0};
  const std::size_t magnitude = action.magnitude_index();
  AgentState* state = impl_->controller->getBeliefs()->getAgentState();
  if (action.type() == domain::ActionType::Forward) {
    request.target_distance_m =
      state->getMovement(static_cast<int>(magnitude));
  } else if (
    action.type() == domain::ActionType::TurnLeft ||
    action.type() == domain::ActionType::TurnRight) {
    request.target_angle_rad =
      std::fabs(state->getRotation(static_cast<int>(magnitude)));
  }
  return request;
}

void NavigationEngineAdapter::markDecisionComplete(double mission_time_s)
{
  impl_->controller->gethighwayExploration()->setHighwaysComplete(
    mission_time_s);
  impl_->controller->getfrontierExploration()->setFrontiersComplete(
    mission_time_s);
}

Controller& NavigationEngineAdapter::visualizationModel() noexcept
{
  return *impl_->controller;
}

}  // namespace semaforr::ros
