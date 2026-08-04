#ifndef SEMAFORR_ROS_NAVIGATION_ENGINE_ADAPTER_HPP
#define SEMAFORR_ROS_NAVIGATION_ENGINE_ADAPTER_HPP

#include <memory>
#include <semaforr/config/navigation_configuration.hpp>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/domain/world_model.hpp>
#include <semaforr/ros/command_executor.hpp>
#include <semaforr/ros/sensor_synchronizer.hpp>

namespace semaforr::ros {

// Owns the ROS-independent navigation composition used by the ROS adapter.
class NavigationEngineAdapter {
 public:
  explicit NavigationEngineAdapter(config::Configuration configuration);
  ~NavigationEngineAdapter();

  NavigationEngineAdapter(const NavigationEngineAdapter&) = delete;
  NavigationEngineAdapter& operator=(const NavigationEngineAdapter&) = delete;
  NavigationEngineAdapter(NavigationEngineAdapter&&) noexcept;
  NavigationEngineAdapter& operator=(NavigationEngineAdapter&&) noexcept;

  void observe(const SynchronizedSensors& sensors,
               const domain::CrowdState& crowd);

  bool missionComplete();
  navigation::NavigationPhase phase() const noexcept;
  decision::DecisionResult decide();
  ActionExecutionRequest executionRequest(const domain::Action& action) const;
  const domain::WorldModel& worldModel() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_NAVIGATION_ENGINE_ADAPTER_HPP
