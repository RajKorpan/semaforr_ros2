#ifndef SEMAFORR_ROS_NAVIGATION_ENGINE_ADAPTER_HPP
#define SEMAFORR_ROS_NAVIGATION_ENGINE_ADAPTER_HPP

#include <memory>

#include <semaforr/config/Configuration.hpp>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/domain/SensorTypes.hpp>
#include <semaforr/ros/command_executor.hpp>
#include <semaforr/ros/sensor_synchronizer.hpp>

class Controller;

namespace semaforr::ros {

// Compatibility boundary between the event-driven ROS node and the legacy
// Controller facade. It keeps decision selection and legacy model updates out
// of SemaFORRNode while the ROS-independent NavigationEngine is adopted.
class NavigationEngineAdapter {
public:
  explicit NavigationEngineAdapter(config::Configuration configuration);
  ~NavigationEngineAdapter();

  NavigationEngineAdapter(const NavigationEngineAdapter&) = delete;
  NavigationEngineAdapter& operator=(const NavigationEngineAdapter&) = delete;
  NavigationEngineAdapter(NavigationEngineAdapter&&) noexcept;
  NavigationEngineAdapter& operator=(NavigationEngineAdapter&&) noexcept;

  void observe(
    const SynchronizedSensors& sensors,
    const domain::CrowdState& crowd);

  bool missionComplete();
  decision::DecisionResult decide();
  ActionExecutionRequest executionRequest(
    const domain::Action& action) const;
  void markDecisionComplete(double mission_time_s);

  // Visualization still reads the established Controller model.
  Controller& visualizationModel() noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_NAVIGATION_ENGINE_ADAPTER_HPP
