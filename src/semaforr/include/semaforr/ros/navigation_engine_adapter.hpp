/**
 * @file navigation_engine_adapter.hpp
 * @brief Navigation engine adapter responsibilities.
 *
 * @details This file defines navigation engine adapter behavior for the ROS 2
 * composition and message-adaptation boundary. It centers on
 * `NavigationEngineAdapter`, `Impl`. Its package-relative location is
 * `include/semaforr/ros/navigation_engine_adapter.hpp`.
 */
#ifndef SEMAFORR_ROS_NAVIGATION_ENGINE_ADAPTER_HPP
#define SEMAFORR_ROS_NAVIGATION_ENGINE_ADAPTER_HPP

#include <memory>
#include <semaforr/config/navigation_configuration.hpp>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/domain/world_model.hpp>
#include <semaforr/ros/command_executor.hpp>
#include <semaforr/ros/sensor_synchronizer.hpp>
#include <string>
#include <vector>

namespace semaforr::ros {

// Owns the ROS-independent navigation composition used by the ROS adapter.
/**
 * @brief Encapsulates navigation engine adapter state and behavior for this
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
class NavigationEngineAdapter {
 public:
  /**
   * @brief Performs the navigation engine adapter operation for this
   * subsystem.
   *
   * Arguments:
   * - @p configuration: Supplies configuration input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit NavigationEngineAdapter(config::Configuration configuration);
  /**
   * @brief Performs the navigation engine adapter operation for this
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
  ~NavigationEngineAdapter();

  /**
   * @brief Performs the navigation engine adapter operation for this
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
  NavigationEngineAdapter(const NavigationEngineAdapter&) = delete;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `NavigationEngineAdapter&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  NavigationEngineAdapter& operator=(const NavigationEngineAdapter&) = delete;
  /**
   * @brief Performs the navigation engine adapter operation for this
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
  NavigationEngineAdapter(NavigationEngineAdapter&&) noexcept;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `NavigationEngineAdapter&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  NavigationEngineAdapter& operator=(NavigationEngineAdapter&&) noexcept;

  /**
   * @brief Processes package content for this subsystem.
   *
   * Arguments:
   * - @p sensors: Supplies sensors input to the operation.
   * - @p crowd: Supplies crowd input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void observe(const SynchronizedSensors& sensors,
               const domain::CrowdState& crowd);

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
  bool missionComplete();
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
  /**
   * @brief Selects package content for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `decision::DecisionResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  decision::DecisionResult decide();
  /**
   * @brief Performs the execution request operation for this subsystem.
   *
   * Arguments:
   * - @p decision: Supplies decision input to the operation.
   *
   * Returns:
   * - `ActionExecutionRequest` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ActionExecutionRequest executionRequest(
      const decision::DecisionResult& decision) const;
  /**
   * @brief Performs the on action started operation for this subsystem.
   *
   * Arguments:
   * - @p update: Supplies update input to the operation.
   *
   * Returns:
   * - `domain::FeedbackDisposition` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::FeedbackDisposition onActionStarted(
      const ActionExecutionUpdate& update);
  /**
   * @brief Performs the on action progress operation for this subsystem.
   *
   * Arguments:
   * - @p update: Supplies update input to the operation.
   *
   * Returns:
   * - `domain::FeedbackDisposition` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::FeedbackDisposition onActionProgress(
      const ActionExecutionUpdate& update);
  /**
   * @brief Performs the on action terminal operation for this subsystem.
   *
   * Arguments:
   * - @p update: Supplies update input to the operation.
   * - @p status: Supplies status input to the operation.
   * - @p detail: Supplies detail input to the operation.
   *
   * Returns:
   * - `domain::FeedbackDisposition` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::FeedbackDisposition onActionTerminal(
      const ActionExecutionUpdate& update,
      domain::ExecutionCompletionStatus status, std::string detail = {});
  /**
   * @brief Performs the on controller restart operation for this subsystem.
   *
   * Arguments:
   * - @p pose: Supplies pose input to the operation.
   *
   * Returns:
   * - `domain::FeedbackDisposition` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::FeedbackDisposition onControllerRestart(
      const domain::Pose2D& pose);
  /**
   * @brief Performs the world model operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const domain::WorldModel&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const domain::WorldModel& worldModel() const noexcept;
  /**
   * @brief Performs the startup diagnostics operation for this subsystem.
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
  const std::vector<std::string>& startupDiagnostics() const noexcept;

 private:
  /**
   * @brief Encapsulates impl state and behavior for this subsystem.
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
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_NAVIGATION_ENGINE_ADAPTER_HPP
