/**
 * @file visualization_publisher.hpp
 * @brief Visualization publisher responsibilities.
 *
 * @details This file defines visualization publisher behavior for the ROS 2
 * composition and message-adaptation boundary. It centers on `Node`,
 * `VisualizationPublisher`, `Impl`. Its package-relative location is
 * `include/semaforr/ros/visualization_publisher.hpp`.
 */
#ifndef SEMAFORR_ROS_VISUALIZATION_PUBLISHER_HPP
#define SEMAFORR_ROS_VISUALIZATION_PUBLISHER_HPP

#include <memory>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/domain/world_model.hpp>

namespace rclcpp {
/**
 * @brief Encapsulates node state and behavior for this subsystem.
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
class Node;
}

namespace semaforr::ros {

/**
 * @brief Encapsulates visualization publisher state and behavior for this
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
class VisualizationPublisher {
 public:
  /**
   * @brief Performs the visualization publisher operation for this
   * subsystem.
   *
   * Arguments:
   * - @p node: Supplies node input to the operation.
   * - @p world: Supplies world input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  VisualizationPublisher(rclcpp::Node& node, const domain::WorldModel& world);
  /**
   * @brief Performs the visualization publisher operation for this
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
  ~VisualizationPublisher();

  /**
   * @brief Performs the visualization publisher operation for this
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
  VisualizationPublisher(const VisualizationPublisher&) = delete;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `VisualizationPublisher&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  VisualizationPublisher& operator=(const VisualizationPublisher&) = delete;
  /**
   * @brief Performs the visualization publisher operation for this
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
  VisualizationPublisher(VisualizationPublisher&&) noexcept;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `VisualizationPublisher&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  VisualizationPublisher& operator=(VisualizationPublisher&&) noexcept;

  /**
   * @brief Publishes snapshot for this subsystem.
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
  void publishSnapshot();
  /**
   * @brief Publishes decision for this subsystem.
   *
   * Arguments:
   * - @p result: Supplies result input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void publishDecision(const decision::DecisionResult& result);

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

#endif  // SEMAFORR_ROS_VISUALIZATION_PUBLISHER_HPP
