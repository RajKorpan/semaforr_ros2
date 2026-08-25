/**
 * @file semaforr_node.hpp
 * @brief Semaforr node responsibilities.
 *
 * @details This file defines semaforr node behavior for the ROS 2 composition and
 * message-adaptation boundary. It centers on `NavigationNodeState`,
 * `SemaFORRNode`, `Impl`. Its package-relative location is
 * `include/semaforr/ros/semaforr_node.hpp`.
 */
#ifndef SEMAFORR_ROS_SEMAFORR_NODE_HPP
#define SEMAFORR_ROS_SEMAFORR_NODE_HPP

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <string>
#include <string_view>

namespace semaforr::ros {

/**
 * @brief Enumerates the supported navigation node state values used by this
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
enum class NavigationNodeState {
  WaitingForSensors,
  ReadyToDecide,
  ExecutingAction,
  Stopped
};

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p state: Supplies state input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(NavigationNodeState state) noexcept;

/**
 * @brief Encapsulates sema forrnode state and behavior for this subsystem.
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
class SemaFORRNode final : public rclcpp::Node {
 public:
  /**
   * @brief Performs the sema forrnode operation for this subsystem.
   *
   * Arguments:
   * - @p options: Supplies options input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit SemaFORRNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  /**
   * @brief Performs the sema forrnode operation for this subsystem.
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
  ~SemaFORRNode() override;

  /**
   * @brief Performs the sema forrnode operation for this subsystem.
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
  SemaFORRNode(const SemaFORRNode&) = delete;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `SemaFORRNode&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  SemaFORRNode& operator=(const SemaFORRNode&) = delete;

  /**
   * @brief Performs the start operation for this subsystem.
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
  void start();
  /**
   * @brief Performs the stop operation for this subsystem.
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
  void stop();
  /**
   * @brief Performs the state operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `NavigationNodeState` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  NavigationNodeState state() const noexcept;
  /**
   * @brief Performs the last failure operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string lastFailure() const;

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

#endif  // SEMAFORR_ROS_SEMAFORR_NODE_HPP
