#ifndef SEMAFORR_ROS_SEMAFORR_NODE_HPP
#define SEMAFORR_ROS_SEMAFORR_NODE_HPP

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <string>
#include <string_view>

namespace semaforr::ros {

enum class NavigationNodeState {
  WaitingForSensors,
  ReadyToDecide,
  ExecutingAction,
  Stopped
};

std::string_view toString(NavigationNodeState state) noexcept;

class SemaFORRNode final : public rclcpp::Node {
 public:
  explicit SemaFORRNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~SemaFORRNode() override;

  SemaFORRNode(const SemaFORRNode&) = delete;
  SemaFORRNode& operator=(const SemaFORRNode&) = delete;

  void start();
  void stop();
  NavigationNodeState state() const noexcept;
  std::string lastFailure() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_SEMAFORR_NODE_HPP
