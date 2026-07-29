#include <exception>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <semaforr/ros/semaforr_node.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<semaforr::ros::SemaFORRNode>();
    node->start();
    const std::weak_ptr<semaforr::ros::SemaFORRNode> weak_node = node;
    const auto context = node->get_node_base_interface()->get_context();
    context->add_pre_shutdown_callback(
      [weak_node]() {
        if (const auto active = weak_node.lock()) {
          active->stop();
        }
      });
    rclcpp::spin(node);
    node->stop();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "SemaFORR startup failed: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
