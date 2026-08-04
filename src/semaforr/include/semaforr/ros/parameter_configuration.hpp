#ifndef SEMAFORR_ROS_PARAMETER_CONFIGURATION_HPP
#define SEMAFORR_ROS_PARAMETER_CONFIGURATION_HPP

#include <semaforr/config/navigation_configuration.hpp>

namespace rclcpp {
class Node;
}

namespace semaforr::ros {

void declareConfigurationParameters(rclcpp::Node& node);
config::Configuration configurationFromParameters(rclcpp::Node& node);

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_PARAMETER_CONFIGURATION_HPP
