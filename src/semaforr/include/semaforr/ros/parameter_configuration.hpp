/**
 * @file parameter_configuration.hpp
 * @brief Parameter configuration responsibilities.
 *
 * @details This file defines parameter configuration behavior for the ROS 2
 * composition and message-adaptation boundary. It centers on `Node`. Its
 * package-relative location is
 * `include/semaforr/ros/parameter_configuration.hpp`.
 */
#ifndef SEMAFORR_ROS_PARAMETER_CONFIGURATION_HPP
#define SEMAFORR_ROS_PARAMETER_CONFIGURATION_HPP

#include <semaforr/config/navigation_configuration.hpp>

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
}  // namespace rclcpp

namespace semaforr::ros {

/**
 * @brief Performs the declare configuration parameters operation for this
 * subsystem.
 *
 * Arguments:
 * - @p node: Supplies node input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void declareConfigurationParameters(rclcpp::Node& node);
/**
 * @brief Performs the configuration from parameters operation for this
 * subsystem.
 *
 * Arguments:
 * - @p node: Supplies node input to the operation.
 *
 * Returns:
 * - `config::Configuration` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
config::Configuration configurationFromParameters(rclcpp::Node& node);

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_PARAMETER_CONFIGURATION_HPP
