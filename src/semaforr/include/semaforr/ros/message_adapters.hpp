#ifndef SEMAFORR_ROS_MESSAGE_ADAPTERS_H
#define SEMAFORR_ROS_MESSAGE_ADAPTERS_H

#include <rclcpp/time.hpp>
#include <semaforr/domain/social.hpp>
#include <social_context_msgs/msg/social_observation.hpp>

namespace semaforr {
namespace ros {

domain::CrowdObservation toDomain(
    const social_context_msgs::msg::SocialObservation& message,
    const rclcpp::Time& received_at);

}  // namespace ros
}  // namespace semaforr

#endif
