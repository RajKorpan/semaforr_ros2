#ifndef SEMAFORR_ROS_MESSAGE_ADAPTERS_H
#define SEMAFORR_ROS_MESSAGE_ADAPTERS_H

#include <rclcpp/time.hpp>
#include <semaforr/domain/SensorTypes.hpp>
#include <semaforr/domain/social.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <social_context_msgs/msg/social_observation.hpp>

namespace semaforr {
namespace ros {

domain::LaserScan toDomain(const sensor_msgs::msg::LaserScan& message);
domain::CrowdObservation toDomain(
  const social_context_msgs::msg::SocialObservation& message,
  const rclcpp::Time& received_at);

sensor_msgs::msg::LaserScan toRos(const domain::LaserScan& scan);

}  // namespace ros
}  // namespace semaforr

#endif
