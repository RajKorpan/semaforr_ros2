#ifndef SEMAFORR_ROS_MESSAGE_ADAPTERS_H
#define SEMAFORR_ROS_MESSAGE_ADAPTERS_H

#include <geometry_msgs/msg/pose_array.hpp>
#include <semaforr/domain/SensorTypes.hpp>
#include <semaforr/msg/crowd_model.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace semaforr {
namespace ros {

domain::LaserScan toDomain(const sensor_msgs::msg::LaserScan& message);
domain::PoseArray toDomain(const geometry_msgs::msg::PoseArray& message);
domain::CrowdModel toDomain(const semaforr::msg::CrowdModel& message);

sensor_msgs::msg::LaserScan toRos(const domain::LaserScan& scan);

}  // namespace ros
}  // namespace semaforr

#endif
