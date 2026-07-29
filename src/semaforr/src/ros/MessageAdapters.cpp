#include <semaforr/ros/MessageAdapters.hpp>

#include <std_msgs/msg/header.hpp>

namespace {

semaforr::domain::Header toDomainHeader(const std_msgs::msg::Header& header)
{
  semaforr::domain::Header result;
  result.stamp_sec = header.stamp.sec;
  result.stamp_nanosec = header.stamp.nanosec;
  result.frame_id = header.frame_id;
  return result;
}

std_msgs::msg::Header toRosHeader(const semaforr::domain::Header& header)
{
  std_msgs::msg::Header result;
  result.stamp.sec = header.stamp_sec;
  result.stamp.nanosec = header.stamp_nanosec;
  result.frame_id = header.frame_id;
  return result;
}

}  // namespace

namespace semaforr {
namespace ros {

domain::LaserScan toDomain(const sensor_msgs::msg::LaserScan& message)
{
  domain::LaserScan result;
  result.header = toDomainHeader(message.header);
  result.angle_min = message.angle_min;
  result.angle_max = message.angle_max;
  result.angle_increment = message.angle_increment;
  result.time_increment = message.time_increment;
  result.scan_time = message.scan_time;
  result.range_min = message.range_min;
  result.range_max = message.range_max;
  result.ranges = message.ranges;
  result.intensities = message.intensities;
  return result;
}

domain::PoseArray toDomain(const geometry_msgs::msg::PoseArray& message)
{
  domain::PoseArray result;
  result.header = toDomainHeader(message.header);
  result.poses.reserve(message.poses.size());
  for (const geometry_msgs::msg::Pose& pose : message.poses) {
    domain::Pose converted;
    converted.position.x = pose.position.x;
    converted.position.y = pose.position.y;
    converted.position.z = pose.position.z;
    converted.orientation.x = pose.orientation.x;
    converted.orientation.y = pose.orientation.y;
    converted.orientation.z = pose.orientation.z;
    converted.orientation.w = pose.orientation.w;
    result.poses.push_back(converted);
  }
  return result;
}

domain::CrowdModel toDomain(const semaforr::msg::CrowdModel& message)
{
  domain::CrowdModel result;
  result.header = toDomainHeader(message.header);
  result.child_frame_id = message.child_frame_id;
  result.height = message.height;
  result.width = message.width;
  result.resolution = message.resolution;
  result.densities = message.densities;
  result.risk = message.risk;
  result.up = message.up;
  result.down = message.down;
  result.left = message.left;
  result.right = message.right;
  result.up_left = message.up_left;
  result.up_right = message.up_right;
  result.down_left = message.down_left;
  result.down_right = message.down_right;
  result.crowd_count = message.crowd_count;
  result.crowd_observations = message.crowd_observations;
  result.risk_count = message.risk_count;
  result.risk_experiences = message.risk_experiences;
  return result;
}

sensor_msgs::msg::LaserScan toRos(const domain::LaserScan& scan)
{
  sensor_msgs::msg::LaserScan result;
  result.header = toRosHeader(scan.header);
  result.angle_min = scan.angle_min;
  result.angle_max = scan.angle_max;
  result.angle_increment = scan.angle_increment;
  result.time_increment = scan.time_increment;
  result.scan_time = scan.scan_time;
  result.range_min = scan.range_min;
  result.range_max = scan.range_max;
  result.ranges = scan.ranges;
  result.intensities = scan.intensities;
  return result;
}

}  // namespace ros
}  // namespace semaforr
