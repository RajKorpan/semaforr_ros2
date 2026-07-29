#include <semaforr/msg/crowd_model.hpp>
#include <semaforr/ros/MessageAdapters.hpp>

#include <cassert>
#include <cmath>

int main()
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp.sec = 12;
  scan.header.stamp.nanosec = 34;
  scan.header.frame_id = "laser";
  scan.angle_min = -1.0F;
  scan.angle_max = 1.0F;
  scan.angle_increment = 0.5F;
  scan.time_increment = 0.01F;
  scan.scan_time = 0.1F;
  scan.range_min = 0.2F;
  scan.range_max = 10.0F;
  scan.ranges = {1.0F, 2.0F};
  scan.intensities = {3.0F, 4.0F};

  const semaforr::domain::LaserScan domain_scan =
    semaforr::ros::toDomain(scan);
  assert(domain_scan.header.stamp_sec == 12);
  assert(domain_scan.header.stamp_nanosec == 34U);
  assert(domain_scan.header.frame_id == "laser");
  assert(domain_scan.angle_min == scan.angle_min);
  assert(domain_scan.angle_max == scan.angle_max);
  assert(domain_scan.angle_increment == scan.angle_increment);
  assert(domain_scan.time_increment == scan.time_increment);
  assert(domain_scan.scan_time == scan.scan_time);
  assert(domain_scan.range_min == scan.range_min);
  assert(domain_scan.range_max == scan.range_max);
  assert(domain_scan.ranges == scan.ranges);
  assert(domain_scan.intensities == scan.intensities);

  const sensor_msgs::msg::LaserScan round_trip =
    semaforr::ros::toRos(domain_scan);
  assert(round_trip.header.stamp == scan.header.stamp);
  assert(round_trip.header.frame_id == scan.header.frame_id);
  assert(round_trip.ranges == scan.ranges);
  assert(round_trip.intensities == scan.intensities);

  geometry_msgs::msg::PoseArray poses;
  poses.header.frame_id = "map";
  poses.poses.resize(1);
  poses.poses[0].position.x = 2.5;
  const double expected_yaw = 0.75;
  poses.poses[0].orientation.z = std::sin(expected_yaw / 2.0);
  poses.poses[0].orientation.w = std::cos(expected_yaw / 2.0);

  const semaforr::domain::PoseArray domain_poses =
    semaforr::ros::toDomain(poses);
  assert(domain_poses.header.frame_id == "map");
  assert(domain_poses.poses.size() == 1U);
  assert(domain_poses.poses[0].position.x == 2.5);
  assert(std::abs(domain_poses.poses[0].yaw() - expected_yaw) < 1e-12);

  semaforr::msg::CrowdModel crowd;
  crowd.header.frame_id = "map";
  crowd.child_frame_id = "crowd";
  crowd.width = 2;
  crowd.height = 1;
  crowd.resolution = 4;
  crowd.densities = {0.25, 0.75};
  crowd.risk = {0.5, 1.0};
  crowd.up = {1.0};
  crowd.down = {2.0};
  crowd.left = {3.0};
  crowd.right = {4.0};
  crowd.up_left = {5.0};
  crowd.up_right = {6.0};
  crowd.down_left = {7.0};
  crowd.down_right = {8.0};
  crowd.crowd_count = {9.0};
  crowd.crowd_observations = {10.0};
  crowd.risk_count = {11.0};
  crowd.risk_experiences = {12.0};

  const semaforr::domain::CrowdModel domain_crowd =
    semaforr::ros::toDomain(crowd);
  assert(domain_crowd.header.frame_id == "map");
  assert(domain_crowd.child_frame_id == "crowd");
  assert(domain_crowd.width == 2);
  assert(domain_crowd.height == 1);
  assert(domain_crowd.resolution == 4);
  assert(domain_crowd.densities == crowd.densities);
  assert(domain_crowd.risk == crowd.risk);
  assert(domain_crowd.up == crowd.up);
  assert(domain_crowd.down == crowd.down);
  assert(domain_crowd.left == crowd.left);
  assert(domain_crowd.right == crowd.right);
  assert(domain_crowd.up_left == crowd.up_left);
  assert(domain_crowd.up_right == crowd.up_right);
  assert(domain_crowd.down_left == crowd.down_left);
  assert(domain_crowd.down_right == crowd.down_right);
  assert(domain_crowd.crowd_count == crowd.crowd_count);
  assert(domain_crowd.crowd_observations == crowd.crowd_observations);
  assert(domain_crowd.risk_count == crowd.risk_count);
  assert(domain_crowd.risk_experiences == crowd.risk_experiences);
  return 0;
}
