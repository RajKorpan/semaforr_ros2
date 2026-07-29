#include <semaforr/ros/MessageAdapters.hpp>
#include <social_context_msgs/msg/social_observation.hpp>

#include <cassert>
#include <chrono>
#include <cmath>

#include <rcl/time.h>

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

  social_context_msgs::msg::SocialObservation crowd;
  crowd.header.frame_id = "map";
  crowd.header.stamp.sec = 12;
  crowd.pedestrians.resize(1);
  auto& person = crowd.pedestrians.front();
  person.id = "person-7";
  person.position.x = 2.5;
  person.position.y = 1.0;
  person.velocity.x = -0.2;
  person.confidence = 0.8;
  person.position_covariance = {0.1, 0.0, 0.0, 0.2};
  person.predicted_positions.resize(1);
  person.predicted_positions.front().x = 2.3;
  person.predicted_positions.front().y = 1.0;
  person.prediction_stamps.resize(1);
  person.prediction_stamps.front().sec = 13;

  const rclcpp::Time received(
    12'100'000'000LL, RCL_ROS_TIME);
  const semaforr::domain::CrowdObservation domain_crowd =
    semaforr::ros::toDomain(crowd, received);
  assert(domain_crowd.frame_id == "map");
  assert(domain_crowd.data_age == std::chrono::milliseconds(100));
  assert(domain_crowd.pedestrians.size() == 1U);
  assert(domain_crowd.pedestrians.front().id == "person-7");
  assert(domain_crowd.pedestrians.front().velocity_mps.x_m == -0.2);
  assert(
    domain_crowd.pedestrians.front().predicted_trajectory.size() == 1U);
  return 0;
}
