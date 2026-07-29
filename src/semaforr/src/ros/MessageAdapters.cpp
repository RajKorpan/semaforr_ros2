#include <semaforr/ros/MessageAdapters.hpp>

#include <algorithm>
#include <chrono>
#include <stdexcept>

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

domain::CrowdObservation toDomain(
  const social_context_msgs::msg::SocialObservation& message,
  const rclcpp::Time& received_at)
{
  const rclcpp::Time observed_at(
    message.header.stamp, received_at.get_clock_type());
  if (observed_at.nanoseconds() < 0 || received_at < observed_at) {
    throw std::invalid_argument(
      "social observation timestamp is in the future");
  }
  domain::CrowdObservation result;
  result.frame_id = message.header.frame_id;
  result.observed_at =
    std::chrono::nanoseconds(observed_at.nanoseconds());
  result.data_age =
    std::chrono::nanoseconds(
      (received_at - observed_at).nanoseconds());
  result.pedestrians.reserve(message.pedestrians.size());
  for (const auto& pedestrian : message.pedestrians) {
    if (pedestrian.predicted_positions.size() !=
        pedestrian.prediction_stamps.size()) {
      throw std::invalid_argument(
        "pedestrian '" + pedestrian.id +
        "' prediction positions and timestamps have different lengths");
    }
    domain::PedestrianObservation converted;
    converted.id = pedestrian.id;
    converted.position = {
      pedestrian.position.x, pedestrian.position.y};
    converted.velocity_mps = {
      pedestrian.velocity.x, pedestrian.velocity.y};
    converted.confidence = pedestrian.confidence;
    std::copy(
      pedestrian.position_covariance.begin(),
      pedestrian.position_covariance.end(),
      converted.position_covariance.begin());
    converted.predicted_trajectory.reserve(
      pedestrian.predicted_positions.size());
    for (std::size_t index = 0U;
         index < pedestrian.predicted_positions.size(); ++index) {
      const rclcpp::Time predicted_at(
        pedestrian.prediction_stamps[index],
        received_at.get_clock_type());
      converted.predicted_trajectory.push_back({
        {
          pedestrian.predicted_positions[index].x,
          pedestrian.predicted_positions[index].y},
        std::chrono::nanoseconds(predicted_at.nanoseconds())});
    }
    result.pedestrians.push_back(std::move(converted));
  }
  result.validate();
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
