#include <algorithm>
#include <chrono>
#include <semaforr/ros/message_adapters.hpp>
#include <stdexcept>

namespace semaforr {
namespace ros {

domain::CrowdObservation toDomain(
    const social_context_msgs::msg::SocialObservation& message,
    const rclcpp::Time& received_at) {
  const rclcpp::Time observed_at(message.header.stamp,
                                 received_at.get_clock_type());
  if (observed_at.nanoseconds() < 0 || received_at < observed_at) {
    throw std::invalid_argument(
        "social observation timestamp is in the future");
  }
  domain::CrowdObservation result;
  result.frame_id = message.header.frame_id;
  result.observed_at = std::chrono::nanoseconds(observed_at.nanoseconds());
  result.data_age =
      std::chrono::nanoseconds((received_at - observed_at).nanoseconds());
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
    converted.position = {pedestrian.position.x, pedestrian.position.y};
    converted.velocity_mps = {pedestrian.velocity.x, pedestrian.velocity.y};
    converted.confidence = pedestrian.confidence;
    std::copy(pedestrian.position_covariance.begin(),
              pedestrian.position_covariance.end(),
              converted.position_covariance.begin());
    converted.predicted_trajectory.reserve(
        pedestrian.predicted_positions.size());
    for (std::size_t index = 0U; index < pedestrian.predicted_positions.size();
         ++index) {
      const rclcpp::Time predicted_at(pedestrian.prediction_stamps[index],
                                      received_at.get_clock_type());
      converted.predicted_trajectory.push_back(
          {{pedestrian.predicted_positions[index].x,
            pedestrian.predicted_positions[index].y},
           std::chrono::nanoseconds(predicted_at.nanoseconds())});
    }
    result.pedestrians.push_back(std::move(converted));
  }
  result.validate();
  return result;
}

}  // namespace ros
}  // namespace semaforr
