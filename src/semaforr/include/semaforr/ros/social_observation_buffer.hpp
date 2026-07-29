#ifndef SEMAFORR_ROS_SOCIAL_OBSERVATION_BUFFER_HPP
#define SEMAFORR_ROS_SOCIAL_OBSERVATION_BUFFER_HPP

#include <optional>
#include <string>
#include <string_view>

#include <rclcpp/time.hpp>
#include <social_context_msgs/msg/social_observation.hpp>

#include <semaforr/domain/social.hpp>

namespace semaforr::ros {

struct SocialObservationConfiguration {
  std::string frame{"map"};
  double maximum_age_s{0.75};
  double minimum_confidence{0.25};
};

enum class SocialObservationStatus {
  NoData,
  Ready,
  FrameMismatch,
  Invalid,
  Stale,
  ClockReset
};

class SocialObservationBuffer {
public:
  explicit SocialObservationBuffer(
    SocialObservationConfiguration configuration);

  bool accept(
    const social_context_msgs::msg::SocialObservation& message,
    const rclcpp::Time& received_at);
  SocialObservationStatus status(const rclcpp::Time& now) const;
  std::optional<domain::CrowdObservation> snapshot(
    const rclcpp::Time& now) const;
  void clear() noexcept;

private:
  SocialObservationConfiguration configuration_;
  std::optional<domain::CrowdObservation> observation_;
  std::optional<rclcpp::Time> received_at_;
  SocialObservationStatus last_status_{SocialObservationStatus::NoData};
};

std::string_view toString(SocialObservationStatus status) noexcept;

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_SOCIAL_OBSERVATION_BUFFER_HPP
