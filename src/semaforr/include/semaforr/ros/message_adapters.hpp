#ifndef SEMAFORR_ROS_MESSAGE_ADAPTERS_H
#define SEMAFORR_ROS_MESSAGE_ADAPTERS_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hunav_msgs/msg/agents.hpp>
#include <optional>
#include <rclcpp/time.hpp>
#include <semaforr/domain/social.hpp>
#include <social_context_msgs/msg/formation_group_array.hpp>
#include <social_context_msgs/msg/tracked_person_array.hpp>
#include <string>

namespace semaforr {
namespace ros {

struct SocialAdapterConfiguration {
  double history_step_s{0.1};
  double default_position_variance{0.09};
  double minimum_covariance_confidence{0.05};
  double hunav_confidence{1.0};
};

struct PredictionIdentity {
  std::string pedestrian_id;
  std::size_t step{0U};
};

domain::CrowdObservation trackedPeopleToDomain(
    const social_context_msgs::msg::TrackedPersonArray& message,
    const rclcpp::Time& received_at,
    const SocialAdapterConfiguration& configuration = {});

domain::CrowdObservation hunavAgentsToDomain(
    const hunav_msgs::msg::Agents& message, const rclcpp::Time& received_at,
    const SocialAdapterConfiguration& configuration = {});

std::vector<domain::FormationObservation> formationsToDomain(
    const social_context_msgs::msg::FormationGroupArray& message,
    double minimum_confidence);

std::optional<PredictionIdentity> parsePredictionIdentity(
    const std::string& encoded_frame_id);

}  // namespace ros
}  // namespace semaforr

#endif
