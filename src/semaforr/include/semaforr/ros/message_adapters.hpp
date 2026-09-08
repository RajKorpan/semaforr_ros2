/**
 * @file message_adapters.hpp
 * @brief Message adapters responsibilities.
 *
 * @details This file defines message adapters behavior for the ROS 2
 * composition and message-adaptation boundary. It centers on
 * `SocialAdapterConfiguration`, `PredictionIdentity`. Its package-relative
 * location is `include/semaforr/ros/message_adapters.hpp`.
 */
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

/**
 * @brief Encapsulates social adapter configuration state and behavior for
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct SocialAdapterConfiguration {
  double history_step_s{0.1};
  double default_position_variance{0.09};
  double minimum_covariance_confidence{0.05};
  double hunav_confidence{1.0};
};

/**
 * @brief Encapsulates prediction identity state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct PredictionIdentity {
  std::string pedestrian_id;
  std::size_t step{0U};
};

/**
 * @brief Performs the tracked people to domain operation for this
 * subsystem.
 *
 * Arguments:
 * - @p message: Supplies message input to the operation.
 * - @p received_at: Supplies received at input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `domain::CrowdObservation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::CrowdObservation trackedPeopleToDomain(
    const social_context_msgs::msg::TrackedPersonArray& message,
    const rclcpp::Time& received_at,
    const SocialAdapterConfiguration& configuration = {});

/**
 * @brief Performs the hunav agents to domain operation for this subsystem.
 *
 * Arguments:
 * - @p message: Supplies message input to the operation.
 * - @p received_at: Supplies received at input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `domain::CrowdObservation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::CrowdObservation hunavAgentsToDomain(
    const hunav_msgs::msg::Agents& message, const rclcpp::Time& received_at,
    const SocialAdapterConfiguration& configuration = {});

/**
 * @brief Performs the formations to domain operation for this subsystem.
 *
 * Arguments:
 * - @p message: Supplies message input to the operation.
 * - @p minimum_confidence: Supplies minimum confidence input to the
 * operation.
 *
 * Returns:
 * - `std::vector<domain::FormationObservation>` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<domain::FormationObservation> formationsToDomain(
    const social_context_msgs::msg::FormationGroupArray& message,
    double minimum_confidence);

/**
 * @brief Parses prediction identity for this subsystem.
 *
 * Arguments:
 * - @p encoded_frame_id: Supplies encoded frame id input to the operation.
 *
 * Returns:
 * - `std::optional<PredictionIdentity>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<PredictionIdentity> parsePredictionIdentity(
    const std::string& encoded_frame_id);

}  // namespace ros
}  // namespace semaforr

#endif
