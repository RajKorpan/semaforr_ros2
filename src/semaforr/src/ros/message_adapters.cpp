/**
 * @file message_adapters.cpp
 * @brief Message adapters responsibilities.
 *
 * @details This file implements message adapters behavior for the ROS 2
 * composition and message-adaptation boundary. It records the declarations,
 * settings, fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/ros/message_adapters.cpp`.
 */
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <semaforr/ros/message_adapters.hpp>
#include <stdexcept>
#include <unordered_set>

namespace semaforr {
namespace ros {

namespace {

/**
 * @brief Performs the observation time operation for this subsystem.
 *
 * Arguments:
 * - @p stamp: Supplies stamp input to the operation.
 * - @p received_at: Supplies received at input to the operation.
 *
 * Returns:
 * - `domain::SocialTimestamp` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::SocialTimestamp observationTime(
    const builtin_interfaces::msg::Time& stamp,
    const rclcpp::Time& received_at) {
  const rclcpp::Time observed_at(stamp, received_at.get_clock_type());
  if (observed_at.nanoseconds() < 0 || received_at < observed_at) {
    throw std::invalid_argument(
        "social observation timestamp is in the future");
  }
  return std::chrono::nanoseconds(observed_at.nanoseconds());
}

/**
 * @brief Performs the observation age operation for this subsystem.
 *
 * Arguments:
 * - @p observed_at: Supplies observed at input to the operation.
 * - @p received_at: Supplies received at input to the operation.
 *
 * Returns:
 * - `std::chrono::nanoseconds` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::chrono::nanoseconds observationAge(domain::SocialTimestamp observed_at,
                                        const rclcpp::Time& received_at) {
  return std::chrono::nanoseconds(received_at.nanoseconds() -
                                  observed_at.count());
}

/**
 * @brief Performs the finite history operation for this subsystem.
 *
 * Arguments:
 * - @p x: Supplies x input to the operation.
 * - @p y: Supplies y input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool finiteHistory(const std::vector<float>& x, const std::vector<float>& y) {
  return x.size() == y.size() &&
         std::all_of(x.begin(), x.end(),
                     [](float value) { return std::isfinite(value); }) &&
         std::all_of(y.begin(), y.end(),
                     [](float value) { return std::isfinite(value); });
}

/**
 * @brief Performs the covariance operation for this subsystem.
 *
 * Arguments:
 * - @p confidence: Supplies confidence input to the operation.
 * - @p config: Supplies config input to the operation.
 *
 * Returns:
 * - `std::array<double, 4>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::array<double, 4> covariance(double confidence,
                                 const SocialAdapterConfiguration& config) {
  const double variance =
      config.default_position_variance /
      std::max(confidence, config.minimum_covariance_confidence);
  return {variance, 0.0, 0.0, variance};
}

/**
 * @brief Validates configuration for this subsystem.
 *
 * Arguments:
 * - @p config: Supplies config input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void validateConfiguration(const SocialAdapterConfiguration& config) {
  if (!std::isfinite(config.history_step_s) || config.history_step_s <= 0.0 ||
      !std::isfinite(config.default_position_variance) ||
      config.default_position_variance <= 0.0 ||
      !std::isfinite(config.minimum_covariance_confidence) ||
      config.minimum_covariance_confidence <= 0.0 ||
      config.minimum_covariance_confidence > 1.0 ||
      !std::isfinite(config.hunav_confidence) ||
      config.hunav_confidence < 0.0 || config.hunav_confidence > 1.0) {
    throw std::invalid_argument("invalid social adapter configuration");
  }
}

}  // namespace

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
    const SocialAdapterConfiguration& configuration) {
  validateConfiguration(configuration);
  domain::CrowdObservation result;
  result.frame_id = message.header.frame_id;
  result.observed_at = observationTime(message.header.stamp, received_at);
  result.data_age = observationAge(result.observed_at, received_at);
  result.provenance = "social_context_tracked";
  result.pedestrians.reserve(message.people.size());
  for (const auto& person : message.people) {
    if (!std::isfinite(person.x) || !std::isfinite(person.y) ||
        !std::isfinite(person.confidence) || person.confidence < 0.0F ||
        person.confidence > 1.0F ||
        !finiteHistory(person.history_x, person.history_y)) {
      continue;
    }
    domain::PedestrianObservation converted;
    converted.id = std::to_string(person.id);
    converted.position = {person.x, person.y};
    if (person.history_x.size() >= 2U) {
      const auto last = person.history_x.size() - 1U;
      converted.velocity_mps = {
          (person.history_x[last] - person.history_x[last - 1U]) /
              configuration.history_step_s,
          (person.history_y[last] - person.history_y[last - 1U]) /
              configuration.history_step_s};
    }
    converted.confidence = person.confidence;
    converted.position_covariance =
        covariance(converted.confidence, configuration);
    result.pedestrians.push_back(std::move(converted));
  }
  result.validate();
  return result;
}

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
    const SocialAdapterConfiguration& configuration) {
  validateConfiguration(configuration);
  domain::CrowdObservation result;
  result.frame_id = message.header.frame_id;
  result.observed_at = observationTime(message.header.stamp, received_at);
  result.data_age = observationAge(result.observed_at, received_at);
  result.provenance = "hunav_agents";
  for (const auto& agent : message.agents) {
    if (agent.type != hunav_msgs::msg::Agent::PERSON ||
        !std::isfinite(agent.position.position.x) ||
        !std::isfinite(agent.position.position.y) ||
        !std::isfinite(agent.velocity.linear.x) ||
        !std::isfinite(agent.velocity.linear.y)) {
      continue;
    }
    domain::PedestrianObservation converted;
    converted.id = std::to_string(agent.id);
    converted.position = {agent.position.position.x, agent.position.position.y};
    converted.velocity_mps = {agent.velocity.linear.x, agent.velocity.linear.y};
    converted.confidence = configuration.hunav_confidence;
    converted.position_covariance =
        covariance(converted.confidence, configuration);
    result.pedestrians.push_back(std::move(converted));
  }
  result.validate();
  return result;
}

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
    double minimum_confidence) {
  if (!std::isfinite(minimum_confidence) || minimum_confidence < 0.0 ||
      minimum_confidence > 1.0) {
    throw std::invalid_argument("formation confidence must be within [0, 1]");
  }
  std::vector<domain::FormationObservation> result;
  for (const auto& group : message.groups) {
    if (!std::isfinite(group.confidence) ||
        group.confidence < minimum_confidence ||
        !std::isfinite(group.center_x) || !std::isfinite(group.center_y) ||
        group.formation_type.empty() || group.member_ids.empty()) {
      continue;
    }
    domain::FormationObservation converted;
    converted.formation_type = group.formation_type;
    converted.center = {group.center_x, group.center_y};
    converted.confidence = group.confidence;
    for (const auto id : group.member_ids) {
      converted.member_ids.push_back(std::to_string(id));
    }
    try {
      converted.validate();
      result.push_back(std::move(converted));
    } catch (const std::invalid_argument&) {
      // Invalid groups are isolated from otherwise usable current-state data.
    }
  }
  return result;
}

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
    const std::string& encoded_frame_id) {
  const auto marker = encoded_frame_id.rfind("_pred_");
  if (marker == std::string::npos || marker == 0U ||
      marker + 6U >= encoded_frame_id.size()) {
    return std::nullopt;
  }
  const std::string id = encoded_frame_id.substr(0U, marker);
  const std::string step_text = encoded_frame_id.substr(marker + 6U);
  if (!std::all_of(step_text.begin(), step_text.end(),
                   [](unsigned char value) { return std::isdigit(value); })) {
    return std::nullopt;
  }
  try {
    const auto step = std::stoull(step_text);
    if (step > std::numeric_limits<std::size_t>::max()) return std::nullopt;
    return PredictionIdentity{id, static_cast<std::size_t>(step)};
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

}  // namespace ros
}  // namespace semaforr
