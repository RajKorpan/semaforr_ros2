/**
 * @file social_observation_buffer.cpp
 * @brief Social observation buffer responsibilities.
 *
 * @details This file implements social observation buffer behavior for the ROS
 * 2 composition and message-adaptation boundary. It records the declarations,
 * settings, fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/ros/social_observation_buffer.cpp`.
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <semaforr/ros/social_observation_buffer.hpp>
#include <stdexcept>
#include <utility>

namespace semaforr::ros {
namespace {

/**
 * @brief Performs the elapsed exceeds operation for this subsystem.
 *
 * Arguments:
 * - @p now: Supplies now input to the operation.
 * - @p then: Supplies then input to the operation.
 * - @p seconds: Supplies seconds input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool elapsedExceeds(const rclcpp::Time& now, const rclcpp::Time& then,
                    double seconds) {
  return now.get_clock_type() != then.get_clock_type() || now < then ||
         (now - then).seconds() > seconds;
}

}  // namespace

/**
 * @brief Performs the social input mode from string operation for this
 * subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `SocialInputMode` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
SocialInputMode socialInputModeFromString(std::string_view value) {
  if (value == "none") return SocialInputMode::None;
  if (value == "tracked") return SocialInputMode::Tracked;
  if (value == "hunav") return SocialInputMode::Hunav;
  throw std::invalid_argument(
      "social input mode must be 'none', 'tracked', or 'hunav'");
}

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p mode: Supplies mode input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(SocialInputMode mode) noexcept {
  switch (mode) {
    case SocialInputMode::None:
      return "none";
    case SocialInputMode::Tracked:
      return "tracked";
    case SocialInputMode::Hunav:
      return "hunav";
  }
  return "unknown";
}

/**
 * @brief Performs the social observation buffer operation for this
 * subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
SocialObservationBuffer::SocialObservationBuffer(
    SocialObservationConfiguration configuration)
    : configuration_(std::move(configuration)) {
  if (configuration_.frame.empty()) {
    throw std::invalid_argument("social observation frame must not be empty");
  }
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  if (!positive(configuration_.current_maximum_age_s) ||
      !positive(configuration_.prediction_maximum_age_s) ||
      !positive(configuration_.formation_maximum_age_s) ||
      !positive(configuration_.prediction_step_s) ||
      configuration_.prediction_steps == 0U) {
    throw std::invalid_argument(
        "social ages, prediction step, and prediction count must be positive");
  }
  if (!std::isfinite(configuration_.minimum_confidence) ||
      configuration_.minimum_confidence < 0.0 ||
      configuration_.minimum_confidence > 1.0 ||
      !std::isfinite(configuration_.minimum_formation_confidence) ||
      configuration_.minimum_formation_confidence < 0.0 ||
      configuration_.minimum_formation_confidence > 1.0) {
    throw std::invalid_argument(
        "social confidence thresholds must be within [0, 1]");
  }
}

/**
 * @brief Performs the source matches operation for this subsystem.
 *
 * Arguments:
 * - @p provenance: Supplies provenance input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool SocialObservationBuffer::sourceMatches(
    std::string_view provenance) const noexcept {
  switch (configuration_.input_mode) {
    case SocialInputMode::None:
      return false;
    case SocialInputMode::Tracked:
      return provenance == "social_context_tracked";
    case SocialInputMode::Hunav:
      return provenance == "hunav_agents";
  }
  return false;
}

/**
 * @brief Performs the accept operation for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 * - @p received_at: Supplies received at input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool SocialObservationBuffer::accept(domain::CrowdObservation observation,
                                     const rclcpp::Time& received_at) {
  if (!sourceMatches(observation.provenance)) {
    observation_.reset();
    received_at_.reset();
    last_status_ = SocialObservationStatus::SourceMismatch;
    return false;
  }
  if (observation.frame_id != configuration_.frame) {
    observation_.reset();
    received_at_.reset();
    last_status_ = SocialObservationStatus::FrameMismatch;
    return false;
  }
  try {
    observation.validate();
    recordLifecycle(observation);
    observation_ = std::move(observation);
    received_at_ = received_at;
    last_status_ = SocialObservationStatus::Ready;
    return true;
  } catch (const std::exception&) {
    observation_.reset();
    received_at_.reset();
    last_status_ = SocialObservationStatus::Invalid;
    return false;
  }
}

/**
 * @brief Performs the accept tracked operation for this subsystem.
 *
 * Arguments:
 * - @p message: Supplies message input to the operation.
 * - @p received_at: Supplies received at input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool SocialObservationBuffer::acceptTracked(
    const social_context_msgs::msg::TrackedPersonArray& message,
    const rclcpp::Time& received_at) {
  if (configuration_.input_mode != SocialInputMode::Tracked) {
    observation_.reset();
    received_at_.reset();
    last_status_ = SocialObservationStatus::SourceMismatch;
    return false;
  }
  try {
    return accept(
        trackedPeopleToDomain(message, received_at, configuration_.adapter),
        received_at);
  } catch (const std::exception&) {
    observation_.reset();
    received_at_.reset();
    last_status_ = SocialObservationStatus::Invalid;
    return false;
  }
}

/**
 * @brief Performs the accept hunav operation for this subsystem.
 *
 * Arguments:
 * - @p message: Supplies message input to the operation.
 * - @p received_at: Supplies received at input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool SocialObservationBuffer::acceptHunav(
    const hunav_msgs::msg::Agents& message, const rclcpp::Time& received_at) {
  if (configuration_.input_mode != SocialInputMode::Hunav) {
    observation_.reset();
    received_at_.reset();
    last_status_ = SocialObservationStatus::SourceMismatch;
    return false;
  }
  try {
    return accept(
        hunavAgentsToDomain(message, received_at, configuration_.adapter),
        received_at);
  } catch (const std::exception&) {
    observation_.reset();
    received_at_.reset();
    last_status_ = SocialObservationStatus::Invalid;
    return false;
  }
}

/**
 * @brief Performs the accept prediction operation for this subsystem.
 *
 * Arguments:
 * - @p message: Supplies message input to the operation.
 * - @p received_at: Supplies received at input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool SocialObservationBuffer::acceptPrediction(
    const geometry_msgs::msg::PoseStamped& message,
    const rclcpp::Time& received_at) {
  if (configuration_.input_mode == SocialInputMode::None) return false;
  const auto identity = parsePredictionIdentity(message.header.frame_id);
  const rclcpp::Time generated_at(message.header.stamp,
                                  received_at.get_clock_type());
  if (!identity || identity->step >= configuration_.prediction_steps ||
      generated_at > received_at ||
      elapsedExceeds(received_at, generated_at,
                     configuration_.prediction_maximum_age_s) ||
      !std::isfinite(message.pose.position.x) ||
      !std::isfinite(message.pose.position.y)) {
    return false;
  }
  if (!observation_ || std::none_of(observation_->pedestrians.begin(),
                                    observation_->pedestrians.end(),
                                    [&identity](const auto& pedestrian) {
                                      return pedestrian.id ==
                                             identity->pedestrian_id;
                                    })) {
    return false;
  }
  auto& cycle = predictions_[identity->pedestrian_id];
  if (identity->step == 0U) {
    if (!cycle.steps.empty()) {
      const bool previous_complete =
          cycle.steps.size() == configuration_.prediction_steps;
      const bool previous_expired =
          cycle.received_at && elapsedExceeds(received_at, *cycle.received_at,
                                              configuration_.prediction_step_s);
      if (!previous_complete && !previous_expired) return false;
    }
    cycle.steps.clear();
  } else if (cycle.steps.contains(identity->step)) {
    return false;
  }
  cycle.steps[identity->step] = {message.pose.position.x,
                                 message.pose.position.y};
  cycle.received_at = received_at;
  return true;
}

/**
 * @brief Performs the accept formations operation for this subsystem.
 *
 * Arguments:
 * - @p message: Supplies message input to the operation.
 * - @p received_at: Supplies received at input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool SocialObservationBuffer::acceptFormations(
    const social_context_msgs::msg::FormationGroupArray& message,
    const rclcpp::Time& received_at) {
  const rclcpp::Time observed_at(message.header.stamp,
                                 received_at.get_clock_type());
  if (message.header.frame_id != configuration_.frame ||
      observed_at > received_at ||
      elapsedExceeds(received_at, observed_at,
                     configuration_.formation_maximum_age_s)) {
    return false;
  }
  try {
    formations_ = formationsToDomain(
        message, configuration_.minimum_formation_confidence);
    formations_received_at_ = received_at;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

/**
 * @brief Records lifecycle for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void SocialObservationBuffer::recordLifecycle(
    const domain::CrowdObservation& observation) {
  std::unordered_set<std::string> next;
  for (const auto& pedestrian : observation.pedestrians) {
    next.insert(pedestrian.id);
    if (!active_ids_.contains(pedestrian.id)) {
      const auto type = seen_ids_.contains(pedestrian.id)
                            ? TrackLifecycleEvent::Type::Reappeared
                            : TrackLifecycleEvent::Type::Appeared;
      lifecycle_events_.push_back({pedestrian.id, type});
      seen_ids_.insert(pedestrian.id);
    }
  }
  for (const auto& id : active_ids_) {
    if (!next.contains(id)) {
      lifecycle_events_.push_back({id, TrackLifecycleEvent::Type::Disappeared});
    }
  }
  active_ids_ = std::move(next);
}

/**
 * @brief Performs the status operation for this subsystem.
 *
 * Arguments:
 * - @p now: Supplies now input to the operation.
 *
 * Returns:
 * - `SocialObservationStatus` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
SocialObservationStatus SocialObservationBuffer::status(
    const rclcpp::Time& now) const {
  if (!observation_ || !received_at_) return last_status_;
  if (received_at_->get_clock_type() != now.get_clock_type() ||
      now < *received_at_) {
    return SocialObservationStatus::ClockReset;
  }
  const auto observed_at =
      rclcpp::Time(observation_->observed_at.count(), now.get_clock_type());
  if (now < observed_at) return SocialObservationStatus::ClockReset;
  if (elapsedExceeds(now, observed_at, configuration_.current_maximum_age_s) ||
      elapsedExceeds(now, *received_at_,
                     configuration_.current_maximum_age_s)) {
    return SocialObservationStatus::Stale;
  }
  return SocialObservationStatus::Ready;
}

/**
 * @brief Performs the snapshot operation for this subsystem.
 *
 * Arguments:
 * - @p now: Supplies now input to the operation.
 *
 * Returns:
 * - `std::optional<domain::CrowdObservation>` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<domain::CrowdObservation> SocialObservationBuffer::snapshot(
    const rclcpp::Time& now) const {
  if (status(now) != SocialObservationStatus::Ready) return std::nullopt;
  auto result = *observation_;
  result.data_age =
      std::chrono::nanoseconds(now.nanoseconds() - result.observed_at.count());
  result.pedestrians.erase(
      std::remove_if(result.pedestrians.begin(), result.pedestrians.end(),
                     [this](const auto& pedestrian) {
                       return pedestrian.confidence <
                              configuration_.minimum_confidence;
                     }),
      result.pedestrians.end());

  for (auto& pedestrian : result.pedestrians) {
    const auto found = predictions_.find(pedestrian.id);
    const bool complete =
        found != predictions_.end() && found->second.received_at &&
        found->second.steps.size() == configuration_.prediction_steps &&
        !elapsedExceeds(now, *found->second.received_at,
                        configuration_.prediction_maximum_age_s);
    if (complete) {
      for (std::size_t step = 0U; step < configuration_.prediction_steps;
           ++step) {
        const auto point = found->second.steps.find(step);
        if (point == found->second.steps.end()) break;
        const auto horizon =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(configuration_.prediction_step_s *
                                              static_cast<double>(step + 1U)));
        pedestrian.predicted_trajectory.push_back(
            {point->second, result.observed_at + horizon});
      }
      if (pedestrian.predicted_trajectory.size() ==
          configuration_.prediction_steps) {
        pedestrian.prediction_source = "gst";
        continue;
      }
      pedestrian.predicted_trajectory.clear();
    }
    if (configuration_.constant_velocity_fallback) {
      for (std::size_t step = 0U; step < configuration_.prediction_steps;
           ++step) {
        const double horizon_s =
            configuration_.prediction_step_s * static_cast<double>(step + 1U);
        const auto horizon =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(horizon_s));
        pedestrian.predicted_trajectory.push_back(
            {{pedestrian.position.x_m + pedestrian.velocity_mps.x_m * horizon_s,
              pedestrian.position.y_m +
                  pedestrian.velocity_mps.y_m * horizon_s},
             result.observed_at + horizon});
      }
      pedestrian.prediction_source = "constant_velocity";
    }
  }

  if (formations_received_at_ &&
      !elapsedExceeds(now, *formations_received_at_,
                      configuration_.formation_maximum_age_s)) {
    std::unordered_set<std::string> current_ids;
    for (const auto& pedestrian : result.pedestrians)
      current_ids.insert(pedestrian.id);
    for (const auto& formation : formations_) {
      const bool known = std::all_of(
          formation.member_ids.begin(), formation.member_ids.end(),
          [&current_ids](const auto& id) { return current_ids.contains(id); });
      if (!known) continue;
      const std::size_t index = result.formations.size();
      result.formations.push_back(formation);
      for (auto& pedestrian : result.pedestrians) {
        if (std::find(formation.member_ids.begin(), formation.member_ids.end(),
                      pedestrian.id) != formation.member_ids.end()) {
          pedestrian.formation_index = index;
        }
      }
    }
  }
  result.validate();
  return result;
}

std::
    vector<TrackLifecycleEvent>
    /**
     * @brief Performs the take lifecycle events operation for this subsystem.
     *
     * Arguments:
     * - None.
     *
     * Returns:
     * - No value; effects are applied to owned state or outputs.
     *
     * Exceptions:
     * - None documented; validation or dependency failures may propagate.
     */
    SocialObservationBuffer::takeLifecycleEvents() {
  auto result = std::move(lifecycle_events_);
  lifecycle_events_.clear();
  return result;
}

/**
 * @brief Clears package content for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void SocialObservationBuffer::clear() noexcept {
  observation_.reset();
  received_at_.reset();
  predictions_.clear();
  formations_.clear();
  formations_received_at_.reset();
  active_ids_.clear();
  last_status_ = SocialObservationStatus::NoData;
}

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p status: Supplies status input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(SocialObservationStatus status) noexcept {
  switch (status) {
    case SocialObservationStatus::NoData:
      return "no_data";
    case SocialObservationStatus::Ready:
      return "ready";
    case SocialObservationStatus::SourceMismatch:
      return "source_mismatch";
    case SocialObservationStatus::FrameMismatch:
      return "frame_mismatch";
    case SocialObservationStatus::Invalid:
      return "invalid";
    case SocialObservationStatus::Stale:
      return "stale";
    case SocialObservationStatus::ClockReset:
      return "clock_reset";
  }
  return "unknown";
}

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p type: Supplies type input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(TrackLifecycleEvent::Type type) noexcept {
  switch (type) {
    case TrackLifecycleEvent::Type::Appeared:
      return "appeared";
    case TrackLifecycleEvent::Type::Disappeared:
      return "disappeared";
    case TrackLifecycleEvent::Type::Reappeared:
      return "reappeared";
  }
  return "unknown";
}

}  // namespace semaforr::ros
