#include <semaforr/ros/social_observation_buffer.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <semaforr/ros/MessageAdapters.hpp>

namespace semaforr::ros {

SocialObservationBuffer::SocialObservationBuffer(
  SocialObservationConfiguration configuration)
  : configuration_(std::move(configuration))
{
  if (configuration_.frame.empty()) {
    throw std::invalid_argument(
      "social observation frame must not be empty");
  }
  if (!std::isfinite(configuration_.maximum_age_s) ||
      configuration_.maximum_age_s <= 0.0) {
    throw std::invalid_argument(
      "social observation maximum age must be finite and positive");
  }
  if (!std::isfinite(configuration_.minimum_confidence) ||
      configuration_.minimum_confidence < 0.0 ||
      configuration_.minimum_confidence > 1.0) {
    throw std::invalid_argument(
      "social minimum confidence must be within [0, 1]");
  }
}

bool SocialObservationBuffer::accept(
  const social_context_msgs::msg::SocialObservation& message,
  const rclcpp::Time& received_at)
{
  if (message.header.frame_id != configuration_.frame) {
    observation_.reset();
    received_at_.reset();
    last_status_ = SocialObservationStatus::FrameMismatch;
    return false;
  }
  try {
    observation_ = toDomain(message, received_at);
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

SocialObservationStatus SocialObservationBuffer::status(
  const rclcpp::Time& now) const
{
  if (!observation_ || !received_at_) {
    return last_status_;
  }
  if (received_at_->get_clock_type() != now.get_clock_type() ||
      now < *received_at_) {
    return SocialObservationStatus::ClockReset;
  }
  const auto observed_at = rclcpp::Time(
    observation_->observed_at.count(), now.get_clock_type());
  if (now < observed_at) {
    return SocialObservationStatus::ClockReset;
  }
  if ((now - observed_at).seconds() > configuration_.maximum_age_s ||
      (now - *received_at_).seconds() > configuration_.maximum_age_s) {
    return SocialObservationStatus::Stale;
  }
  return SocialObservationStatus::Ready;
}

std::optional<domain::CrowdObservation>
SocialObservationBuffer::snapshot(const rclcpp::Time& now) const
{
  if (status(now) != SocialObservationStatus::Ready) {
    return std::nullopt;
  }
  auto result = *observation_;
  result.data_age = std::chrono::nanoseconds(
    now.nanoseconds() - result.observed_at.count());
  result.pedestrians.erase(
    std::remove_if(
      result.pedestrians.begin(),
      result.pedestrians.end(),
      [this](const auto& pedestrian) {
        return pedestrian.confidence <
          configuration_.minimum_confidence;
      }),
    result.pedestrians.end());
  // A valid empty observation is retained: it is negative evidence for the
  // visibility-normalized learned crowd field. Live advisors independently
  // require at least one sufficiently confident pedestrian.
  return result;
}

void SocialObservationBuffer::clear() noexcept
{
  observation_.reset();
  received_at_.reset();
  last_status_ = SocialObservationStatus::NoData;
}

std::string_view toString(SocialObservationStatus status) noexcept
{
  switch (status) {
    case SocialObservationStatus::NoData: return "no_data";
    case SocialObservationStatus::Ready: return "ready";
    case SocialObservationStatus::FrameMismatch: return "frame_mismatch";
    case SocialObservationStatus::Invalid: return "invalid";
    case SocialObservationStatus::Stale: return "stale";
    case SocialObservationStatus::ClockReset: return "clock_reset";
  }
  return "unknown";
}

}  // namespace semaforr::ros
