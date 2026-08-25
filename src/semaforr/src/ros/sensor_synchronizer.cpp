/**
 * @file sensor_synchronizer.cpp
 * @brief Sensor synchronizer responsibilities.
 *
 * @details This file implements sensor synchronizer behavior for the ROS 2
 * composition and message-adaptation boundary. It records the
 * declarations, settings, fixtures, or guidance needed by that
 * responsibility. Its package-relative location is
 * `src/ros/sensor_synchronizer.cpp`.
 */
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/ros/sensor_synchronizer.hpp>
#include <stdexcept>
#include <utility>

namespace semaforr::ros {
namespace {

/**
 * @brief Performs the source stamp operation for this subsystem.
 *
 * Arguments:
 * - @p stamp: Supplies stamp input to the operation.
 * - @p received_at: Supplies received at input to the operation.
 *
 * Returns:
 * - `rclcpp::Time` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
rclcpp::Time sourceStamp(const builtin_interfaces::msg::Time& stamp,
                         const rclcpp::Time& received_at) {
  rclcpp::Time result(stamp, received_at.get_clock_type());
  return result.nanoseconds() == 0 ? received_at : result;
}

/**
 * @brief Performs the finite quaternion operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool finiteQuaternion(const geometry_msgs::msg::Quaternion& value) noexcept {
  const double norm = value.x * value.x + value.y * value.y +
                      value.z * value.z + value.w * value.w;
  return std::isfinite(norm) && norm > 1.0e-12;
}

}  // namespace

/**
 * @brief Performs the sensor synchronizer operation for this subsystem.
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
SensorSynchronizer::SensorSynchronizer(
    SensorSynchronizerConfiguration configuration)
    : configuration_(std::move(configuration)) {
  if (configuration_.pose_frame.empty()) {
    throw std::invalid_argument("sensor pose frame must not be empty");
  }
  if (configuration_.scan_frame.empty()) {
    throw std::invalid_argument("sensor scan frame must not be empty");
  }
  if (!std::isfinite(configuration_.maximum_age_s) ||
      configuration_.maximum_age_s <= 0.0) {
    throw std::invalid_argument(
        "sensor maximum age must be finite and positive");
  }
  if (!std::isfinite(configuration_.maximum_skew_s) ||
      configuration_.maximum_skew_s < 0.0) {
    throw std::invalid_argument(
        "sensor maximum skew must be finite and non-negative");
  }
}

/**
 * @brief Performs the accept pose operation for this subsystem.
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
bool SensorSynchronizer::acceptPose(
    const geometry_msgs::msg::PoseStamped& message,
    const rclcpp::Time& received_at) {
  if (message.header.frame_id != configuration_.pose_frame) {
    pose_.reset();
    pose_error_ = SensorStatus::PoseFrameMismatch;
    return false;
  }
  if (!std::isfinite(message.pose.position.x) ||
      !std::isfinite(message.pose.position.y) ||
      !finiteQuaternion(message.pose.orientation)) {
    pose_.reset();
    pose_error_ = SensorStatus::InvalidPose;
    return false;
  }

  const auto& orientation = message.pose.orientation;
  tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z,
                             orientation.w);
  quaternion.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  if (!std::isfinite(yaw)) {
    pose_.reset();
    pose_error_ = SensorStatus::InvalidPose;
    return false;
  }

  pose_ = PoseSample{
      domain::Pose2D{{message.pose.position.x, message.pose.position.y},
                     domain::Angle(yaw)},
      sourceStamp(message.header.stamp, received_at), received_at};
  pose_error_.reset();
  ++generation_;
  return true;
}

/**
 * @brief Performs the accept scan operation for this subsystem.
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
bool SensorSynchronizer::acceptScan(const sensor_msgs::msg::LaserScan& message,
                                    const rclcpp::Time& received_at) {
  if (message.header.frame_id != configuration_.scan_frame) {
    scan_.reset();
    scan_error_ = SensorStatus::ScanFrameMismatch;
    return false;
  }
  if (!std::isfinite(message.angle_min) || !std::isfinite(message.angle_max) ||
      !std::isfinite(message.angle_increment) ||
      message.angle_increment <= 0.0F || !std::isfinite(message.range_min) ||
      !std::isfinite(message.range_max) || message.range_min < 0.0F ||
      message.range_max < message.range_min) {
    scan_.reset();
    scan_error_ = SensorStatus::InvalidScan;
    return false;
  }

  std::vector<double> ranges;
  ranges.reserve(message.ranges.size());
  for (const float range : message.ranges) {
    ranges.push_back(static_cast<double>(range));
  }
  scan_ =
      ScanSample{domain::LaserObservation{
                     domain::Angle(message.angle_min),
                     domain::Angle(message.angle_increment),
                     domain::Distance(message.range_min),
                     domain::Distance(message.range_max), std::move(ranges)},
                 sourceStamp(message.header.stamp, received_at), received_at};
  scan_error_.reset();
  ++generation_;
  return true;
}

/**
 * @brief Performs the status operation for this subsystem.
 *
 * Arguments:
 * - @p now: Supplies now input to the operation.
 *
 * Returns:
 * - `SensorStatus` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
SensorStatus SensorSynchronizer::status(const rclcpp::Time& now) const {
  if (pose_error_) {
    return *pose_error_;
  }
  if (scan_error_) {
    return *scan_error_;
  }
  if (!pose_) {
    return SensorStatus::WaitingForPose;
  }
  if (!scan_) {
    return SensorStatus::WaitingForScan;
  }
  if (pose_->received_at.get_clock_type() != now.get_clock_type() ||
      scan_->received_at.get_clock_type() != now.get_clock_type() ||
      now < pose_->received_at || now < scan_->received_at) {
    return SensorStatus::ClockReset;
  }
  if ((now - pose_->received_at).seconds() > configuration_.maximum_age_s) {
    return SensorStatus::PoseStale;
  }
  if ((now - scan_->received_at).seconds() > configuration_.maximum_age_s) {
    return SensorStatus::ScanStale;
  }
  if (pose_->stamp.get_clock_type() != now.get_clock_type() ||
      scan_->stamp.get_clock_type() != now.get_clock_type() ||
      now < pose_->stamp || now < scan_->stamp) {
    return SensorStatus::ClockReset;
  }
  if ((now - pose_->stamp).seconds() > configuration_.maximum_age_s) {
    return SensorStatus::PoseStale;
  }
  if ((now - scan_->stamp).seconds() > configuration_.maximum_age_s) {
    return SensorStatus::ScanStale;
  }
  if (std::fabs((pose_->stamp - scan_->stamp).seconds()) >
      configuration_.maximum_skew_s) {
    return SensorStatus::Unsynchronized;
  }
  return SensorStatus::Ready;
}

/**
 * @brief Performs the snapshot operation for this subsystem.
 *
 * Arguments:
 * - @p now: Supplies now input to the operation.
 *
 * Returns:
 * - `std::optional<SynchronizedSensors>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<SynchronizedSensors> SensorSynchronizer::snapshot(
    const rclcpp::Time& now) const {
  if (status(now) != SensorStatus::Ready) {
    return std::nullopt;
  }
  return SynchronizedSensors{pose_->pose, scan_->scan, pose_->stamp,
                             scan_->stamp, generation_};
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
void SensorSynchronizer::clear() noexcept {
  pose_.reset();
  scan_.reset();
  pose_error_.reset();
  scan_error_.reset();
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
std::string_view toString(SensorStatus status) noexcept {
  switch (status) {
    case SensorStatus::WaitingForPose:
      return "waiting_for_pose";
    case SensorStatus::WaitingForScan:
      return "waiting_for_scan";
    case SensorStatus::PoseFrameMismatch:
      return "pose_frame_mismatch";
    case SensorStatus::ScanFrameMismatch:
      return "scan_frame_mismatch";
    case SensorStatus::InvalidPose:
      return "invalid_pose";
    case SensorStatus::InvalidScan:
      return "invalid_scan";
    case SensorStatus::PoseStale:
      return "pose_stale";
    case SensorStatus::ScanStale:
      return "scan_stale";
    case SensorStatus::Unsynchronized:
      return "unsynchronized";
    case SensorStatus::ClockReset:
      return "clock_reset";
    case SensorStatus::Ready:
      return "ready";
  }
  return "unknown";
}

}  // namespace semaforr::ros
