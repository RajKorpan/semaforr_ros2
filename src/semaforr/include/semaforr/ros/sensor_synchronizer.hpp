/**
 * @file sensor_synchronizer.hpp
 * @brief Sensor synchronizer responsibilities.
 *
 * @details This file defines sensor synchronizer behavior for the ROS 2 composition
 * and message-adaptation boundary. It centers on
 * `SensorSynchronizerConfiguration`, `SensorStatus`,
 * `SynchronizedSensors`, `SensorSynchronizer`, `PoseSample`, `ScanSample`.
 * Its package-relative location is
 * `include/semaforr/ros/sensor_synchronizer.hpp`.
 */
#ifndef SEMAFORR_ROS_SENSOR_SYNCHRONIZER_HPP
#define SEMAFORR_ROS_SENSOR_SYNCHRONIZER_HPP

#include <cstddef>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <rclcpp/time.hpp>
#include <semaforr/domain/observation.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <string_view>

namespace semaforr::ros {

/**
 * @brief Encapsulates sensor synchronizer configuration state and behavior
 * for this subsystem.
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
struct SensorSynchronizerConfiguration {
  std::string pose_frame{"map"};
  std::string scan_frame{"base_laser_link"};
  double maximum_age_s{0.5};
  double maximum_skew_s{0.1};
};

/**
 * @brief Enumerates the supported sensor status values used by this
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
enum class SensorStatus {
  WaitingForPose,
  WaitingForScan,
  PoseFrameMismatch,
  ScanFrameMismatch,
  InvalidPose,
  InvalidScan,
  PoseStale,
  ScanStale,
  Unsynchronized,
  ClockReset,
  Ready
};

/**
 * @brief Encapsulates synchronized sensors state and behavior for this
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
struct SynchronizedSensors {
  domain::Pose2D pose;
  domain::LaserObservation scan;
  rclcpp::Time pose_stamp;
  rclcpp::Time scan_stamp;
  std::size_t generation{0U};
};

/**
 * @brief Encapsulates sensor synchronizer state and behavior for this
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
class SensorSynchronizer {
 public:
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
  explicit SensorSynchronizer(SensorSynchronizerConfiguration configuration);

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
  bool acceptPose(const geometry_msgs::msg::PoseStamped& message,
                  const rclcpp::Time& received_at);
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
  bool acceptScan(const sensor_msgs::msg::LaserScan& message,
                  const rclcpp::Time& received_at);

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
  SensorStatus status(const rclcpp::Time& now) const;
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
  std::optional<SynchronizedSensors> snapshot(const rclcpp::Time& now) const;
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
  void clear() noexcept;

  /**
   * @brief Performs the configuration operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const SensorSynchronizerConfiguration&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const SensorSynchronizerConfiguration& configuration() const noexcept {
    return configuration_;
  }

 private:
  /**
   * @brief Encapsulates pose sample state and behavior for this subsystem.
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
  struct PoseSample {
    domain::Pose2D pose;
    rclcpp::Time stamp;
    rclcpp::Time received_at;
  };

  /**
   * @brief Encapsulates scan sample state and behavior for this subsystem.
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
  struct ScanSample {
    domain::LaserObservation scan;
    rclcpp::Time stamp;
    rclcpp::Time received_at;
  };

  SensorSynchronizerConfiguration configuration_;
  std::optional<PoseSample> pose_;
  std::optional<ScanSample> scan_;
  std::optional<SensorStatus> pose_error_;
  std::optional<SensorStatus> scan_error_;
  std::size_t generation_{0U};
};

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
std::string_view toString(SensorStatus status) noexcept;

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_SENSOR_SYNCHRONIZER_HPP
