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

struct SensorSynchronizerConfiguration {
  std::string pose_frame{"map"};
  std::string scan_frame{"base_laser_link"};
  double maximum_age_s{0.5};
  double maximum_skew_s{0.1};
};

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

struct SynchronizedSensors {
  domain::Pose2D pose;
  domain::LaserObservation scan;
  rclcpp::Time pose_stamp;
  rclcpp::Time scan_stamp;
  std::size_t generation{0U};
};

class SensorSynchronizer {
 public:
  explicit SensorSynchronizer(SensorSynchronizerConfiguration configuration);

  bool acceptPose(const geometry_msgs::msg::PoseStamped& message,
                  const rclcpp::Time& received_at);
  bool acceptScan(const sensor_msgs::msg::LaserScan& message,
                  const rclcpp::Time& received_at);

  SensorStatus status(const rclcpp::Time& now) const;
  std::optional<SynchronizedSensors> snapshot(const rclcpp::Time& now) const;
  void clear() noexcept;

  const SensorSynchronizerConfiguration& configuration() const noexcept {
    return configuration_;
  }

 private:
  struct PoseSample {
    domain::Pose2D pose;
    rclcpp::Time stamp;
    rclcpp::Time received_at;
  };

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

std::string_view toString(SensorStatus status) noexcept;

}  // namespace semaforr::ros

#endif  // SEMAFORR_ROS_SENSOR_SYNCHRONIZER_HPP
