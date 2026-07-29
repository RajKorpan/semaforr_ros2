#ifndef SEMAFORR_DOMAIN_SENSOR_TYPES_H
#define SEMAFORR_DOMAIN_SENSOR_TYPES_H

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace semaforr {
namespace domain {

struct Header
{
  std::int32_t stamp_sec = 0;
  std::uint32_t stamp_nanosec = 0;
  std::string frame_id;
};

struct LaserScan
{
  Header header;
  float angle_min = 0.0F;
  float angle_max = 0.0F;
  float angle_increment = 0.0F;
  float time_increment = 0.0F;
  float scan_time = 0.0F;
  float range_min = 0.0F;
  float range_max = 0.0F;
  std::vector<float> ranges;
  std::vector<float> intensities;
};

struct Point
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Pose
{
  Point position;
  Quaternion orientation;

  double yaw() const
  {
    const double sin_yaw =
      2.0 * (orientation.w * orientation.z +
             orientation.x * orientation.y);
    const double cos_yaw =
      1.0 - 2.0 * (orientation.y * orientation.y +
                   orientation.z * orientation.z);
    return std::atan2(sin_yaw, cos_yaw);
  }
};

struct PoseArray
{
  Header header;
  std::vector<Pose> poses;
};

}  // namespace domain
}  // namespace semaforr

#endif
