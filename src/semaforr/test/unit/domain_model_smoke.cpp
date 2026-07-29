#include <semaforr/core/FORRAction.hpp>
#include <semaforr/domain/SensorTypes.hpp>

#include <cassert>
#include <cmath>

int main()
{
  semaforr::domain::LaserScan scan;
  scan.angle_min = -1.0F;
  scan.angle_increment = 0.5F;
  scan.ranges = {1.0F, 2.0F, 3.0F};

  assert(scan.ranges.size() == 3);
  assert(scan.ranges[1] == 2.0F);

  semaforr::domain::Pose pose;
  const double expected_yaw = 1.2;
  pose.orientation.z = std::sin(expected_yaw / 2.0);
  pose.orientation.w = std::cos(expected_yaw / 2.0);
  assert(std::abs(pose.yaw() - expected_yaw) < 1e-12);

  semaforr::domain::CrowdModel crowd;
  crowd.width = 2;
  crowd.height = 1;
  crowd.densities = {0.25, 0.75};
  assert(crowd.densities.size() ==
         static_cast<std::size_t>(crowd.width * crowd.height));

  const FORRAction action(FORWARD, 2);
  assert(action.type == FORWARD);
  assert(action.parameter == 2);
  return 0;
}
