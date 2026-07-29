#ifndef SEMAFORR_SPATIAL_LEARNING_GEOMETRY_HPP
#define SEMAFORR_SPATIAL_LEARNING_GEOMETRY_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include <semaforr/domain/observation.hpp>

namespace semaforr::spatial::detail {

inline std::vector<domain::Point2D> laserEndpoints(
  const domain::RobotObservation& observation)
{
  std::vector<domain::Point2D> endpoints;
  endpoints.reserve(observation.laser.ranges_m.size());
  const double heading = observation.pose.heading.radians();
  const double first_angle = observation.laser.angle_min.radians();
  const double increment = observation.laser.angle_increment.radians();
  for (std::size_t index = 0U;
       index < observation.laser.ranges_m.size(); ++index) {
    const double range = observation.laser.ranges_m[index];
    const double angle =
      heading + first_angle + increment * static_cast<double>(index);
    endpoints.push_back({
      observation.pose.position.x_m + range * std::cos(angle),
      observation.pose.position.y_m + range * std::sin(angle)});
  }
  return endpoints;
}

inline bool near(
  const domain::Point2D& first,
  const domain::Point2D& second,
  double tolerance_m)
{
  return domain::distance(first, second).meters() <= tolerance_m;
}

inline bool equivalent(
  const domain::Segment2D& first,
  const domain::Segment2D& second,
  double tolerance_m)
{
  return
    (near(first.start, second.start, tolerance_m) &&
     near(first.end, second.end, tolerance_m)) ||
    (near(first.start, second.end, tolerance_m) &&
     near(first.end, second.start, tolerance_m));
}

inline void appendUnique(
  std::vector<domain::Segment2D>& segments,
  domain::Segment2D candidate,
  double tolerance_m)
{
  if (candidate.length().meters() <= domain::geometry_tolerance_m) {
    return;
  }
  if (std::none_of(
      segments.begin(), segments.end(),
      [&](const auto& existing) {
        return equivalent(existing, candidate, tolerance_m);
      })) {
    segments.push_back(std::move(candidate));
  }
}

}  // namespace semaforr::spatial::detail

#endif  // SEMAFORR_SPATIAL_LEARNING_GEOMETRY_HPP
