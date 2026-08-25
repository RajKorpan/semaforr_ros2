/**
 * @file learning_geometry.hpp
 * @brief Learning geometry responsibilities.
 *
 * @details This file defines learning geometry behavior for learned spatial
 * representations and their lifecycle. It records the declarations,
 * settings, fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/spatial/learning_geometry.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_LEARNING_GEOMETRY_HPP
#define SEMAFORR_SPATIAL_LEARNING_GEOMETRY_HPP

#include <algorithm>
#include <cmath>
#include <semaforr/domain/observation.hpp>
#include <vector>

namespace semaforr::spatial::detail {

/**
 * @brief Performs the laser endpoints operation for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 *
 * Returns:
 * - `std::vector<domain::Point2D>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
inline std::vector<domain::Point2D> laserEndpoints(
    const domain::RobotObservation& observation) {
  std::vector<domain::Point2D> endpoints;
  endpoints.reserve(observation.laser.ranges_m.size());
  const double heading = observation.pose.heading.radians();
  const double first_angle = observation.laser.angle_min.radians();
  const double increment = observation.laser.angle_increment.radians();
  for (std::size_t index = 0U; index < observation.laser.ranges_m.size();
       ++index) {
    const double range = observation.laser.ranges_m[index];
    const double angle =
        heading + first_angle + increment * static_cast<double>(index);
    endpoints.push_back(
        {observation.pose.position.x_m + range * std::cos(angle),
         observation.pose.position.y_m + range * std::sin(angle)});
  }
  return endpoints;
}

/**
 * @brief Performs the near operation for this subsystem.
 *
 * Arguments:
 * - @p first: Supplies first input to the operation.
 * - @p second: Supplies second input to the operation.
 * - @p tolerance_m: Supplies tolerance m input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
inline bool near(const domain::Point2D& first, const domain::Point2D& second,
                 double tolerance_m) {
  return domain::distance(first, second).meters() <= tolerance_m;
}

/**
 * @brief Performs the equivalent operation for this subsystem.
 *
 * Arguments:
 * - @p first: Supplies first input to the operation.
 * - @p second: Supplies second input to the operation.
 * - @p tolerance_m: Supplies tolerance m input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
inline bool equivalent(const domain::Segment2D& first,
                       const domain::Segment2D& second, double tolerance_m) {
  return (near(first.start, second.start, tolerance_m) &&
          near(first.end, second.end, tolerance_m)) ||
         (near(first.start, second.end, tolerance_m) &&
          near(first.end, second.start, tolerance_m));
}

/**
 * @brief Performs the append unique operation for this subsystem.
 *
 * Arguments:
 * - @p segments: Supplies segments input to the operation.
 * - @p candidate: Supplies candidate input to the operation.
 * - @p tolerance_m: Supplies tolerance m input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
inline void appendUnique(std::vector<domain::Segment2D>& segments,
                         domain::Segment2D candidate, double tolerance_m) {
  if (candidate.length().meters() <= domain::geometry_tolerance_m) {
    return;
  }
  if (std::none_of(segments.begin(), segments.end(), [&](const auto& existing) {
        return equivalent(existing, candidate, tolerance_m);
      })) {
    segments.push_back(std::move(candidate));
  }
}

}  // namespace semaforr::spatial::detail

#endif  // SEMAFORR_SPATIAL_LEARNING_GEOMETRY_HPP
