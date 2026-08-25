/**
 * @file motion_model.cpp
 * @brief Motion model responsibilities.
 *
 * @details This file implements motion model behavior for ROS-independent domain
 * state and value types. It records the declarations, settings, fixtures,
 * or guidance needed by that responsibility. Its package-relative location
 * is `src/domain/motion_model.cpp`.
 */
#include <cmath>
#include <semaforr/domain/motion_model.hpp>
#include <stdexcept>

namespace semaforr::domain {

/**
 * @brief Performs the expected pose after action operation for this
 * subsystem.
 *
 * Arguments:
 * - @p pose: Supplies pose input to the operation.
 * - @p action: Supplies action input to the operation.
 * - @p action_space: Supplies action space input to the operation.
 *
 * Returns:
 * - `Pose2D` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
Pose2D expectedPoseAfterAction(const Pose2D& pose, const Action& action,
                               const ActionSpace& action_space) {
  if (!action_space.contains(action)) {
    throw std::out_of_range("action is not in the configured action space");
  }

  Pose2D result = pose;
  const std::size_t index =
      action.magnitude_index() == 0U ? 0U : action.magnitude_index() - 1U;
  switch (action.type()) {
    case ActionType::Forward: {
      const double distance_m = action_space.move_distances_m().at(index);
      result.position.x_m += distance_m * std::cos(pose.heading.radians());
      result.position.y_m += distance_m * std::sin(pose.heading.radians());
      break;
    }
    case ActionType::TurnRight:
      result.heading = Angle(pose.heading.radians() -
                             action_space.rotation_angles_rad().at(index));
      break;
    case ActionType::TurnLeft:
      result.heading = Angle(pose.heading.radians() +
                             action_space.rotation_angles_rad().at(index));
      break;
    case ActionType::Pause:
      break;
  }
  return result;
}

/**
 * @brief Performs the laser endpoints operation for this subsystem.
 *
 * Arguments:
 * - @p pose: Supplies pose input to the operation.
 * - @p laser: Supplies laser input to the operation.
 *
 * Returns:
 * - `std::vector<Point2D>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<Point2D> laserEndpoints(const Pose2D& pose,
                                    const LaserObservation& laser) {
  laser.validate();
  std::vector<Point2D> endpoints;
  endpoints.reserve(laser.ranges_m.size());
  double angle = pose.heading.radians() + laser.angle_min.radians();
  for (const double range_m : laser.ranges_m) {
    const double endpoint_range_m =
        std::isfinite(range_m) ? range_m : laser.maximum_range.meters();
    endpoints.push_back(
        {pose.position.x_m + endpoint_range_m * std::cos(angle),
         pose.position.y_m + endpoint_range_m * std::sin(angle)});
    angle += laser.angle_increment.radians();
  }
  return endpoints;
}

/**
 * @brief Performs the goal reached operation for this subsystem.
 *
 * Arguments:
 * - @p pose: Supplies pose input to the operation.
 * - @p goal: Supplies goal input to the operation.
 * - @p tolerance: Supplies tolerance input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool goalReached(const Pose2D& pose, const Point2D& goal, Distance tolerance) {
  return distance(pose.position, goal).meters() <=
         tolerance.meters() + geometry_tolerance_m;
}

}  // namespace semaforr::domain
