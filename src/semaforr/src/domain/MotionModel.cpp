#include <cmath>
#include <semaforr/domain/motion_model.hpp>
#include <stdexcept>

namespace semaforr::domain {

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

bool goalReached(const Pose2D& pose, const Point2D& goal, Distance tolerance) {
  return distance(pose.position, goal).meters() <=
         tolerance.meters() + geometry_tolerance_m;
}

}  // namespace semaforr::domain
