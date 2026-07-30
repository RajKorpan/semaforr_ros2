#ifndef SEMAFORR_DOMAIN_MOTION_MODEL_HPP
#define SEMAFORR_DOMAIN_MOTION_MODEL_HPP

#include <semaforr/domain/observation.hpp>
#include <semaforr/domain/world_model.hpp>
#include <vector>

namespace semaforr::domain {

Pose2D expectedPoseAfterAction(const Pose2D& pose, const Action& action,
                               const ActionSpace& action_space);

std::vector<Point2D> laserEndpoints(const Pose2D& pose,
                                    const LaserObservation& laser);

bool goalReached(const Pose2D& pose, const Point2D& goal, Distance tolerance);

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_MOTION_MODEL_HPP
