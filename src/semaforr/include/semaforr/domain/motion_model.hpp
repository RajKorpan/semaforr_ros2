/**
 * @file motion_model.hpp
 * @brief Motion model responsibilities.
 *
 * @details This file defines motion model behavior for ROS-independent domain state
 * and value types. It records the declarations, settings, fixtures, or
 * guidance needed by that responsibility. Its package-relative location is
 * `include/semaforr/domain/motion_model.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_MOTION_MODEL_HPP
#define SEMAFORR_DOMAIN_MOTION_MODEL_HPP

#include <semaforr/domain/observation.hpp>
#include <semaforr/domain/world_model.hpp>
#include <vector>

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
                               const ActionSpace& action_space);

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
                                    const LaserObservation& laser);

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
bool goalReached(const Pose2D& pose, const Point2D& goal, Distance tolerance);

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_MOTION_MODEL_HPP
