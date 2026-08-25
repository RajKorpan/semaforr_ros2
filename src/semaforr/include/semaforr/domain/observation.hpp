/**
 * @file observation.hpp
 * @brief Observation responsibilities.
 *
 * @details This file defines observation behavior for ROS-independent domain state
 * and value types. It centers on `LaserObservation`, `VelocityCommand`,
 * `RobotObservation`. Its package-relative location is
 * `include/semaforr/domain/observation.hpp`.
 */
#ifndef SEMAFORR_DOMAIN_OBSERVATION_HPP
#define SEMAFORR_DOMAIN_OBSERVATION_HPP

#include <chrono>
#include <cmath>
#include <optional>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/social.hpp>
#include <stdexcept>
#include <vector>

namespace semaforr::domain {

/**
 * @brief Encapsulates laser observation state and behavior for this
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
struct LaserObservation {
  Angle angle_min = Angle::zero();
  Angle angle_increment = Angle::zero();
  Distance minimum_range = Distance::zero();
  Distance maximum_range = Distance::zero();
  std::vector<double> ranges_m;

  /**
   * @brief Validates package content for this subsystem.
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
  void validate() const {
    if (maximum_range.meters() < minimum_range.meters()) {
      throw std::invalid_argument(
          "laser maximum range must not be below minimum range");
    }
    // Individual beams may legally be NaN, infinite, below range_min, or
    // above range_max. Consumers classify each beam rather than rejecting an
    // otherwise coherent scan; occupancy integration ignores invalid beams.
  }
};

/**
 * @brief Encapsulates velocity command state and behavior for this
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
struct VelocityCommand {
  double linear_mps = 0.0;
  double angular_radps = 0.0;

  /**
   * @brief Performs the finite operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool finite() const noexcept {
    return std::isfinite(linear_mps) && std::isfinite(angular_radps);
  }

  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const VelocityCommand&) const = default;
};

/**
 * @brief Encapsulates robot observation state and behavior for this
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
struct RobotObservation {
  Pose2D pose;
  LaserObservation laser;
  std::optional<CrowdObservation> crowd;
  std::chrono::steady_clock::time_point observed_at{};
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_OBSERVATION_HPP
