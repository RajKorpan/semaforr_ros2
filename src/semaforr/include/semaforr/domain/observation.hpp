#ifndef SEMAFORR_DOMAIN_OBSERVATION_HPP
#define SEMAFORR_DOMAIN_OBSERVATION_HPP

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/social.hpp>

namespace semaforr::domain {

struct LaserObservation {
  Angle angle_min = Angle::zero();
  Angle angle_increment = Angle::zero();
  Distance minimum_range = Distance::zero();
  Distance maximum_range = Distance::zero();
  std::vector<double> ranges_m;

  void validate() const
  {
    if (maximum_range.meters() < minimum_range.meters()) {
      throw std::invalid_argument(
        "laser maximum range must not be below minimum range");
    }
    for (const double range : ranges_m) {
      if (!std::isfinite(range) || range < minimum_range.meters() ||
          range > maximum_range.meters()) {
        throw std::invalid_argument(
          "laser ranges must be finite and within configured bounds");
      }
    }
  }
};

struct VelocityCommand {
  double linear_mps = 0.0;
  double angular_radps = 0.0;

  bool finite() const noexcept
  {
    return std::isfinite(linear_mps) && std::isfinite(angular_radps);
  }

  bool operator==(const VelocityCommand&) const = default;
};

struct RobotObservation {
  Pose2D pose;
  LaserObservation laser;
  std::optional<CrowdObservation> crowd;
  std::chrono::steady_clock::time_point observed_at{};
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_OBSERVATION_HPP
