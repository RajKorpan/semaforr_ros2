#include <cmath>
#include <semaforr/spatial/learners/door_exit_learner.hpp>
#include <stdexcept>

#include "learning_geometry.hpp"

namespace semaforr::spatial {

DoorExitLearner::DoorExitLearner(double minimum_range_jump_m,
                                 double maximum_opening_width_m,
                                 SpatialLearningMode mode,
                                 DoorLearningConfiguration compatibility)
    : SpatialLearnerBase(
          SpatialRepresentation::DoorsAndExits, "door_exit",
          UpdateMode::RebuildOnDemand,
          {true,
           true,
           false,
           false,
           mode == SpatialLearningMode::Compatibility
               ? "derive exits from region/path intersections and doors from exit arcs"
               : "infer sensor-opening scan discontinuities",
           {"EnterLinear", "EnterRotation", "region and skeleton planners"},
           UpdateSchedule::EndOfTarget}),
      minimum_range_jump_m_(minimum_range_jump_m),
      maximum_opening_width_m_(maximum_opening_width_m),
      mode_(mode),
      compatibility_(compatibility) {
  if (!std::isfinite(minimum_range_jump_m_) || minimum_range_jump_m_ <= 0.0 ||
      !std::isfinite(maximum_opening_width_m_) ||
      maximum_opening_width_m_ <= 0.0) {
    throw std::invalid_argument(
        "door inference thresholds must be finite and positive");
  }
}

void DoorExitLearner::onObserve(const NavigationEpisode&) {}

void DoorExitLearner::onRebuild() {
  if (mode_ == SpatialLearningMode::Compatibility) {
    auto regions = learnDecisionRegions(episodes());
    auto model = learnRegionExitsAndDoors(
        regions, completedPathsFromEpisodes(episodes()), compatibility_);
    publish(model,
            model.exits.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
            model.exits.empty()
                ? "no traveled path intersects a learned region circumference"
                : "region exits and exit-derived door arcs rebuilt");
    return;
  }
  DoorExitModel model;
  for (const NavigationEpisode& episode : episodes()) {
    const auto& ranges = episode.observation.laser.ranges_m;
    const auto endpoints = detail::laserEndpoints(episode.observation);
    for (std::size_t index = 1U; index < ranges.size(); ++index) {
      if (std::abs(ranges[index] - ranges[index - 1U]) <
          minimum_range_jump_m_) {
        continue;
      }
      domain::Segment2D opening{endpoints[index - 1U], endpoints[index]};
      const double width = opening.length().meters();
      if (width <= maximum_opening_width_m_) {
        detail::appendUnique(model.openings, opening, 0.2);
      }
    }
  }
  for (const auto& opening : model.openings)
    model.sensor_openings.push_back({opening, 0U, 0.5});
  publish(model,
          model.openings.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
          model.openings.empty()
              ? "no qualifying laser discontinuity found"
              : "sensor openings rebuilt from laser discontinuities; not learned doors");
}

}  // namespace semaforr::spatial
