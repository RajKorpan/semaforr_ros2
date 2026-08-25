/**
 * @file barrier_learner.cpp
 * @brief Barrier learner responsibilities.
 *
 * @details This file implements barrier learner behavior for learned spatial
 * representations and their lifecycle. It records the declarations,
 * settings, fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/spatial/barrier_learner.cpp`.
 */
#include <cmath>
#include <semaforr/spatial/learners/barrier_learner.hpp>
#include <stdexcept>

#include "learning_geometry.hpp"

namespace semaforr::spatial {

/**
 * @brief Performs the barrier learner operation for this subsystem.
 *
 * Arguments:
 * - @p maximum_segment_length_m: Supplies maximum segment length m input to
 * the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
BarrierLearner::BarrierLearner(double maximum_segment_length_m)
    : SpatialLearnerBase(
          SpatialRepresentation::Barriers, "barrier", UpdateMode::Incremental,
          {true,
           true,
           false,
           false,
           "append local obstacle segments after each laser observation",
           {"AvoidObstacles", "UnlikelyField", "collision-aware planners"},
           UpdateSchedule::EveryObservation}),
      maximum_segment_length_m_(maximum_segment_length_m) {
  if (!std::isfinite(maximum_segment_length_m_) ||
      maximum_segment_length_m_ <= 0.0) {
    throw std::invalid_argument(
        "barrier segment length must be finite and positive");
  }
}

/**
 * @brief Performs the on observe operation for this subsystem.
 *
 * Arguments:
 * - @p episode: Supplies episode input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void BarrierLearner::onObserve(const NavigationEpisode& episode) {
  const auto endpoints = detail::laserEndpoints(episode.observation);
  const auto& ranges = episode.observation.laser.ranges_m;
  for (std::size_t index = 1U; index < endpoints.size(); ++index) {
    if (ranges[index] >= episode.observation.laser.maximum_range.meters() ||
        ranges[index - 1U] >=
            episode.observation.laser.maximum_range.meters()) {
      continue;
    }
    domain::Segment2D barrier{endpoints[index - 1U], endpoints[index]};
    if (barrier.length().meters() <= maximum_segment_length_m_) {
      detail::appendUnique(model_.barriers, barrier, 0.1);
    }
  }
  publish(
      model_,
      model_.barriers.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
      model_.barriers.empty() ? "no adjacent obstacle returns found"
                              : "barrier segments updated incrementally");
}

/**
 * @brief Performs the on rebuild operation for this subsystem.
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
void BarrierLearner::onRebuild() {
  publish(
      model_,
      model_.barriers.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
      model_.barriers.empty() ? "no barrier evidence"
                              : "barrier snapshot rebuilt");
}

}  // namespace semaforr::spatial
