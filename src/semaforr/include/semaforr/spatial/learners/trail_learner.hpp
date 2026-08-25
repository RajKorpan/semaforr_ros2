/**
 * @file trail_learner.hpp
 * @brief Trail learner responsibilities.
 *
 * @details This file defines trail learner behavior for learned spatial
 * representations and their lifecycle. It centers on `TrailLearner`. Its
 * package-relative location is
 * `include/semaforr/spatial/learners/trail_learner.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_TRAIL_LEARNER_HPP
#define SEMAFORR_SPATIAL_TRAIL_LEARNER_HPP

#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

/**
 * @brief Encapsulates trail learner state and behavior for this subsystem.
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
class TrailLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the trail learner operation for this subsystem.
   *
   * Arguments:
   * - @p minimum_sample_distance_m: Supplies minimum sample distance m
   * input to the operation.
   * - @p mode: Supplies mode input to the operation.
   * - @p compatibility: Supplies compatibility input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit TrailLearner(
      double minimum_sample_distance_m = 0.05,
      SpatialLearningMode mode = SpatialLearningMode::Modernized,
      TrailLearningConfiguration compatibility = {});

 private:
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
  void onObserve(const NavigationEpisode& episode) override;
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
  void onRebuild() override;

  double minimum_sample_distance_m_;
  SpatialLearningMode mode_;
  TrailLearningConfiguration compatibility_;
  TrailModel model_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_TRAIL_LEARNER_HPP
