/**
 * @file hallway_learner.hpp
 * @brief Hallway learner responsibilities.
 *
 * @details This file defines hallway learner behavior for learned spatial
 * representations and their lifecycle. It centers on `HallwayLearner`. Its
 * package-relative location is
 * `include/semaforr/spatial/learners/hallway_learner.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_HALLWAY_LEARNER_HPP
#define SEMAFORR_SPATIAL_HALLWAY_LEARNER_HPP

#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

/**
 * @brief Encapsulates hallway learner state and behavior for this
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
class HallwayLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the hallway learner operation for this subsystem.
   *
   * Arguments:
   * - @p minimum_centerline_length_m: Supplies minimum centerline length m
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
  explicit HallwayLearner(
      double minimum_centerline_length_m = 0.5,
      SpatialLearningMode mode = SpatialLearningMode::Modernized,
      HallwayLearningConfiguration compatibility = {});

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

  double minimum_centerline_length_m_;
  SpatialLearningMode mode_;
  HallwayLearningConfiguration compatibility_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_HALLWAY_LEARNER_HPP
