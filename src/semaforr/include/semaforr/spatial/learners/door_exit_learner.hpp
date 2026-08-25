/**
 * @file door_exit_learner.hpp
 * @brief Door exit learner responsibilities.
 *
 * @details This file defines door exit learner behavior for learned spatial
 * representations and their lifecycle. It centers on `DoorExitLearner`.
 * Its package-relative location is
 * `include/semaforr/spatial/learners/door_exit_learner.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_DOOR_EXIT_LEARNER_HPP
#define SEMAFORR_SPATIAL_DOOR_EXIT_LEARNER_HPP

#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

/**
 * @brief Encapsulates door exit learner state and behavior for this
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
class DoorExitLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the door exit learner operation for this subsystem.
   *
   * Arguments:
   * - @p minimum_range_jump_m: Supplies minimum range jump m input to the
   * operation.
   * - @p maximum_opening_width_m: Supplies maximum opening width m input to
   * the operation.
   * - @p mode: Supplies mode input to the operation.
   * - @p compatibility: Supplies compatibility input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  DoorExitLearner(double minimum_range_jump_m = 0.75,
                  double maximum_opening_width_m = 2.5,
                  SpatialLearningMode mode = SpatialLearningMode::Modernized,
                  DoorLearningConfiguration compatibility = {});

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

  double minimum_range_jump_m_;
  double maximum_opening_width_m_;
  SpatialLearningMode mode_;
  DoorLearningConfiguration compatibility_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_DOOR_EXIT_LEARNER_HPP
