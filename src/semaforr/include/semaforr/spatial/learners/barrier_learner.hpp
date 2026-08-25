/**
 * @file barrier_learner.hpp
 * @brief Barrier learner responsibilities.
 *
 * @details This file defines barrier learner behavior for learned spatial
 * representations and their lifecycle. It centers on `BarrierLearner`. Its
 * package-relative location is
 * `include/semaforr/spatial/learners/barrier_learner.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_BARRIER_LEARNER_HPP
#define SEMAFORR_SPATIAL_BARRIER_LEARNER_HPP

#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

/**
 * @brief Encapsulates barrier learner state and behavior for this
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
class BarrierLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the barrier learner operation for this subsystem.
   *
   * Arguments:
   * - @p maximum_segment_length_m: Supplies maximum segment length m input
   * to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit BarrierLearner(double maximum_segment_length_m = 0.5);

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

  double maximum_segment_length_m_;
  BarrierModel model_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_BARRIER_LEARNER_HPP
