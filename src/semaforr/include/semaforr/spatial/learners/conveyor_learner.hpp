/**
 * @file conveyor_learner.hpp
 * @brief Conveyor learner responsibilities.
 *
 * @details This file defines conveyor learner behavior for learned spatial
 * representations and their lifecycle. It centers on `ConveyorLearner`.
 * Its package-relative location is
 * `include/semaforr/spatial/learners/conveyor_learner.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_CONVEYOR_LEARNER_HPP
#define SEMAFORR_SPATIAL_CONVEYOR_LEARNER_HPP

#include <optional>
#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

/**
 * @brief Encapsulates conveyor learner state and behavior for this
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
class ConveyorLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the conveyor learner operation for this subsystem.
   *
   * Arguments:
   * - @p minimum_traversal_distance_m: Supplies minimum traversal distance
   * m input to the operation.
   * - @p mode: Supplies mode input to the operation.
   * - @p compatibility: Supplies compatibility input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit ConveyorLearner(
      double minimum_traversal_distance_m = 0.05,
      SpatialLearningMode mode = SpatialLearningMode::Modernized,
      ConveyorLearningConfiguration compatibility = {});

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

  double minimum_traversal_distance_m_;
  SpatialLearningMode mode_;
  ConveyorLearningConfiguration compatibility_;
  std::optional<domain::Point2D> previous_position_;
  std::optional<domain::Action> previous_action_;
  ConveyorModel model_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_CONVEYOR_LEARNER_HPP
