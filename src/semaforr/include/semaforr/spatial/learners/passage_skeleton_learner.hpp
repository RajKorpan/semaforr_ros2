/**
 * @file passage_skeleton_learner.hpp
 * @brief Passage skeleton learner responsibilities.
 *
 * @details This file defines passage skeleton learner behavior for learned spatial
 * representations and their lifecycle. It centers on
 * `PassageSkeletonLearner`. Its package-relative location is
 * `include/semaforr/spatial/learners/passage_skeleton_learner.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_PASSAGE_SKELETON_LEARNER_HPP
#define SEMAFORR_SPATIAL_PASSAGE_SKELETON_LEARNER_HPP

#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

/**
 * @brief Encapsulates passage skeleton learner state and behavior for this
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
class PassageSkeletonLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the passage skeleton learner operation for this
   * subsystem.
   *
   * Arguments:
   * - @p minimum_node_spacing_m: Supplies minimum node spacing m input to
   * the operation.
   * - @p mode: Supplies mode input to the operation.
   * - @p region_configuration: Supplies region configuration input to the
   * operation.
   * - @p trail_configuration: Supplies trail configuration input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit PassageSkeletonLearner(
      double minimum_node_spacing_m = 0.5,
      SpatialLearningMode mode = SpatialLearningMode::Modernized,
      RegionLearningConfiguration region_configuration = {},
      TrailLearningConfiguration trail_configuration = {});

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

  double minimum_node_spacing_m_;
  SpatialLearningMode mode_;
  RegionLearningConfiguration region_configuration_;
  TrailLearningConfiguration trail_configuration_;
  PassageSkeletonModel model_;
  std::optional<domain::TaskId> last_task_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_PASSAGE_SKELETON_LEARNER_HPP
