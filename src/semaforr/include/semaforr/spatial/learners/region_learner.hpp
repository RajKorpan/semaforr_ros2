/**
 * @file region_learner.hpp
 * @brief Region learner responsibilities.
 *
 * @details This file defines region learner behavior for learned spatial
 * representations and their lifecycle. It centers on `RegionLearner`. Its
 * package-relative location is
 * `include/semaforr/spatial/learners/region_learner.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_REGION_LEARNER_HPP
#define SEMAFORR_SPATIAL_REGION_LEARNER_HPP

#include <cstdint>
#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learner_base.hpp>
#include <unordered_map>

namespace semaforr::spatial {

/**
 * @brief Encapsulates region learner state and behavior for this subsystem.
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
class RegionLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the region learner operation for this subsystem.
   *
   * Arguments:
   * - @p cluster_radius_m: Supplies cluster radius m input to the
   * operation.
   * - @p minimum_observations: Supplies minimum observations input to the
   * operation.
   * - @p mode: Supplies mode input to the operation.
   * - @p compatibility: Supplies compatibility input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  RegionLearner(double cluster_radius_m = 1.0,
                std::size_t minimum_observations = 3U,
                SpatialLearningMode mode = SpatialLearningMode::Modernized,
                RegionLearningConfiguration compatibility = {});

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

  double cluster_radius_m_;
  std::size_t minimum_observations_;
  SpatialLearningMode mode_;
  RegionLearningConfiguration compatibility_;
  RegionModel model_;
  std::vector<std::size_t> observation_counts_;
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> spatial_index_;
  std::vector<std::uint64_t> region_buckets_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_REGION_LEARNER_HPP
