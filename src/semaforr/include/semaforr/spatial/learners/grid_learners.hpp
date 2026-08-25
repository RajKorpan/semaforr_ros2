/**
 * @file grid_learners.hpp
 * @brief Grid learners responsibilities.
 *
 * @details This file defines grid learners behavior for learned spatial
 * representations and their lifecycle. It centers on `GridExtentPolicy`,
 * `KnownGridLearner`, `SensedOccupancyLearningConfiguration`,
 * `SensedOccupancyLearner`, `InclusionGridLearner`. Its package-relative
 * location is `include/semaforr/spatial/learners/grid_learners.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_GRID_LEARNERS_HPP
#define SEMAFORR_SPATIAL_GRID_LEARNERS_HPP

#include <semaforr/spatial/learner_base.hpp>
#include <string>
#include <unordered_map>

namespace semaforr::spatial {

/**
 * @brief Enumerates the supported grid extent policy values used by this
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
enum class GridExtentPolicy { Fixed, Expand };

/**
 * @brief Encapsulates known grid learner state and behavior for this
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
class KnownGridLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the known grid learner operation for this subsystem.
   *
   * Arguments:
   * - @p columns: Supplies columns input to the operation.
   * - @p rows: Supplies rows input to the operation.
   * - @p resolution_m: Supplies resolution m input to the operation.
   * - @p origin: Supplies origin input to the operation.
   * - @p extent_policy: Supplies extent policy input to the operation.
   * - @p expansion_policy: Supplies expansion policy input to the
   * operation.
   * - @p initialize_around_first_pose: Supplies initialize around first
   * pose input to the operation.
   * - @p frame_id: Supplies frame id input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  KnownGridLearner(std::size_t columns = 200U, std::size_t rows = 200U,
                   double resolution_m = 1.0,
                   domain::Point2D origin = {},
                   GridExtentPolicy extent_policy = GridExtentPolicy::Expand,
                   domain::GridExpansionPolicy expansion_policy = {},
                   bool initialize_around_first_pose = false,
                   std::string frame_id = "map");

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
  GridGeometry geometry_;
  std::unordered_map<std::size_t, std::uint32_t> observations_;
  std::unordered_map<std::size_t, std::size_t> last_observed_sequence_;
  GridExtentPolicy extent_policy_;
  domain::GridExpansionPolicy expansion_policy_;
  bool initialize_around_first_pose_ = false;
  std::size_t out_of_bounds_evidence_ = 0U;
};

/**
 * @brief Encapsulates sensed occupancy learning configuration state and
 * behavior for this subsystem.
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
struct SensedOccupancyLearningConfiguration {
  std::uint16_t free_observations_to_clear = 3U;
  std::size_t dynamic_expiry_observations = 30U;
  bool treat_obstacle_returns_as_dynamic = true;
};

/**
 * @brief Encapsulates sensed occupancy learner state and behavior for this
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
class SensedOccupancyLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the sensed occupancy learner operation for this
   * subsystem.
   *
   * Arguments:
   * - @p columns: Supplies columns input to the operation.
   * - @p rows: Supplies rows input to the operation.
   * - @p resolution_m: Supplies resolution m input to the operation.
   * - @p origin: Supplies origin input to the operation.
   * - @p configuration: Supplies configuration input to the operation.
   * - @p extent_policy: Supplies extent policy input to the operation.
   * - @p expansion_policy: Supplies expansion policy input to the
   * operation.
   * - @p initialize_around_first_pose: Supplies initialize around first
   * pose input to the operation.
   * - @p frame_id: Supplies frame id input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  SensedOccupancyLearner(
      std::size_t columns = 200U, std::size_t rows = 200U,
      double resolution_m = 1.0, domain::Point2D origin = {},
      SensedOccupancyLearningConfiguration configuration = {},
      GridExtentPolicy extent_policy = GridExtentPolicy::Expand,
      domain::GridExpansionPolicy expansion_policy = {},
      bool initialize_around_first_pose = false,
      std::string frame_id = "map");

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
  /**
   * @brief Performs the integrate free operation for this subsystem.
   *
   * Arguments:
   * - @p index: Supplies index input to the operation.
   * - @p sequence: Supplies sequence input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void integrateFree(std::size_t index, std::size_t sequence);
  /**
   * @brief Performs the integrate occupied operation for this subsystem.
   *
   * Arguments:
   * - @p index: Supplies index input to the operation.
   * - @p sequence: Supplies sequence input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void integrateOccupied(std::size_t index, std::size_t sequence);
  /**
   * @brief Performs the expire dynamic operation for this subsystem.
   *
   * Arguments:
   * - @p sequence: Supplies sequence input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void expireDynamic(std::size_t sequence);
  /**
   * @brief Performs the snapshot model operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `SensedOccupancyModel` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  SensedOccupancyModel snapshotModel() const;

  GridGeometry geometry_;
  SensedOccupancyLearningConfiguration configuration_;
  std::unordered_map<std::size_t, domain::SensedOccupancyCell> cells_;
  GridExtentPolicy extent_policy_;
  domain::GridExpansionPolicy expansion_policy_;
  bool initialize_around_first_pose_ = false;
  std::size_t out_of_bounds_evidence_ = 0U;
};

/**
 * @brief Encapsulates inclusion grid learner state and behavior for this
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
class InclusionGridLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the inclusion grid learner operation for this
   * subsystem.
   *
   * Arguments:
   * - @p columns: Supplies columns input to the operation.
   * - @p rows: Supplies rows input to the operation.
   * - @p resolution_m: Supplies resolution m input to the operation.
   * - @p origin: Supplies origin input to the operation.
   * - @p extent_policy: Supplies extent policy input to the operation.
   * - @p expansion_policy: Supplies expansion policy input to the
   * operation.
   * - @p initialize_around_first_pose: Supplies initialize around first
   * pose input to the operation.
   * - @p frame_id: Supplies frame id input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  InclusionGridLearner(std::size_t columns = 200U, std::size_t rows = 200U,
                       double resolution_m = 1.0,
                       domain::Point2D origin = {},
                       GridExtentPolicy extent_policy = GridExtentPolicy::Expand,
                       domain::GridExpansionPolicy expansion_policy = {},
                       bool initialize_around_first_pose = false,
                       std::string frame_id = "map");
  /**
   * @brief Performs the replace represented operation for this subsystem.
   *
   * Arguments:
   * - @p regions: Supplies regions input to the operation.
   * - @p skeleton: Supplies skeleton input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void replaceRepresented(const RegionModel& regions,
                          const PassageSkeletonModel& skeleton);

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
  GridGeometry geometry_;
  std::unordered_map<std::size_t, std::uint32_t> included_;
  std::unordered_map<std::size_t, std::uint32_t> lle_included_;
  GridExtentPolicy extent_policy_;
  domain::GridExpansionPolicy expansion_policy_;
  bool initialize_around_first_pose_ = false;
};

}  // namespace semaforr::spatial

#endif
