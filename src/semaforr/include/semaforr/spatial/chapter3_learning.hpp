/**
 * @file chapter3_learning.hpp
 * @brief Chapter3 learning responsibilities.
 *
 * @details This file defines chapter3 learning behavior for learned spatial
 * representations and their lifecycle. It centers on
 * `TrailLearningConfiguration`, `ConveyorLearningConfiguration`,
 * `RegionLearningConfiguration`, `DoorLearningConfiguration`,
 * `HallwayLearningConfiguration`. Its package-relative location is
 * `include/semaforr/spatial/chapter3_learning.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_CHAPTER3_LEARNING_HPP
#define SEMAFORR_SPATIAL_CHAPTER3_LEARNING_HPP

#include <semaforr/spatial/learner.hpp>

namespace semaforr::spatial {

/**
 * @brief Encapsulates trail learning configuration state and behavior for
 * this subsystem.
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
struct TrailLearningConfiguration {
  double visibility_tolerance_m{0.05};
  double simplification_tolerance_m{0.0};
  bool include_partial_movement{true};
};

/**
 * @brief Encapsulates conveyor learning configuration state and behavior
 * for this subsystem.
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
struct ConveyorLearningConfiguration {
  double resolution_m{1.0};
  double decay_factor{1.0};
  double bounds_padding_m{1.0};
  bool directional{true};
};

/**
 * @brief Encapsulates region learning configuration state and behavior for
 * this subsystem.
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
struct RegionLearningConfiguration {
  double minimum_radius_m{0.05};
  double overlap_tolerance_m{0.01};
};

/**
 * @brief Encapsulates door learning configuration state and behavior for
 * this subsystem.
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
struct DoorLearningConfiguration {
  double exit_merge_angle_rad{0.10};
  double sensor_opening_minimum_jump_m{0.75};
  double sensor_opening_maximum_width_m{2.5};
};

/**
 * @brief Encapsulates hallway learning configuration state and behavior for
 * this subsystem.
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
struct HallwayLearningConfiguration {
  double heatmap_resolution_m{1.0};
  std::uint32_t smoothing_threshold{1U};
  double smoothing_neighbor_fraction{0.70};
  double initial_sigma{3.0};
  double sigma_decrement{0.25};
  double comparison_radius_m{20.0};
  double minimum_segment_length_m{0.05};
};

/**
 * @brief Performs the completed paths from episodes operation for this
 * subsystem.
 *
 * Arguments:
 * - @p episodes: Supplies episodes input to the operation.
 *
 * Returns:
 * - `std::vector<domain::CompletedPath>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<domain::CompletedPath> completedPathsFromEpisodes(
    const std::vector<NavigationEpisode>& episodes);

/**
 * @brief Performs the historically visible operation for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 * - @p marker: Supplies marker input to the operation.
 * - @p tolerance_m: Supplies tolerance m input to the operation.
 * - @p evidence: Supplies evidence input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool historicallyVisible(const domain::RobotObservation& observation,
                         domain::Point2D marker, double tolerance_m,
                         domain::VisibilityEvidence* evidence = nullptr);

/**
 * @brief Performs the learn visibility trail operation for this subsystem.
 *
 * Arguments:
 * - @p path: Supplies path input to the operation.
 * - @p id: Supplies id input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `domain::LearnedTrail` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::LearnedTrail learnVisibilityTrail(
    const domain::CompletedPath& path, domain::TrailId id,
    const TrailLearningConfiguration& configuration = {});

/**
 * @brief Performs the learn decision regions operation for this subsystem.
 *
 * Arguments:
 * - @p decision_episodes: Supplies decision episodes input to the
 * operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `RegionModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
RegionModel learnDecisionRegions(
    const std::vector<NavigationEpisode>& decision_episodes,
    const RegionLearningConfiguration& configuration = {});

/**
 * @brief Performs the learn region exits and doors operation for this
 * subsystem.
 *
 * Arguments:
 * - @p regions: Supplies regions input to the operation.
 * - @p paths: Supplies paths input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `DoorExitModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
DoorExitModel learnRegionExitsAndDoors(
    const RegionModel& regions,
    const std::vector<domain::CompletedPath>& paths,
    const DoorLearningConfiguration& configuration = {});

/**
 * @brief Performs the learn compatibility hallways operation for this
 * subsystem.
 *
 * Arguments:
 * - @p paths: Supplies paths input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `HallwayModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
HallwayModel learnCompatibilityHallways(
    const std::vector<domain::CompletedPath>& paths,
    const HallwayLearningConfiguration& configuration = {});

/**
 * @brief Performs the learn conveyor grid operation for this subsystem.
 *
 * Arguments:
 * - @p trails: Supplies trails input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `ConveyorModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
ConveyorModel learnConveyorGrid(
    const std::vector<domain::LearnedTrail>& trails,
    const ConveyorLearningConfiguration& configuration = {});

/**
 * @brief Performs the learn region skeleton operation for this subsystem.
 *
 * Arguments:
 * - @p regions: Supplies regions input to the operation.
 * - @p trails: Supplies trails input to the operation.
 * - @p paths: Supplies paths input to the operation.
 *
 * Returns:
 * - `PassageSkeletonModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PassageSkeletonModel learnRegionSkeleton(
    const RegionModel& regions,
    const std::vector<domain::LearnedTrail>& trails,
    const std::vector<domain::CompletedPath>& paths);

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_CHAPTER3_LEARNING_HPP
