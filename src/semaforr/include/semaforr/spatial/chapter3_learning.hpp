#ifndef SEMAFORR_SPATIAL_CHAPTER3_LEARNING_HPP
#define SEMAFORR_SPATIAL_CHAPTER3_LEARNING_HPP

#include <semaforr/spatial/learner.hpp>

namespace semaforr::spatial {

struct TrailLearningConfiguration {
  double visibility_tolerance_m{0.05};
  double simplification_tolerance_m{0.0};
  bool include_partial_movement{true};
};

struct ConveyorLearningConfiguration {
  double resolution_m{1.0};
  double decay_factor{1.0};
  double bounds_padding_m{1.0};
  bool directional{true};
};

struct RegionLearningConfiguration {
  double minimum_radius_m{0.05};
  double overlap_tolerance_m{0.01};
};

struct DoorLearningConfiguration {
  double exit_merge_angle_rad{0.10};
  double sensor_opening_minimum_jump_m{0.75};
  double sensor_opening_maximum_width_m{2.5};
};

struct HallwayLearningConfiguration {
  double heatmap_resolution_m{1.0};
  std::uint32_t smoothing_threshold{1U};
  double smoothing_neighbor_fraction{0.70};
  double initial_sigma{3.0};
  double sigma_decrement{0.25};
  double comparison_radius_m{20.0};
  double minimum_segment_length_m{0.05};
};

std::vector<domain::CompletedPath> completedPathsFromEpisodes(
    const std::vector<NavigationEpisode>& episodes);

bool historicallyVisible(const domain::RobotObservation& observation,
                         domain::Point2D marker, double tolerance_m,
                         domain::VisibilityEvidence* evidence = nullptr);

domain::LearnedTrail learnVisibilityTrail(
    const domain::CompletedPath& path, domain::TrailId id,
    const TrailLearningConfiguration& configuration = {});

RegionModel learnDecisionRegions(
    const std::vector<NavigationEpisode>& decision_episodes,
    const RegionLearningConfiguration& configuration = {});

DoorExitModel learnRegionExitsAndDoors(
    const RegionModel& regions,
    const std::vector<domain::CompletedPath>& paths,
    const DoorLearningConfiguration& configuration = {});

HallwayModel learnCompatibilityHallways(
    const std::vector<domain::CompletedPath>& paths,
    const HallwayLearningConfiguration& configuration = {});

ConveyorModel learnConveyorGrid(
    const std::vector<domain::LearnedTrail>& trails,
    const ConveyorLearningConfiguration& configuration = {});

PassageSkeletonModel learnRegionSkeleton(
    const RegionModel& regions,
    const std::vector<domain::LearnedTrail>& trails,
    const std::vector<domain::CompletedPath>& paths);

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_CHAPTER3_LEARNING_HPP
