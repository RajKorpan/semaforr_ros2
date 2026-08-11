#ifndef SEMAFORR_SPATIAL_SPATIAL_LEARNING_COORDINATOR_HPP
#define SEMAFORR_SPATIAL_SPATIAL_LEARNING_COORDINATOR_HPP

#include <cstddef>
#include <memory>
#include <optional>
#include <semaforr/domain/world_model.hpp>
#include <semaforr/spatial/learner.hpp>
#include <semaforr/spatial/learners/circumstance_learner.hpp>
#include <semaforr/spatial/learners/grid_learners.hpp>
#include <string>
#include <vector>

namespace semaforr::spatial {

struct LearnerInspection {
  SpatialRepresentation representation = SpatialRepresentation::Trails;
  std::string name;
  bool enabled = false;
  ObservationContract contract;
  SpatialModelUpdate update;
};

struct LearnedGridConfiguration {
  std::string frame_id{"map"};
  double initial_width_m{20.0};
  double initial_height_m{20.0};
  double resolution_m{0.5};
  GridExtentPolicy extent_policy = GridExtentPolicy::Expand;
  domain::GridExpansionPolicy expansion;
  bool initialize_around_first_pose{true};
};

class SpatialLearningCoordinator {
 public:
  explicit SpatialLearningCoordinator(
      std::size_t automatic_rebuild_interval = 10U);

  static SpatialLearningCoordinator defaults(
      std::size_t automatic_rebuild_interval = 10U);
  static SpatialLearningCoordinator defaults(
      std::size_t automatic_rebuild_interval,
      CircumstanceLearningConfiguration circumstance_configuration);
  static SpatialLearningCoordinator defaults(
      std::size_t automatic_rebuild_interval,
      CircumstanceLearningConfiguration circumstance_configuration,
      SensedOccupancyLearningConfiguration occupancy_configuration);
  static SpatialLearningCoordinator defaults(
      std::size_t automatic_rebuild_interval,
      CircumstanceLearningConfiguration circumstance_configuration,
      SensedOccupancyLearningConfiguration occupancy_configuration,
      GridExtentPolicy extent_policy);
  static SpatialLearningCoordinator defaults(
      std::size_t automatic_rebuild_interval,
      CircumstanceLearningConfiguration circumstance_configuration,
      SensedOccupancyLearningConfiguration occupancy_configuration,
      LearnedGridConfiguration grid_configuration);
  void addLearner(std::unique_ptr<SpatialLearner> learner, bool enabled = true);
  void setEnabled(SpatialRepresentation representation, bool enabled);
  bool enabled(SpatialRepresentation representation) const;

  void observe(const NavigationEpisode& episode);
  void observeSensor(NavigationEpisode episode);
  void observeDecision(NavigationEpisode episode);
  void observeActionStarted(NavigationEpisode episode);
  void observeActionProgress(NavigationEpisode episode);
  void observeActionTerminal(NavigationEpisode episode);
  void rebuild(SpatialRepresentation representation);
  void rebuildStale();
  void rebuildAll();
  void finalizeInitialExploration();
  void finalizeTarget();

  std::optional<SpatialModelUpdate> snapshot(
      SpatialRepresentation representation) const;
  std::vector<SpatialModelUpdate> snapshots() const;
  std::vector<LearnerInspection> inspect() const;
  std::string serialize(SpatialRepresentation representation) const;
  std::string serializeAll() const;

  void applyTo(domain::SpatialModel& model) const;
  std::size_t learnerCount() const noexcept { return learners_.size(); }
  std::size_t enabledCount() const noexcept;

 private:
  void dispatch(const NavigationEpisode& episode);
  struct Entry {
    std::unique_ptr<SpatialLearner> learner;
    bool enabled = true;
  };

  Entry& require(SpatialRepresentation representation);
  const Entry& require(SpatialRepresentation representation) const;

  std::size_t automatic_rebuild_interval_;
  std::size_t observed_episodes_ = 0U;
  std::vector<Entry> learners_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_SPATIAL_LEARNING_COORDINATOR_HPP
