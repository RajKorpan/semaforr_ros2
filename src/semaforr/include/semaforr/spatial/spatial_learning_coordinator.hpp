#ifndef SEMAFORR_SPATIAL_SPATIAL_LEARNING_COORDINATOR_HPP
#define SEMAFORR_SPATIAL_SPATIAL_LEARNING_COORDINATOR_HPP

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <semaforr/domain/world_model.hpp>
#include <semaforr/spatial/spatial_learner.hpp>

namespace semaforr::spatial {

struct LearnerInspection {
  SpatialRepresentation representation = SpatialRepresentation::Trails;
  std::string name;
  bool enabled = false;
  ObservationContract contract;
  SpatialModelUpdate update;
};

class SpatialLearningCoordinator {
public:
  explicit SpatialLearningCoordinator(
    std::size_t automatic_rebuild_interval = 10U);

  static SpatialLearningCoordinator defaults(
    std::size_t automatic_rebuild_interval = 10U);

  void addLearner(
    std::unique_ptr<SpatialLearner> learner,
    bool enabled = true);
  void setEnabled(SpatialRepresentation representation, bool enabled);
  bool enabled(SpatialRepresentation representation) const;

  void observe(const NavigationEpisode& episode);
  void rebuild(SpatialRepresentation representation);
  void rebuildStale();
  void rebuildAll();

  std::optional<SpatialModelUpdate> snapshot(
    SpatialRepresentation representation) const;
  std::vector<SpatialModelUpdate> snapshots() const;
  std::vector<LearnerInspection> inspect() const;
  std::string serialize(SpatialRepresentation representation) const;

  void applyTo(domain::SpatialModel& model) const;
  std::size_t learnerCount() const noexcept { return learners_.size(); }
  std::size_t enabledCount() const noexcept;

private:
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
