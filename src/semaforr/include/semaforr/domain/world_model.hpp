#ifndef SEMAFORR_DOMAIN_WORLD_MODEL_HPP
#define SEMAFORR_DOMAIN_WORLD_MODEL_HPP

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/action_execution.hpp>
#include <semaforr/domain/crowd_model.hpp>
#include <semaforr/domain/circumstance.hpp>
#include <semaforr/domain/completed_path.hpp>
#include <semaforr/domain/highway.hpp>
#include <semaforr/domain/grid_layers.hpp>
#include <semaforr/domain/mission.hpp>
#include <semaforr/domain/model_revision.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/domain/static_map.hpp>
#include <semaforr/domain/spatial_affordances.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace semaforr::domain {

class ActionSpace {
 public:
  ActionSpace(std::vector<double> move_distances_m,
              std::vector<double> rotation_angles_rad)
      : move_distances_m_(std::move(move_distances_m)),
        rotation_angles_rad_(std::move(rotation_angles_rad)) {
    validate(move_distances_m_, "move distances");
    validate(rotation_angles_rad_, "rotation angles");
    if (move_distances_m_.size() > Action::maximum_magnitude_index ||
        rotation_angles_rad_.size() > Action::maximum_magnitude_index) {
      throw std::invalid_argument(
          "action arrays may contain at most 299 values");
    }
  }

  const std::vector<double>& move_distances_m() const noexcept {
    return move_distances_m_;
  }
  const std::vector<double>& rotation_angles_rad() const noexcept {
    return rotation_angles_rad_;
  }

  bool contains(const Action& action) const noexcept {
    switch (action.type()) {
      case ActionType::Pause:
        return action.magnitude_index() == 0U;
      case ActionType::Forward:
        return action.magnitude_index() <= move_distances_m_.size();
      case ActionType::TurnRight:
      case ActionType::TurnLeft:
        return action.magnitude_index() <= rotation_angles_rad_.size();
    }
    return false;
  }

 private:
  static void validate(const std::vector<double>& values, const char* name) {
    if (values.empty()) {
      throw std::invalid_argument(std::string(name) + " must not be empty");
    }
    if (!std::all_of(values.begin(), values.end(), [](double value) {
          return std::isfinite(value) && value > 0.0;
        })) {
      throw std::invalid_argument(std::string(name) +
                                  " must contain finite positive values");
    }
    if (!std::is_sorted(values.begin(), values.end()) ||
        std::adjacent_find(values.begin(), values.end()) != values.end()) {
      throw std::invalid_argument(std::string(name) +
                                  " must be strictly increasing");
    }
  }

  std::vector<double> move_distances_m_;
  std::vector<double> rotation_angles_rad_;
};

struct RobotState {
  Pose2D pose;
  std::optional<LaserObservation> laser;
  std::chrono::steady_clock::time_point observed_at{};
};

struct NavigationHistoryEntry {
  Pose2D pose;
  LaserObservation laser;
  Action action = Action::pause();
  std::optional<TaskId> task_id;
  DecisionId decision_id{0U};
  ActionId action_id{0U};
  ExecutionCompletionStatus execution_status =
      ExecutionCompletionStatus::NoMovement;
  double distance_achieved_m{0.0};
  double rotation_achieved_rad{0.0};

  NavigationHistoryEntry() = default;
  NavigationHistoryEntry(Pose2D pose_value, LaserObservation laser_value,
                         Action action_value,
                         std::optional<TaskId> task_value = std::nullopt)
      : pose(std::move(pose_value)),
        laser(std::move(laser_value)),
        action(action_value),
        task_id(task_value) {}
};

class NavigationHistory {
 public:
  void record(NavigationHistoryEntry entry) {
    entries_.push_back(std::move(entry));
  }

  const std::vector<NavigationHistoryEntry>& entries() const noexcept {
    return entries_;
  }

 private:
  std::vector<NavigationHistoryEntry> entries_;
};

template <typename Record>
class AppendOnlyHistory {
 public:
  void record(Record entry) { entries_.push_back(std::move(entry)); }
  const std::vector<Record>& entries() const noexcept { return entries_; }

 private:
  std::vector<Record> entries_;
};

struct ObservationHistoryEntry {
  Pose2D pose;
  ExecutionTimestamp observed_at{};
};

using DecisionHistory = AppendOnlyHistory<SelectedActionRecord>;
using CommandHistory = AppendOnlyHistory<ActionStartedEvent>;
using ExecutionHistory = AppendOnlyHistory<ActionExecutionResult>;
using CompletedPathHistory = AppendOnlyHistory<NavigationHistoryEntry>;
using ObservationHistory = AppendOnlyHistory<ObservationHistoryEntry>;

struct RecoveryState {
  bool confined = false;
  std::size_t get_out_attempts = 0U;
  std::size_t reposition_attempts = 0U;
};

using FreespaceGrid = SparseCountGrid;

struct ExplorationCue {
  std::uint64_t id = 0U;
  Point2D start;
  Point2D target;
};

struct SpatialModel {
  std::vector<Polygon> obstacle_polygons;
  std::vector<std::vector<Point2D>> trails;
  std::vector<LearnedTrail> learned_trails;
  std::vector<Segment2D> conveyor_flows;
  std::vector<std::size_t> conveyor_traversals;
  ConveyorGrid conveyor_grid;
  std::vector<Circle> learned_regions;
  std::vector<LearnedRegion> regions;
  std::vector<Segment2D> doorways;
  std::vector<RegionExit> exits;
  std::vector<LearnedDoor> doors;
  std::vector<SensorOpening> sensor_openings;
  std::vector<Segment2D> hallways;
  std::vector<LearnedHallway> hallway_entities;
  std::vector<Segment2D> barriers;
  std::vector<Point2D> skeleton_nodes;
  std::vector<std::pair<std::size_t, std::size_t>> skeleton_edges;
  std::vector<RegionSkeletonNode> region_skeleton_nodes;
  std::vector<RegionSkeletonEdge> region_skeleton_edges;
  // The incremental sampled-pose graph is retained as a distinct engineering
  // representation and is never advertised as the region skeleton.
  std::vector<Point2D> sampled_path_nodes;
  std::vector<std::pair<std::size_t, std::size_t>> sampled_path_edges;
  FamiliarityGrid known_grid;
  SensedOccupancyGrid sensed_occupancy;
  FreespaceGrid inclusion_grid;
  std::vector<ExplorationCue> unfinished_hle_candidates;
  HighwayGraph highways;
  CircumstanceModel circumstances;
  DependencyRevisions revisions;
  std::map<ModelDependency, std::shared_ptr<const void>> snapshot_handles;
  Revision mutation_sequence = 0U;
  std::vector<ModelMutation> mutation_history;
  // Compatibility diagnostic sequence. This is never used as a dependency.
  std::size_t revision = 0U;

  Revision revisionOf(ModelDependency dependency) const noexcept {
    const auto found = revisions.find(dependency);
    return found == revisions.end() ? 0U : found->second;
  }
};

struct WorldModel {
  RobotState robot;
  Mission mission;
  NavigationHistory navigation_history;
  DecisionHistory decision_history;
  CommandHistory command_history;
  ExecutionHistory execution_history;
  CompletedPathHistory completed_path_history;
  PathHistory path_history;
  ObservationHistory observation_history;
  RecoveryState recovery;
  CrowdModel crowd;
  SpatialModel spatial;
  // Non-owning read-only view. The composition root owns this startup-lifetime
  // prior; spatial learners never mutate or replace it.
  const StaticMap* static_map = nullptr;
  MapCapabilities map_capabilities;
  Revision mutation_sequence = 0U;
  std::vector<ModelMutation> mutation_history;

  // Imports representation-local mutations into one diagnostic ordering.
  // Consumers must continue to validate the exact representation revisions.
  void synchronizeMutationJournal() {
    while (imported_spatial_mutations_ < spatial.mutation_history.size()) {
      auto mutation = spatial.mutation_history[imported_spatial_mutations_++];
      mutation.sequence = ++mutation_sequence;
      mutation_history.push_back(std::move(mutation));
    }
    const auto& crowd_mutations = crowd.mutationHistory();
    while (imported_crowd_mutations_ < crowd_mutations.size()) {
      auto mutation = crowd_mutations[imported_crowd_mutations_++];
      mutation.sequence = ++mutation_sequence;
      mutation_history.push_back(std::move(mutation));
    }
  }

 private:
  std::size_t imported_spatial_mutations_ = 0U;
  std::size_t imported_crowd_mutations_ = 0U;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_WORLD_MODEL_HPP
