/**
 * @file world_model.hpp
 * @brief World model responsibilities.
 *
 * @details This file defines world model behavior for ROS-independent domain state
 * and value types. It centers on `ActionSpace`, `RobotState`,
 * `NavigationHistoryEntry`, `NavigationHistory`, `AppendOnlyHistory`,
 * `ObservationHistoryEntry`, `RecoveryState`, `ExplorationCue`. Its
 * package-relative location is `include/semaforr/domain/world_model.hpp`.
 */
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

/**
 * @brief Encapsulates action space state and behavior for this subsystem.
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
class ActionSpace {
 public:
  /**
   * @brief Performs the action space operation for this subsystem.
   *
   * Arguments:
   * - @p move_distances_m: Supplies move distances m input to the
   * operation.
   * - @p rotation_angles_rad: Supplies rotation angles rad input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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

  /**
   * @brief Performs the move distances m operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<double>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<double>& move_distances_m() const noexcept {
    return move_distances_m_;
  }
  /**
   * @brief Performs the rotation angles rad operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<double>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<double>& rotation_angles_rad() const noexcept {
    return rotation_angles_rad_;
  }

  /**
   * @brief Performs the contains operation for this subsystem.
   *
   * Arguments:
   * - @p action: Supplies action input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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
  /**
   * @brief Validates package content for this subsystem.
   *
   * Arguments:
   * - @p values: Supplies values input to the operation.
   * - @p name: Supplies name input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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

/**
 * @brief Encapsulates robot state state and behavior for this subsystem.
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
struct RobotState {
  Pose2D pose;
  std::optional<LaserObservation> laser;
  std::chrono::steady_clock::time_point observed_at{};
};

/**
 * @brief Encapsulates navigation history entry state and behavior for this
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
struct NavigationHistoryEntry {
  Pose2D pose;
  Pose2D observation_pose;
  LaserObservation laser;
  Action action = Action::pause();
  std::optional<TaskId> task_id;
  DecisionId decision_id{0U};
  ActionId action_id{0U};
  ExecutionCompletionStatus execution_status =
      ExecutionCompletionStatus::NoMovement;
  double distance_achieved_m{0.0};
  double rotation_achieved_rad{0.0};

  /**
   * @brief Performs the navigation history entry operation for this
   * subsystem.
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
  NavigationHistoryEntry() = default;
  /**
   * @brief Performs the navigation history entry operation for this
   * subsystem.
   *
   * Arguments:
   * - @p pose_value: Supplies pose value input to the operation.
   * - @p laser_value: Supplies laser value input to the operation.
   * - @p action_value: Supplies action value input to the operation.
   * - @p task_value: Supplies task value input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  NavigationHistoryEntry(Pose2D pose_value, LaserObservation laser_value,
                         Action action_value,
                         std::optional<TaskId> task_value = std::nullopt)
      : pose(std::move(pose_value)),
        observation_pose(pose),
        laser(std::move(laser_value)),
        action(action_value),
        task_id(task_value) {}
};

/**
 * @brief Encapsulates navigation history state and behavior for this
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
class NavigationHistory {
 public:
  /**
   * @brief Records package content for this subsystem.
   *
   * Arguments:
   * - @p entry: Supplies entry input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void record(NavigationHistoryEntry entry) {
    entries_.push_back(std::move(entry));
  }

  /**
   * @brief Performs the entries operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<NavigationHistoryEntry>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<NavigationHistoryEntry>& entries() const noexcept {
    return entries_;
  }

 private:
  std::vector<NavigationHistoryEntry> entries_;
};

/**
 * @brief Encapsulates append only history state and behavior for this
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
template <typename Record>
class AppendOnlyHistory {
 public:
  /**
   * @brief Records package content for this subsystem.
   *
   * Arguments:
   * - @p entry: Supplies entry input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void record(Record entry) { entries_.push_back(std::move(entry)); }
  /**
   * @brief Performs the entries operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<Record>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<Record>& entries() const noexcept { return entries_; }

 private:
  std::vector<Record> entries_;
};

/**
 * @brief Encapsulates observation history entry state and behavior for this
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
struct ObservationHistoryEntry {
  Pose2D pose;
  ExecutionTimestamp observed_at{};
};

using DecisionHistory = AppendOnlyHistory<SelectedActionRecord>;
using CommandHistory = AppendOnlyHistory<ActionStartedEvent>;
using ExecutionHistory = AppendOnlyHistory<ActionExecutionResult>;
using CompletedPathHistory = AppendOnlyHistory<NavigationHistoryEntry>;
using ObservationHistory = AppendOnlyHistory<ObservationHistoryEntry>;

/**
 * @brief Encapsulates recovery state state and behavior for this subsystem.
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
struct RecoveryState {
  bool confined = false;
  std::size_t get_out_attempts = 0U;
  std::size_t reposition_attempts = 0U;
  bool planning_attempted = false;
  bool plan_available = false;
  bool completed_plan_failed_target = false;
  std::size_t tier_two_attempts = 0U;
  std::size_t consecutive_immediate_plan_failures = 0U;
  bool plan_abandoned = false;
};

using FreespaceGrid = SparseCountGrid;

/**
 * @brief Encapsulates exploration cue state and behavior for this
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
struct ExplorationCue {
  std::uint64_t id = 0U;
  Point2D start;
  Point2D target;
};

/**
 * @brief Encapsulates spatial model state and behavior for this subsystem.
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

  /**
   * @brief Performs the revision of operation for this subsystem.
   *
   * Arguments:
   * - @p dependency: Supplies dependency input to the operation.
   *
   * Returns:
   * - `Revision` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Revision revisionOf(ModelDependency dependency) const noexcept {
    const auto found = revisions.find(dependency);
    return found == revisions.end() ? 0U : found->second;
  }
};

/**
 * @brief Encapsulates world model state and behavior for this subsystem.
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
  /**
   * @brief Performs the synchronize mutation journal operation for this
   * subsystem.
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
