/**
 * @file spatial_learning_coordinator.hpp
 * @brief Spatial learning coordinator responsibilities.
 *
 * @details This file defines spatial learning coordinator behavior for learned
 * spatial representations and their lifecycle. It centers on
 * `LearnerInspection`, `LearnedGridConfiguration`,
 * `SnapshotProjectionMetrics`, `SpatialLearningCoordinator`, `Entry`. Its
 * package-relative location is
 * `include/semaforr/spatial/spatial_learning_coordinator.hpp`.
 */
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

/**
 * @brief Encapsulates learner inspection state and behavior for this
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
struct LearnerInspection {
  SpatialRepresentation representation = SpatialRepresentation::Trails;
  std::string name;
  bool enabled = false;
  ObservationContract contract;
  SpatialModelUpdate update;
};

/**
 * @brief Encapsulates learned grid configuration state and behavior for
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
struct LearnedGridConfiguration {
  std::string frame_id{"map"};
  double initial_width_m{20.0};
  double initial_height_m{20.0};
  double resolution_m{0.5};
  domain::Point2D highway_origin;
  std::string highway_smoothing_policy{"profile"};
  std::string highway_component_selection_policy{"profile"};
  GridExtentPolicy extent_policy = GridExtentPolicy::Expand;
  domain::GridExpansionPolicy expansion;
  bool initialize_around_first_pose{true};
  SpatialLearningMode learning_mode{SpatialLearningMode::Modernized};
};

/**
 * @brief Encapsulates snapshot projection metrics state and behavior for
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
struct SnapshotProjectionMetrics {
  std::size_t snapshots_examined = 0U;
  std::size_t unchanged_snapshots_reused = 0U;
  std::size_t representations_projected = 0U;
  std::size_t dense_cells_copied = 0U;
  std::size_t sparse_cells_copied = 0U;
  std::size_t sparse_cells_shared = 0U;
  std::size_t entities_copied = 0U;
  std::size_t estimated_allocations = 0U;
  std::size_t estimated_bytes_copied = 0U;
  std::size_t estimated_bytes_shared = 0U;
  std::size_t peak_projection_bytes = 0U;
  double projection_time_s = 0.0;
  double lock_duration_s = 0.0;
};

/**
 * @brief Encapsulates spatial learning coordinator state and behavior for
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
class SpatialLearningCoordinator {
 public:
  /**
   * @brief Performs the spatial learning coordinator operation for this
   * subsystem.
   *
   * Arguments:
   * - @p automatic_rebuild_interval: Supplies automatic rebuild interval
   * input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit SpatialLearningCoordinator(
      std::size_t automatic_rebuild_interval = 10U);

  /**
   * @brief Performs the defaults operation for this subsystem.
   *
   * Arguments:
   * - @p automatic_rebuild_interval: Supplies automatic rebuild interval
   * input to the operation.
   *
   * Returns:
   * - `SpatialLearningCoordinator` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static SpatialLearningCoordinator defaults(
      std::size_t automatic_rebuild_interval = 10U);
  /**
   * @brief Performs the defaults operation for this subsystem.
   *
   * Arguments:
   * - @p automatic_rebuild_interval: Supplies automatic rebuild interval
   * input to the operation.
   * - @p circumstance_configuration: Supplies circumstance configuration
   * input to the operation.
   *
   * Returns:
   * - `SpatialLearningCoordinator` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static SpatialLearningCoordinator defaults(
      std::size_t automatic_rebuild_interval,
      CircumstanceLearningConfiguration circumstance_configuration);
  /**
   * @brief Performs the defaults operation for this subsystem.
   *
   * Arguments:
   * - @p automatic_rebuild_interval: Supplies automatic rebuild interval
   * input to the operation.
   * - @p circumstance_configuration: Supplies circumstance configuration
   * input to the operation.
   * - @p occupancy_configuration: Supplies occupancy configuration input to
   * the operation.
   *
   * Returns:
   * - `SpatialLearningCoordinator` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static SpatialLearningCoordinator defaults(
      std::size_t automatic_rebuild_interval,
      CircumstanceLearningConfiguration circumstance_configuration,
      SensedOccupancyLearningConfiguration occupancy_configuration);
  /**
   * @brief Performs the defaults operation for this subsystem.
   *
   * Arguments:
   * - @p automatic_rebuild_interval: Supplies automatic rebuild interval
   * input to the operation.
   * - @p circumstance_configuration: Supplies circumstance configuration
   * input to the operation.
   * - @p occupancy_configuration: Supplies occupancy configuration input to
   * the operation.
   * - @p extent_policy: Supplies extent policy input to the operation.
   *
   * Returns:
   * - `SpatialLearningCoordinator` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static SpatialLearningCoordinator defaults(
      std::size_t automatic_rebuild_interval,
      CircumstanceLearningConfiguration circumstance_configuration,
      SensedOccupancyLearningConfiguration occupancy_configuration,
      GridExtentPolicy extent_policy);
  /**
   * @brief Performs the defaults operation for this subsystem.
   *
   * Arguments:
   * - @p automatic_rebuild_interval: Supplies automatic rebuild interval
   * input to the operation.
   * - @p circumstance_configuration: Supplies circumstance configuration
   * input to the operation.
   * - @p occupancy_configuration: Supplies occupancy configuration input to
   * the operation.
   * - @p grid_configuration: Supplies grid configuration input to the
   * operation.
   *
   * Returns:
   * - `SpatialLearningCoordinator` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static SpatialLearningCoordinator defaults(
      std::size_t automatic_rebuild_interval,
      CircumstanceLearningConfiguration circumstance_configuration,
      SensedOccupancyLearningConfiguration occupancy_configuration,
      LearnedGridConfiguration grid_configuration);
  /**
   * @brief Performs the add learner operation for this subsystem.
   *
   * Arguments:
   * - @p learner: Supplies learner input to the operation.
   * - @p enabled: Supplies enabled input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void addLearner(std::unique_ptr<SpatialLearner> learner, bool enabled = true);
  /**
   * @brief Sets enabled for this subsystem.
   *
   * Arguments:
   * - @p representation: Supplies representation input to the operation.
   * - @p enabled: Supplies enabled input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void setEnabled(SpatialRepresentation representation, bool enabled);
  /**
   * @brief Performs the enabled operation for this subsystem.
   *
   * Arguments:
   * - @p representation: Supplies representation input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool enabled(SpatialRepresentation representation) const;

  /**
   * @brief Processes package content for this subsystem.
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
  void observe(const NavigationEpisode& episode);
  /**
   * @brief Processes sensor for this subsystem.
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
  void observeSensor(NavigationEpisode episode);
  /**
   * @brief Processes decision for this subsystem.
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
  void observeDecision(NavigationEpisode episode);
  /**
   * @brief Processes action started for this subsystem.
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
  void observeActionStarted(NavigationEpisode episode);
  /**
   * @brief Processes action progress for this subsystem.
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
  void observeActionProgress(NavigationEpisode episode);
  /**
   * @brief Processes action terminal for this subsystem.
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
  void observeActionTerminal(NavigationEpisode episode);
  /**
   * @brief Performs the rebuild operation for this subsystem.
   *
   * Arguments:
   * - @p representation: Supplies representation input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void rebuild(SpatialRepresentation representation);
  /**
   * @brief Performs the rebuild stale operation for this subsystem.
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
  void rebuildStale();
  /**
   * @brief Performs the rebuild all operation for this subsystem.
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
  void rebuildAll();
  /**
   * @brief Performs the finalize initial exploration operation for this
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
  void finalizeInitialExploration();
  /**
   * @brief Performs the finalize target operation for this subsystem.
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
  void finalizeTarget();

  /**
   * @brief Performs the snapshot operation for this subsystem.
   *
   * Arguments:
   * - @p representation: Supplies representation input to the operation.
   *
   * Returns:
   * - `std::optional<SpatialModelUpdate>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<SpatialModelUpdate> snapshot(
      SpatialRepresentation representation) const;
  /**
   * @brief Performs the snapshots operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<SpatialModelUpdate>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<SpatialModelUpdate> snapshots() const;
  /**
   * @brief Performs the inspect operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<LearnerInspection>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<LearnerInspection> inspect() const;
  /**
   * @brief Performs the serialize operation for this subsystem.
   *
   * Arguments:
   * - @p representation: Supplies representation input to the operation.
   *
   * Returns:
   * - `std::string` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string serialize(SpatialRepresentation representation) const;
  /**
   * @brief Performs the serialize all operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string serializeAll() const;

  /**
   * @brief Applies to for this subsystem.
   *
   * Arguments:
   * - @p model: Supplies model input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void applyTo(domain::SpatialModel& model) const;
  /**
   * @brief Performs the last projection metrics operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const SnapshotProjectionMetrics&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const SnapshotProjectionMetrics& lastProjectionMetrics() const noexcept {
    return last_projection_metrics_;
  }
  /**
   * @brief Performs the cumulative projection metrics operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const SnapshotProjectionMetrics&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const SnapshotProjectionMetrics& cumulativeProjectionMetrics() const noexcept {
    return cumulative_projection_metrics_;
  }
  /**
   * @brief Performs the learner count operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t learnerCount() const noexcept { return learners_.size(); }
  /**
   * @brief Performs the enabled count operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t enabledCount() const noexcept;

 private:
  /**
   * @brief Performs the synchronize inclusion operation for this subsystem.
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
  void synchronizeInclusion();
  /**
   * @brief Performs the dispatch operation for this subsystem.
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
  void dispatch(const NavigationEpisode& episode);
  /**
   * @brief Encapsulates entry state and behavior for this subsystem.
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
  struct Entry {
    std::unique_ptr<SpatialLearner> learner;
    bool enabled = true;
  };

  /**
   * @brief Performs the require operation for this subsystem.
   *
   * Arguments:
   * - @p representation: Supplies representation input to the operation.
   *
   * Returns:
   * - `Entry&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Entry& require(SpatialRepresentation representation);
  /**
   * @brief Performs the require operation for this subsystem.
   *
   * Arguments:
   * - @p representation: Supplies representation input to the operation.
   *
   * Returns:
   * - `const Entry&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const Entry& require(SpatialRepresentation representation) const;

  std::size_t automatic_rebuild_interval_;
  std::size_t observed_episodes_ = 0U;
  std::size_t learning_event_sequence_ = 0U;
  std::optional<std::size_t> last_legacy_observation_sequence_;
  std::vector<Entry> learners_;
  mutable SnapshotProjectionMetrics last_projection_metrics_;
  mutable SnapshotProjectionMetrics cumulative_projection_metrics_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_SPATIAL_LEARNING_COORDINATOR_HPP
