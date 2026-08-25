/**
 * @file learner.hpp
 * @brief Learner responsibilities.
 *
 * @details This file defines learner behavior for learned spatial representations
 * and their lifecycle. It centers on `SpatialRepresentation`,
 * `UpdateMode`, `SpatialLearningMode`, `UpdateSchedule`, `LearningEvent`,
 * `ModelStatus`, `ObservationContract`, `NavigationEpisode`. Its
 * package-relative location is `include/semaforr/spatial/learner.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_LEARNER_HPP
#define SEMAFORR_SPATIAL_LEARNER_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <memory>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/action_execution.hpp>
#include <semaforr/domain/mission.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/spatial/representations/models.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace semaforr::spatial {

/**
 * @brief Enumerates the supported spatial representation values used by
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
enum class SpatialRepresentation {
  Trails,
  Conveyors,
  Regions,
  DoorsAndExits,
  Hallways,
  Barriers,
  PassagesAndSkeleton,
  KnownGrid,
  SensedOccupancy,
  InclusionGrid,
  Highways,
  Circumstances
};

/**
 * @brief Enumerates the supported update mode values used by this
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
enum class UpdateMode { Incremental, RebuildOnDemand };
/**
 * @brief Enumerates the supported spatial learning mode values used by this
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
enum class SpatialLearningMode { Compatibility, Modernized };
/**
 * @brief Enumerates the supported update schedule values used by this
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
enum class UpdateSchedule {
  EveryObservation,
  EveryDecisionCycle,
  AfterActionStart,
  AfterSuccessfulActionCompletion,
  AfterAnyTerminalActionResult,
  EndOfTarget,
  EndOfTask,
  EndOfInitialExploration,
  DuringHLEOnly,
  DuringLLEOnly,
  Periodic,
  OnShutdown,
  OnDemand
};

/**
 * @brief Enumerates the supported learning event values used by this
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
enum class LearningEvent {
  SensorObservation,
  DecisionSelected,
  ActionStarted,
  ActionProgress,
  ActionTerminal,
  TargetCompleted,
  TaskCompleted,
  InitialExplorationCompleted,
  Periodic,
  Shutdown
};

/**
 * @brief Enumerates the supported model status values used by this
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
enum class ModelStatus { Empty, Incomplete, Fresh, Stale };

/**
 * @brief Encapsulates observation contract state and behavior for this
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
struct ObservationContract {
  bool pose = true;
  bool laser = false;
  bool selected_action = false;
  bool task_boundaries = false;
  std::string update_trigger;
  std::vector<std::string> consumers;
  UpdateSchedule schedule = UpdateSchedule::EveryObservation;
};

/**
 * @brief Encapsulates navigation episode state and behavior for this
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
struct NavigationEpisode {
  std::size_t sequence = 0U;
  domain::RobotObservation observation;
  std::optional<domain::Action> selected_action;
  std::optional<domain::TaskId> active_task;
  bool task_started = false;
  bool task_finished = false;
  bool initial_exploration = false;
  bool action_completed = false;
  std::optional<domain::Point2D> active_target;
  std::vector<domain::Action> viable_actions;
  std::vector<double> move_distances_m;
  std::vector<double> rotation_angles_rad;
  LearningEvent event = LearningEvent::SensorObservation;
  std::optional<domain::SelectedActionRecord> selection;
  std::optional<domain::ActionExecutionResult> execution_result;
  // Appended to preserve the established aggregate-initialization order.
  bool target_reached = false;
  bool task_skipped = false;
  bool action_started = false;

  /**
   * @brief Performs the action succeeded operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool actionSucceeded() const noexcept {
    return execution_result ? action_started && execution_result->successful()
                            : action_completed;
  }
  /**
   * @brief Performs the action terminated operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool actionTerminated() const noexcept {
    return execution_result.has_value() || action_completed;
  }
};

using SpatialPayload = std::variant<std::monostate, TrailModel, ConveyorModel,
                                    RegionModel, DoorExitModel, HallwayModel,
                                    BarrierModel, PassageSkeletonModel,
                                    KnownGridModel, SensedOccupancyModel,
                                    InclusionGridModel,
                                    HighwayModel, CircumstanceModel>;

/**
 * @brief Encapsulates changed cell range state and behavior for this
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
struct ChangedCellRange {
  std::size_t first = 0U;
  std::size_t last = 0U;
};

/**
 * @brief Encapsulates representation change set state and behavior for this
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
struct RepresentationChangeSet {
  std::size_t revision = 0U;
  std::vector<ChangedCellRange> changed_cell_ranges;
  std::size_t added_entities = 0U;
  std::size_t removed_entities = 0U;
  std::size_t updated_entities = 0U;
  std::size_t added_graph_nodes = 0U;
  std::size_t removed_graph_nodes = 0U;
  std::size_t updated_graph_nodes = 0U;
  std::size_t added_graph_edges = 0U;
  std::size_t removed_graph_edges = 0U;
  std::size_t updated_graph_edges = 0U;

  /**
   * @brief Performs the empty operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool empty() const noexcept {
    return changed_cell_ranges.empty() && added_entities == 0U &&
           removed_entities == 0U && updated_entities == 0U &&
           added_graph_nodes == 0U && removed_graph_nodes == 0U &&
           updated_graph_nodes == 0U && added_graph_edges == 0U &&
           removed_graph_edges == 0U && updated_graph_edges == 0U;
  }
};

/**
 * @brief Encapsulates spatial model update state and behavior for this
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
struct SpatialModelUpdate {
  SpatialRepresentation representation = SpatialRepresentation::Trails;
  std::string learner;
  std::size_t revision = 0U;
  std::size_t observed_episodes = 0U;
  std::optional<std::size_t> last_observation_sequence;
  UpdateMode update_mode = UpdateMode::Incremental;
  UpdateSchedule update_schedule = UpdateSchedule::EveryObservation;
  ModelStatus status = ModelStatus::Empty;
  SpatialPayload payload;
  std::vector<std::string> consumers;
  std::string diagnostic;
  RepresentationChangeSet changes;

  /**
   * @brief Performs the usable operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool usable() const noexcept { return status == ModelStatus::Fresh; }
};

using SharedSpatialSnapshot = std::shared_ptr<const SpatialModelUpdate>;

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p representation: Supplies representation input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(SpatialRepresentation representation) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p mode: Supplies mode input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(UpdateMode mode) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p schedule: Supplies schedule input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(UpdateSchedule schedule) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p status: Supplies status input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(ModelStatus status) noexcept;
/**
 * @brief Performs the serialize operation for this subsystem.
 *
 * Arguments:
 * - @p update: Supplies update input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string serialize(const SpatialModelUpdate& update);

/**
 * @brief Encapsulates spatial learner state and behavior for this
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
class SpatialLearner {
 public:
  /**
   * @brief Performs the spatial learner operation for this subsystem.
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
  virtual ~SpatialLearner() = default;

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
  virtual void observe(const NavigationEpisode& episode) = 0;
  /**
   * @brief Performs the rebuild operation for this subsystem.
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
  virtual void rebuild() = 0;
  /**
   * @brief Performs the snapshot operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `SpatialModelUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual SpatialModelUpdate snapshot() const = 0;
  /**
   * @brief Performs the shared snapshot operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `SharedSpatialSnapshot` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual SharedSpatialSnapshot sharedSnapshot() const {
    return std::make_shared<const SpatialModelUpdate>(snapshot());
  }

  /**
   * @brief Performs the representation operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `SpatialRepresentation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual SpatialRepresentation representation() const noexcept = 0;
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::string_view name() const noexcept = 0;
  /**
   * @brief Performs the contract operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const ObservationContract&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual const ObservationContract& contract() const noexcept = 0;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_LEARNER_HPP
