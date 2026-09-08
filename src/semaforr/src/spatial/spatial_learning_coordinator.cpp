/**
 * @file spatial_learning_coordinator.cpp
 * @brief Spatial learning coordinator responsibilities.
 *
 * @details This file implements spatial learning coordinator behavior for
 * learned spatial representations and their lifecycle. It centers on
 * `ProjectionEstimate`. Its package-relative location is
 * `src/spatial/spatial_learning_coordinator.cpp`.
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <semaforr/spatial/learners/all.hpp>
#include <semaforr/spatial/spatial_learning_coordinator.hpp>
#include <stdexcept>
#include <type_traits>

namespace semaforr::spatial {
namespace {

/**
 * @brief Performs the dependency for operation for this subsystem.
 *
 * Arguments:
 * - @p representation: Supplies representation input to the operation.
 *
 * Returns:
 * - `domain::ModelDependency` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::ModelDependency dependencyFor(SpatialRepresentation representation) {
  using D = domain::ModelDependency;
  switch (representation) {
    case SpatialRepresentation::Trails:
      return D::Trails;
    case SpatialRepresentation::Conveyors:
      return D::Conveyors;
    case SpatialRepresentation::Regions:
      return D::Regions;
    case SpatialRepresentation::DoorsAndExits:
      return D::DoorsAndExits;
    case SpatialRepresentation::Hallways:
      return D::Hallways;
    case SpatialRepresentation::Barriers:
      return D::Barriers;
    case SpatialRepresentation::PassagesAndSkeleton:
      return D::Skeleton;
    case SpatialRepresentation::KnownGrid:
      return D::Familiarity;
    case SpatialRepresentation::SensedOccupancy:
      return D::SensedOccupancy;
    case SpatialRepresentation::InclusionGrid:
      return D::Inclusion;
    case SpatialRepresentation::Highways:
      return D::Highways;
    case SpatialRepresentation::Circumstances:
      return D::Circumstances;
  }
  return D::Trails;
}

/**
 * @brief Encapsulates projection estimate state and behavior for this
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
struct ProjectionEstimate {
  std::size_t dense_cells = 0U;
  std::size_t sparse_cells = 0U;
  std::size_t entities = 0U;
  std::size_t allocations = 0U;
  std::size_t bytes = 0U;
  std::size_t shared_bytes = 0U;
};

/**
 * @brief Performs the estimate projection operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `ProjectionEstimate` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
ProjectionEstimate estimateProjection(const SpatialPayload& value) {
  ProjectionEstimate result;
  std::visit(
      [&](const auto& payload) {
        using Payload = std::decay_t<decltype(payload)>;
        const auto add = [&](const auto& entries) {
          result.entities += entries.size();
          result.allocations += entries.empty() ? 0U : 1U;
          result.bytes +=
              entries.size() *
              sizeof(typename std::decay_t<decltype(entries)>::value_type);
        };
        if constexpr (std::is_same_v<Payload, KnownGridModel>) {
          result.dense_cells = payload.observations.size();
          result.sparse_cells = payload.sparse_observations.size();
          result.bytes = payload.observations.size() * sizeof(std::uint32_t);
          result.shared_bytes =
              payload.sparse_observations.size() * sizeof(SparseGridCell) +
              payload.sparse_metadata.size() * sizeof(FamiliarityCellMetadata);
          result.allocations = payload.observations.empty() ? 0U : 1U;
        } else if constexpr (std::is_same_v<Payload, SensedOccupancyModel>) {
          result.dense_cells = payload.cells.size();
          result.sparse_cells = payload.sparse_cells.size();
          result.bytes =
              payload.cells.size() * sizeof(domain::SensedOccupancyCell);
          result.shared_bytes = payload.sparse_cells.size() *
                                sizeof(domain::SparseSensedOccupancyCell);
          result.allocations = payload.cells.empty() ? 0U : 1U;
        } else if constexpr (std::is_same_v<Payload, InclusionGridModel>) {
          result.dense_cells = payload.included.size();
          result.sparse_cells = payload.sparse_included.size();
          result.bytes = payload.included.size() * sizeof(std::uint32_t);
          result.shared_bytes =
              payload.sparse_included.size() * sizeof(SparseGridCell);
          result.allocations = payload.included.empty() ? 0U : 1U;
        } else if constexpr (std::is_same_v<Payload, TrailModel>) {
          add(payload.trails);
          for (const auto& trail : payload.trails) add(trail);
          add(payload.learned_trails);
          for (const auto& trail : payload.learned_trails) {
            add(trail.markers);
            add(trail.subtrail_geometry);
          }
        } else if constexpr (std::is_same_v<Payload, ConveyorModel>) {
          add(payload.flows);
          add(payload.grid.cells);
        } else if constexpr (std::is_same_v<Payload, RegionModel>) {
          add(payload.regions);
          add(payload.learned_regions);
        } else if constexpr (std::is_same_v<Payload, DoorExitModel>) {
          add(payload.openings);
          add(payload.exits);
          add(payload.doors);
          add(payload.sensor_openings);
        } else if constexpr (std::is_same_v<Payload, HallwayModel>) {
          add(payload.centerlines);
          add(payload.hallways);
        } else if constexpr (std::is_same_v<Payload, BarrierModel>) {
          add(payload.barriers);
        } else if constexpr (std::is_same_v<Payload, PassageSkeletonModel>) {
          add(payload.nodes);
          add(payload.edges);
          add(payload.region_nodes);
          add(payload.region_edges);
          add(payload.sampled_path_nodes);
          add(payload.sampled_path_edges);
        } else if constexpr (std::is_same_v<Payload, HighwayModel>) {
          add(payload.highways);
          add(payload.graph.vertices);
          add(payload.graph.edges);
        } else if constexpr (std::is_same_v<Payload, CircumstanceModel>) {
          add(payload.clusters);
          add(payload.cases);
        }
      },
      value);
  return result;
}

/**
 * @brief Clears representation for this subsystem.
 *
 * Arguments:
 * - @p model: Supplies model input to the operation.
 * - @p representation: Supplies representation input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void clearRepresentation(domain::SpatialModel& model,
                         SpatialRepresentation representation) {
  switch (representation) {
    case SpatialRepresentation::Trails:
      model.trails.clear();
      model.learned_trails.clear();
      break;
    case SpatialRepresentation::Conveyors:
      model.conveyor_flows.clear();
      model.conveyor_traversals.clear();
      model.conveyor_grid = {};
      break;
    case SpatialRepresentation::Regions:
      model.learned_regions.clear();
      model.regions.clear();
      break;
    case SpatialRepresentation::DoorsAndExits:
      model.doorways.clear();
      model.exits.clear();
      model.doors.clear();
      model.sensor_openings.clear();
      break;
    case SpatialRepresentation::Hallways:
      model.hallways.clear();
      model.hallway_entities.clear();
      break;
    case SpatialRepresentation::Barriers:
      model.barriers.clear();
      break;
    case SpatialRepresentation::PassagesAndSkeleton:
      model.skeleton_nodes.clear();
      model.skeleton_edges.clear();
      model.region_skeleton_nodes.clear();
      model.region_skeleton_edges.clear();
      model.sampled_path_nodes.clear();
      model.sampled_path_edges.clear();
      break;
    case SpatialRepresentation::KnownGrid:
      model.known_grid = {};
      break;
    case SpatialRepresentation::SensedOccupancy:
      model.sensed_occupancy = {};
      break;
    case SpatialRepresentation::InclusionGrid:
      model.inclusion_grid = {};
      break;
    case SpatialRepresentation::Highways:
      model.highways = {};
      break;
    case SpatialRepresentation::Circumstances:
      model.circumstances = {};
      break;
  }
  const auto dependency = dependencyFor(representation);
  model.snapshot_handles.erase(dependency);
  model.revisions.erase(dependency);
  if (dependency == domain::ModelDependency::Highways)
    model.revisions.erase(domain::ModelDependency::HighwayGraph);
  if (dependency == domain::ModelDependency::Barriers)
    model.revisions.erase(domain::ModelDependency::VisibilityGeometry);
}

}  // namespace

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
SpatialLearningCoordinator::SpatialLearningCoordinator(
    std::size_t automatic_rebuild_interval)
    : automatic_rebuild_interval_(automatic_rebuild_interval) {
  if (automatic_rebuild_interval_ == 0U) {
    throw std::invalid_argument(
        "spatial automatic rebuild interval must be positive");
  }
}

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
SpatialLearningCoordinator SpatialLearningCoordinator::defaults(
    std::size_t automatic_rebuild_interval) {
  return defaults(automatic_rebuild_interval, {});
}

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
SpatialLearningCoordinator SpatialLearningCoordinator::defaults(
    std::size_t automatic_rebuild_interval,
    CircumstanceLearningConfiguration circumstance_configuration) {
  return defaults(automatic_rebuild_interval,
                  std::move(circumstance_configuration), {});
}

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
SpatialLearningCoordinator SpatialLearningCoordinator::defaults(
    std::size_t automatic_rebuild_interval,
    CircumstanceLearningConfiguration circumstance_configuration,
    SensedOccupancyLearningConfiguration occupancy_configuration) {
  return defaults(automatic_rebuild_interval,
                  std::move(circumstance_configuration),
                  occupancy_configuration, GridExtentPolicy::Expand);
}

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
SpatialLearningCoordinator SpatialLearningCoordinator::defaults(
    std::size_t automatic_rebuild_interval,
    CircumstanceLearningConfiguration circumstance_configuration,
    SensedOccupancyLearningConfiguration occupancy_configuration,
    GridExtentPolicy extent_policy) {
  LearnedGridConfiguration grids;
  grids.extent_policy = extent_policy;
  grids.initialize_around_first_pose = false;
  grids.initial_width_m = 200.0;
  grids.initial_height_m = 200.0;
  grids.resolution_m = 1.0;
  grids.expansion.margin_m = 0.0;
  return defaults(automatic_rebuild_interval,
                  std::move(circumstance_configuration),
                  occupancy_configuration, grids);
}

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
SpatialLearningCoordinator SpatialLearningCoordinator::defaults(
    std::size_t automatic_rebuild_interval,
    CircumstanceLearningConfiguration circumstance_configuration,
    SensedOccupancyLearningConfiguration occupancy_configuration,
    LearnedGridConfiguration grid_configuration) {
  SpatialLearningCoordinator coordinator(automatic_rebuild_interval);
  coordinator.addLearner(
      std::make_unique<TrailLearner>(0.05, grid_configuration.learning_mode));
  coordinator.addLearner(std::make_unique<ConveyorLearner>(
      0.05, grid_configuration.learning_mode,
      ConveyorLearningConfiguration{grid_configuration.resolution_m, 1.0,
                                    grid_configuration.resolution_m, true}));
  coordinator.addLearner(std::make_unique<RegionLearner>(
      1.0, 3U, grid_configuration.learning_mode));
  coordinator.addLearner(std::make_unique<DoorExitLearner>(
      0.75, 2.5, grid_configuration.learning_mode));
  coordinator.addLearner(
      std::make_unique<HallwayLearner>(0.5, grid_configuration.learning_mode));
  coordinator.addLearner(std::make_unique<BarrierLearner>());
  coordinator.addLearner(std::make_unique<PassageSkeletonLearner>(
      0.5, grid_configuration.learning_mode));
  const auto columns = static_cast<std::size_t>(std::ceil(
      grid_configuration.initial_width_m / grid_configuration.resolution_m));
  const auto rows = static_cast<std::size_t>(std::ceil(
      grid_configuration.initial_height_m / grid_configuration.resolution_m));
  coordinator.addLearner(std::make_unique<KnownGridLearner>(
      columns, rows, grid_configuration.resolution_m, domain::Point2D{},
      grid_configuration.extent_policy, grid_configuration.expansion,
      grid_configuration.initialize_around_first_pose,
      grid_configuration.frame_id));
  coordinator.addLearner(std::make_unique<SensedOccupancyLearner>(
      columns, rows, grid_configuration.resolution_m, domain::Point2D{},
      occupancy_configuration, grid_configuration.extent_policy,
      grid_configuration.expansion,
      grid_configuration.initialize_around_first_pose,
      grid_configuration.frame_id));
  coordinator.addLearner(std::make_unique<InclusionGridLearner>(
      columns, rows, grid_configuration.resolution_m, domain::Point2D{},
      grid_configuration.extent_policy, grid_configuration.expansion,
      grid_configuration.initialize_around_first_pose,
      grid_configuration.frame_id));
  HighwayLearningConfiguration highway_configuration;
  highway_configuration.grid_resolution_m = grid_configuration.resolution_m;
  highway_configuration.grid_origin = grid_configuration.highway_origin;
  highway_configuration.frame_id = grid_configuration.frame_id;
  const bool modernized =
      grid_configuration.learning_mode == SpatialLearningMode::Modernized;
  if (grid_configuration.highway_smoothing_policy == "profile") {
    highway_configuration.smoothing_policy =
        modernized ? HighwaySmoothingPolicy::DirectionalGapFill
                   : HighwaySmoothingPolicy::VonNeumannThreeOfFour;
  } else if (grid_configuration.highway_smoothing_policy ==
             "von_neumann_three_of_four") {
    highway_configuration.smoothing_policy =
        HighwaySmoothingPolicy::VonNeumannThreeOfFour;
  } else if (grid_configuration.highway_smoothing_policy ==
             "directional_gap_fill") {
    highway_configuration.smoothing_policy =
        HighwaySmoothingPolicy::DirectionalGapFill;
  } else {
    throw std::invalid_argument("unknown highway smoothing policy '" +
                                grid_configuration.highway_smoothing_policy +
                                "'");
  }
  if (grid_configuration.highway_component_selection_policy == "profile") {
    highway_configuration.component_selection_policy =
        modernized ? HighwayComponentSelectionPolicy::LargestVertexCount
                   : HighwayComponentSelectionPolicy::MostIntersections;
  } else if (grid_configuration.highway_component_selection_policy ==
             "most_intersections") {
    highway_configuration.component_selection_policy =
        HighwayComponentSelectionPolicy::MostIntersections;
  } else if (grid_configuration.highway_component_selection_policy ==
             "largest_vertex_count") {
    highway_configuration.component_selection_policy =
        HighwayComponentSelectionPolicy::LargestVertexCount;
  } else {
    throw std::invalid_argument(
        "unknown highway component selection policy '" +
        grid_configuration.highway_component_selection_policy + "'");
  }
  coordinator.addLearner(
      std::make_unique<HighwayLearner>(highway_configuration));
  coordinator.addLearner(std::make_unique<CircumstanceLearner>(
      std::move(circumstance_configuration)));
  return coordinator;
}

/**
 * @brief Performs the add learner operation for this subsystem.
 *
 * Arguments:
 * - @p learner: Supplies learner input to the operation.
 * - @p is_enabled: Supplies is enabled input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void SpatialLearningCoordinator::addLearner(
    std::unique_ptr<SpatialLearner> learner, bool is_enabled) {
  if (!learner) {
    throw std::invalid_argument("spatial learner must not be null");
  }
  const auto representation = learner->representation();
  const auto duplicate =
      std::find_if(learners_.begin(), learners_.end(), [&](const Entry& entry) {
        return entry.learner->representation() == representation ||
               entry.learner->name() == learner->name();
      });
  if (duplicate != learners_.end()) {
    throw std::invalid_argument(
        "duplicate spatial learner or representation: " +
        std::string(learner->name()));
  }
  learners_.push_back({std::move(learner), is_enabled});
}

/**
 * @brief Performs the require operation for this subsystem.
 *
 * Arguments:
 * - @p representation: Supplies representation input to the operation.
 *
 * Returns:
 * - `SpatialLearningCoordinator::Entry&` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
SpatialLearningCoordinator::Entry& SpatialLearningCoordinator::require(
    SpatialRepresentation representation) {
  const auto found =
      std::find_if(learners_.begin(), learners_.end(), [&](const Entry& entry) {
        return entry.learner->representation() == representation;
      });
  if (found == learners_.end()) {
    throw std::out_of_range("spatial learner is not registered: " +
                            std::string(toString(representation)));
  }
  return *found;
}

/**
 * @brief Performs the require operation for this subsystem.
 *
 * Arguments:
 * - @p representation: Supplies representation input to the operation.
 *
 * Returns:
 * - `const SpatialLearningCoordinator::Entry&` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const SpatialLearningCoordinator::Entry& SpatialLearningCoordinator::require(
    SpatialRepresentation representation) const {
  const auto found =
      std::find_if(learners_.begin(), learners_.end(), [&](const Entry& entry) {
        return entry.learner->representation() == representation;
      });
  if (found == learners_.end()) {
    throw std::out_of_range("spatial learner is not registered: " +
                            std::string(toString(representation)));
  }
  return *found;
}

/**
 * @brief Sets enabled for this subsystem.
 *
 * Arguments:
 * - @p representation: Supplies representation input to the operation.
 * - @p is_enabled: Supplies is enabled input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void SpatialLearningCoordinator::setEnabled(
    SpatialRepresentation representation, bool is_enabled) {
  require(representation).enabled = is_enabled;
}

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
bool SpatialLearningCoordinator::enabled(
    SpatialRepresentation representation) const {
  return require(representation).enabled;
}

namespace {

/**
 * @brief Performs the accepts event operation for this subsystem.
 *
 * Arguments:
 * - @p contract: Supplies contract input to the operation.
 * - @p episode: Supplies episode input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool acceptsEvent(const ObservationContract& contract,
                  const NavigationEpisode& episode) {
  switch (contract.schedule) {
    case UpdateSchedule::EveryObservation:
      return episode.event == LearningEvent::SensorObservation;
    case UpdateSchedule::EveryDecisionCycle:
      return episode.event == LearningEvent::DecisionSelected;
    case UpdateSchedule::AfterActionStart:
      return episode.event == LearningEvent::ActionStarted;
    case UpdateSchedule::AfterSuccessfulActionCompletion:
      return episode.event == LearningEvent::ActionTerminal &&
             episode.actionSucceeded();
    case UpdateSchedule::AfterAnyTerminalActionResult:
    case UpdateSchedule::EndOfTarget:
    case UpdateSchedule::EndOfTask:
      return episode.event == LearningEvent::ActionTerminal;
    case UpdateSchedule::EndOfInitialExploration:
    case UpdateSchedule::DuringHLEOnly:
      return episode.initial_exploration &&
             episode.event == LearningEvent::ActionTerminal &&
             episode.actionSucceeded();
    case UpdateSchedule::DuringLLEOnly:
      return episode.event == LearningEvent::ActionTerminal &&
             episode.selection &&
             episode.selection->provenance.find("LLE") != std::string::npos;
    case UpdateSchedule::Periodic:
      return episode.event == LearningEvent::Periodic;
    case UpdateSchedule::OnShutdown:
      return episode.event == LearningEvent::Shutdown;
    case UpdateSchedule::OnDemand:
      return false;
  }
  return false;
}

}  // namespace

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
void SpatialLearningCoordinator::dispatch(const NavigationEpisode& episode) {
  bool accepted = false;
  std::optional<NavigationEpisode> sequenced;
  for (Entry& entry : learners_) {
    const bool circumstance_event =
        entry.learner->representation() ==
            SpatialRepresentation::Circumstances &&
        (episode.event == LearningEvent::SensorObservation ||
         episode.event == LearningEvent::DecisionSelected ||
         episode.event == LearningEvent::ActionTerminal);
    if (entry.enabled && (circumstance_event ||
                          acceptsEvent(entry.learner->contract(), episode))) {
      if (!sequenced) {
        sequenced = episode;
        sequenced->sequence = ++learning_event_sequence_;
      }
      entry.learner->observe(*sequenced);
      accepted = true;
    }
  }
  if (!accepted) return;
  ++observed_episodes_;
  if (observed_episodes_ % automatic_rebuild_interval_ == 0U) {
    rebuildStale();
  }
}

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
void SpatialLearningCoordinator::observe(const NavigationEpisode& episode) {
  if (last_legacy_observation_sequence_ &&
      episode.sequence <= *last_legacy_observation_sequence_)
    throw std::invalid_argument(
        "navigation episodes must have strictly increasing sequence numbers");
  last_legacy_observation_sequence_ = episode.sequence;
  NavigationEpisode sensed = episode;
  sensed.event = LearningEvent::SensorObservation;
  sensed.action_completed = false;
  sensed.execution_result.reset();
  dispatch(sensed);
  if (episode.execution_result || episode.action_completed) {
    NavigationEpisode terminal = episode;
    terminal.event = LearningEvent::ActionTerminal;
    dispatch(terminal);
  }
}

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
void SpatialLearningCoordinator::observeSensor(NavigationEpisode episode) {
  episode.event = LearningEvent::SensorObservation;
  episode.action_completed = false;
  episode.execution_result.reset();
  dispatch(episode);
}

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
void SpatialLearningCoordinator::observeDecision(NavigationEpisode episode) {
  episode.event = LearningEvent::DecisionSelected;
  dispatch(episode);
}

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
void SpatialLearningCoordinator::observeActionStarted(
    NavigationEpisode episode) {
  episode.event = LearningEvent::ActionStarted;
  dispatch(episode);
}

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
void SpatialLearningCoordinator::observeActionProgress(
    NavigationEpisode episode) {
  episode.event = LearningEvent::ActionProgress;
  dispatch(episode);
}

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
void SpatialLearningCoordinator::observeActionTerminal(
    NavigationEpisode episode) {
  if (!episode.execution_result)
    throw std::invalid_argument(
        "terminal learning episode requires an execution result");
  episode.event = LearningEvent::ActionTerminal;
  episode.action_completed = episode.execution_result->successful();
  dispatch(episode);
}

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
void SpatialLearningCoordinator::rebuild(SpatialRepresentation representation) {
  Entry& entry = require(representation);
  if (entry.enabled) {
    entry.learner->rebuild();
  }
}

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
void SpatialLearningCoordinator::rebuildStale() {
  for (Entry& entry : learners_) {
    if (!entry.enabled) {
      continue;
    }
    const SpatialModelUpdate update = entry.learner->snapshot();
    if (entry.learner->contract().schedule == UpdateSchedule::OnDemand &&
        update.update_mode == UpdateMode::RebuildOnDemand &&
        (update.status == ModelStatus::Stale ||
         update.status == ModelStatus::Incomplete)) {
      entry.learner->rebuild();
    }
  }
}

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
void SpatialLearningCoordinator::rebuildAll() {
  for (Entry& entry : learners_) {
    if (entry.enabled) {
      entry.learner->rebuild();
    }
  }
}

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
void SpatialLearningCoordinator::finalizeInitialExploration() {
  for (Entry& entry : learners_) {
    if (entry.enabled &&
        (entry.learner->contract().schedule ==
             UpdateSchedule::EndOfInitialExploration ||
         entry.learner->representation() == SpatialRepresentation::Regions ||
         entry.learner->representation() ==
             SpatialRepresentation::PassagesAndSkeleton) &&
        entry.learner->representation() != SpatialRepresentation::InclusionGrid)
      entry.learner->rebuild();
  }
  synchronizeInclusion();
}

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
void SpatialLearningCoordinator::finalizeTarget() {
  for (Entry& entry : learners_)
    if (entry.enabled &&
        (entry.learner->contract().schedule == UpdateSchedule::EndOfTarget ||
         entry.learner->representation() == SpatialRepresentation::Regions ||
         entry.learner->representation() ==
             SpatialRepresentation::PassagesAndSkeleton) &&
        entry.learner->representation() != SpatialRepresentation::InclusionGrid)
      entry.learner->rebuild();
  synchronizeInclusion();
}

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
void SpatialLearningCoordinator::synchronizeInclusion() {
  auto inclusion =
      std::find_if(learners_.begin(), learners_.end(), [](const Entry& entry) {
        return entry.learner->representation() ==
               SpatialRepresentation::InclusionGrid;
      });
  if (inclusion == learners_.end() || !inclusion->enabled) return;
  RegionModel regions;
  PassageSkeletonModel skeleton;
  if (const auto update = snapshot(SpatialRepresentation::Regions);
      update && update->usable() &&
      std::holds_alternative<RegionModel>(update->payload))
    regions = std::get<RegionModel>(update->payload);
  if (const auto update = snapshot(SpatialRepresentation::PassagesAndSkeleton);
      update && update->usable() &&
      std::holds_alternative<PassageSkeletonModel>(update->payload))
    skeleton = std::get<PassageSkeletonModel>(update->payload);
  auto* learner = dynamic_cast<InclusionGridLearner*>(inclusion->learner.get());
  if (!learner)
    throw std::logic_error(
        "inclusion representation is not provided by InclusionGridLearner");
  learner->replaceRepresented(regions, skeleton);
}

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
std::optional<SpatialModelUpdate> SpatialLearningCoordinator::snapshot(
    SpatialRepresentation representation) const {
  const Entry& entry = require(representation);
  if (!entry.enabled) {
    return std::nullopt;
  }
  return entry.learner->snapshot();
}

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
std::vector<SpatialModelUpdate> SpatialLearningCoordinator::snapshots() const {
  std::vector<SpatialModelUpdate> result;
  result.reserve(enabledCount());
  for (const Entry& entry : learners_) {
    if (entry.enabled) {
      result.push_back(entry.learner->snapshot());
    }
  }
  std::sort(result.begin(), result.end(),
            [](const auto& first, const auto& second) {
              return first.representation < second.representation;
            });
  return result;
}

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
std::vector<LearnerInspection> SpatialLearningCoordinator::inspect() const {
  std::vector<LearnerInspection> result;
  result.reserve(learners_.size());
  for (const Entry& entry : learners_) {
    result.push_back({entry.learner->representation(),
                      std::string(entry.learner->name()), entry.enabled,
                      entry.learner->contract(), entry.learner->snapshot()});
  }
  std::sort(result.begin(), result.end(),
            [](const auto& first, const auto& second) {
              return first.representation < second.representation;
            });
  return result;
}

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
std::string SpatialLearningCoordinator::serialize(
    SpatialRepresentation representation) const {
  const Entry& entry = require(representation);
  if (!entry.enabled) {
    throw std::logic_error("cannot serialize a disabled spatial learner: " +
                           std::string(toString(representation)));
  }
  return spatial::serialize(entry.learner->snapshot());
}

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
std::string SpatialLearningCoordinator::serializeAll() const {
  std::string result{"{\"schema\":\"semaforr.spatial.v1\",\"models\":["};
  bool first = true;
  for (const auto& update : snapshots()) {
    if (!first) result += ',';
    first = false;
    result += spatial::serialize(update);
  }
  result += "]}";
  return result;
}

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
void SpatialLearningCoordinator::applyTo(domain::SpatialModel& model) const {
  const auto started = std::chrono::steady_clock::now();
  SnapshotProjectionMetrics metrics;
  for (const Entry& entry : learners_) {
    if (!entry.enabled) {
      clearRepresentation(model, entry.learner->representation());
      continue;
    }
    const auto snapshot = entry.learner->sharedSnapshot();
    ++metrics.snapshots_examined;
    if (!snapshot) continue;
    const SpatialModelUpdate& update = *snapshot;
    if (!update.usable()) {
      continue;
    }
    const auto dependency = dependencyFor(update.representation);
    if (model.revisionOf(dependency) == update.revision) {
      ++metrics.unchanged_snapshots_reused;
      continue;
    }
    ++metrics.representations_projected;
    const auto estimate = estimateProjection(update.payload);
    metrics.dense_cells_copied += estimate.dense_cells;
    metrics.sparse_cells_shared += estimate.sparse_cells;
    metrics.entities_copied += estimate.entities;
    metrics.estimated_allocations += estimate.allocations;
    metrics.estimated_bytes_copied += estimate.bytes;
    metrics.estimated_bytes_shared += estimate.shared_bytes;
    // Shared immutable storage remains resident in the world model even when
    // projection performs no cell copy. Include it in the high-water mark.
    metrics.peak_projection_bytes = std::max(
        metrics.peak_projection_bytes, estimate.bytes + estimate.shared_bytes);
    std::visit(
        [&model, &update, &snapshot](const auto& payload) {
          using Payload = std::decay_t<decltype(payload)>;
          if constexpr (std::is_same_v<Payload, TrailModel>) {
            model.trails = payload.trails;
            model.learned_trails = payload.learned_trails;
          } else if constexpr (std::is_same_v<Payload, ConveyorModel>) {
            model.conveyor_flows.clear();
            model.conveyor_traversals.clear();
            for (const ConveyorFlow& flow : payload.flows) {
              model.conveyor_flows.push_back(flow.axis);
              model.conveyor_traversals.push_back(flow.traversals);
            }
            model.conveyor_grid = payload.grid;
          } else if constexpr (std::is_same_v<Payload, RegionModel>) {
            model.learned_regions = payload.regions;
            model.regions = payload.learned_regions;
          } else if constexpr (std::is_same_v<Payload, DoorExitModel>) {
            model.doorways = payload.openings;
            model.exits = payload.exits;
            model.doors = payload.doors;
            model.sensor_openings = payload.sensor_openings;
          } else if constexpr (std::is_same_v<Payload, HallwayModel>) {
            model.hallways = payload.centerlines;
            model.hallway_entities = payload.hallways;
          } else if constexpr (std::is_same_v<Payload, BarrierModel>) {
            model.barriers = payload.barriers;
          } else if constexpr (std::is_same_v<Payload, PassageSkeletonModel>) {
            model.skeleton_nodes = payload.nodes;
            model.skeleton_edges.clear();
            for (const SkeletonEdge& edge : payload.edges) {
              model.skeleton_edges.emplace_back(edge.from, edge.to);
            }
            model.region_skeleton_nodes = payload.region_nodes;
            model.region_skeleton_edges = payload.region_edges;
            model.sampled_path_nodes = payload.sampled_path_nodes;
            model.sampled_path_edges.clear();
            for (const SkeletonEdge& edge : payload.sampled_path_edges)
              model.sampled_path_edges.emplace_back(edge.from, edge.to);
          } else if constexpr (std::is_same_v<Payload, KnownGridModel>) {
            model.known_grid = {
                payload.geometry.columns,      payload.geometry.rows,
                payload.geometry.resolution_m, payload.geometry.origin,
                payload.observations,          update.revision};
            model.known_grid.sparse_snapshot =
                std::shared_ptr<const std::vector<domain::SparseCountCell>>(
                    snapshot, &payload.sparse_observations);
            model.known_grid.sparse_metadata_snapshot = std::shared_ptr<
                const std::vector<domain::SparseFamiliarityMetadata>>(
                snapshot, &payload.sparse_metadata);
            model.known_grid.frame_id = payload.geometry.frame_id;
            model.known_grid.geometry_revision =
                payload.geometry.geometry_revision;
            model.known_grid.extent_mode = payload.geometry.extent_mode;
            model.known_grid.extent_source = payload.geometry.extent_source;
          } else if constexpr (std::is_same_v<Payload, SensedOccupancyModel>) {
            model.sensed_occupancy = {};
            model.sensed_occupancy.geometry = payload.geometry;
            model.sensed_occupancy.cells = payload.cells;
            model.sensed_occupancy.sparse_snapshot = std::shared_ptr<
                const std::vector<domain::SparseSensedOccupancyCell>>(
                snapshot, &payload.sparse_cells);
            model.sensed_occupancy.revision = update.revision;
          } else if constexpr (std::is_same_v<Payload, InclusionGridModel>) {
            model.inclusion_grid = domain::SparseCountGrid(
                payload.geometry.columns, payload.geometry.rows,
                payload.geometry.resolution_m, payload.geometry.origin,
                payload.included, update.revision);
            model.inclusion_grid.sparse_snapshot =
                std::shared_ptr<const std::vector<domain::SparseCountCell>>(
                    snapshot, &payload.sparse_included);
          } else if constexpr (std::is_same_v<Payload, HighwayModel>) {
            model.highways.graph = payload.graph;
            model.highways.highways = payload.highways;
            model.highways.serialized_schema_version =
                payload.serialized_schema_version;
            model.highways.nodes = payload.nodes;
            model.highways.edges.clear();
            for (const SkeletonEdge& edge : payload.edges)
              model.highways.edges.emplace_back(edge.from, edge.to);
            model.highways.intersections.clear();
            for (const HighwayIntersection& intersection :
                 payload.intersections)
              model.highways.intersections.push_back(
                  {intersection.node, intersection.degree});
            model.highways.geometry = payload.geometry;
            model.highways.smoothing_policy = payload.smoothing_policy;
            model.highways.component_selection_policy =
                payload.component_selection_policy;
            model.highways.revision = update.revision;
          } else if constexpr (std::is_same_v<Payload, CircumstanceModel>) {
            model.circumstances = payload;
            model.circumstances.revision = update.revision;
          }
        },
        update.payload);
    model.revisions[dependency] = update.revision;
    model.snapshot_handles[dependency] = snapshot;
    ++model.mutation_sequence;
    model.revision = static_cast<std::size_t>(model.mutation_sequence);
    model.mutation_history.push_back(
        {model.mutation_sequence, dependency, update.revision,
         std::chrono::steady_clock::now(),
         std::string(update.learner) + ":" + update.diagnostic});
    if (dependency == domain::ModelDependency::Highways) {
      model.revisions[domain::ModelDependency::HighwayGraph] = update.revision;
      model.highways.revision = update.revision;
    }
    if (dependency == domain::ModelDependency::Barriers)
      model.revisions[domain::ModelDependency::VisibilityGeometry] =
          update.revision;
  }
  metrics.projection_time_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
          .count();
  last_projection_metrics_ = metrics;
  auto& total = cumulative_projection_metrics_;
  total.snapshots_examined += metrics.snapshots_examined;
  total.unchanged_snapshots_reused += metrics.unchanged_snapshots_reused;
  total.representations_projected += metrics.representations_projected;
  total.dense_cells_copied += metrics.dense_cells_copied;
  total.sparse_cells_copied += metrics.sparse_cells_copied;
  total.sparse_cells_shared += metrics.sparse_cells_shared;
  total.entities_copied += metrics.entities_copied;
  total.estimated_allocations += metrics.estimated_allocations;
  total.estimated_bytes_copied += metrics.estimated_bytes_copied;
  total.estimated_bytes_shared += metrics.estimated_bytes_shared;
  total.peak_projection_bytes =
      std::max(total.peak_projection_bytes, metrics.peak_projection_bytes);
  total.projection_time_s += metrics.projection_time_s;
  total.lock_duration_s += metrics.lock_duration_s;
}

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
std::size_t SpatialLearningCoordinator::enabledCount() const noexcept {
  return static_cast<std::size_t>(
      std::count_if(learners_.begin(), learners_.end(),
                    [](const Entry& entry) { return entry.enabled; }));
}

}  // namespace semaforr::spatial
