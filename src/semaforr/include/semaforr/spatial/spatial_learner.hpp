#ifndef SEMAFORR_SPATIAL_SPATIAL_LEARNER_HPP
#define SEMAFORR_SPATIAL_SPATIAL_LEARNER_HPP

#include <cstddef>
#include <cstdint>
#include <cstdint>
#include <cstdint>
#include <optional>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/mission.hpp>
#include <semaforr/domain/observation.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace semaforr::spatial {

enum class SpatialRepresentation {
  Trails,
  Conveyors,
  Regions,
  DoorsAndExits,
  Hallways,
  Barriers,
  PassagesAndSkeleton,
  KnownGrid,
  InclusionGrid
};

enum class UpdateMode { Incremental, RebuildOnDemand };
enum class UpdateSchedule {
  EveryObservation,
  AfterCompletedAction,
  EndOfTarget,
  EndOfInitialExploration,
  OnDemand
};

enum class ModelStatus { Empty, Incomplete, Fresh, Stale };

struct ObservationContract {
  bool pose = true;
  bool laser = false;
  bool selected_action = false;
  bool task_boundaries = false;
  std::string update_trigger;
  std::vector<std::string> consumers;
  UpdateSchedule schedule = UpdateSchedule::EveryObservation;
};

struct NavigationEpisode {
  std::size_t sequence = 0U;
  domain::RobotObservation observation;
  std::optional<domain::Action> selected_action;
  std::optional<domain::TaskId> active_task;
  bool task_started = false;
  bool task_finished = false;
};

struct TrailModel {
  std::vector<std::vector<domain::Point2D>> trails;
};

struct ConveyorFlow {
  domain::Segment2D axis;
  std::size_t traversals = 1U;
};

struct ConveyorModel {
  std::vector<ConveyorFlow> flows;
};

struct RegionModel {
  std::vector<domain::Circle> regions;
};

struct DoorExitModel {
  std::vector<domain::Segment2D> openings;
};

struct HallwayModel {
  std::vector<domain::Segment2D> centerlines;
};

struct BarrierModel {
  std::vector<domain::Segment2D> barriers;
};

struct SkeletonEdge {
  std::size_t from = 0U;
  std::size_t to = 0U;
};

struct PassageSkeletonModel {
  std::vector<domain::Point2D> nodes;
  std::vector<SkeletonEdge> edges;
};

struct GridGeometry {
  std::size_t columns = 0U;
  std::size_t rows = 0U;
  double resolution_m = 1.0;
  domain::Point2D origin;
};

struct KnownGridModel {
  GridGeometry geometry;
  std::vector<std::uint32_t> observations;
};

struct InclusionGridModel {
  GridGeometry geometry;
  std::vector<std::uint32_t> included;
};

using SpatialPayload = std::variant<std::monostate, TrailModel, ConveyorModel,
                                    RegionModel, DoorExitModel, HallwayModel,
                                    BarrierModel, PassageSkeletonModel,
                                    KnownGridModel, InclusionGridModel>;

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

  bool usable() const noexcept { return status == ModelStatus::Fresh; }
};

std::string_view toString(SpatialRepresentation representation) noexcept;
std::string_view toString(UpdateMode mode) noexcept;
std::string_view toString(UpdateSchedule schedule) noexcept;
std::string_view toString(ModelStatus status) noexcept;
std::string serialize(const SpatialModelUpdate& update);

class SpatialLearner {
 public:
  virtual ~SpatialLearner() = default;

  virtual void observe(const NavigationEpisode& episode) = 0;
  virtual void rebuild() = 0;
  virtual SpatialModelUpdate snapshot() const = 0;

  virtual SpatialRepresentation representation() const noexcept = 0;
  virtual std::string_view name() const noexcept = 0;
  virtual const ObservationContract& contract() const noexcept = 0;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_SPATIAL_LEARNER_HPP
