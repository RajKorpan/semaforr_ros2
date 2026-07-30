#include <algorithm>
#include <semaforr/spatial/learners.hpp>
#include <semaforr/spatial/spatial_learning_coordinator.hpp>
#include <stdexcept>
#include <type_traits>

namespace semaforr::spatial {
namespace {

void clearRepresentation(domain::SpatialModel& model,
                         SpatialRepresentation representation) {
  switch (representation) {
    case SpatialRepresentation::Trails:
      model.trails.clear();
      break;
    case SpatialRepresentation::Conveyors:
      model.conveyor_flows.clear();
      break;
    case SpatialRepresentation::Regions:
      model.learned_regions.clear();
      break;
    case SpatialRepresentation::DoorsAndExits:
      model.doorways.clear();
      break;
    case SpatialRepresentation::Hallways:
      model.hallways.clear();
      break;
    case SpatialRepresentation::Barriers:
      model.barriers.clear();
      break;
    case SpatialRepresentation::PassagesAndSkeleton:
      model.skeleton_nodes.clear();
      model.skeleton_edges.clear();
      break;
    case SpatialRepresentation::KnownGrid:
      model.known_grid = {};
      break;
    case SpatialRepresentation::InclusionGrid:
      model.inclusion_grid = {};
      break;
    case SpatialRepresentation::Highways:
      model.highways = {};
      break;
  }
}

}  // namespace

SpatialLearningCoordinator::SpatialLearningCoordinator(
    std::size_t automatic_rebuild_interval)
    : automatic_rebuild_interval_(automatic_rebuild_interval) {
  if (automatic_rebuild_interval_ == 0U) {
    throw std::invalid_argument(
        "spatial automatic rebuild interval must be positive");
  }
}

SpatialLearningCoordinator SpatialLearningCoordinator::defaults(
    std::size_t automatic_rebuild_interval) {
  SpatialLearningCoordinator coordinator(automatic_rebuild_interval);
  coordinator.addLearner(std::make_unique<TrailLearner>());
  coordinator.addLearner(std::make_unique<ConveyorLearner>());
  coordinator.addLearner(std::make_unique<RegionLearner>());
  coordinator.addLearner(std::make_unique<DoorExitLearner>());
  coordinator.addLearner(std::make_unique<HallwayLearner>());
  coordinator.addLearner(std::make_unique<BarrierLearner>());
  coordinator.addLearner(std::make_unique<PassageSkeletonLearner>());
  coordinator.addLearner(std::make_unique<KnownGridLearner>());
  coordinator.addLearner(std::make_unique<InclusionGridLearner>());
  coordinator.addLearner(std::make_unique<HighwayLearner>());
  return coordinator;
}

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

void SpatialLearningCoordinator::setEnabled(
    SpatialRepresentation representation, bool is_enabled) {
  require(representation).enabled = is_enabled;
}

bool SpatialLearningCoordinator::enabled(
    SpatialRepresentation representation) const {
  return require(representation).enabled;
}

void SpatialLearningCoordinator::observe(const NavigationEpisode& episode) {
  for (Entry& entry : learners_) {
    if (entry.enabled) {
      entry.learner->observe(episode);
    }
  }
  ++observed_episodes_;
  if (observed_episodes_ % automatic_rebuild_interval_ == 0U) {
    rebuildStale();
  }
}

void SpatialLearningCoordinator::rebuild(SpatialRepresentation representation) {
  Entry& entry = require(representation);
  if (entry.enabled) {
    entry.learner->rebuild();
  }
}

void SpatialLearningCoordinator::rebuildStale() {
  for (Entry& entry : learners_) {
    if (!entry.enabled) {
      continue;
    }
    const SpatialModelUpdate update = entry.learner->snapshot();
    if (update.update_mode == UpdateMode::RebuildOnDemand &&
        (update.status == ModelStatus::Stale ||
         update.status == ModelStatus::Incomplete)) {
      entry.learner->rebuild();
    }
  }
}

void SpatialLearningCoordinator::rebuildAll() {
  for (Entry& entry : learners_) {
    if (entry.enabled) {
      entry.learner->rebuild();
    }
  }
}

std::optional<SpatialModelUpdate> SpatialLearningCoordinator::snapshot(
    SpatialRepresentation representation) const {
  const Entry& entry = require(representation);
  if (!entry.enabled) {
    return std::nullopt;
  }
  return entry.learner->snapshot();
}

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

std::string SpatialLearningCoordinator::serialize(
    SpatialRepresentation representation) const {
  const Entry& entry = require(representation);
  if (!entry.enabled) {
    throw std::logic_error("cannot serialize a disabled spatial learner: " +
                           std::string(toString(representation)));
  }
  return spatial::serialize(entry.learner->snapshot());
}

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

void SpatialLearningCoordinator::applyTo(domain::SpatialModel& model) const {
  for (const Entry& entry : learners_) {
    if (!entry.enabled) {
      clearRepresentation(model, entry.learner->representation());
      continue;
    }
    const SpatialModelUpdate update = entry.learner->snapshot();
    if (!update.usable()) {
      continue;
    }
    std::visit(
        [&model, &update](const auto& payload) {
          using Payload = std::decay_t<decltype(payload)>;
          if constexpr (std::is_same_v<Payload, TrailModel>) {
            model.trails = payload.trails;
          } else if constexpr (std::is_same_v<Payload, ConveyorModel>) {
            model.conveyor_flows.clear();
            for (const ConveyorFlow& flow : payload.flows) {
              model.conveyor_flows.push_back(flow.axis);
            }
          } else if constexpr (std::is_same_v<Payload, RegionModel>) {
            model.learned_regions = payload.regions;
          } else if constexpr (std::is_same_v<Payload, DoorExitModel>) {
            model.doorways = payload.openings;
          } else if constexpr (std::is_same_v<Payload, HallwayModel>) {
            model.hallways = payload.centerlines;
          } else if constexpr (std::is_same_v<Payload, BarrierModel>) {
            model.barriers = payload.barriers;
          } else if constexpr (std::is_same_v<Payload, PassageSkeletonModel>) {
            model.skeleton_nodes = payload.nodes;
            model.skeleton_edges.clear();
            for (const SkeletonEdge& edge : payload.edges) {
              model.skeleton_edges.emplace_back(edge.from, edge.to);
            }
          } else if constexpr (std::is_same_v<Payload, KnownGridModel>) {
            model.known_grid = {
                payload.geometry.columns, payload.geometry.rows,
                payload.geometry.resolution_m, payload.geometry.origin,
                payload.observations, update.revision};
          } else if constexpr (std::is_same_v<Payload,
                                               InclusionGridModel>) {
            model.inclusion_grid = {
                payload.geometry.columns, payload.geometry.rows,
                payload.geometry.resolution_m, payload.geometry.origin,
                payload.included, update.revision};
          } else if constexpr (std::is_same_v<Payload, HighwayModel>) {
            model.highways.nodes = payload.nodes;
            model.highways.edges.clear();
            for (const SkeletonEdge& edge : payload.edges)
              model.highways.edges.emplace_back(edge.from, edge.to);
            model.highways.intersections.clear();
            for (const HighwayIntersection& intersection :
                 payload.intersections)
              model.highways.intersections.push_back(
                  {intersection.node, intersection.degree});
            model.highways.revision = update.revision;
          }
        },
        update.payload);
  }
  std::size_t revision = 0U;
  for (const Entry& entry : learners_)
    if (entry.enabled)
      revision = std::max(revision, entry.learner->snapshot().revision);
  model.revision = revision;
}

std::size_t SpatialLearningCoordinator::enabledCount() const noexcept {
  return static_cast<std::size_t>(
      std::count_if(learners_.begin(), learners_.end(),
                    [](const Entry& entry) { return entry.enabled; }));
}

}  // namespace semaforr::spatial
