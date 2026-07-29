#include <semaforr/spatial/spatial_learner_base.hpp>

#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace semaforr::spatial {
namespace {

std::string quote(std::string_view value)
{
  std::string result{"\""};
  for (const char character : value) {
    switch (character) {
      case '\\': result += "\\\\"; break;
      case '"': result += "\\\""; break;
      case '\n': result += "\\n"; break;
      case '\r': result += "\\r"; break;
      case '\t': result += "\\t"; break;
      default: result += character; break;
    }
  }
  result += '"';
  return result;
}

void point(std::ostream& output, const domain::Point2D& value)
{
  output << "{\"x_m\":" << value.x_m << ",\"y_m\":" << value.y_m << '}';
}

void segment(std::ostream& output, const domain::Segment2D& value)
{
  output << "{\"start\":";
  point(output, value.start);
  output << ",\"end\":";
  point(output, value.end);
  output << '}';
}

template<typename Range, typename Writer>
void array(std::ostream& output, const Range& values, Writer writer)
{
  output << '[';
  bool first = true;
  for (const auto& value : values) {
    if (!first) {
      output << ',';
    }
    first = false;
    writer(output, value);
  }
  output << ']';
}

void payload(std::ostream& output, const SpatialPayload& value)
{
  std::visit([&output](const auto& model) {
    using Model = std::decay_t<decltype(model)>;
    if constexpr (std::is_same_v<Model, std::monostate>) {
      output << "null";
    } else if constexpr (std::is_same_v<Model, TrailModel>) {
      output << "{\"trails\":";
      array(output, model.trails, [](std::ostream& stream, const auto& trail) {
        array(stream, trail, point);
      });
      output << '}';
    } else if constexpr (std::is_same_v<Model, ConveyorModel>) {
      output << "{\"flows\":";
      array(output, model.flows, [](std::ostream& stream, const auto& flow) {
        stream << "{\"axis\":";
        segment(stream, flow.axis);
        stream << ",\"traversals\":" << flow.traversals << '}';
      });
      output << '}';
    } else if constexpr (std::is_same_v<Model, RegionModel>) {
      output << "{\"regions\":";
      array(output, model.regions, [](std::ostream& stream, const auto& region) {
        stream << "{\"center\":";
        point(stream, region.center);
        stream << ",\"radius_m\":" << region.radius.meters() << '}';
      });
      output << '}';
    } else if constexpr (std::is_same_v<Model, DoorExitModel>) {
      output << "{\"openings\":";
      array(output, model.openings, segment);
      output << '}';
    } else if constexpr (std::is_same_v<Model, HallwayModel>) {
      output << "{\"centerlines\":";
      array(output, model.centerlines, segment);
      output << '}';
    } else if constexpr (std::is_same_v<Model, BarrierModel>) {
      output << "{\"barriers\":";
      array(output, model.barriers, segment);
      output << '}';
    } else if constexpr (std::is_same_v<Model, PassageSkeletonModel>) {
      output << "{\"nodes\":";
      array(output, model.nodes, point);
      output << ",\"edges\":";
      array(output, model.edges, [](std::ostream& stream, const auto& edge) {
        stream << "{\"from\":" << edge.from << ",\"to\":" << edge.to << '}';
      });
      output << '}';
    }
  }, value);
}

}  // namespace

std::string_view toString(SpatialRepresentation representation) noexcept
{
  switch (representation) {
    case SpatialRepresentation::Trails: return "trails";
    case SpatialRepresentation::Conveyors: return "conveyors";
    case SpatialRepresentation::Regions: return "regions";
    case SpatialRepresentation::DoorsAndExits: return "doors_and_exits";
    case SpatialRepresentation::Hallways: return "hallways";
    case SpatialRepresentation::Barriers: return "barriers";
    case SpatialRepresentation::PassagesAndSkeleton:
      return "passages_and_skeleton";
  }
  return "unknown";
}

std::string_view toString(UpdateMode mode) noexcept
{
  return mode == UpdateMode::Incremental ? "incremental" : "rebuild_on_demand";
}

std::string_view toString(ModelStatus status) noexcept
{
  switch (status) {
    case ModelStatus::Empty: return "empty";
    case ModelStatus::Incomplete: return "incomplete";
    case ModelStatus::Fresh: return "fresh";
    case ModelStatus::Stale: return "stale";
  }
  return "unknown";
}

std::string serialize(const SpatialModelUpdate& update)
{
  std::ostringstream output;
  output << std::setprecision(17)
         << "{\"representation\":" << quote(toString(update.representation))
         << ",\"learner\":" << quote(update.learner)
         << ",\"revision\":" << update.revision
         << ",\"observed_episodes\":" << update.observed_episodes
         << ",\"last_observation_sequence\":";
  if (update.last_observation_sequence) {
    output << *update.last_observation_sequence;
  } else {
    output << "null";
  }
  output << ",\"update_mode\":" << quote(toString(update.update_mode))
         << ",\"status\":" << quote(toString(update.status))
         << ",\"consumers\":";
  array(output, update.consumers, [](std::ostream& stream, const auto& consumer) {
    stream << quote(consumer);
  });
  output << ",\"diagnostic\":" << quote(update.diagnostic) << ",\"payload\":";
  payload(output, update.payload);
  output << '}';
  return output.str();
}

SpatialLearnerBase::SpatialLearnerBase(
  SpatialRepresentation representation,
  std::string name,
  UpdateMode mode,
  ObservationContract contract)
  : contract_(std::move(contract))
{
  if (name.empty()) {
    throw std::invalid_argument("spatial learner name must not be empty");
  }
  if (contract_.update_trigger.empty()) {
    throw std::invalid_argument(
      "spatial learner update trigger must not be empty");
  }
  if (contract_.consumers.empty()) {
    throw std::invalid_argument(
      "spatial learner must declare at least one consumer");
  }
  update_.representation = representation;
  update_.learner = std::move(name);
  update_.update_mode = mode;
  update_.consumers = contract_.consumers;
}

void SpatialLearnerBase::observe(const NavigationEpisode& episode)
{
  if (!episode.observation.pose.position.finite()) {
    throw std::invalid_argument("navigation episode pose must be finite");
  }
  episode.observation.laser.validate();
  if (episode.sequence == 0U) {
    throw std::invalid_argument("navigation episode sequence must be positive");
  }
  if (update_.last_observation_sequence &&
      episode.sequence <= *update_.last_observation_sequence) {
    throw std::invalid_argument(
      "navigation episodes must have strictly increasing sequence numbers");
  }

  if (update_.update_mode == UpdateMode::RebuildOnDemand) {
    episodes_.push_back(episode);
  }
  ++update_.observed_episodes;
  update_.last_observation_sequence = episode.sequence;
  if (update_.update_mode == UpdateMode::RebuildOnDemand) {
    update_.status =
      update_.revision == 0U ? ModelStatus::Incomplete : ModelStatus::Stale;
    update_.diagnostic =
      update_.revision == 0U
      ? "observations collected; rebuild required"
      : "new observations collected after the last rebuild";
  }
  onObserve(episode);
}

void SpatialLearnerBase::rebuild()
{
  onRebuild();
}

void SpatialLearnerBase::publish(
  SpatialPayload payload_value,
  ModelStatus status,
  std::string diagnostic)
{
  if (status == ModelStatus::Empty || status == ModelStatus::Stale) {
    throw std::invalid_argument(
      "published spatial model must be fresh or explicitly incomplete");
  }
  ++update_.revision;
  update_.payload = std::move(payload_value);
  update_.status = status;
  update_.diagnostic = std::move(diagnostic);
}

void SpatialLearnerBase::markIncomplete(std::string diagnostic)
{
  publish(std::monostate{}, ModelStatus::Incomplete, std::move(diagnostic));
}

}  // namespace semaforr::spatial
