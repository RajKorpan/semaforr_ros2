#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <semaforr/exploration/high_level_explorer.hpp>
#include <set>
#include <stdexcept>
#include <tuple>

namespace semaforr::exploration {
namespace {

domain::Point2D endpoint(const domain::RobotObservation& observation,
                         std::size_t beam, double range_m) {
  const double angle = observation.pose.heading.radians() +
                       observation.laser.angle_min.radians() +
                       static_cast<double>(beam) *
                           observation.laser.angle_increment.radians();
  return {observation.pose.position.x_m + range_m * std::cos(angle),
          observation.pose.position.y_m + range_m * std::sin(angle)};
}

double clampedRange(const domain::LaserObservation& laser, std::size_t beam) {
  if (beam >= laser.ranges_m.size()) return -1.0;
  const double value = laser.ranges_m[beam];
  if (!std::isfinite(value) || value < laser.minimum_range.meters())
    return -1.0;
  return std::min(value, laser.maximum_range.meters());
}

domain::Action turnToward(double heading,
                          const domain::ActionSpace& action_space) {
  const auto& turns = action_space.rotation_angles_rad();
  if (turns.empty()) return domain::Action::pause();
  const auto found =
      std::lower_bound(turns.begin(), turns.end(), std::abs(heading));
  const std::size_t index =
      found == turns.end()
          ? turns.size()
          : static_cast<std::size_t>(found - turns.begin()) + 1U;
  return heading < 0.0
             ? domain::Action(domain::ActionType::TurnRight, index)
             : domain::Action(domain::ActionType::TurnLeft, index);
}

double projection(domain::Point2D origin, domain::Angle direction,
                  domain::Point2D point) {
  return (point.x_m - origin.x_m) * std::cos(direction.radians()) +
         (point.y_m - origin.y_m) * std::sin(direction.radians());
}

double pointSegmentDistance(domain::Point2D point,
                            const domain::Segment2D& segment) {
  const double dx = segment.end.x_m - segment.start.x_m;
  const double dy = segment.end.y_m - segment.start.y_m;
  const double squared = dx * dx + dy * dy;
  if (squared <= domain::geometry_tolerance_m)
    return domain::distance(point, segment.start).meters();
  const double fraction = std::clamp(
      ((point.x_m - segment.start.x_m) * dx +
       (point.y_m - segment.start.y_m) * dy) /
          squared,
      0.0, 1.0);
  return std::hypot(point.x_m - (segment.start.x_m + fraction * dx),
                    point.y_m - (segment.start.y_m + fraction * dy));
}

bool similarSegments(const ExplorationCandidate& left,
                     const ExplorationCandidate& right, double tolerance) {
  const double angle = std::abs(domain::Angle::normalize(
      left.direction.radians() - right.direction.radians()));
  if (angle > std::numbers::pi / 9.0) return false;
  const domain::Segment2D left_segment{left.start, left.endpoint};
  const domain::Segment2D right_segment{right.start, right.endpoint};
  const double lateral = std::min(
      {pointSegmentDistance(right.start, left_segment),
       pointSegmentDistance(right.endpoint, left_segment),
       pointSegmentDistance(left.start, right_segment),
       pointSegmentDistance(left.endpoint, right_segment)});
  if (lateral > tolerance) return false;
  const double first = projection(left.start, left.direction, right.start);
  const double second = projection(left.start, left.direction, right.endpoint);
  const double low = std::min(first, second);
  const double high = std::max(first, second);
  const double overlap = std::min(left.current_extension.meters(), high) -
                         std::max(0.0, low);
  return overlap >= -tolerance;
}

std::optional<std::size_t> closestBeam(
    const domain::RobotObservation& observation, domain::Point2D point) {
  if (observation.laser.ranges_m.empty()) return std::nullopt;
  const double relative = domain::Angle::normalize(
      std::atan2(point.y_m - observation.pose.position.y_m,
                 point.x_m - observation.pose.position.x_m) -
      observation.pose.heading.radians());
  std::size_t best = 0U;
  double error = std::numeric_limits<double>::infinity();
  for (std::size_t beam = 0U; beam < observation.laser.ranges_m.size(); ++beam) {
    const double beam_angle = observation.laser.angle_min.radians() +
                              static_cast<double>(beam) *
                                  observation.laser.angle_increment.radians();
    const double candidate =
        std::abs(domain::Angle::normalize(relative - beam_angle));
    if (candidate < error) {
      error = candidate;
      best = beam;
    }
  }
  if (error > std::abs(observation.laser.angle_increment.radians()) * 0.75 +
                  domain::angle_tolerance_rad)
    return std::nullopt;
  return best;
}

bool pointClear(const domain::RobotObservation& observation,
                domain::Point2D point) {
  const double required =
      domain::distance(observation.pose.position, point).meters();
  if (required <= domain::geometry_tolerance_m) return true;
  const auto beam = closestBeam(observation, point);
  if (!beam) return false;
  const double available = clampedRange(observation.laser, *beam);
  return available >= 0.0 &&
         required <= available + domain::geometry_tolerance_m;
}

ExplorationCandidate focusCandidate(
    const domain::RobotObservation& observation,
    const HighLevelExplorationConfiguration& configuration,
    std::size_t first, std::size_t last, PassageCueType cue_type) {
  ExplorationCandidate candidate;
  candidate.start = observation.pose.position;
  candidate.first_beam = first;
  candidate.last_beam = last;
  candidate.cue_type = cue_type;
  std::vector<double> valid;
  valid.reserve(last - first + 1U);
  for (std::size_t beam = first; beam <= last; ++beam) {
    const double range = clampedRange(observation.laser, beam);
    if (range >= configuration.minimum_clearance.meters())
      valid.push_back(range);
  }
  if (valid.empty()) return candidate;
  std::sort(valid.begin(), valid.end());
  const double length = valid[valid.size() / 2U];
  const std::size_t middle = first + (last - first) / 2U;
  const double relative = observation.laser.angle_min.radians() +
                          static_cast<double>(middle) *
                              observation.laser.angle_increment.radians();
  const double global = domain::Angle::normalize(
      observation.pose.heading.radians() + relative);
  const double span = std::abs(static_cast<double>(last - first) *
                               observation.laser.angle_increment.radians());
  const double width = 2.0 * length * std::sin(span / 2.0);
  const double ratio = length / std::max(width, 0.01);
  const double valid_fraction = static_cast<double>(valid.size()) /
                                static_cast<double>(last - first + 1U);
  candidate.heading = domain::Angle(relative);
  candidate.direction = domain::Angle(global);
  candidate.clearance = domain::Distance(length);
  candidate.length = domain::Distance(length);
  candidate.width = domain::Distance(width);
  candidate.current_width = candidate.width;
  const double usable_length =
      std::max(0.0, length - configuration.cue_clearance_margin.meters());
  candidate.current_extension = domain::Distance(usable_length);
  candidate.endpoint =
      {candidate.start.x_m + usable_length * std::cos(global),
       candidate.start.y_m + usable_length * std::sin(global)};
  const bool large_room =
      length >= configuration.large_room_length.meters() &&
      width >= configuration.large_room_width.meters();
  candidate.kind = large_room ? PassageKind::LargeRoom : PassageKind::Corridor;
  candidate.confidence =
      std::clamp(valid_fraction *
                     (large_room ? 1.0
                                 : ratio /
                                       configuration.minimum_length_to_width_ratio),
                 0.0, 1.0);
  candidate.priority = candidate.confidence * length;
  return candidate;
}

}  // namespace

const char* toString(PassageCellState state) noexcept {
  switch (state) {
    case PassageCellState::Free: return "free";
    case PassageCellState::Obstructed: return "obstructed";
    case PassageCellState::Passage: return "passage";
  }
  return "free";
}

const char* toString(PassageCompletionState state) noexcept {
  switch (state) {
    case PassageCompletionState::Unassigned: return "unassigned";
    case PassageCompletionState::InProgress: return "in_progress";
    case PassageCompletionState::Suspended: return "suspended";
    case PassageCompletionState::Completed: return "completed";
    case PassageCompletionState::Abandoned: return "abandoned";
  }
  return "unassigned";
}

void HighLevelExplorationConfiguration::validate() const {
  if (!(minimum_clearance.meters() > 0.0) ||
      !(heading_tolerance.radians() > 0.0) ||
      !(candidate_completion_distance.meters() > 0.0) ||
      !(cue_similarity_radius.meters() > 0.0) ||
      !(passage_grid_resolution.meters() > 0.0) ||
      minimum_bundle_beams == 0U || compatibility_focus_bundle_beams != 41U ||
      !(minimum_length_to_width_ratio > 0.0) ||
      !(minimum_passage_length.meters() > 0.0) ||
      !(large_room_width.meters() > 0.0) ||
      !(large_room_length.meters() > 0.0) ||
      !(maximum_width_change_ratio > 0.0) ||
      !(hard_turn_threshold.radians() > 0.0) ||
      !(end_of_passage_clearance.meters() > 0.0) ||
      !(minimum_extension.meters() > 0.0) || !(time_budget.count() > 0.0) ||
      decision_budget == 0U)
    throw std::invalid_argument(
        "HLE geometry, thresholds, budgets, and fixed focus bundle must be "
        "positive; compatibility focus bundles must contain 41 rays");
  if (passage_grid_geometry.valid() &&
      std::abs(passage_grid_geometry.resolution_m -
               passage_grid_resolution.meters()) >
          domain::geometry_tolerance_m)
    throw std::invalid_argument(
        "HLE passage grid geometry and configured resolution disagree");
}

std::string_view toString(HleState state) noexcept {
  switch (state) {
    case HleState::Initialize: return "initialize";
    case HleState::DiscoverCandidate: return "discover_candidate";
    case HleState::ReturnToCandidateStart: return "return_to_candidate_start";
    case HleState::PursueCandidate: return "pursue_candidate";
    case HleState::RecordPassage: return "record_passage";
    case HleState::SelectNextCandidate: return "select_next_candidate";
    case HleState::FinalizeModel: return "finalize_model";
    case HleState::Complete: return "complete";
  }
  return "complete";
}

std::string_view toString(CandidateLifecycleEvent event) noexcept {
  switch (event) {
    case CandidateLifecycleEvent::None: return "none";
    case CandidateLifecycleEvent::Discovered: return "candidate_discovered";
    case CandidateLifecycleEvent::Merged: return "candidate_merged";
    case CandidateLifecycleEvent::Rejected: return "candidate_rejected";
    case CandidateLifecycleEvent::Selected: return "candidate_selected";
    case CandidateLifecycleEvent::PursuitStarted:
      return "candidate_pursuit_started";
    case CandidateLifecycleEvent::Suspended: return "candidate_suspended";
    case CandidateLifecycleEvent::Completed: return "candidate_completed";
    case CandidateLifecycleEvent::Abandoned: return "candidate_abandoned";
    case CandidateLifecycleEvent::Exhausted: return "candidate_exhausted";
  }
  return "none";
}

std::string_view toString(ExplorationCompletionReason reason) noexcept {
  switch (reason) {
    case ExplorationCompletionReason::None: return "none";
    case ExplorationCompletionReason::CandidateQueueExhausted:
      return "candidate_queue_exhausted";
    case ExplorationCompletionReason::TimeBudgetExceeded:
      return "time_budget_exceeded";
    case ExplorationCompletionReason::DecisionBudgetExceeded:
      return "decision_budget_exceeded";
    case ExplorationCompletionReason::ExplicitlyFinished:
      return "explicitly_finished";
  }
  return "none";
}

std::string_view toString(PursuitTerminationReason reason) noexcept {
  switch (reason) {
    case PursuitTerminationReason::None: return "none";
    case PursuitTerminationReason::EndpointReached: return "endpoint_reached";
    case PursuitTerminationReason::EndOfPassageClearance:
      return "end_of_passage_clearance";
    case PursuitTerminationReason::WidthChanged: return "width_changed";
    case PursuitTerminationReason::HardTurn: return "hard_turn";
    case PursuitTerminationReason::LargeRoom: return "large_room";
    case PursuitTerminationReason::CandidateUnreachable:
      return "candidate_unreachable";
    case PursuitTerminationReason::TimeBudgetExceeded:
      return "time_budget_exceeded";
    case PursuitTerminationReason::DecisionBudgetExceeded:
      return "decision_budget_exceeded";
    case PursuitTerminationReason::ExplicitlyFinished:
      return "explicitly_finished";
  }
  return "none";
}

std::string_view toString(CandidateDiagnosticKind kind) noexcept {
  switch (kind) {
    case CandidateDiagnosticKind::Created: return "created";
    case CandidateDiagnosticKind::Merged: return "merged";
    case CandidateDiagnosticKind::Rejected: return "rejected";
    case CandidateDiagnosticKind::Selected: return "selected";
    case CandidateDiagnosticKind::Suspended: return "suspended";
    case CandidateDiagnosticKind::Completed: return "completed";
    case CandidateDiagnosticKind::Abandoned: return "abandoned";
  }
  return "created";
}

HighLevelExplorer::HighLevelExplorer(
    HighLevelExplorationConfiguration configuration)
    : configuration_(std::move(configuration)) {
  configuration_.validate();
}

std::int64_t HighLevelExplorer::cellKey(int row, int column) noexcept {
  const auto packed =
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(row)) << 32U) |
      static_cast<std::uint32_t>(column);
  return static_cast<std::int64_t>(packed);
}

std::int64_t HighLevelExplorer::cueKey(const domain::Point2D& point) const
    noexcept {
  const double size = configuration_.cue_similarity_radius.meters();
  return cellKey(static_cast<int>(std::floor(point.y_m / size)),
                 static_cast<int>(std::floor(point.x_m / size)));
}

std::vector<ExplorationCandidate> HighLevelExplorer::discoverCandidates(
    const domain::RobotObservation& observation,
    const HighLevelExplorationConfiguration& configuration) {
  std::vector<ExplorationCandidate> result;
  const auto& ranges = observation.laser.ranges_m;
  if (configuration.behavior_policy == HleBehaviorPolicy::Compatibility) {
    const auto bundle = configuration.compatibility_focus_bundle_beams;
    if (ranges.size() < bundle * 2U) return result;
    for (const auto& [first, last, type] :
         {std::tuple<std::size_t, std::size_t, PassageCueType>{
              0U, bundle - 1U, PassageCueType::RightOpen},
          {ranges.size() - bundle, ranges.size() - 1U,
           PassageCueType::LeftOpen}}) {
      auto candidate =
          focusCandidate(observation, configuration, first, last, type);
      const double ratio = candidate.length.meters() /
                           std::max(candidate.width.meters(), 0.01);
      const bool passage =
          candidate.length.meters() >=
              configuration.minimum_passage_length.meters() &&
          ratio >= configuration.minimum_length_to_width_ratio;
      if (passage || candidate.kind == PassageKind::LargeRoom)
        result.push_back(std::move(candidate));
    }
    return result;
  }

  std::size_t begin = 0U;
  while (begin < ranges.size()) {
    while (begin < ranges.size() &&
           clampedRange(observation.laser, begin) <
               configuration.minimum_clearance.meters())
      ++begin;
    if (begin == ranges.size()) break;
    std::size_t end = begin;
    double clearance = clampedRange(observation.laser, begin);
    while (end + 1U < ranges.size() &&
           clampedRange(observation.laser, end + 1U) >=
               configuration.minimum_clearance.meters()) {
      ++end;
      clearance =
          std::min(clearance, clampedRange(observation.laser, end));
    }
    const std::size_t count = end - begin + 1U;
    if (count >= configuration.minimum_bundle_beams) {
      const std::size_t middle = begin + (end - begin) / 2U;
      const double relative = observation.laser.angle_min.radians() +
                              static_cast<double>(middle) *
                                  observation.laser.angle_increment.radians();
      const double global = domain::Angle::normalize(
          observation.pose.heading.radians() + relative);
      ExplorationCandidate candidate;
      candidate.start = observation.pose.position;
      candidate.endpoint =
          {candidate.start.x_m + clearance * std::cos(global),
           candidate.start.y_m + clearance * std::sin(global)};
      candidate.direction = domain::Angle(global);
      candidate.heading = domain::Angle(relative);
      candidate.clearance = domain::Distance(clearance);
      candidate.length = candidate.clearance;
      candidate.current_extension = candidate.length;
      const double width =
          std::max(0.0, 2.0 * clearance *
                            std::sin(std::abs(static_cast<double>(end - begin) *
                                              observation.laser.angle_increment
                                                  .radians()) /
                                     2.0));
      candidate.width = domain::Distance(width);
      candidate.current_width = candidate.width;
      candidate.kind = count <= 2U ? PassageKind::Doorway
                                    : PassageKind::Corridor;
      candidate.priority = static_cast<double>(count) /
                           static_cast<double>(
                               std::max<std::size_t>(1U, ranges.size()));
      candidate.confidence = candidate.priority;
      candidate.first_beam = begin;
      candidate.last_beam = end;
      result.push_back(std::move(candidate));
    }
    begin = end + 1U;
  }
  if (result.size() >= 3U)
    for (auto& candidate : result)
      candidate.kind = PassageKind::IntersectionBranch;
  return result;
}

CueValidation HighLevelExplorer::evaluateCue(
    const ExplorationCandidate& candidate,
    const domain::RobotObservation& observation) const {
  CueValidation result;
  const domain::Point2D midpoint{
      (candidate.start.x_m + candidate.endpoint.x_m) / 2.0,
      (candidate.start.y_m + candidate.endpoint.y_m) / 2.0};
  result.start_clear = pointClear(observation, candidate.start);
  result.midpoint_clear = pointClear(observation, midpoint);
  result.endpoint_clear = pointClear(observation, candidate.endpoint);
  if (!result.start_clear) result.reason = "cue_start_not_clear";
  else if (!result.midpoint_clear) result.reason = "cue_midpoint_not_clear";
  else if (!result.endpoint_clear) result.reason = "cue_endpoint_not_clear";
  if (!result.reason.empty()) return result;
  std::set<std::uint64_t> passages;
  const auto samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(
              candidate.length.meters() /
              configuration_.passage_grid_resolution.meters())));
  const domain::Point2D reference =
      configuration_.passage_grid_geometry.valid()
          ? configuration_.passage_grid_geometry.origin
          : passage_grid_reference_.value_or(observation.pose.position);
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double fraction =
        static_cast<double>(sample) / static_cast<double>(samples);
    const domain::Point2D point{
        candidate.start.x_m +
            fraction * (candidate.endpoint.x_m - candidate.start.x_m),
        candidate.start.y_m +
            fraction * (candidate.endpoint.y_m - candidate.start.y_m)};
    int row = 0;
    int column = 0;
    if (configuration_.passage_grid_geometry.valid()) {
      const auto cell = configuration_.passage_grid_geometry.cell(point);
      if (!cell) {
        result.reason = "cue_outside_fixed_grid";
        return result;
      }
      row = static_cast<int>(cell->second);
      column = static_cast<int>(cell->first);
    } else {
      row = static_cast<int>(std::floor(
          (point.y_m - reference.y_m) /
          configuration_.passage_grid_resolution.meters()));
      column = static_cast<int>(std::floor(
          (point.x_m - reference.x_m) /
          configuration_.passage_grid_resolution.meters()));
    }
    const auto found = passage_cells_.find(cellKey(row, column));
    if (found != passage_cells_.end() && found->second.passage_id)
      passages.insert(*found->second.passage_id);
  }
  result.passage_identities = passages.size();
  if (passages.size() > 1U) {
    result.reason = "cue_intersects_multiple_passages";
    return result;
  }
  result.geometrically_reachable = true;
  for (std::size_t sample = 1U; sample <= 5U; ++sample) {
    const double fraction = static_cast<double>(sample) / 5.0;
    if (!pointClear(observation,
                    {candidate.start.x_m +
                         fraction *
                             (candidate.endpoint.x_m - candidate.start.x_m),
                     candidate.start.y_m +
                         fraction *
                             (candidate.endpoint.y_m - candidate.start.y_m)})) {
      result.geometrically_reachable = false;
      result.reason = "cue_not_geometrically_reachable";
      return result;
    }
  }
  result.accepted = true;
  result.reason = "cue_clear_and_reachable";
  return result;
}

bool HighLevelExplorer::cuesSimilar(const ExplorationCandidate& left,
                                    const ExplorationCandidate& right,
                                    double tolerance_m) {
  return similarSegments(left, right, tolerance_m);
}

std::optional<ExplorationCandidateId> HighLevelExplorer::mergeTarget(
    const ExplorationCandidate& candidate) const {
  for (const auto& [id, stored] : candidate_registry_)
    if (stored.state != ExplorationCandidateState::Abandoned &&
        similarSegments(stored, candidate,
                        configuration_.cue_similarity_radius.meters()))
      return id;
  return std::nullopt;
}

void HighLevelExplorer::mergeCandidate(
    ExplorationCandidateId id, const ExplorationCandidate& observation) {
  auto& stored = candidate_registry_.at(id);
  if (observation.current_extension > stored.current_extension) {
    stored.endpoint = observation.endpoint;
    stored.current_extension = observation.current_extension;
    stored.length = observation.length;
  }
  stored.current_width = observation.width;
  stored.confidence = std::max(stored.confidence, observation.confidence);
  stored.priority = std::max(stored.priority, observation.priority);
  ++stored.revision;
  if (stored.state == ExplorationCandidateState::Queued)
    candidates_.push(stored);
}

void HighLevelExplorer::recordDiagnostic(ExplorationCandidateId id,
                                         CandidateDiagnosticKind kind,
                                         std::string reason) {
  ExplorationCandidate snapshot;
  const auto found = candidate_registry_.find(id);
  if (found != candidate_registry_.end()) snapshot = found->second;
  else if (active_ && active_->id == id) snapshot = *active_;
  candidate_diagnostics_.push_back(
      {++diagnostic_sequence_, id, kind, std::move(reason),
       std::move(snapshot)});
}

void HighLevelExplorer::recordTrace(
    const domain::RobotObservation& observation,
    const ExplorationResult& result) {
  trace_.push_back({static_cast<std::uint64_t>(decisions_), observation,
                    result});
}

std::vector<ExplorationResult> HighLevelExplorer::replay(
    const std::vector<HleTraceEntry>& trace) {
  std::vector<ExplorationResult> results;
  results.reserve(trace.size());
  for (const auto& entry : trace) results.push_back(entry.result);
  return results;
}

std::vector<ExplorationCandidate> HighLevelExplorer::discover(
    const domain::RobotObservation& observation) {
  std::vector<ExplorationCandidate> discovered;
  for (auto candidate : discoverCandidates(observation, configuration_)) {
    candidate.discovery_observation_id = observation_sequence_;
    CueValidation validation;
    if (configuration_.behavior_policy == HleBehaviorPolicy::Compatibility)
      validation = evaluateCue(candidate, observation);
    if (configuration_.behavior_policy == HleBehaviorPolicy::Compatibility &&
        !validation.accepted) {
      candidate.id = next_candidate_id_++;
      candidate.state = ExplorationCandidateState::Abandoned;
      candidate_diagnostics_.push_back(
          {++diagnostic_sequence_, candidate.id,
           CandidateDiagnosticKind::Rejected, std::move(validation.reason),
           candidate});
      continue;
    }
    if (configuration_.behavior_policy == HleBehaviorPolicy::Modernized &&
        !cue_cells_.insert(cueKey(candidate.endpoint)).second)
      continue;
    if (configuration_.behavior_policy == HleBehaviorPolicy::Compatibility)
      if (const auto target = mergeTarget(candidate)) {
        mergeCandidate(*target, candidate);
        recordDiagnostic(*target, CandidateDiagnosticKind::Merged,
                         "segment_distance_and_interval_overlap");
        continue;
      }
    candidate.id = next_candidate_id_++;
    candidate.state = ExplorationCandidateState::Queued;
    candidate_registry_[candidate.id] = candidate;
    candidates_.push(candidate);
    discovered.push_back(candidate);
    recordDiagnostic(candidate.id, CandidateDiagnosticKind::Created,
                     validation.reason.empty() ? "clear_bundle"
                                               : validation.reason);
  }
  return discovered;
}

void HighLevelExplorer::updateCandidateExtension(
    const domain::RobotObservation& observation) {
  if (!active_ ||
      configuration_.behavior_policy != HleBehaviorPolicy::Compatibility)
    return;
  const auto visible = discoverCandidates(observation, configuration_);
  const ExplorationCandidate* best = nullptr;
  double best_angle = std::numeric_limits<double>::infinity();
  for (const auto& candidate : visible) {
    if (candidate.cue_type != active_->cue_type) continue;
    const double angle = std::abs(domain::Angle::normalize(
        candidate.direction.radians() - active_->direction.radians()));
    if (angle < best_angle) {
      best = &candidate;
      best_angle = angle;
    }
  }
  if (!best) return;
  const double extension =
      projection(active_->start, active_->direction, best->endpoint);
  active_->current_width = best->width;
  if (extension >= active_->current_extension.meters() +
                       configuration_.minimum_extension.meters()) {
    active_->endpoint =
        {active_->start.x_m +
             extension * std::cos(active_->direction.radians()),
         active_->start.y_m +
             extension * std::sin(active_->direction.radians())};
    active_->current_extension = domain::Distance(extension);
    active_->length = active_->current_extension;
    ++active_->revision;
    candidate_registry_[active_->id] = *active_;
  }
}

PursuitTerminationReason HighLevelExplorer::pursuitTermination(
    const domain::RobotObservation& observation) const {
  if (!active_) return PursuitTerminationReason::CandidateUnreachable;
  if (configuration_.behavior_policy == HleBehaviorPolicy::Modernized)
    return domain::distance(observation.pose.position, active_->start).meters() >=
                   configuration_.candidate_completion_distance.meters()
               ? PursuitTerminationReason::EndpointReached
               : PursuitTerminationReason::None;
  const auto visible = discoverCandidates(observation, configuration_);
  const ExplorationCandidate* best = nullptr;
  double best_angle = std::numeric_limits<double>::infinity();
  for (const auto& candidate : visible) {
    if (candidate.cue_type != active_->cue_type) continue;
    const double angle = std::abs(domain::Angle::normalize(
        candidate.direction.radians() - active_->direction.radians()));
    if (angle < best_angle) {
      best = &candidate;
      best_angle = angle;
    }
  }
  if (!best) {
    const domain::Point2D ahead{
        observation.pose.position.x_m +
            configuration_.end_of_passage_clearance.meters() *
                std::cos(active_->direction.radians()),
        observation.pose.position.y_m +
            configuration_.end_of_passage_clearance.meters() *
                std::sin(active_->direction.radians())};
    return pointClear(observation, ahead)
               ? PursuitTerminationReason::CandidateUnreachable
               : PursuitTerminationReason::EndOfPassageClearance;
  }
  if (best_angle >= configuration_.hard_turn_threshold.radians())
    return PursuitTerminationReason::HardTurn;
  if (best->kind == PassageKind::LargeRoom &&
      active_->kind != PassageKind::LargeRoom)
    return PursuitTerminationReason::LargeRoom;
  const double width_change =
      std::abs(best->width.meters() - active_->width.meters()) /
      std::max(active_->width.meters(), 0.01);
  if (width_change >= configuration_.maximum_width_change_ratio)
    return PursuitTerminationReason::WidthChanged;
  const double extension =
      projection(active_->start, active_->direction, best->endpoint);
  const double progress = projection(active_->start, active_->direction,
                                     observation.pose.position);
  if (extension < active_->current_extension.meters() +
                      configuration_.minimum_extension.meters() &&
      progress >= active_->current_extension.meters() -
                      configuration_.candidate_completion_distance.meters())
    return PursuitTerminationReason::EndpointReached;
  return PursuitTerminationReason::None;
}

void HighLevelExplorer::updatePassageGrid(
    const domain::RobotObservation& observation,
    ExplorationCandidateId candidate_id, std::uint64_t passage_number,
    domain::Point2D passage_start,
    PassageCompletionState completion_state) {
  bool changed = false;
  const double resolution = configuration_.passage_grid_resolution.meters();
  if (!passage_grid_reference_)
    passage_grid_reference_ = configuration_.passage_grid_geometry.valid()
                                  ? configuration_.passage_grid_geometry.origin
                                  : observation.pose.position;
  const auto coordinates = [&](domain::Point2D point)
      -> std::optional<std::pair<int, int>> {
    if (configuration_.passage_grid_geometry.valid()) {
      const auto cell = configuration_.passage_grid_geometry.cell(point);
      if (!cell) return std::nullopt;
      return std::pair{static_cast<int>(cell->second),
                       static_cast<int>(cell->first)};
    }
    return std::pair{
        static_cast<int>(std::floor(
            (point.y_m - passage_grid_reference_->y_m) / resolution)),
        static_cast<int>(std::floor(
            (point.x_m - passage_grid_reference_->x_m) / resolution))};
  };
  const auto apply = [&](domain::Point2D point, PassageCellState state,
                         std::optional<std::uint64_t> numbered_passage,
                         std::optional<ExplorationCandidateId> association,
                         PassageCompletionState completion) {
    const auto coordinate = coordinates(point);
    if (!coordinate) return;
    auto& cell = passage_cells_[cellKey(coordinate->first, coordinate->second)];
    cell.row = coordinate->first;
    cell.column = coordinate->second;
    if (state == PassageCellState::Obstructed) {
      cell.state = state;
      cell.passage_id.reset();
      cell.candidate_id.reset();
      cell.completion_state = PassageCompletionState::Unassigned;
    } else if (state == PassageCellState::Passage) {
      // The robot's execution-confirmed centerline is stronger evidence for
      // this cell than a range endpoint quantized into the same coarse cell.
      cell.state = state;
      cell.passage_id = numbered_passage;
      cell.candidate_id = association;
      cell.completion_state = completion;
    } else if (state == PassageCellState::Free &&
               cell.state != PassageCellState::Obstructed &&
               cell.state != PassageCellState::Passage) {
      cell.state = state;
    }
    ++cell.evidence_count;
    changed = true;
  };
  for (std::size_t beam = 0U; beam < observation.laser.ranges_m.size(); ++beam) {
    const double range = observation.laser.ranges_m[beam];
    if (!std::isfinite(range) ||
        range < observation.laser.minimum_range.meters())
      continue;
    const double extent =
        std::min(range, observation.laser.maximum_range.meters());
    const bool obstacle_hit =
        range + domain::geometry_tolerance_m <
        observation.laser.maximum_range.meters();
    const std::size_t samples = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(extent / resolution)));
    for (std::size_t sample = 0U; sample <= samples; ++sample)
      apply(endpoint(observation, beam,
                     extent * static_cast<double>(sample) /
                         static_cast<double>(samples)),
            obstacle_hit && sample == samples
                ? PassageCellState::Obstructed
                : PassageCellState::Free,
            std::nullopt, std::nullopt,
            PassageCompletionState::Unassigned);
  }
  const double path_length =
      domain::distance(passage_start, observation.pose.position).meters();
  const auto path_samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(path_length / resolution)));
  for (std::size_t sample = 0U; sample <= path_samples; ++sample) {
    const double fraction =
        static_cast<double>(sample) / static_cast<double>(path_samples);
    apply({passage_start.x_m +
               fraction * (observation.pose.position.x_m - passage_start.x_m),
           passage_start.y_m +
               fraction * (observation.pose.position.y_m - passage_start.y_m)},
          PassageCellState::Passage, passage_number, candidate_id,
          completion_state);
  }
  if (changed) ++passage_revision_;
}

domain::Action HighLevelExplorer::pursue(const ExplorationInput& input) const {
  if (!active_) return domain::Action::pause();
  const double desired = std::atan2(
      active_->endpoint.y_m - input.observation.pose.position.y_m,
      active_->endpoint.x_m - input.observation.pose.position.x_m);
  const double relative = domain::Angle::normalize(
      desired - input.observation.pose.heading.radians());
  if (std::abs(relative) > configuration_.heading_tolerance.radians())
    return turnToward(relative, input.action_space);
  const std::size_t magnitude =
      active_->clearance.meters() > 1.5
          ? input.action_space.move_distances_m().size()
          : 1U;
  return magnitude == 0U
             ? domain::Action::pause()
             : domain::Action(domain::ActionType::Forward, magnitude);
}

ExplorationResult HighLevelExplorer::update(const ExplorationInput& input) {
  ++decisions_;
  ++observation_sequence_;
  const auto diagnostic_begin = candidate_diagnostics_.size();
  exploration_path_.push_back(input.observation.pose.position);
  ExplorationResult result;
  const auto finish_result = [&](ExplorationResult value) {
    value.passage_grid_revision = passage_revision_;
    value.diagnostics.insert(value.diagnostics.end(),
                             candidate_diagnostics_.begin() +
                                 static_cast<std::ptrdiff_t>(diagnostic_begin),
                             candidate_diagnostics_.end());
    recordTrace(input.observation, value);
    return value;
  };

  if (state_ != HleState::Complete &&
      input.elapsed >= configuration_.time_budget) {
    completion_reason_ = ExplorationCompletionReason::TimeBudgetExceeded;
    active_termination_reason_ = PursuitTerminationReason::TimeBudgetExceeded;
    if (active_) {
      active_->state = ExplorationCandidateState::Abandoned;
      candidate_registry_[active_->id] = *active_;
      recordDiagnostic(active_->id, CandidateDiagnosticKind::Abandoned,
                       "time_budget_exceeded");
      if (active_->passage_id)
        updatePassageGrid(input.observation, active_->id, *active_->passage_id,
                          active_->start,
                          PassageCompletionState::Abandoned);
      result.event = CandidateLifecycleEvent::Abandoned;
      result.candidate_id = active_->id;
    }
    state_ = HleState::FinalizeModel;
  } else if (state_ != HleState::Complete &&
             decisions_ > configuration_.decision_budget) {
    completion_reason_ = ExplorationCompletionReason::DecisionBudgetExceeded;
    active_termination_reason_ =
        PursuitTerminationReason::DecisionBudgetExceeded;
    if (active_) {
      active_->state = ExplorationCandidateState::Abandoned;
      candidate_registry_[active_->id] = *active_;
      recordDiagnostic(active_->id, CandidateDiagnosticKind::Abandoned,
                       "decision_budget_exceeded");
      if (active_->passage_id)
        updatePassageGrid(input.observation, active_->id, *active_->passage_id,
                          active_->start,
                          PassageCompletionState::Abandoned);
      result.event = CandidateLifecycleEvent::Abandoned;
      result.candidate_id = active_->id;
    }
    state_ = HleState::FinalizeModel;
  }

  result.state = state_;
  result.pursuit_termination_reason = active_termination_reason_;
  if (state_ == HleState::Initialize) {
    state_ = HleState::DiscoverCandidate;
    result.state = state_;
    result.action = turnToward(1.0, input.action_space);
    result.rationale = "initialize exploration and survey";
    return finish_result(std::move(result));
  }
  if (state_ == HleState::DiscoverCandidate) {
    const auto before = candidate_diagnostics_.size();
    result.discovered = discover(input.observation);
    const auto begin = candidate_diagnostics_.begin() +
                       static_cast<std::ptrdiff_t>(before);
    if (!result.discovered.empty())
      result.event = CandidateLifecycleEvent::Discovered;
    else if (std::any_of(begin, candidate_diagnostics_.end(), [](const auto& d) {
               return d.kind == CandidateDiagnosticKind::Merged;
             }))
      result.event = CandidateLifecycleEvent::Merged;
    else if (begin != candidate_diagnostics_.end())
      result.event = CandidateLifecycleEvent::Rejected;
    state_ = HleState::SelectNextCandidate;
  }
  if (state_ == HleState::SelectNextCandidate) {
    while (!candidates_.empty()) {
      const auto queued = candidates_.top();
      const auto found = candidate_registry_.find(queued.id);
      if (found != candidate_registry_.end() &&
          found->second.state == ExplorationCandidateState::Queued &&
          found->second.revision == queued.revision)
        break;
      candidates_.pop();
    }
    if (candidates_.empty()) {
      if (completion_reason_ == ExplorationCompletionReason::None)
        completion_reason_ =
            ExplorationCompletionReason::CandidateQueueExhausted;
      state_ = HleState::FinalizeModel;
    } else {
      const auto queued = candidates_.top();
      candidates_.pop();
      auto& stored = candidate_registry_.at(queued.id);
      stored.state = ExplorationCandidateState::Selected;
      if (!stored.passage_id) stored.passage_id = next_passage_id_++;
      ++stored.revision;
      active_ = stored;
      pursuit_started_ = false;
      active_termination_reason_ = PursuitTerminationReason::None;
      state_ = HleState::ReturnToCandidateStart;
      recordDiagnostic(active_->id, CandidateDiagnosticKind::Selected,
                       "highest_priority_valid_candidate");
      result.event = CandidateLifecycleEvent::Selected;
      result.candidate_id = active_->id;
      result.subgoal = ExplorationSubgoal{active_->start, active_->id};
      result.state = state_;
      result.rationale = "return to stable candidate start";
      return finish_result(std::move(result));
    }
  }
  if (state_ == HleState::ReturnToCandidateStart) {
    result.candidate_id = active_->id;
    if (domain::distance(input.observation.pose.position, active_->start)
            .meters() >
        configuration_.candidate_completion_distance.meters()) {
      result.subgoal = ExplorationSubgoal{active_->start, active_->id};
      result.rationale = "return to candidate start";
      return finish_result(std::move(result));
    }
    state_ = HleState::PursueCandidate;
  }
  if (state_ == HleState::PursueCandidate) {
    result.candidate_id = active_->id;
    if (!pursuit_started_) {
      pursuit_started_ = true;
      active_->state = ExplorationCandidateState::Pursuing;
      candidate_registry_[active_->id] = *active_;
      result.event = CandidateLifecycleEvent::PursuitStarted;
    } else {
      active_termination_reason_ = pursuitTermination(input.observation);
      if (active_termination_reason_ == PursuitTerminationReason::None) {
        updateCandidateExtension(input.observation);
      } else {
        if (active_termination_reason_ == PursuitTerminationReason::WidthChanged ||
            active_termination_reason_ == PursuitTerminationReason::HardTurn) {
          active_->state = ExplorationCandidateState::Suspended;
          recordDiagnostic(active_->id, CandidateDiagnosticKind::Suspended,
                           std::string(toString(active_termination_reason_)));
        } else if (active_termination_reason_ ==
                   PursuitTerminationReason::CandidateUnreachable) {
          active_->state = ExplorationCandidateState::Abandoned;
          recordDiagnostic(active_->id, CandidateDiagnosticKind::Abandoned,
                           "candidate_unreachable");
        } else {
          active_->state = ExplorationCandidateState::Completed;
          recordDiagnostic(active_->id, CandidateDiagnosticKind::Completed,
                           std::string(toString(active_termination_reason_)));
        }
        candidate_registry_[active_->id] = *active_;
        state_ = HleState::RecordPassage;
      }
    }
    if (state_ == HleState::PursueCandidate && active_->passage_id)
      updatePassageGrid(input.observation, active_->id, *active_->passage_id,
                        active_->start, PassageCompletionState::InProgress);
    if (state_ == HleState::PursueCandidate) {
      result.action = pursue(input);
      result.state = state_;
      result.rationale = "extend and pursue selected passage";
      return finish_result(std::move(result));
    }
  }
  if (state_ == HleState::RecordPassage) {
    PassageCompletionState completion = PassageCompletionState::Completed;
    if (active_->state == ExplorationCandidateState::Suspended)
      completion = PassageCompletionState::Suspended;
    else if (active_->state == ExplorationCandidateState::Abandoned)
      completion = PassageCompletionState::Abandoned;
    updatePassageGrid(input.observation, active_->id, *active_->passage_id,
                      active_->start, completion);
    result.event = active_->state == ExplorationCandidateState::Suspended
                       ? CandidateLifecycleEvent::Suspended
                   : active_->state == ExplorationCandidateState::Abandoned
                       ? CandidateLifecycleEvent::Abandoned
                       : CandidateLifecycleEvent::Completed;
    result.candidate_id = active_->id;
    result.pursuit_termination_reason = active_termination_reason_;
    active_.reset();
    state_ = HleState::DiscoverCandidate;
    result.state = state_;
    result.rationale = "passage terminal state recorded; discover next cue";
    return finish_result(std::move(result));
  }
  if (state_ == HleState::FinalizeModel) {
    result.state = HleState::FinalizeModel;
    result.completion_reason = completion_reason_;
    result.pursuit_termination_reason = active_termination_reason_;
    result.rationale = "finalize exploration models";
    state_ = HleState::Complete;
    return finish_result(std::move(result));
  }
  result.state = HleState::Complete;
  result.completion_reason = completion_reason_;
  result.pursuit_termination_reason = active_termination_reason_;
  result.rationale = "exploration complete";
  return finish_result(std::move(result));
}

void HighLevelExplorer::finish() noexcept {
  if (completion_reason_ == ExplorationCompletionReason::None)
    completion_reason_ = ExplorationCompletionReason::ExplicitlyFinished;
  active_termination_reason_ = PursuitTerminationReason::ExplicitlyFinished;
  if (active_) {
    active_->state = ExplorationCandidateState::Abandoned;
    const auto found = candidate_registry_.find(active_->id);
    if (found != candidate_registry_.end()) found->second = *active_;
    try {
      recordDiagnostic(active_->id, CandidateDiagnosticKind::Abandoned,
                       "explicitly_finished");
    } catch (...) {
      // The terminal state remains authoritative if diagnostics cannot be
      // allocated during this noexcept shutdown path.
    }
  }
  state_ = HleState::FinalizeModel;
}

PassageGridSnapshot HighLevelExplorer::passageGrid() const {
  PassageGridSnapshot result;
  result.revision = passage_revision_;
  if (passage_cells_.empty() || !passage_grid_reference_) return result;
  int minimum_row = std::numeric_limits<int>::max();
  int maximum_row = std::numeric_limits<int>::min();
  int minimum_column = std::numeric_limits<int>::max();
  int maximum_column = std::numeric_limits<int>::min();
  for (const auto& [key, cell] : passage_cells_) {
    (void)key;
    minimum_row = std::min(minimum_row, cell.row);
    maximum_row = std::max(maximum_row, cell.row);
    minimum_column = std::min(minimum_column, cell.column);
    maximum_column = std::max(maximum_column, cell.column);
  }
  if (configuration_.passage_grid_geometry.valid()) {
    result.geometry = configuration_.passage_grid_geometry;
    minimum_row = 0;
    minimum_column = 0;
  } else {
    const double resolution = configuration_.passage_grid_resolution.meters();
    const domain::Point2D minimum{
        passage_grid_reference_->x_m +
            static_cast<double>(minimum_column) * resolution,
        passage_grid_reference_->y_m +
            static_cast<double>(minimum_row) * resolution};
    result.geometry = domain::GridGeometry::fromBounds(
        "map", minimum,
        {passage_grid_reference_->x_m +
             static_cast<double>(maximum_column + 1) * resolution,
         passage_grid_reference_->y_m +
             static_cast<double>(maximum_row + 1) * resolution},
        resolution, domain::GridExtentMode::Expandable,
        domain::GridExtentSource::SensorDerivedExpansion,
        domain::GridOutOfBoundsBehavior::ExpandBeforeInsert,
        static_cast<std::size_t>(passage_revision_));
  }
  result.cells.reserve(passage_cells_.size());
  for (const auto& [key, cell] : passage_cells_) {
    (void)key;
    auto projected = cell;
    projected.row -= minimum_row;
    projected.column -= minimum_column;
    result.cells.push_back(std::move(projected));
  }
  std::sort(result.cells.begin(), result.cells.end(),
            [](const PassageCell& left, const PassageCell& right) {
              return left.row < right.row ||
                     (left.row == right.row && left.column < right.column);
            });
  return result;
}

void HighLevelExplorer::restorePassageGrid(
    const PassageGridSnapshot& snapshot) {
  if (!snapshot.geometry.valid())
    throw std::invalid_argument("restored HLE passage grid requires geometry");
  if (std::abs(snapshot.geometry.resolution_m -
               configuration_.passage_grid_resolution.meters()) >
      domain::geometry_tolerance_m)
    throw std::invalid_argument(
        "restored HLE passage grid resolution is incompatible");
  std::unordered_map<std::int64_t, PassageCell> restored;
  for (const auto& cell : snapshot.cells) {
    if (cell.row < 0 || cell.column < 0 ||
        static_cast<std::size_t>(cell.row) >= snapshot.geometry.rows ||
        static_cast<std::size_t>(cell.column) >= snapshot.geometry.columns)
      throw std::invalid_argument(
          "restored HLE passage cell is outside its geometry");
    restored[cellKey(cell.row, cell.column)] = cell;
  }
  passage_cells_ = std::move(restored);
  passage_grid_reference_ = snapshot.geometry.origin;
  passage_revision_ = snapshot.revision;
  for (const auto& [key, cell] : passage_cells_) {
    (void)key;
    if (cell.passage_id)
      next_passage_id_ = std::max(next_passage_id_, *cell.passage_id + 1U);
  }
}

std::vector<ExplorationCandidate>
HighLevelExplorer::unfinishedCandidates() const {
  std::vector<ExplorationCandidate> result;
  for (const auto& [id, candidate] : candidate_registry_) {
    (void)id;
    if (candidate.state == ExplorationCandidateState::Queued ||
        candidate.state == ExplorationCandidateState::Selected ||
        candidate.state == ExplorationCandidateState::Pursuing ||
        candidate.state == ExplorationCandidateState::Suspended)
      result.push_back(candidate);
  }
  std::stable_sort(result.begin(), result.end(),
                   [](const auto& left, const auto& right) {
                     return left.id < right.id;
                   });
  return result;
}

}  // namespace semaforr::exploration
