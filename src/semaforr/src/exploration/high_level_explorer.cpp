#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <semaforr/exploration/high_level_explorer.hpp>
#include <stdexcept>

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

}  // namespace

void HighLevelExplorationConfiguration::validate() const {
  if (!(minimum_clearance.meters() > 0.0) ||
      !(heading_tolerance.radians() > 0.0) ||
      !(candidate_completion_distance.meters() > 0.0) ||
      !(cue_similarity_radius.meters() > 0.0) ||
      !(passage_grid_resolution.meters() > 0.0) ||
      minimum_bundle_beams == 0U || !(time_budget.count() > 0.0) ||
      decision_budget == 0U)
    throw std::invalid_argument(
        "HLE distances, angles, budgets, and bundle size must be positive");
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
    case CandidateLifecycleEvent::Selected: return "candidate_selected";
    case CandidateLifecycleEvent::PursuitStarted: return "candidate_pursuit_started";
    case CandidateLifecycleEvent::Completed: return "candidate_completed";
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
  std::size_t begin = 0U;
  while (begin < ranges.size()) {
    while (begin < ranges.size() &&
           ranges[begin] < configuration.minimum_clearance.meters())
      ++begin;
    if (begin == ranges.size()) break;
    std::size_t end = begin;
    double clearance = ranges[begin];
    while (end + 1U < ranges.size() &&
           ranges[end + 1U] >= configuration.minimum_clearance.meters()) {
      ++end;
      clearance = std::min(clearance, ranges[end]);
    }
    const std::size_t width = end - begin + 1U;
    if (width >= configuration.minimum_bundle_beams) {
      const std::size_t middle = begin + (end - begin) / 2U;
      const double heading =
          observation.laser.angle_min.radians() +
          static_cast<double>(middle) *
              observation.laser.angle_increment.radians();
      result.push_back(
          {0U, observation.pose.position, domain::Angle(heading),
           domain::Distance(clearance),
           width <= 2U ? PassageKind::Doorway : PassageKind::Corridor,
           static_cast<double>(width) /
               static_cast<double>(std::max<std::size_t>(1U, ranges.size())),
           begin, end});
    }
    begin = end + 1U;
  }
  if (result.size() >= 3U)
    for (auto& candidate : result)
      candidate.kind = PassageKind::IntersectionBranch;
  return result;
}

std::vector<ExplorationCandidate> HighLevelExplorer::discover(
    const domain::RobotObservation& observation) {
  std::vector<ExplorationCandidate> discovered;
  for (auto candidate : discoverCandidates(observation, configuration_)) {
    const auto cue = endpoint(observation, (candidate.first_beam +
                                             candidate.last_beam) /
                                                2U,
                              candidate.clearance.meters());
    if (!cue_cells_.insert(cueKey(cue)).second) continue;
    candidate.id = next_candidate_id_++;
    candidates_.push(candidate);
    discovered.push_back(candidate);
  }
  return discovered;
}

void HighLevelExplorer::updatePassageGrid(
    const domain::RobotObservation& observation) {
  bool changed = false;
  const double resolution = configuration_.passage_grid_resolution.meters();
  for (std::size_t beam = 0U; beam < observation.laser.ranges_m.size(); ++beam) {
    const double range = observation.laser.ranges_m[beam];
    if (!std::isfinite(range) || range <= 0.0) continue;
    const std::size_t samples = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(range / resolution)));
    for (std::size_t sample = 0U; sample <= samples; ++sample) {
      const auto point =
          endpoint(observation, beam,
                   range * static_cast<double>(sample) /
                       static_cast<double>(samples));
      const int row = static_cast<int>(std::floor(point.y_m / resolution));
      const int column = static_cast<int>(std::floor(point.x_m / resolution));
      auto& cell = passage_cells_[cellKey(row, column)];
      cell.row = row;
      cell.column = column;
      ++cell.observations;
      changed = true;
    }
  }
  if (changed) ++passage_revision_;
}

domain::Action HighLevelExplorer::pursue(const ExplorationInput& input) const {
  if (!active_) return domain::Action::pause();
  const double heading = active_->heading.radians();
  if (std::abs(heading) > configuration_.heading_tolerance.radians())
    return turnToward(heading, input.action_space);
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
  exploration_path_.push_back(input.observation.pose.position);
  if (state_ != HleState::Complete &&
      input.elapsed >= configuration_.time_budget) {
    completion_reason_ = ExplorationCompletionReason::TimeBudgetExceeded;
    state_ = HleState::FinalizeModel;
  } else if (state_ != HleState::Complete &&
             decisions_ > configuration_.decision_budget) {
    completion_reason_ = ExplorationCompletionReason::DecisionBudgetExceeded;
    state_ = HleState::FinalizeModel;
  }

  ExplorationResult result;
  result.state = state_;
  result.passage_grid_revision = passage_revision_;
  if (state_ == HleState::Initialize) {
    state_ = HleState::DiscoverCandidate;
    result.state = state_;
    result.action = turnToward(1.0, input.action_space);
    result.rationale = "initialize exploration and survey";
    return result;
  }
  if (state_ == HleState::DiscoverCandidate) {
    result.discovered = discover(input.observation);
    if (!result.discovered.empty())
      result.event = CandidateLifecycleEvent::Discovered;
    state_ = HleState::SelectNextCandidate;
  }
  if (state_ == HleState::SelectNextCandidate) {
    if (candidates_.empty()) {
      if (completion_reason_ == ExplorationCompletionReason::None)
        completion_reason_ =
            ExplorationCompletionReason::CandidateQueueExhausted;
      state_ = HleState::FinalizeModel;
    } else {
      active_ = candidates_.top();
      candidates_.pop();
      pursuit_started_ = false;
      state_ = HleState::ReturnToCandidateStart;
      result.event = CandidateLifecycleEvent::Selected;
      result.candidate_id = active_->id;
      result.subgoal = ExplorationSubgoal{active_->start, active_->id};
      result.state = state_;
      result.rationale = "return to stable candidate start";
      return result;
    }
  }
  if (state_ == HleState::ReturnToCandidateStart) {
    result.candidate_id = active_->id;
    if (domain::distance(input.observation.pose.position, active_->start)
            .meters() >
        configuration_.candidate_completion_distance.meters()) {
      result.subgoal = ExplorationSubgoal{active_->start, active_->id};
      result.rationale = "return to candidate start";
      return result;
    }
    state_ = HleState::PursueCandidate;
  }
  if (state_ == HleState::PursueCandidate) {
    result.candidate_id = active_->id;
    if (!pursuit_started_) {
      pursuit_started_ = true;
      result.event = CandidateLifecycleEvent::PursuitStarted;
    } else if (domain::distance(input.observation.pose.position, active_->start)
                   .meters() >=
               configuration_.candidate_completion_distance.meters()) {
      state_ = HleState::RecordPassage;
    }
    if (state_ == HleState::PursueCandidate) {
      result.action = pursue(input);
      result.state = state_;
      result.rationale = "pursue selected candidate";
      return result;
    }
  }
  if (state_ == HleState::RecordPassage) {
    updatePassageGrid(input.observation);
    result.event = CandidateLifecycleEvent::Completed;
    result.candidate_id = active_ ? std::optional(active_->id) : std::nullopt;
    result.passage_grid_revision = passage_revision_;
    active_.reset();
    state_ = HleState::DiscoverCandidate;
    result.state = state_;
    result.rationale = "passage recorded; discover next candidate";
    return result;
  }
  if (state_ == HleState::FinalizeModel) {
    result.state = HleState::FinalizeModel;
    result.completion_reason = completion_reason_;
    result.rationale = "finalize exploration models";
    state_ = HleState::Complete;
    return result;
  }
  result.state = HleState::Complete;
  result.completion_reason = completion_reason_;
  result.rationale = "exploration complete";
  return result;
}

void HighLevelExplorer::finish() noexcept {
  if (completion_reason_ == ExplorationCompletionReason::None)
    completion_reason_ = ExplorationCompletionReason::ExplicitlyFinished;
  state_ = HleState::FinalizeModel;
}

PassageGridSnapshot HighLevelExplorer::passageGrid() const {
  PassageGridSnapshot result;
  result.resolution_m = configuration_.passage_grid_resolution.meters();
  result.revision = passage_revision_;
  result.cells.reserve(passage_cells_.size());
  for (const auto& [key, cell] : passage_cells_) {
    (void)key;
    result.cells.push_back(cell);
  }
  std::sort(result.cells.begin(), result.cells.end(),
            [](const PassageCell& left, const PassageCell& right) {
              return left.row < right.row ||
                     (left.row == right.row && left.column < right.column);
            });
  return result;
}

std::vector<ExplorationCandidate>
HighLevelExplorer::unfinishedCandidates() const {
  auto queue = candidates_;
  std::vector<ExplorationCandidate> result;
  if (active_) result.push_back(*active_);
  while (!queue.empty()) {
    result.push_back(queue.top());
    queue.pop();
  }
  std::stable_sort(result.begin(), result.end(),
                   [](const auto& left, const auto& right) {
                     return left.id < right.id;
                   });
  return result;
}

}  // namespace semaforr::exploration
