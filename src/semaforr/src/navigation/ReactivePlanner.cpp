#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/planning/reactive_planner.hpp>
#include <stdexcept>

namespace semaforr::planning {
namespace {

std::optional<domain::Point2D> waypoint(const domain::WorldModel& world) {
  if (!world.mission.active()) return std::nullopt;
  return world.mission.active()->waypoint().value_or(
      world.mission.active()->target);
}

double headingError(const domain::Pose2D& pose, domain::Point2D target) {
  return domain::Angle::normalize(
      std::atan2(target.y_m - pose.position.y_m,
                 target.x_m - pose.position.x_m) -
      pose.heading.radians());
}

domain::Action turn(double error, const domain::ActionSpace& actions) {
  const auto& values = actions.rotation_angles_rad();
  if (values.empty()) return domain::Action::pause();
  const auto found =
      std::lower_bound(values.begin(), values.end(), std::abs(error));
  const std::size_t magnitude =
      found == values.end()
          ? values.size()
          : static_cast<std::size_t>(found - values.begin()) + 1U;
  return domain::Action(error < 0.0 ? domain::ActionType::TurnRight
                                    : domain::ActionType::TurnLeft,
                        magnitude);
}

bool targetSensed(const domain::WorldModel& world) {
  if (!world.mission.active() || !world.robot.laser ||
      world.robot.laser->ranges_m.empty())
    return false;
  const auto target = world.mission.active()->target;
  const double range = domain::distance(world.robot.pose.position, target).meters();
  const double bearing = headingError(world.robot.pose, target);
  const double first = world.robot.laser->angle_min.radians();
  const double increment = world.robot.laser->angle_increment.radians();
  if (increment <= 0.0 || bearing < first) return false;
  const auto beam = static_cast<std::size_t>(
      std::llround((bearing - first) / increment));
  return beam < world.robot.laser->ranges_m.size() &&
         world.robot.laser->ranges_m[beam] + 0.05 >= range;
}

ReactiveResult resultFrom(std::string_view planner,
                          ReactivePlanUpdate update) {
  return {update.status, update.action, std::string(planner),
          std::move(update.explanation), update.completion_reason};
}

}  // namespace

ReactiveResult ReactivePlanner::evaluate(const ReactiveRequest& request) {
  decision::DecisionContext context{request.world, &request.action_space};
  if (!evaluateTrigger(context).triggered) return {};
  return resultFrom(name(), update(context));
}

std::string_view toString(ReactiveCompletionReason reason) noexcept {
  switch (reason) {
    case ReactiveCompletionReason::None: return "none";
    case ReactiveCompletionReason::TargetSensed: return "target_sensed";
    case ReactiveCompletionReason::NewPlanAvailable: return "new_plan_available";
    case ReactiveCompletionReason::CandidateExhausted:
      return "candidate_exhausted";
    case ReactiveCompletionReason::NoCandidates: return "no_candidates";
    case ReactiveCompletionReason::BudgetExceeded: return "budget_exceeded";
    case ReactiveCompletionReason::SensorLost: return "sensor_lost";
    case ReactiveCompletionReason::MissionChanged: return "mission_changed";
  }
  return "none";
}

std::string_view toString(LowLevelExplorationState state) noexcept {
  switch (state) {
    case LowLevelExplorationState::DetectMissingGuidance:
      return "detect_missing_guidance";
    case LowLevelExplorationState::AssembleCandidateRays:
      return "assemble_candidate_rays";
    case LowLevelExplorationState::RankByTargetRelevance:
      return "rank_by_target_relevance";
    case LowLevelExplorationState::PlanToCandidateStart:
      return "plan_to_candidate_start";
    case LowLevelExplorationState::PursueCandidate:
      return "pursue_candidate";
    case LowLevelExplorationState::CheckConnectivity:
      return "check_connectivity";
    case LowLevelExplorationState::Complete: return "complete";
  }
  return "complete";
}

TriggerEvaluation Thru::evaluateTrigger(
    const decision::DecisionContext& context) const {
  const auto target = waypoint(context.world);
  return {target && std::abs(headingError(context.world.robot.pose, *target)) <=
                        0.35,
          "waypoint lies through the current opening"};
}

ReactivePlanUpdate Thru::update(
    const decision::DecisionContext& context) {
  if (!context.action_space || !evaluateTrigger(context).triggered) return {};
  return {ReactiveStatus::Action,
          domain::Action(domain::ActionType::Forward, 1U), {},
          ReactiveCompletionReason::None, std::nullopt,
          "waypoint lies through the current opening"};
}

TriggerEvaluation Behind::evaluateTrigger(
    const decision::DecisionContext& context) const {
  const auto target = waypoint(context.world);
  return {target && std::abs(headingError(context.world.robot.pose, *target)) >=
                        2.35,
          "active waypoint is behind the robot"};
}

ReactivePlanUpdate Behind::update(
    const decision::DecisionContext& context) {
  const auto target = waypoint(context.world);
  if (!target || !context.action_space || !evaluateTrigger(context).triggered)
    return {};
  return {ReactiveStatus::Action,
          turn(headingError(context.world.robot.pose, *target),
               *context.action_space),
          {}, ReactiveCompletionReason::None, std::nullopt,
          "active waypoint is behind the robot"};
}

TriggerEvaluation Out::evaluateTrigger(
    const decision::DecisionContext& context) const {
  bool outside_known_space = false;
  const auto& grid = context.world.spatial.known_grid;
  if (!grid.cells.empty() && grid.columns > 0U && grid.rows > 0U &&
      grid.resolution_m > 0.0) {
    const int column = static_cast<int>(std::floor(
        (context.world.robot.pose.position.x_m - grid.origin.x_m) /
        grid.resolution_m));
    const int row = static_cast<int>(std::floor(
        (context.world.robot.pose.position.y_m - grid.origin.y_m) /
        grid.resolution_m));
    outside_known_space =
        row < 0 || column < 0 ||
        static_cast<std::size_t>(row) >= grid.rows ||
        static_cast<std::size_t>(column) >= grid.columns ||
        grid.cells[static_cast<std::size_t>(row) * grid.columns +
                   static_cast<std::size_t>(column)] == 0U;
  }
  return {context.world.recovery.confined || outside_known_space,
          outside_known_space ? "robot left known-grid support"
                              : "robot is confined"};
}

ReactivePlanUpdate Out::update(
    const decision::DecisionContext& context) {
  if (!context.action_space || !evaluateTrigger(context).triggered) return {};
  const auto& grid = context.world.spatial.known_grid;
  if (grid.cells.empty())
    return {ReactiveStatus::Action,
            domain::Action(domain::ActionType::TurnLeft, 1U), {},
            ReactiveCompletionReason::None, std::nullopt,
            "survey while leaving confinement"};
  const auto strongest =
      std::max_element(grid.cells.begin(), grid.cells.end());
  const std::size_t index =
      static_cast<std::size_t>(strongest - grid.cells.begin());
  const std::size_t robot_column =
      grid.columns == 0U ? 0U : grid.columns / 2U;
  const std::size_t target_column =
      grid.columns == 0U ? 0U : index % grid.columns;
  if (target_column == robot_column)
    return {ReactiveStatus::Action,
            domain::Action(domain::ActionType::Forward, 1U), {},
            ReactiveCompletionReason::None, std::nullopt,
            "move toward strongest known-grid support"};
  return {ReactiveStatus::Action,
          domain::Action(target_column < robot_column
                             ? domain::ActionType::TurnRight
                             : domain::ActionType::TurnLeft,
                         1U),
          {}, ReactiveCompletionReason::None, std::nullopt,
          "turn toward strongest known-grid support"};
}

ReactivePlannerCoordinator::ReactivePlannerCoordinator() = default;

ReactivePlannerCoordinator::ReactivePlannerCoordinator(
    std::vector<std::unique_ptr<ReactivePlanner>> planners) {
  for (auto& planner : planners) add(std::move(planner));
}

void ReactivePlannerCoordinator::add(
    std::unique_ptr<ReactivePlanner> planner) {
  if (!planner) throw std::invalid_argument("reactive planner is null");
  if (std::any_of(planners_.begin(), planners_.end(), [&](const auto& item) {
        return item->name() == planner->name();
      }))
    throw std::invalid_argument("duplicate reactive planner");
  planners_.push_back(std::move(planner));
}

ReactiveResult ReactivePlannerCoordinator::evaluate(
    const ReactiveRequest& request) {
  decision::DecisionContext context{request.world, &request.action_space};
  for (const auto& planner : planners_) {
    if (!planner->evaluateTrigger(context).triggered) continue;
    auto result = resultFrom(planner->name(), planner->update(context));
    if (result.status != ReactiveStatus::NotApplicable) return result;
  }
  return {};
}

void ReactivePlannerCoordinator::cancelAll(InterruptionReason reason) {
  for (auto& planner : planners_) planner->cancel(reason);
}

LowLevelExplorer::LowLevelExplorer(std::size_t history_window,
                                   double progress_threshold_m,
                                   std::size_t decision_budget)
    : history_window_(history_window),
      progress_threshold_m_(progress_threshold_m),
      decision_budget_(decision_budget) {
  if (history_window_ < 2U || !(progress_threshold_m_ > 0.0) ||
      decision_budget_ == 0U)
    throw std::invalid_argument("invalid LLE progress/budget configuration");
}

TriggerEvaluation LowLevelExplorer::evaluateTrigger(
    const decision::DecisionContext& context) const {
  if (!context.world.mission.active()) return {};
  const auto& task = *context.world.mission.active();
  const bool only_direct_guidance = task.plan.size() <= 1U;
  const bool lacks_connectivity =
      context.world.spatial.skeleton_nodes.empty() &&
      context.world.spatial.highways.nodes.empty() &&
      context.world.spatial.highways.graph.vertices.empty();
  const auto& history = context.world.navigation_history.entries();
  bool stalled = false;
  if (history.size() >= history_window_) {
    const auto first =
        history.end() - static_cast<std::ptrdiff_t>(history_window_);
    const double displacement =
        domain::distance(first->pose.position, history.back().pose.position)
            .meters();
    stalled = displacement < progress_threshold_m_ &&
              std::any_of(first, history.end(), [](const auto& entry) {
                return entry.action.type() == domain::ActionType::Forward;
              });
  }
  return {(only_direct_guidance && lacks_connectivity) || stalled,
          stalled ? "target navigation has stalled"
                  : "target-directed planning lacks learned connectivity"};
}

decision::ReplanningRequest LowLevelExplorer::evaluateReplan(
    const decision::DecisionContext& context) const {
  const auto trigger = evaluateTrigger(context);
  return {trigger.triggered, trigger.rationale};
}

void LowLevelExplorer::assembleCandidates(const domain::WorldModel& world) {
  ranked_candidates_.clear();
  candidate_cursor_ = 0U;
  const auto target = world.mission.active()->target;
  const auto add = [&](LLECandidateSource source, domain::Point2D start,
                       domain::Point2D point, std::uint64_t stable_id = 0U) {
    const double relevance = -domain::distance(point, target).meters();
    ranked_candidates_.push_back(
        {stable_id == 0U ? next_candidate_id_++ : stable_id, source, start,
         point, relevance});
  };
  for (const auto& cue : world.spatial.unfinished_hle_candidates)
    add(LLECandidateSource::UnfinishedHle, cue.start, cue.target, cue.id);

  const auto& laser = *world.robot.laser;
  for (std::size_t beam = 0U; beam < laser.ranges_m.size(); ++beam) {
    const double range = laser.ranges_m[beam];
    if (!std::isfinite(range) || range < laser.minimum_range.meters()) continue;
    const double angle =
        world.robot.pose.heading.radians() + laser.angle_min.radians() +
        static_cast<double>(beam) * laser.angle_increment.radians();
    add(LLECandidateSource::CurrentTargetObservation,
        world.robot.pose.position,
        {world.robot.pose.position.x_m + range * std::cos(angle),
         world.robot.pose.position.y_m + range * std::sin(angle)});
  }
  for (const auto& region : world.spatial.learned_regions)
    add(LLECandidateSource::RegionVisibility, world.robot.pose.position,
        region.center);
  const auto& grid = world.spatial.inclusion_grid;
  if (grid.columns > 0U && grid.rows > 0U) {
    for (std::size_t index = 0U; index < grid.cells.size(); ++index) {
      if (grid.cells[index] != 0U) continue;
      const auto row = index / grid.columns;
      const auto column = index % grid.columns;
      add(LLECandidateSource::InclusionGap, world.robot.pose.position,
          {grid.origin.x_m + (static_cast<double>(column) + 0.5) *
                                 grid.resolution_m,
           grid.origin.y_m + (static_cast<double>(row) + 0.5) *
                                 grid.resolution_m});
    }
  }
}

domain::Action LowLevelExplorer::actionToward(
    const domain::Pose2D& pose, domain::Point2D target,
    const domain::ActionSpace& actions) const {
  const double error = headingError(pose, target);
  return std::abs(error) > 0.2
             ? turn(error, actions)
             : domain::Action(domain::ActionType::Forward, 1U);
}

ReactivePlanUpdate LowLevelExplorer::complete(
    ReactiveCompletionReason reason, std::string explanation,
    ReactiveStatus status) {
  completion_reason_ = reason;
  state_ = LowLevelExplorationState::Complete;
  return {status, std::nullopt, state_, reason, std::nullopt,
          std::move(explanation)};
}

ReactivePlanUpdate LowLevelExplorer::update(
    const decision::DecisionContext& context) {
  if (!context.world.mission.active())
    return complete(ReactiveCompletionReason::MissionChanged,
                    "mission is no longer active");
  if (mission_id_ && *mission_id_ != context.world.mission.active()->id)
    return complete(ReactiveCompletionReason::MissionChanged,
                    "active mission changed");
  if (!context.world.robot.laser ||
      context.world.robot.laser->ranges_m.empty())
    return complete(ReactiveCompletionReason::SensorLost,
                    "laser observation is unavailable");
  if (state_ != LowLevelExplorationState::DetectMissingGuidance &&
      context.world.mission.active()->plan.size() > 1U)
    return complete(ReactiveCompletionReason::NewPlanAvailable,
                    "a new target-directed plan is available");
  if (targetSensed(context.world))
    return complete(ReactiveCompletionReason::TargetSensed,
                    "target is directly sensed");
  if (++decisions_ > decision_budget_)
    return complete(ReactiveCompletionReason::BudgetExceeded,
                    "LLE decision budget exceeded");
  if (!context.action_space)
    return complete(ReactiveCompletionReason::SensorLost,
                    "action space is unavailable");

  if (state_ == LowLevelExplorationState::Complete) {
    state_ = LowLevelExplorationState::DetectMissingGuidance;
    completion_reason_ = ReactiveCompletionReason::None;
    decisions_ = 1U;
  }
  if (state_ == LowLevelExplorationState::DetectMissingGuidance) {
    if (!evaluateTrigger(context).triggered) return {};
    mission_id_ = context.world.mission.active()->id;
    source_revision_ = context.world.spatial.revision;
    state_ = LowLevelExplorationState::AssembleCandidateRays;
  }
  if (state_ == LowLevelExplorationState::AssembleCandidateRays) {
    assembleCandidates(context.world);
    if (ranked_candidates_.empty())
      return complete(ReactiveCompletionReason::NoCandidates,
                      "no LLE candidates are available");
    state_ = LowLevelExplorationState::RankByTargetRelevance;
  }
  if (state_ == LowLevelExplorationState::RankByTargetRelevance) {
    std::stable_sort(
        ranked_candidates_.begin(), ranked_candidates_.end(),
        [](const auto& left, const auto& right) {
          return left.target_relevance > right.target_relevance ||
                 (left.target_relevance == right.target_relevance &&
                  left.id < right.id);
        });
    state_ = LowLevelExplorationState::PlanToCandidateStart;
  }
  if (candidate_cursor_ >= ranked_candidates_.size())
    return complete(ReactiveCompletionReason::CandidateExhausted,
                    "all LLE candidates were exhausted");
  const auto& candidate = ranked_candidates_[candidate_cursor_];
  if (state_ == LowLevelExplorationState::PlanToCandidateStart) {
    if (domain::distance(context.world.robot.pose.position, candidate.start)
            .meters() > progress_threshold_m_) {
      return {ReactiveStatus::Action,
              actionToward(context.world.robot.pose, candidate.start,
                           *context.action_space),
              state_, ReactiveCompletionReason::None, candidate.id,
              "plan to candidate start"};
    }
    state_ = LowLevelExplorationState::PursueCandidate;
  }
  if (state_ == LowLevelExplorationState::PursueCandidate) {
    state_ = LowLevelExplorationState::CheckConnectivity;
    return {ReactiveStatus::Action,
            actionToward(context.world.robot.pose, candidate.target,
                         *context.action_space),
            LowLevelExplorationState::PursueCandidate,
            ReactiveCompletionReason::None, candidate.id,
            "pursue candidate ray"};
  }
  if (state_ == LowLevelExplorationState::CheckConnectivity) {
    const bool connectivity =
        context.world.spatial.revision != source_revision_ &&
        (!context.world.spatial.skeleton_nodes.empty() ||
         !context.world.spatial.highways.nodes.empty() ||
         !context.world.spatial.highways.graph.vertices.empty());
    if (connectivity)
      return complete(ReactiveCompletionReason::NewPlanAvailable,
                      "new connectivity found; request Tier-2 replanning",
                      ReactiveStatus::RequestReplan);
    ++candidate_cursor_;
    state_ = LowLevelExplorationState::PlanToCandidateStart;
    if (candidate_cursor_ >= ranked_candidates_.size())
      return complete(ReactiveCompletionReason::CandidateExhausted,
                      "all LLE candidates were exhausted");
    return update(context);
  }
  return {};
}

void LowLevelExplorer::cancel(InterruptionReason reason) {
  switch (reason) {
    case InterruptionReason::TargetSensed:
      completion_reason_ = ReactiveCompletionReason::TargetSensed;
      break;
    case InterruptionReason::NewPlanAvailable:
      completion_reason_ = ReactiveCompletionReason::NewPlanAvailable;
      break;
    case InterruptionReason::SensorLost:
      completion_reason_ = ReactiveCompletionReason::SensorLost;
      break;
    case InterruptionReason::MissionChanged:
    case InterruptionReason::Disabled:
      completion_reason_ = ReactiveCompletionReason::MissionChanged;
      break;
  }
  state_ = LowLevelExplorationState::Complete;
}

ReactiveResult LowLevelExplorer::evaluate(const ReactiveRequest& request) {
  decision::DecisionContext context{request.world, &request.action_space};
  if (state_ == LowLevelExplorationState::DetectMissingGuidance &&
      !evaluateTrigger(context).triggered)
    return {};
  return resultFrom(name(), update(context));
}

}  // namespace semaforr::planning
