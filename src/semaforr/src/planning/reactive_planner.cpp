#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/planning/reactive_planner.hpp>
#include <semaforr/domain/motion_model.hpp>
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

bool sensed(const domain::Pose2D&, const domain::LaserObservation&,
            domain::Point2D);

bool targetSensed(const domain::WorldModel& world) {
  if (!world.mission.active() || !world.robot.laser ||
      world.robot.laser->ranges_m.empty())
    return false;
  return sensed(world.robot.pose, *world.robot.laser,
                world.mission.active()->target);
}

bool sensed(const domain::Pose2D& pose,
            const domain::LaserObservation& laser,
            domain::Point2D point) {
  if (laser.ranges_m.empty() || laser.angle_increment.radians() <= 0.0)
    return false;
  const double distance = domain::distance(pose.position, point).meters();
  const double bearing = headingError(pose, point);
  const double coordinate =
      (bearing - laser.angle_min.radians()) /
      laser.angle_increment.radians();
  if (coordinate < 0.0 ||
      coordinate > static_cast<double>(laser.ranges_m.size() - 1U))
    return false;
  const auto center = static_cast<std::ptrdiff_t>(std::llround(coordinate));
  std::size_t visible = 0U;
  std::size_t sampled = 0U;
  for (std::ptrdiff_t offset = -2; offset <= 2; ++offset) {
    const auto beam = center + offset;
    if (beam < 0 ||
        beam >= static_cast<std::ptrdiff_t>(laser.ranges_m.size()))
      continue;
    ++sampled;
    const double range = laser.ranges_m[static_cast<std::size_t>(beam)];
    if (!std::isnan(range) && range + domain::geometry_tolerance_m >= distance)
      ++visible;
  }
  return sampled >= 3U && visible >= 3U;
}

bool forwardBlocked(const domain::LaserObservation& laser,
                    const domain::ActionSpace& actions,
                    double clearance_m = 0.35) {
  double nearest = std::numeric_limits<double>::infinity();
  double angle = laser.angle_min.radians();
  for (const double range : laser.ranges_m) {
    if (std::isfinite(range)) {
      const double longitudinal = range * std::cos(angle);
      const double lateral = std::abs(range * std::sin(angle));
      if (longitudinal > 0.0 && lateral <= clearance_m)
        nearest = std::min(nearest, longitudinal);
    }
    angle += laser.angle_increment.radians();
  }
  return actions.move_distances_m().front() + clearance_m >= nearest;
}

template <typename Grid>
std::optional<std::size_t> gridIndex(const Grid& grid,
                                     domain::Point2D point) {
  return grid.extent().index(point);
}

domain::Action stepToward(const domain::Pose2D& pose,
                          domain::Point2D point,
                          const domain::ActionSpace& actions,
                          double desired_step_m) {
  const double error = headingError(pose, point);
  if (std::abs(error) > 0.2) return turn(error, actions);
  const auto& distances = actions.move_distances_m();
  const auto found = std::upper_bound(distances.begin(), distances.end(),
                                      desired_step_m);
  const std::size_t magnitude =
      found == distances.begin()
          ? 1U
          : static_cast<std::size_t>(found - distances.begin());
  return domain::Action(domain::ActionType::Forward, magnitude);
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

Thru::Thru(std::size_t decision_budget, double desired_step_m,
           double endpoint_tolerance_m)
    : decision_budget_(decision_budget),
      desired_step_m_(desired_step_m),
      endpoint_tolerance_m_(endpoint_tolerance_m) {
  if (decision_budget_ == 0U || desired_step_m_ <= 0.0 ||
      endpoint_tolerance_m_ <= 0.0)
    throw std::invalid_argument("invalid Thru configuration");
}

TriggerEvaluation Thru::evaluateTrigger(
    const decision::DecisionContext& context) const {
  const auto target = waypoint(context.world);
  if (endpoint_) return {true, "Thru repositioning is active"};
  if (!target || !context.action_space || !context.world.robot.laser)
    return {};
  return {sensed(context.world.robot.pose, *context.world.robot.laser,
                 *target) &&
              forwardBlocked(*context.world.robot.laser,
                             *context.action_space),
          "sensed waypoint is blocked by a tight opening"};
}

std::optional<domain::Point2D> Thru::chooseEndpoint(
    const domain::WorldModel& world) const {
  const auto target = waypoint(world);
  if (!target || !world.robot.laser || world.robot.laser->ranges_m.size() < 3U)
    return std::nullopt;
  const auto& laser = *world.robot.laser;
  const double bearing = headingError(world.robot.pose, *target);
  const auto center = static_cast<std::ptrdiff_t>(std::llround(
      (bearing - laser.angle_min.radians()) /
      laser.angle_increment.radians()));
  if (center < 0 || center >= static_cast<std::ptrdiff_t>(laser.ranges_m.size()))
    return std::nullopt;
  const auto bundle = [&](std::ptrdiff_t begin, std::ptrdiff_t end) {
    double x = 0.0;
    double y = 0.0;
    std::size_t count = 0U;
    for (auto beam = begin; beam != end; beam += begin < end ? 1 : -1) {
      if (beam < 0 || beam >= static_cast<std::ptrdiff_t>(laser.ranges_m.size()))
        break;
      const double measured = laser.ranges_m[static_cast<std::size_t>(beam)];
      const double range = std::isfinite(measured)
                               ? measured
                               : laser.maximum_range.meters();
      if (!(range > 0.0)) continue;
      const double angle = world.robot.pose.heading.radians() +
                           laser.angle_min.radians() +
                           static_cast<double>(beam) *
                               laser.angle_increment.radians();
      x += world.robot.pose.position.x_m + range * std::cos(angle);
      y += world.robot.pose.position.y_m + range * std::sin(angle);
      ++count;
      if (count == 5U) break;
    }
    return count == 0U
               ? std::optional<domain::Point2D>{}
               : std::optional<domain::Point2D>{{x / count, y / count}};
  };
  const auto left = bundle(center + 1, static_cast<std::ptrdiff_t>(laser.ranges_m.size()));
  const auto right = bundle(center - 1, -1);
  if (!left) return right;
  if (!right) return left;
  return domain::distance(world.robot.pose.position, *left).meters() >
                 domain::distance(world.robot.pose.position, *right).meters()
             ? left
             : right;
}

ReactivePlanUpdate Thru::update(
    const decision::DecisionContext& context) {
  if (!context.action_space || !evaluateTrigger(context).triggered) return {};
  if (!context.world.mission.active()) return {};
  if (mission_id_ && *mission_id_ != context.world.mission.active()->id) {
    cancel(InterruptionReason::MissionChanged);
    return {};
  }
  if (!endpoint_) {
    endpoint_ = chooseEndpoint(context.world);
    mission_id_ = context.world.mission.active()->id;
    decisions_ = 0U;
  }
  if (!endpoint_) return {};
  if (domain::distance(context.world.robot.pose.position, *endpoint_).meters() <=
          endpoint_tolerance_m_ ||
      decisions_++ >= decision_budget_) {
    cancel(InterruptionReason::Disabled);
    return {};
  }
  return {ReactiveStatus::Action,
          stepToward(context.world.robot.pose, *endpoint_,
                     *context.action_space, desired_step_m_), {},
          ReactiveCompletionReason::None, std::nullopt,
          "reposition along the clearer side of the opening"};
}

void Thru::cancel(InterruptionReason) {
  endpoint_.reset();
  mission_id_.reset();
  decisions_ = 0U;
}

TriggerEvaluation Behind::evaluateTrigger(
    const decision::DecisionContext& context) const {
  const auto target = waypoint(context.world);
  if (!target || !context.world.robot.laser ||
      domain::distance(context.world.robot.pose.position, *target).meters() >
          1.5)
    return {};
  const auto& history = context.world.navigation_history.entries();
  const bool visible_now = sensed(context.world.robot.pose,
                                  *context.world.robot.laser, *target);
  const bool visible_before = !history.empty() &&
      sensed(history.back().pose, history.back().laser, *target);
  bool last_was_quarter_turn = false;
  if (!history.empty() && context.action_space &&
      (history.back().action.type() == domain::ActionType::TurnLeft ||
       history.back().action.type() == domain::ActionType::TurnRight)) {
    const auto magnitude = history.back().action.magnitude_index();
    const auto& turns = context.action_space->rotation_angles_rad();
    last_was_quarter_turn =
        magnitude > 0U && magnitude <= turns.size() &&
        std::abs(turns[magnitude - 1U] - 1.5707963267948966) <= 0.1;
  }
  return {!visible_now && !visible_before && !last_was_quarter_turn,
          "nearby waypoint was outside both recent views"};
}

ReactivePlanUpdate Behind::update(
    const decision::DecisionContext& context) {
  const auto target = waypoint(context.world);
  if (!target || !context.action_space || !evaluateTrigger(context).triggered)
    return {};
  const auto& turns = context.action_space->rotation_angles_rad();
  const auto closest = std::min_element(
      turns.begin(), turns.end(), [](double left, double right) {
        return std::abs(left - 1.5707963267948966) <
               std::abs(right - 1.5707963267948966);
      });
  if (closest == turns.end() ||
      std::abs(*closest - 1.5707963267948966) > 0.1)
    return {};
  const auto magnitude =
      static_cast<std::size_t>(closest - turns.begin()) + 1U;
  return {ReactiveStatus::Action,
          domain::Action(domain::ActionType::TurnRight, magnitude),
          {}, ReactiveCompletionReason::None, std::nullopt,
          "turn 90 degrees to reveal the nearby waypoint"};
}

Out::Out(std::size_t coverage_threshold, double covered_fraction,
         std::size_t maximum_new_cells)
    : coverage_threshold_(coverage_threshold),
      covered_fraction_(covered_fraction),
      maximum_new_cells_(maximum_new_cells) {
  if (coverage_threshold_ == 0U || covered_fraction_ <= 0.0 ||
      covered_fraction_ > 1.0)
    throw std::invalid_argument("invalid Out configuration");
}

TriggerEvaluation Out::evaluateTrigger(
    const decision::DecisionContext& context) const {
  if (state_ != State::Idle) return {true, "Out recovery is active"};
  const auto& grid = context.world.spatial.known_grid;
  const auto nonzero = static_cast<std::size_t>(std::count_if(
      grid.cells.begin(), grid.cells.end(),
      [](std::uint32_t value) { return value > 0U; }));
  const auto well_covered = static_cast<std::size_t>(std::count_if(
      grid.cells.begin(), grid.cells.end(), [&](std::uint32_t value) {
        return value >= coverage_threshold_;
      }));
  std::size_t new_cells = 0U;
  std::vector<std::size_t> observed_new_cells;
  if (context.world.robot.laser) {
    for (const auto& endpoint : domain::laserEndpoints(
             context.world.robot.pose, *context.world.robot.laser)) {
      const auto index = gridIndex(grid, endpoint);
      if (index && grid.cells[*index] == 0U &&
          std::find(observed_new_cells.begin(), observed_new_cells.end(),
                    *index) == observed_new_cells.end())
        observed_new_cells.push_back(*index);
    }
  }
  new_cells = observed_new_cells.size();
  const bool repeatedly_confined =
      nonzero > 0U &&
      static_cast<double>(well_covered) / static_cast<double>(nonzero) >=
          covered_fraction_ &&
      new_cells <= maximum_new_cells_;
  return {context.world.recovery.confined || repeatedly_confined,
          "recent known-grid support is repeatedly confined"};
}

ReactivePlanUpdate Out::update(
    const decision::DecisionContext& context) {
  if (!context.action_space || !evaluateTrigger(context).triggered) return {};
  const auto& grid = context.world.spatial.known_grid;
  if (!context.world.mission.active()) {
    reset();
    return {};
  }
  if (mission_id_ && *mission_id_ != context.world.mission.active()->id) {
    reset();
    return {};
  }
  if (state_ == State::Idle) {
    mission_id_ = context.world.mission.active()->id;
    state_ = State::Survey;
    rotations_ = 0U;
    baseline_known_cells_ = static_cast<std::size_t>(std::count_if(
        grid.cells.begin(), grid.cells.end(),
        [](std::uint32_t value) { return value > 0U; }));
  }
  if (state_ == State::Survey) {
    const auto known = static_cast<std::size_t>(std::count_if(
        grid.cells.begin(), grid.cells.end(),
        [](std::uint32_t value) { return value > 0U; }));
    if (rotations_ > 0U &&
        known > baseline_known_cells_ + maximum_new_cells_) {
      reset();
      return {ReactiveStatus::NotApplicable, std::nullopt, {},
              ReactiveCompletionReason::NewPlanAvailable, std::nullopt,
              "survey revealed new freespace; cede control"};
    }
    if (rotations_ < 4U) {
      ++rotations_;
      return {ReactiveStatus::Action,
              turn(-1.5707963267948966, *context.action_space), {},
              ReactiveCompletionReason::None, std::nullopt,
              "survey confinement with a 90 degree rotation"};
    }
    buildEscape(context.world);
    state_ = State::Escape;
  }
  if (escape_cursor_ >= escape_points_.size()) {
    reset();
    return {ReactiveStatus::NotApplicable, std::nullopt, {},
            ReactiveCompletionReason::CandidateExhausted, std::nullopt,
            "no uncovered path-history point is available"};
  }
  while (escape_cursor_ < escape_points_.size() &&
         domain::distance(context.world.robot.pose.position,
                          escape_points_[escape_cursor_]).meters() <= 0.5)
    ++escape_cursor_;
  if (escape_cursor_ >= escape_points_.size()) {
    reset();
    return {};
  }
  return {ReactiveStatus::Action,
          stepToward(context.world.robot.pose, escape_points_[escape_cursor_],
                     *context.action_space, 0.8),
          {}, ReactiveCompletionReason::None, std::nullopt,
          "follow the reverse subtrail out of known confinement"};
}

void Out::buildEscape(const domain::WorldModel& world) {
  escape_points_.clear();
  escape_cursor_ = 0U;
  const auto& grid = world.spatial.known_grid;
  const auto& history = world.navigation_history.entries();
  for (auto entry = history.rbegin(); entry != history.rend(); ++entry) {
    if (escape_points_.empty() ||
        domain::distance(escape_points_.back(), entry->pose.position).meters() >=
            0.75)
      escape_points_.push_back(entry->pose.position);
    const auto index = gridIndex(grid, entry->pose.position);
    if (!index || grid.cells[*index] < coverage_threshold_) break;
  }
}

void Out::reset() noexcept {
  state_ = State::Idle;
  mission_id_.reset();
  rotations_ = 0U;
  baseline_known_cells_ = 0U;
  escape_points_.clear();
  escape_cursor_ = 0U;
}

void Out::cancel(InterruptionReason) { reset(); }

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
                                   std::size_t decision_budget,
                                   double minimum_cue_length_m,
                                   double target_cue_tolerance_m,
                                   std::size_t cue_waypoint_count)
    : history_window_(history_window),
      progress_threshold_m_(progress_threshold_m),
      decision_budget_(decision_budget),
      minimum_cue_length_m_(minimum_cue_length_m),
      target_cue_tolerance_m_(target_cue_tolerance_m),
      cue_waypoint_count_(cue_waypoint_count) {
  if (history_window_ < 2U || !(progress_threshold_m_ > 0.0) ||
      decision_budget_ == 0U || minimum_cue_length_m_ <= 0.0 ||
      target_cue_tolerance_m_ <= 0.0 || cue_waypoint_count_ == 0U)
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
  const auto& grid = world.spatial.inclusion_grid;
  const auto add = [&](LLECandidateSource source, domain::Point2D start,
                       domain::Point2D point, std::uint64_t stable_id = 0U,
                       bool require_valid_cue = true) {
    if (require_valid_cue &&
        (domain::distance(start, point).meters() < minimum_cue_length_m_ ||
         domain::distance(point, target).meters() >
             target_cue_tolerance_m_))
      return;
    if (source != LLECandidateSource::UnfinishedHle &&
        std::any_of(ranked_candidates_.begin(), ranked_candidates_.end(),
                    [&](const auto& candidate) {
                      return candidate.source == source &&
                             domain::distance(candidate.target, point).meters() <=
                                 0.25;
                    }))
      return;
    const double relevance = -domain::distance(point, target).meters();
    ranked_candidates_.push_back(
        {stable_id == 0U ? next_candidate_id_++ : stable_id, source, start,
         point, relevance, require_valid_cue});
  };
  for (const auto& cue : world.spatial.unfinished_hle_candidates)
    add(LLECandidateSource::UnfinishedHle, cue.start, cue.target, cue.id);

  std::vector<LLECandidate> fallback_rays;
  const auto addView = [&](const domain::Pose2D& pose,
                           const domain::LaserObservation& laser) {
    for (std::size_t beam = 0U; beam < laser.ranges_m.size(); ++beam) {
      const double range = std::isfinite(laser.ranges_m[beam])
                               ? laser.ranges_m[beam]
                               : laser.maximum_range.meters();
      if (!std::isfinite(range) || range < laser.minimum_range.meters())
        continue;
      const double angle = pose.heading.radians() +
                           laser.angle_min.radians() +
                           static_cast<double>(beam) *
                               laser.angle_increment.radians();
      const domain::Point2D endpoint{
          pose.position.x_m + range * std::cos(angle),
          pose.position.y_m + range * std::sin(angle)};
      const bool valid_cue =
          domain::distance(pose.position, endpoint).meters() >=
              minimum_cue_length_m_ &&
          domain::distance(endpoint, target).meters() <=
              target_cue_tolerance_m_;
      add(LLECandidateSource::CurrentTargetObservation, pose.position,
          endpoint);
      const auto inclusion_index = gridIndex(grid, endpoint);
      const bool uncovered =
          !inclusion_index || grid.cells[*inclusion_index] == 0U;
      if (!valid_cue && uncovered)
        fallback_rays.push_back(
            {next_candidate_id_++,
             LLECandidateSource::CurrentTargetObservation, pose.position,
             endpoint, -domain::distance(endpoint, target).meters(), false});
    }
  };
  addView(world.robot.pose, *world.robot.laser);
  for (const auto& entry : world.navigation_history.entries())
    addView(entry.pose, entry.laser);
  for (const auto& region : world.spatial.learned_regions)
    add(LLECandidateSource::RegionVisibility, world.robot.pose.position,
        region.center);
  std::vector<LLECandidate> fallback_gaps;
  if (grid.columns > 0U && grid.rows > 0U) {
    for (std::size_t index = 0U; index < grid.cells.size(); ++index) {
      if (grid.cells[index] != 0U) continue;
      const auto row = index / grid.columns;
      const auto column = index % grid.columns;
      const domain::Point2D point{
          grid.origin.x_m + (static_cast<double>(column) + 0.5) *
                                grid.resolution_m,
          grid.origin.y_m + (static_cast<double>(row) + 0.5) *
                                grid.resolution_m};
      fallback_gaps.push_back(
          {next_candidate_id_++, LLECandidateSource::InclusionGap,
           world.robot.pose.position, point,
           -domain::distance(point, target).meters(), false});
    }
  }
  if (ranked_candidates_.empty() && !fallback_rays.empty()) {
    const auto closest = std::max_element(
        fallback_rays.begin(), fallback_rays.end(),
        [](const auto& left, const auto& right) {
          return left.target_relevance < right.target_relevance;
        });
    ranked_candidates_.push_back(*closest);
  }
  if (ranked_candidates_.empty() && !fallback_gaps.empty()) {
    const auto closest = std::max_element(
        fallback_gaps.begin(), fallback_gaps.end(),
        [](const auto& left, const auto& right) {
          return left.target_relevance < right.target_relevance;
        });
    ranked_candidates_.push_back(*closest);
  }
}

void LowLevelExplorer::installCandidateWaypoints(
    const LLECandidate& candidate) {
  cue_waypoints_.clear();
  cue_waypoints_.reserve(cue_waypoint_count_);
  for (std::size_t index = 1U; index <= cue_waypoint_count_; ++index) {
    const double fraction = static_cast<double>(index) /
                            static_cast<double>(cue_waypoint_count_);
    cue_waypoints_.push_back(
        {candidate.start.x_m +
             (candidate.target.x_m - candidate.start.x_m) * fraction,
         candidate.start.y_m +
             (candidate.target.y_m - candidate.start.y_m) * fraction});
  }
  waypoint_cursor_ = 0U;
  lost_waypoint_cycles_ = 0U;
}

bool LowLevelExplorer::appendCurrentViewCandidates(
    const domain::WorldModel& world) {
  const auto& laser = *world.robot.laser;
  const auto target = world.mission.active()->target;
  bool appended = false;
  for (std::size_t beam = 0U; beam < laser.ranges_m.size(); ++beam) {
    const double range = std::isfinite(laser.ranges_m[beam])
                             ? laser.ranges_m[beam]
                             : laser.maximum_range.meters();
    if (range < minimum_cue_length_m_) continue;
    const double angle = world.robot.pose.heading.radians() +
                         laser.angle_min.radians() +
                         static_cast<double>(beam) *
                             laser.angle_increment.radians();
    const domain::Point2D endpoint{
        world.robot.pose.position.x_m + range * std::cos(angle),
        world.robot.pose.position.y_m + range * std::sin(angle)};
    if (domain::distance(endpoint, target).meters() >
        target_cue_tolerance_m_)
      continue;
    const bool duplicate = std::any_of(
        ranked_candidates_.begin(), ranked_candidates_.end(),
        [&](const auto& candidate) {
          return candidate.source ==
                     LLECandidateSource::CurrentTargetObservation &&
                 domain::distance(candidate.target, endpoint).meters() <= 0.25;
        });
    if (duplicate) continue;
    ranked_candidates_.push_back(
        {next_candidate_id_++,
         LLECandidateSource::CurrentTargetObservation,
         world.robot.pose.position, endpoint,
         -domain::distance(endpoint, target).meters(), true});
    appended = true;
  }
  return appended;
}

std::size_t LowLevelExplorer::includedCellCount(
    const domain::WorldModel& world) const noexcept {
  return static_cast<std::size_t>(std::count_if(
      world.spatial.inclusion_grid.cells.begin(),
      world.spatial.inclusion_grid.cells.end(),
      [](std::uint32_t value) { return value != 0U; }));
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
    ranked_candidates_.clear();
    candidate_cursor_ = 0U;
    cue_waypoints_.clear();
    waypoint_cursor_ = 0U;
    lost_waypoint_cycles_ = 0U;
  }
  if (state_ != LowLevelExplorationState::DetectMissingGuidance) {
    const auto included = includedCellCount(context.world);
    const bool inclusion_growth =
        included > initial_included_cells_ &&
        (initial_included_cells_ == 0U ||
         static_cast<double>(included) >=
             1.1 * static_cast<double>(initial_included_cells_));
    const bool connectivity_revision_changed = std::any_of(
        source_revisions_.begin(), source_revisions_.end(),
        [&](const auto& entry) {
          return context.world.spatial.revisionOf(entry.first) != entry.second;
        });
    const bool connectivity =
        connectivity_revision_changed &&
        (!context.world.spatial.skeleton_nodes.empty() ||
         !context.world.spatial.highways.nodes.empty() ||
         !context.world.spatial.highways.graph.vertices.empty());
    if (inclusion_growth || connectivity)
      return complete(ReactiveCompletionReason::NewPlanAvailable,
                      "LLE expanded inclusion/connectivity; request Tier-2 replanning",
                      ReactiveStatus::RequestReplan);
  }
  if (state_ == LowLevelExplorationState::DetectMissingGuidance) {
    if (!evaluateTrigger(context).triggered) return {};
    mission_id_ = context.world.mission.active()->id;
    source_revisions_ = {
        {domain::ModelDependency::Inclusion,
         context.world.spatial.revisionOf(domain::ModelDependency::Inclusion)},
        {domain::ModelDependency::Skeleton,
         context.world.spatial.revisionOf(domain::ModelDependency::Skeleton)},
        {domain::ModelDependency::HighwayGraph,
         context.world.spatial.revisionOf(
             domain::ModelDependency::HighwayGraph)}};
    initial_included_cells_ = includedCellCount(context.world);
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
  if (state_ == LowLevelExplorationState::PursueCandidate) {
    const bool current_is_cue =
        ranked_candidates_[candidate_cursor_].validated_cue;
    if (appendCurrentViewCandidates(context.world)) {
      const auto sort_begin = candidate_cursor_ + (current_is_cue ? 1U : 0U);
      std::stable_sort(
          ranked_candidates_.begin() +
              static_cast<std::ptrdiff_t>(sort_begin),
          ranked_candidates_.end(), [](const auto& left, const auto& right) {
            return left.target_relevance > right.target_relevance ||
                   (left.target_relevance == right.target_relevance &&
                    left.id < right.id);
          });
      if (!current_is_cue)
        installCandidateWaypoints(ranked_candidates_[candidate_cursor_]);
    }
  }
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
    installCandidateWaypoints(candidate);
    state_ = LowLevelExplorationState::PursueCandidate;
  }
  if (state_ == LowLevelExplorationState::PursueCandidate) {
    while (waypoint_cursor_ < cue_waypoints_.size() &&
           domain::distance(context.world.robot.pose.position,
                            cue_waypoints_[waypoint_cursor_]).meters() <=
               progress_threshold_m_)
      ++waypoint_cursor_;
    if (waypoint_cursor_ >= cue_waypoints_.size()) {
      ++candidate_cursor_;
      state_ = LowLevelExplorationState::PlanToCandidateStart;
      if (candidate_cursor_ >= ranked_candidates_.size())
        return complete(ReactiveCompletionReason::CandidateExhausted,
                        "all LLE candidates were exhausted");
      return update(context);
    }
    const bool next_visible = sensed(
        context.world.robot.pose, *context.world.robot.laser,
        cue_waypoints_[waypoint_cursor_]);
    const bool following_visible =
        waypoint_cursor_ + 1U < cue_waypoints_.size() &&
        sensed(context.world.robot.pose, *context.world.robot.laser,
               cue_waypoints_[waypoint_cursor_ + 1U]);
    if (!next_visible && !following_visible) {
      if (++lost_waypoint_cycles_ >= 3U) {
        ++candidate_cursor_;
        state_ = LowLevelExplorationState::PlanToCandidateStart;
        cue_waypoints_.clear();
        if (candidate_cursor_ >= ranked_candidates_.size())
          return complete(ReactiveCompletionReason::CandidateExhausted,
                          "LLE lost every remaining cue");
        return update(context);
      }
    } else {
      lost_waypoint_cycles_ = 0U;
    }
    return {ReactiveStatus::Action,
            actionToward(context.world.robot.pose,
                         cue_waypoints_[waypoint_cursor_],
                         *context.action_space),
            LowLevelExplorationState::PursueCandidate,
            ReactiveCompletionReason::None, candidate.id,
            "pursue candidate ray"};
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
  cue_waypoints_.clear();
  waypoint_cursor_ = 0U;
  lost_waypoint_cycles_ = 0U;
}

ReactiveResult LowLevelExplorer::evaluate(const ReactiveRequest& request) {
  decision::DecisionContext context{request.world, &request.action_space};
  if (state_ == LowLevelExplorationState::DetectMissingGuidance &&
      !evaluateTrigger(context).triggered)
    return {};
  return resultFrom(name(), update(context));
}

}  // namespace semaforr::planning
