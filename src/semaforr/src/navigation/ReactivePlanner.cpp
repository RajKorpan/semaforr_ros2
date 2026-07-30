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

}  // namespace

ReactiveResult Thru::evaluate(const ReactiveRequest& request) const {
  const auto target = waypoint(request.world);
  if (!target) return {};
  const double error = headingError(request.world.robot.pose, *target);
  if (std::abs(error) > 0.35) return {};
  return {ReactiveStatus::Action,
          domain::Action(domain::ActionType::Forward, 1U),
          std::string(name()), "waypoint lies through the current opening"};
}

ReactiveResult Behind::evaluate(const ReactiveRequest& request) const {
  const auto target = waypoint(request.world);
  if (!target) return {};
  const double error = headingError(request.world.robot.pose, *target);
  if (std::abs(error) < 2.35) return {};
  return {ReactiveStatus::Action, turn(error, request.action_space),
          std::string(name()), "active waypoint is behind the robot"};
}

ReactiveResult Out::evaluate(const ReactiveRequest& request) const {
  if (!request.world.recovery.confined) return {};
  const auto& grid = request.world.spatial.inclusion_grid;
  if (grid.cells.empty())
    return {ReactiveStatus::Action,
            domain::Action(domain::ActionType::TurnLeft, 1U),
            std::string(name()), "survey while leaving confinement"};
  const auto least = std::min_element(grid.cells.begin(), grid.cells.end());
  const std::size_t index =
      static_cast<std::size_t>(least - grid.cells.begin());
  const std::size_t robot_column =
      grid.columns == 0U ? 0U : grid.columns / 2U;
  const std::size_t target_column =
      grid.columns == 0U ? 0U : index % grid.columns;
  if (target_column == robot_column)
    return {ReactiveStatus::Action,
            domain::Action(domain::ActionType::Forward, 1U),
            std::string(name()), "move toward least-included space"};
  return {ReactiveStatus::Action,
          domain::Action(target_column < robot_column
                             ? domain::ActionType::TurnRight
                             : domain::ActionType::TurnLeft,
                         1U),
          std::string(name()), "turn toward least-included space"};
}

ReactivePlannerCoordinator::ReactivePlannerCoordinator()
    : ReactivePlannerCoordinator({"behind", "out", "thru"}) {}

ReactivePlannerCoordinator::ReactivePlannerCoordinator(
    const std::vector<std::string>& enabled_planners) {
  for (const auto& name : enabled_planners) {
    if (name == "thru")
      add(std::make_unique<Thru>());
    else if (name == "behind")
      add(std::make_unique<Behind>());
    else if (name == "out")
      add(std::make_unique<Out>());
    else if (name != "low_level_exploration")
      throw std::invalid_argument("unknown reactive planner '" + name + "'");
  }
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
    const ReactiveRequest& request) const {
  for (const auto& planner : planners_) {
    ReactiveResult result = planner->evaluate(request);
    if (result.status != ReactiveStatus::NotApplicable) return result;
  }
  return {};
}

LowLevelExplorer::LowLevelExplorer(std::size_t history_window,
                                   double progress_threshold_m)
    : history_window_(history_window),
      progress_threshold_m_(progress_threshold_m) {
  if (history_window_ < 2U || !(progress_threshold_m_ > 0.0))
    throw std::invalid_argument("invalid LLE progress configuration");
}

ReactiveResult LowLevelExplorer::evaluate(
    const ReactiveRequest& request) const {
  const auto& history = request.world.navigation_history.entries();
  if (!request.world.mission.active())
    return {};
  const bool only_direct_guidance =
      request.world.mission.active()->plan.size() <= 1U;
  const bool lacks_connectivity =
      request.world.spatial.skeleton_nodes.empty() &&
      request.world.spatial.highways.nodes.empty();
  if (only_direct_guidance && lacks_connectivity &&
      (!last_missing_knowledge_revision_ ||
       *last_missing_knowledge_revision_ != request.world.spatial.revision)) {
    last_missing_knowledge_revision_ = request.world.spatial.revision;
    return {ReactiveStatus::RequestReplan, std::nullopt, "LLE",
            "target-directed planning lacks learned connectivity"};
  }
  if (request.world.mission.decisions_for_active() < history_window_) return {};
  if (history.size() < history_window_) return {};
  const auto first = history.end() -
                     static_cast<std::ptrdiff_t>(history_window_);
  const double displacement =
      domain::distance(first->pose.position, history.back().pose.position)
          .meters();
  const bool attempted_motion =
      std::any_of(first, history.end(), [](const auto& entry) {
        return entry.action.type() == domain::ActionType::Forward;
      });
  if (attempted_motion && displacement < progress_threshold_m_)
    return {ReactiveStatus::RequestReplan, std::nullopt, "LLE",
            "insufficient progress; request Tier-2 replan"};
  return {};
}

}  // namespace semaforr::planning
