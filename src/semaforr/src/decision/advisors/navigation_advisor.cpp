/**
 * @file navigation_advisor.cpp
 * @brief Navigation advisor responsibilities.
 *
 * @details This file implements navigation advisor behavior for tiered decision
 * making and action arbitration. It records the declarations, settings,
 * fixtures, or guidance needed by that responsibility. Its
 * package-relative location is
 * `src/decision/advisors/navigation_advisor.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/decision/advisors/navigation_advisor.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <stdexcept>
#include <utility>

namespace semaforr::decision {
namespace {

/**
 * @brief Performs the heading error operation for this subsystem.
 *
 * Arguments:
 * - @p pose: Supplies pose input to the operation.
 * - @p target: Supplies target input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double headingError(const domain::Pose2D& pose, domain::Point2D target) {
  const double bearing = std::atan2(target.y_m - pose.position.y_m,
                                    target.x_m - pose.position.x_m);
  return std::abs(domain::Angle::normalize(bearing - pose.heading.radians()));
}

}  // namespace

/**
 * @brief Performs the navigation advisor operation for this subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
NavigationAdvisor::NavigationAdvisor(
    NavigationAdvisorConfiguration configuration)
    : configuration_(std::move(configuration)) {
  if (configuration_.name.empty() || !std::isfinite(configuration_.weight)) {
    throw std::invalid_argument(
        "navigation advisor requires a name and finite weight");
  }
}

/**
 * @brief Performs the accepts operation for this subsystem.
 *
 * Arguments:
 * - @p action: Supplies action input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool NavigationAdvisor::accepts(const domain::Action& action) const noexcept {
  switch (configuration_.selection) {
    case ActionSelection::All:
      return true;
    case ActionSelection::Linear:
      return action.type() == domain::ActionType::Forward ||
             action.type() == domain::ActionType::Pause;
    case ActionSelection::Rotation:
      return action.type() == domain::ActionType::TurnLeft ||
             action.type() == domain::ActionType::TurnRight ||
             action.type() == domain::ActionType::Pause;
  }
  return false;
}

/**
 * @brief Performs the score operation for this subsystem.
 *
 * Arguments:
 * - @p context: Supplies context input to the operation.
 * - @p action: Supplies action input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double NavigationAdvisor::score(const DecisionContext& context,
                                const domain::Action& action) const {
  const auto& world = context.world;
  const domain::Pose2D expected = domain::expectedPoseAfterAction(
      world.robot.pose, action, configuration_.action_space);
  switch (configuration_.objective) {
    case NavigationAdvisorObjective::GoalProgress: {
      if (!world.mission.active()) {
        return 0.0;
      }
      const domain::Point2D target =
          context.active_plan_objective
              ? context.active_plan_objective->target
              : world.mission.active()->waypoint().value_or(
                    world.mission.active()->target);
      const double progress =
          domain::distance(world.robot.pose.position, target).meters() -
          domain::distance(expected.position, target).meters();
      return progress - 0.2 * headingError(expected, target);
    }
    case NavigationAdvisorObjective::Clearance: {
      if (action.type() == domain::ActionType::Forward) {
        return configuration_.action_space.move_distances_m().at(
            action.magnitude_index() - 1U);
      }
      if (!world.robot.laser || world.robot.laser->ranges_m.empty()) {
        return 0.0;
      }
      const auto finite_range = std::max_element(
          world.robot.laser->ranges_m.begin(),
          world.robot.laser->ranges_m.end(), [](double left, double right) {
            const double normalized_left =
                std::isfinite(left) ? left : std::numeric_limits<double>::max();
            const double normalized_right =
                std::isfinite(right) ? right
                                     : std::numeric_limits<double>::max();
            return normalized_left < normalized_right;
          });
      return finite_range == world.robot.laser->ranges_m.end()
                 ? 0.0
                 : std::min(std::isfinite(*finite_range)
                                ? *finite_range
                                : world.robot.laser->maximum_range.meters(),
                            world.robot.laser->maximum_range.meters()) *
                       0.01;
    }
    case NavigationAdvisorObjective::Exploration: {
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto& entry : world.navigation_history.entries()) {
        nearest = std::min(
            nearest,
            domain::distance(expected.position, entry.pose.position).meters());
      }
      return std::isfinite(nearest) ? nearest : 1.0;
    }
  }
  return 0.0;
}

/**
 * @brief Evaluates package content for this subsystem.
 *
 * Arguments:
 * - @p context: Supplies context input to the operation.
 * - @p candidates: Supplies candidates input to the operation.
 *
 * Returns:
 * - `AdvisorEvaluation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
AdvisorEvaluation NavigationAdvisor::evaluate(
    const DecisionContext& context,
    std::span<const domain::Action> candidates) const {
  AdvisorEvaluation result;
  result.weight = configuration_.weight;
  result.explanation = "ROS-independent navigation objective";
  for (const auto& action : candidates) {
    if (accepts(action)) {
      result.scores.push_back({action, score(context, action)});
    }
  }
  result.participated = !result.scores.empty();
  return result;
}

}  // namespace semaforr::decision
