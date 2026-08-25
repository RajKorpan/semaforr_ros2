/**
 * @file social_navigation_advisor.cpp
 * @brief Social navigation advisor responsibilities.
 *
 * @details This file implements social navigation advisor behavior for tiered
 * decision making and action arbitration. It records the declarations,
 * settings, fixtures, or guidance needed by that responsibility. Its
 * package-relative location is
 * `src/decision/advisors/social/social_navigation_advisor.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <semaforr/decision/advisors/social/social_navigation_advisor.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace semaforr::decision {
namespace {

/**
 * @brief Validates positive for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 * - @p name: Supplies name input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void validatePositive(double value, std::string_view name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and positive");
  }
}

/**
 * @brief Performs the interpolate operation for this subsystem.
 *
 * Arguments:
 * - @p from: Supplies from input to the operation.
 * - @p to: Supplies to input to the operation.
 * - @p fraction: Supplies fraction input to the operation.
 *
 * Returns:
 * - `domain::Point2D` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::Point2D interpolate(const domain::Point2D& from,
                            const domain::Point2D& to, double fraction) {
  return {from.x_m + (to.x_m - from.x_m) * fraction,
          from.y_m + (to.y_m - from.y_m) * fraction};
}

/**
 * @brief Performs the euclidean distance operation for this subsystem.
 *
 * Arguments:
 * - @p left: Supplies left input to the operation.
 * - @p right: Supplies right input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double euclideanDistance(const domain::Point2D& left,
                         const domain::Point2D& right) {
  return std::hypot(left.x_m - right.x_m, left.y_m - right.y_m);
}

/**
 * @brief Performs the pedestrian at operation for this subsystem.
 *
 * Arguments:
 * - @p pedestrian: Supplies pedestrian input to the operation.
 * - @p observed_at: Supplies observed at input to the operation.
 * - @p seconds: Supplies seconds input to the operation.
 *
 * Returns:
 * - `domain::Point2D` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::Point2D pedestrianAt(const domain::PedestrianObservation& pedestrian,
                             domain::SocialTimestamp observed_at,
                             double seconds) {
  const auto target =
      observed_at + std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(seconds));
  if (!pedestrian.predicted_trajectory.empty()) {
    domain::Point2D previous_position = pedestrian.position;
    domain::SocialTimestamp previous_time = observed_at;
    for (const auto& prediction : pedestrian.predicted_trajectory) {
      if (target <= prediction.predicted_at) {
        const double span = std::chrono::duration<double>(
                                prediction.predicted_at - previous_time)
                                .count();
        const double elapsed =
            std::chrono::duration<double>(target - previous_time).count();
        return interpolate(previous_position, prediction.position,
                           std::clamp(elapsed / span, 0.0, 1.0));
      }
      previous_position = prediction.position;
      previous_time = prediction.predicted_at;
    }
    return pedestrian.predicted_trajectory.back().position;
  }
  return {pedestrian.position.x_m + pedestrian.velocity_mps.x_m * seconds,
          pedestrian.position.y_m + pedestrian.velocity_mps.y_m * seconds};
}

}  // namespace

/**
 * @brief Performs the social navigation advisor operation for this
 * subsystem.
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
SocialNavigationAdvisor::SocialNavigationAdvisor(
    SocialAdvisorConfiguration configuration)
    : configuration_(std::move(configuration)) {
  if (configuration_.move_distances_m.empty() ||
      configuration_.rotation_angles_rad.empty()) {
    throw std::invalid_argument(
        "social advisor requires configured action magnitudes");
  }
  if (configuration_.advisor_name.empty()) {
    throw std::invalid_argument("social advisor name must not be empty");
  }
  for (double value : configuration_.move_distances_m) {
    validatePositive(value, "social move distance");
  }
  for (double value : configuration_.rotation_angles_rad) {
    validatePositive(value, "social rotation angle");
  }
  if (configuration_.maximum_age < std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("social maximum age must be non-negative");
  }
  if (!std::isfinite(configuration_.minimum_confidence) ||
      configuration_.minimum_confidence < 0.0 ||
      configuration_.minimum_confidence > 1.0) {
    throw std::invalid_argument(
        "social minimum confidence must be within [0, 1]");
  }
  validatePositive(configuration_.prediction_horizon_s,
                   "social prediction horizon");
  validatePositive(configuration_.personal_space_m, "personal space");
  validatePositive(configuration_.collision_distance_m, "collision distance");
  if (configuration_.collision_distance_m > configuration_.personal_space_m) {
    throw std::invalid_argument(
        "collision distance must not exceed personal space");
  }
  if (!std::isfinite(configuration_.weight)) {
    throw std::invalid_argument("social advisor weight must be finite");
  }
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
AdvisorEvaluation SocialNavigationAdvisor::evaluate(
    const DecisionContext& context,
    std::span<const domain::Action> candidates) const {
  AdvisorEvaluation evaluation;
  evaluation.weight = configuration_.weight;
  evaluation.explanation =
      "predicted interpersonal distance, crossing, following, and flow risk";
  const auto& current = context.world.crowd.current();
  if (!current || !current->usable(configuration_.maximum_age,
                                   configuration_.minimum_confidence)) {
    evaluation.explanation =
        "social data absent, invalid, or stale; advisor disabled";
    return evaluation;
  }
  evaluation.model_revision_used =
      static_cast<std::size_t>(current->observed_at.count());

  evaluation.participated = true;
  evaluation.scores.reserve(candidates.size());
  for (const auto& action : candidates) {
    evaluation.scores.push_back(
        {action, score(context.world, *current, action)});
  }
  return evaluation;
}

/**
 * @brief Performs the score operation for this subsystem.
 *
 * Arguments:
 * - @p world: Supplies world input to the operation.
 * - @p crowd: Supplies crowd input to the operation.
 * - @p action: Supplies action input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double SocialNavigationAdvisor::score(const domain::WorldModel& world,
                                      const domain::CrowdObservation& crowd,
                                      const domain::Action& action) const {
  const domain::Point2D start = world.robot.pose.position;
  domain::Point2D end = start;
  double robot_speed = 0.0;
  if (action.type() == domain::ActionType::Forward) {
    const std::size_t index = action.magnitude_index();
    if (index == 0U || index > configuration_.move_distances_m.size()) {
      throw std::out_of_range("social advisor received invalid forward action");
    }
    const double travel = configuration_.move_distances_m[index - 1U];
    end = {start.x_m + travel * std::cos(world.robot.pose.heading.radians()),
           start.y_m + travel * std::sin(world.robot.pose.heading.radians())};
    robot_speed = travel / configuration_.prediction_horizon_s;
  }

  double risk = 0.0;
  constexpr int samples = 20;
  for (const auto& pedestrian : crowd.pedestrians) {
    if (pedestrian.confidence < configuration_.minimum_confidence) {
      continue;
    }
    const double observation_age_s =
        std::chrono::duration<double>(crowd.data_age).count();
    const domain::Point2D current_person = pedestrianAt(
        pedestrian, crowd.observed_at, observation_age_s);
    double minimum_separation = euclideanDistance(start, current_person);
    for (int sample = 1; sample <= samples; ++sample) {
      const double fraction =
          static_cast<double>(sample) / static_cast<double>(samples);
      const domain::Point2D robot = interpolate(start, end, fraction);
      const domain::Point2D person =
          pedestrianAt(pedestrian, crowd.observed_at,
                       observation_age_s +
                           configuration_.prediction_horizon_s * fraction);
      minimum_separation =
          std::min(minimum_separation, euclideanDistance(robot, person));
    }

    if (minimum_separation < configuration_.personal_space_m) {
      risk += pedestrian.confidence *
              (configuration_.personal_space_m - minimum_separation) /
              configuration_.personal_space_m;
    }
    if (minimum_separation < configuration_.collision_distance_m) {
      risk += 10.0 * pedestrian.confidence *
              (configuration_.collision_distance_m - minimum_separation) /
              configuration_.collision_distance_m;
    }

    if (robot_speed > 0.0) {
      const double heading_x = std::cos(world.robot.pose.heading.radians());
      const double heading_y = std::sin(world.robot.pose.heading.radians());
      const double along = pedestrian.velocity_mps.x_m * heading_x +
                           pedestrian.velocity_mps.y_m * heading_y;
      const double lateral = std::fabs(pedestrian.velocity_mps.x_m * heading_y -
                                       pedestrian.velocity_mps.y_m * heading_x);
      const double ahead = (current_person.x_m - start.x_m) * heading_x +
                           (current_person.y_m - start.y_m) * heading_y;
      if (ahead > 0.0 && ahead < 3.0 && lateral < 0.75) {
        if (along < 0.0) {
          risk += pedestrian.confidence * (1.0 + std::fabs(along));
        } else if (along < robot_speed) {
          risk += pedestrian.confidence * (robot_speed - along);
        }
      }
    }
  }
  return -risk;
}

}  // namespace semaforr::decision
