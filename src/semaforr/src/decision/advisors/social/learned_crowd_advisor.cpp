/**
 * @file learned_crowd_advisor.cpp
 * @brief Learned crowd advisor responsibilities.
 *
 * @details This file implements learned crowd advisor behavior for tiered
 * decision making and action arbitration. It records the declarations,
 * settings, fixtures, or guidance needed by that responsibility. Its
 * package-relative location is
 * `src/decision/advisors/social/learned_crowd_advisor.cpp`.
 */
#include <cmath>
#include <semaforr/decision/advisors/social/learned_crowd_advisor.hpp>
#include <stdexcept>
#include <utility>

namespace semaforr::decision {
namespace {

/**
 * @brief Validates positive magnitudes for this subsystem.
 *
 * Arguments:
 * - @p values: Supplies values input to the operation.
 * - @p description: Supplies description input to the operation.
 * - @p allow_empty: Supplies allow empty input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void validatePositiveMagnitudes(const std::vector<double>& values,
                                std::string_view description,
                                bool allow_empty = false) {
  if (values.empty() && !allow_empty) {
    throw std::invalid_argument(std::string(description) +
                                " must not be empty");
  }
  for (const double value : values) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(std::string(description) +
                                  " must be finite and positive");
    }
  }
}

}  // namespace

/**
 * @brief Performs the learned crowd advisor operation for this subsystem.
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
LearnedCrowdAdvisor::LearnedCrowdAdvisor(
    LearnedCrowdAdvisorConfiguration configuration)
    : configuration_(std::move(configuration)) {
  if (configuration_.advisor_name.empty()) {
    throw std::invalid_argument("learned crowd advisor name must not be empty");
  }
  validatePositiveMagnitudes(configuration_.move_distances_m,
                             "learned crowd move distances");
  validatePositiveMagnitudes(configuration_.rotation_angles_rad,
                             "learned crowd rotation angles", true);
  if (!std::isfinite(configuration_.weight) ||
      !std::isfinite(configuration_.minimum_cell_confidence) ||
      configuration_.minimum_cell_confidence < 0.0 ||
      configuration_.minimum_cell_confidence > 1.0 ||
      configuration_.maximum_live_age < std::chrono::nanoseconds::zero() ||
      !std::isfinite(configuration_.minimum_live_confidence) ||
      configuration_.minimum_live_confidence < 0.0 ||
      configuration_.minimum_live_confidence > 1.0) {
    throw std::invalid_argument(
        "learned crowd advisor weight, age, and confidence must be valid");
  }
}

/**
 * @brief Performs the expected operation for this subsystem.
 *
 * Arguments:
 * - @p world: Supplies world input to the operation.
 * - @p action: Supplies action input to the operation.
 *
 * Returns:
 * - `std::pair<domain::Point2D, domain::Angle>` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::pair<domain::Point2D, domain::Angle> LearnedCrowdAdvisor::expected(
    const domain::WorldModel& world, const domain::Action& action) const {
  domain::Point2D position = world.robot.pose.position;
  domain::Angle heading = world.robot.pose.heading;
  const std::size_t magnitude = action.magnitude_index();
  if (action.type() == domain::ActionType::Forward) {
    if (magnitude == 0U || magnitude > configuration_.move_distances_m.size()) {
      throw std::out_of_range(
          "learned crowd advisor received invalid forward action");
    }
    const double distance = configuration_.move_distances_m[magnitude - 1U];
    position.x_m += distance * std::cos(heading.radians());
    position.y_m += distance * std::sin(heading.radians());
  } else if (action.type() == domain::ActionType::TurnLeft ||
             action.type() == domain::ActionType::TurnRight) {
    if (magnitude == 0U ||
        magnitude > configuration_.rotation_angles_rad.size()) {
      throw std::out_of_range(
          "learned crowd advisor received invalid turn action");
    }
    const double sign =
        action.type() == domain::ActionType::TurnLeft ? 1.0 : -1.0;
    heading = domain::Angle(
        heading.radians() +
        sign * configuration_.rotation_angles_rad[magnitude - 1U]);
  }
  return {position, heading};
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
AdvisorEvaluation LearnedCrowdAdvisor::evaluate(
    const DecisionContext& context,
    std::span<const domain::Action> candidates) const {
  AdvisorEvaluation evaluation;
  evaluation.model_revision_used = context.world.crowd.learned().version;
  evaluation.weight = configuration_.weight;
  evaluation.explanation = "shared visibility-normalized learned crowd field";
  const bool live_people = context.world.crowd.current() &&
                           context.world.crowd.current()->usable(
                               configuration_.maximum_live_age,
                               configuration_.minimum_live_confidence);
  const bool learned = context.world.crowd.learnedAvailable();
  if (!learned &&
      (configuration_.objective != LearnedCrowdObjective::AvoidEncounterRisk ||
       !live_people)) {
    evaluation.explanation =
        "learned crowd field unavailable; advisor disabled";
    return evaluation;
  }

  for (const auto& action : candidates) {
    const auto [position, heading] = expected(context.world, action);
    const auto sample = context.world.crowd.learnedAt(position);
    double score = 0.0;
    switch (configuration_.objective) {
      case LearnedCrowdObjective::AvoidDensity:
        if (!sample || sample->stale ||
            sample->cell.confidence < configuration_.minimum_cell_confidence) {
          continue;
        }
        score = -sample->cell.density;
        break;
      case LearnedCrowdObjective::AvoidEncounterRisk:
        if ((!sample || sample->stale ||
             sample->cell.confidence <
                 configuration_.minimum_cell_confidence) &&
            !live_people) {
          continue;
        }
        score = -std::max(
            context.world.crowd.learnedEncounterRiskAt(position),
            live_people
                ? context.world.crowd.predictiveCollisionRiskAt(position)
                : 0.0);
        evaluation.explanation =
            "maximum of learned encounter and live prediction risk";
        break;
      case LearnedCrowdObjective::PreferFollowingFlow:
        if (!sample || sample->stale ||
            sample->cell.confidence < configuration_.minimum_cell_confidence) {
          continue;
        }
        score = context.world.crowd.flowAlignmentAt(position, heading);
        break;
    }
    evaluation.scores.push_back({action, score});
  }
  evaluation.participated = !evaluation.scores.empty();
  return evaluation;
}

}  // namespace semaforr::decision
