#include <cmath>
#include <semaforr/decision/learned_crowd_advisor.hpp>
#include <stdexcept>
#include <utility>

namespace semaforr::decision {

LearnedCrowdAdvisor::LearnedCrowdAdvisor(
    LearnedCrowdAdvisorConfiguration configuration)
    : configuration_(std::move(configuration)) {
  if (configuration_.advisor_name.empty()) {
    throw std::invalid_argument("learned crowd advisor name must not be empty");
  }
  if (!std::isfinite(configuration_.weight) ||
      !std::isfinite(configuration_.minimum_cell_confidence) ||
      configuration_.minimum_cell_confidence < 0.0 ||
      configuration_.minimum_cell_confidence > 1.0) {
    throw std::invalid_argument(
        "learned crowd advisor weight and confidence must be finite and valid");
  }
}

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

AdvisorEvaluation LearnedCrowdAdvisor::evaluate(
    const DecisionContext& context,
    std::span<const domain::Action> candidates) const {
  AdvisorEvaluation evaluation;
  evaluation.model_revision_used = context.world.crowd.learned().version;
  evaluation.weight = configuration_.weight;
  evaluation.explanation = "shared visibility-normalized learned crowd field";
  const bool live_people = context.world.crowd.current() &&
                           !context.world.crowd.current()->pedestrians.empty();
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
        score = -context.world.crowd.navigationRiskAt(position);
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
