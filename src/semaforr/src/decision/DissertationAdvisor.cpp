#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/decision/dissertation_advisor.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <stdexcept>

namespace semaforr::decision {
namespace {

double nearest(domain::Point2D point,
               const std::vector<domain::Point2D>& values) {
  double result = std::numeric_limits<double>::infinity();
  for (const auto& value : values)
    result = std::min(result, domain::distance(point, value).meters());
  return std::isfinite(result) ? result : 0.0;
}

domain::Point2D center(const domain::Segment2D& segment) {
  return {(segment.start.x_m + segment.end.x_m) * 0.5,
          (segment.start.y_m + segment.end.y_m) * 0.5};
}

double targetProgress(const domain::WorldModel& world,
                      const domain::Pose2D& expected) {
  if (!world.mission.active()) return 0.0;
  const auto target = world.mission.active()->waypoint().value_or(
      world.mission.active()->target);
  return domain::distance(world.robot.pose.position, target).meters() -
         domain::distance(expected.position, target).meters();
}

void normalize(std::vector<ActionScore>& scores,
               ScoreNormalization normalization) {
  if (scores.empty() || normalization == ScoreNormalization::None) return;
  const auto [minimum, maximum] = std::minmax_element(
      scores.begin(), scores.end(),
      [](const auto& left, const auto& right) {
        return left.raw_score < right.raw_score;
      });
  const double span = maximum->raw_score - minimum->raw_score;
  if (span <= 1.0e-12) {
    for (auto& score : scores) score.raw_score = 0.0;
    return;
  }
  for (auto& score : scores) {
    const double unit = (score.raw_score - minimum->raw_score) / span;
    score.raw_score =
        normalization == ScoreNormalization::SignedUnit ? 2.0 * unit - 1.0
                                                        : unit;
  }
}

}  // namespace

DissertationAdvisor::DissertationAdvisor(
    DissertationAdvisorConfiguration configuration)
    : configuration_(std::move(configuration)) {
  if (configuration_.name.empty() || !std::isfinite(configuration_.weight) ||
      configuration_.weight < 0.0)
    throw std::invalid_argument("invalid dissertation advisor configuration");
}

std::vector<std::string_view> DissertationAdvisor::dependencies() const {
  using O = DissertationAdvisorObjective;
  switch (configuration_.objective) {
    case O::Novelty:
    case O::Curiosity: return {"navigation_history"};
    case O::ElbowRoom:
    case O::GoAround:
    case O::Enfilade:
    case O::VisualScan: return {"laser"};
    case O::Convey: return {"conveyors"};
    case O::Enter: return {"doors"};
    case O::Exit:
    case O::Access:
    case O::Stay: return {"regions"};
    case O::Trailer: return {"trails"};
    case O::Unlikely: return {"barriers"};
    case O::Crossroads: return {"highways"};
    case O::Follow: return {"hallways"};
    case O::SpatialLearner: return {"inclusion_grid"};
    case O::BigStep:
    case O::Greedy:
    case O::LeastAngle: return {"active_target"};
  }
  return {};
}

AdvisorMetadata DissertationAdvisor::metadata() const {
  using A = domain::ActionType;
  using O = DissertationAdvisorObjective;
  std::vector<A> actions{A::Pause, A::Forward, A::TurnLeft, A::TurnRight};
  if (configuration_.objective == O::BigStep)
    actions = {A::Forward};
  else if (configuration_.objective == O::VisualScan)
    actions = {A::TurnLeft, A::TurnRight};
  const bool target_free =
      configuration_.objective == O::ElbowRoom ||
      configuration_.objective == O::Novelty ||
      configuration_.objective == O::Curiosity ||
      configuration_.objective == O::Enfilade ||
      configuration_.objective == O::VisualScan ||
      configuration_.objective == O::SpatialLearner;
  return {dependencies(), std::move(actions), target_free,
          ScoreNormalization::SignedUnit,
          "dissertation Tier-3 normalized action preference"};
}

bool DissertationAdvisor::accepts(domain::ActionType type) const noexcept {
  const auto actions = metadata().scored_action_types;
  return std::find(actions.begin(), actions.end(), type) != actions.end();
}

double DissertationAdvisor::score(const domain::WorldModel& world,
                                  const domain::Action& action) const {
  using O = DissertationAdvisorObjective;
  const auto expected = domain::expectedPoseAfterAction(
      world.robot.pose, action, configuration_.action_space);
  const double progress = targetProgress(world, expected);
  switch (configuration_.objective) {
    case O::BigStep:
      return action.type() == domain::ActionType::Forward
                 ? static_cast<double>(action.magnitude_index())
                 : 0.0;
    case O::Greedy: return progress;
    case O::LeastAngle:
      if (!world.mission.active()) return 0.0;
      return -std::abs(domain::Angle::normalize(
          std::atan2(world.mission.active()->target.y_m -
                         expected.position.y_m,
                     world.mission.active()->target.x_m -
                         expected.position.x_m) -
          expected.heading.radians()));
    case O::ElbowRoom:
    case O::Enfilade:
      if (!world.robot.laser || world.robot.laser->ranges_m.empty()) return 0.0;
      return *std::max_element(world.robot.laser->ranges_m.begin(),
                               world.robot.laser->ranges_m.end()) +
             (action.type() == domain::ActionType::Forward ? 0.1 : 0.0);
    case O::GoAround:
      if (!world.robot.laser || world.robot.laser->ranges_m.empty()) return 0.0;
      return action.type() == domain::ActionType::TurnLeft ||
                     action.type() == domain::ActionType::TurnRight
                 ? 1.0
                 : *std::min_element(world.robot.laser->ranges_m.begin(),
                                     world.robot.laser->ranges_m.end());
    case O::VisualScan:
      return action.type() == domain::ActionType::TurnLeft ||
                     action.type() == domain::ActionType::TurnRight
                 ? static_cast<double>(action.magnitude_index())
                 : 0.0;
    case O::Novelty:
    case O::Curiosity: {
      std::vector<domain::Point2D> points;
      for (const auto& entry : world.navigation_history.entries())
        points.push_back(entry.pose.position);
      return nearest(expected.position, points) +
             (configuration_.objective == O::Curiosity ? 0.25 * progress
                                                       : 0.0);
    }
    case O::Convey: {
      std::vector<domain::Point2D> points;
      for (const auto& flow : world.spatial.conveyor_flows)
        points.push_back(flow.end);
      return -nearest(expected.position, points);
    }
    case O::Enter: {
      std::vector<domain::Point2D> points;
      for (const auto& door : world.spatial.doorways)
        points.push_back(center(door));
      return -nearest(expected.position, points);
    }
    case O::Exit: {
      double score = 0.0;
      for (const auto& region : world.spatial.learned_regions)
        score = std::max(score,
                         domain::distance(expected.position, region.center)
                             .meters());
      return score;
    }
    case O::Trailer: {
      std::vector<domain::Point2D> points;
      for (const auto& trail : world.spatial.trails)
        points.insert(points.end(), trail.begin(), trail.end());
      return -nearest(expected.position, points);
    }
    case O::Unlikely: {
      std::vector<domain::Point2D> points;
      for (const auto& barrier : world.spatial.barriers)
        points.push_back(center(barrier));
      return nearest(expected.position, points);
    }
    case O::Access: {
      std::vector<domain::Point2D> points;
      for (const auto& region : world.spatial.learned_regions)
        points.push_back(region.center);
      return -nearest(expected.position, points) + progress;
    }
    case O::Crossroads: {
      std::vector<domain::Point2D> points;
      for (const auto& intersection :
           world.spatial.highways.graph.vertices)
        points.push_back(intersection.position);
      return -nearest(expected.position, points);
    }
    case O::Follow: {
      std::vector<domain::Point2D> points;
      for (const auto& hallway : world.spatial.hallways)
        points.push_back(center(hallway));
      return -nearest(expected.position, points);
    }
    case O::SpatialLearner: {
      const auto& grid = world.spatial.inclusion_grid;
      if (grid.cells.empty() || grid.columns == 0U ||
          grid.resolution_m <= 0.0)
        return 0.0;
      const int column = static_cast<int>(std::floor(
          (expected.position.x_m - grid.origin.x_m) / grid.resolution_m));
      const int row = static_cast<int>(std::floor(
          (expected.position.y_m - grid.origin.y_m) / grid.resolution_m));
      if (row < 0 || column < 0 ||
          static_cast<std::size_t>(row) >= grid.rows ||
          static_cast<std::size_t>(column) >= grid.columns)
        return 1.0;
      return -static_cast<double>(
          grid.cells[static_cast<std::size_t>(row) * grid.columns +
                     static_cast<std::size_t>(column)]);
    }
    case O::Stay: {
      for (const auto& region : world.spatial.learned_regions)
        if (domain::distance(expected.position, region.center).meters() <=
            region.radius.meters())
          return 1.0;
      return 0.0;
    }
  }
  return 0.0;
}

AdvisorEvaluation DissertationAdvisor::evaluate(
    const DecisionContext& context,
    std::span<const domain::Action> candidates) const {
  AdvisorEvaluation result;
  const auto contract = metadata();
  if (!contract.participates_without_target &&
      !context.world.mission.active())
    return result;
  result.weight = configuration_.weight;
  result.explanation = std::string(contract.rationale);
  result.model_revision_used = context.world.spatial.revision;
  for (const auto& action : candidates)
    if (accepts(action.type()))
      result.scores.push_back({action, score(context.world, action)});
  normalize(result.scores, contract.normalization);
  result.participated = !result.scores.empty();
  return result;
}

}  // namespace semaforr::decision
