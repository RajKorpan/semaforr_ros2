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

std::vector<domain::Point2D> obstaclePoints(
    const domain::WorldModel& world) {
  std::vector<domain::Point2D> points;
  if (!world.robot.laser) return points;
  const auto& laser = *world.robot.laser;
  points.reserve(laser.ranges_m.size());
  double angle = world.robot.pose.heading.radians() +
                 laser.angle_min.radians();
  for (const double range : laser.ranges_m) {
    if (std::isfinite(range))
      points.push_back(
          {world.robot.pose.position.x_m + range * std::cos(angle),
           world.robot.pose.position.y_m + range * std::sin(angle)});
    angle += laser.angle_increment.radians();
  }
  return points;
}

double forwardClearance(const domain::WorldModel& world, double heading,
                        double corridor_half_width_m = 0.35) {
  if (!world.robot.laser) return std::numeric_limits<double>::infinity();
  double result = std::numeric_limits<double>::infinity();
  for (const auto& point : obstaclePoints(world)) {
    const double dx = point.x_m - world.robot.pose.position.x_m;
    const double dy = point.y_m - world.robot.pose.position.y_m;
    const double longitudinal = dx * std::cos(heading) + dy * std::sin(heading);
    const double lateral = std::abs(-dx * std::sin(heading) +
                                    dy * std::cos(heading));
    if (longitudinal > 0.0 && lateral <= corridor_half_width_m)
      result = std::min(result, longitudinal - corridor_half_width_m);
  }
  if (!std::isfinite(result))
    return world.robot.laser->maximum_range.meters();
  return std::max(0.0, result);
}

domain::Pose2D anticipatedPose(const domain::WorldModel& world,
                               const domain::Action& action,
                               const domain::ActionSpace& action_space) {
  auto result = domain::expectedPoseAfterAction(world.robot.pose, action,
                                                action_space);
  if (action.type() != domain::ActionType::TurnLeft &&
      action.type() != domain::ActionType::TurnRight)
    return result;
  const double distance = std::min(action_space.move_distances_m().back(),
                                   forwardClearance(
                                       world, result.heading.radians()));
  result.position.x_m += distance * std::cos(result.heading.radians());
  result.position.y_m += distance * std::sin(result.heading.radians());
  return result;
}

bool angleInArc(double angle, double center, double width) {
  return std::abs(domain::Angle::normalize(angle - center)) <= width * 0.5;
}

double visualNovelty(const domain::WorldModel& world,
                     const domain::Pose2D& anticipated) {
  if (!world.robot.laser || world.robot.laser->ranges_m.empty()) return 0.0;
  const auto& laser = *world.robot.laser;
  const double width = std::abs(laser.angle_increment.radians()) *
                       static_cast<double>(laser.ranges_m.size() - 1U);
  const double offset = laser.angle_min.radians() + width * 0.5;
  const double candidate_center = anticipated.heading.radians() + offset;
  constexpr std::size_t samples = 21U;
  std::size_t unseen = 0U;
  for (std::size_t sample = 0U; sample < samples; ++sample) {
    const double fraction = static_cast<double>(sample) /
                            static_cast<double>(samples - 1U);
    const double direction =
        candidate_center - width * 0.5 + fraction * width;
    bool seen = angleInArc(direction,
                           world.robot.pose.heading.radians() + offset,
                           width);
    for (const auto& entry : world.navigation_history.entries()) {
      if (seen) break;
      if (domain::distance(entry.pose.position,
                           world.robot.pose.position).meters() > 1.5 ||
          entry.laser.ranges_m.empty())
        continue;
      const double historical_width =
          std::abs(entry.laser.angle_increment.radians()) *
          static_cast<double>(entry.laser.ranges_m.size() - 1U);
      const double historical_center =
          entry.pose.heading.radians() + entry.laser.angle_min.radians() +
          historical_width * 0.5;
      seen = angleInArc(direction, historical_center, historical_width);
    }
    if (!seen) ++unseen;
  }
  return static_cast<double>(unseen) / static_cast<double>(samples);
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
    case O::GoAround: return {"laser"};
    case O::Enfilade: return {"navigation_history"};
    case O::VisualScan: return {"laser", "navigation_history"};
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
  if (configuration_.objective == O::VisualScan)
    actions = {A::TurnLeft, A::TurnRight};
  const bool target_free =
      configuration_.objective == O::ElbowRoom ||
      configuration_.objective == O::Curiosity ||
      configuration_.objective == O::Enfilade ||
      configuration_.objective == O::VisualScan ||
      configuration_.objective == O::SpatialLearner;
  std::string_view rationale =
      "dissertation Tier-3 normalized action preference";
  switch (configuration_.objective) {
    case O::BigStep: rationale = "prefer the longest safe prospective step"; break;
    case O::ElbowRoom: rationale = "maximize predicted obstacle clearance"; break;
    case O::Novelty: rationale = "avoid locations visited for the active target"; break;
    case O::GoAround: rationale = "turn away from the nearest obstacle"; break;
    case O::Greedy: rationale = "reduce distance to the active waypoint or target"; break;
    case O::Curiosity: rationale = "avoid every location visited in the experiment"; break;
    case O::Enfilade: rationale = "return toward recently visited locations"; break;
    case O::VisualScan: rationale = "rotate toward previously unseen orientations"; break;
    default: break;
  }
  return {dependencies(), std::move(actions), target_free,
          ScoreNormalization::SignedUnit, rationale};
}

bool DissertationAdvisor::accepts(domain::ActionType type) const noexcept {
  const auto actions = metadata().scored_action_types;
  return std::find(actions.begin(), actions.end(), type) != actions.end();
}

bool DissertationAdvisor::applicable(
    const domain::WorldModel& world) const {
  using O = DissertationAdvisorObjective;
  const auto& history = world.navigation_history.entries();
  switch (configuration_.objective) {
    case O::ElbowRoom:
    case O::GoAround:
    case O::VisualScan:
      return world.robot.laser && !world.robot.laser->ranges_m.empty();
    case O::Novelty:
      if (!world.mission.active()) return false;
      return std::any_of(history.begin(), history.end(), [&](const auto& entry) {
        return entry.task_id &&
               *entry.task_id == world.mission.active()->id;
      });
    case O::Curiosity: return !history.empty();
    case O::Enfilade:
      return history.size() >= 2U;
    default: return true;
  }
}

double DissertationAdvisor::score(const domain::WorldModel& world,
                                  const domain::Action& action) const {
  using O = DissertationAdvisorObjective;
  const auto expected = anticipatedPose(world, action,
                                        configuration_.action_space);
  const double progress = targetProgress(world, expected);
  switch (configuration_.objective) {
    case O::BigStep:
      if (action.type() == domain::ActionType::Pause) return 0.0;
      if (action.type() == domain::ActionType::Forward)
        return configuration_.action_space.move_distances_m().at(
            action.magnitude_index() - 1U);
      return configuration_.action_space.move_distances_m().back() * 0.5;
    case O::Greedy: return progress;
    case O::LeastAngle:
      if (!world.mission.active()) return 0.0;
      return -std::abs(domain::Angle::normalize(
          std::atan2(world.mission.active()->target.y_m -
                         expected.position.y_m,
                     world.mission.active()->target.x_m -
                         expected.position.x_m) -
          expected.heading.radians()));
    case O::ElbowRoom: {
      const auto obstacles = obstaclePoints(world);
      return obstacles.empty()
                 ? world.robot.laser->maximum_range.meters()
                 : nearest(expected.position, obstacles);
    }
    case O::GoAround: {
      const auto obstacles = obstaclePoints(world);
      if (obstacles.empty()) return 0.0;
      const auto closest = std::min_element(
          obstacles.begin(), obstacles.end(), [&](const auto& left,
                                                   const auto& right) {
            return domain::distance(world.robot.pose.position, left).meters() <
                   domain::distance(world.robot.pose.position, right).meters();
          });
      const double bearing = std::atan2(
          closest->y_m - world.robot.pose.position.y_m,
          closest->x_m - world.robot.pose.position.x_m);
      return std::abs(domain::Angle::normalize(
          expected.heading.radians() - bearing));
    }
    case O::VisualScan: return visualNovelty(world, expected);
    case O::Novelty:
    case O::Curiosity: {
      std::vector<domain::Point2D> points;
      for (const auto& entry : world.navigation_history.entries()) {
        if (configuration_.objective == O::Curiosity ||
            (world.mission.active() &&
             entry.task_id &&
             *entry.task_id == world.mission.active()->id))
          points.push_back(entry.pose.position);
      }
      return nearest(expected.position, points);
    }
    case O::Enfilade: {
      std::vector<domain::Point2D> recent;
      const auto& history = world.navigation_history.entries();
      const auto first = history.size() > 10U ? history.size() - 10U : 0U;
      for (std::size_t index = first; index < history.size(); ++index) {
        if (domain::distance(history[index].pose.position,
                             world.robot.pose.position).meters() > 0.25)
          recent.push_back(history[index].pose.position);
      }
      return recent.empty() ? 0.0 : -nearest(expected.position, recent);
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
  if (!applicable(context.world)) return result;
  result.weight = configuration_.weight;
  result.explanation = std::string(contract.rationale);
  using O = DissertationAdvisorObjective;
  switch (configuration_.objective) {
    case O::Novelty:
    case O::Curiosity:
    case O::Enfilade:
    case O::VisualScan:
      result.model_revision_used =
          context.world.navigation_history.entries().size();
      break;
    case O::BigStep:
    case O::ElbowRoom:
    case O::GoAround:
    case O::Greedy: result.model_revision_used = 0U; break;
    default: result.model_revision_used = context.world.spatial.revision; break;
  }
  for (const auto& action : candidates)
    if (accepts(action.type()))
      result.scores.push_back({action, score(context.world, action)});
  normalize(result.scores, contract.normalization);
  result.participated = !result.scores.empty();
  return result;
}

}  // namespace semaforr::decision
