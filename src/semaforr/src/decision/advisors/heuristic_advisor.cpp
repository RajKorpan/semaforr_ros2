#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/decision/advisors/heuristic_advisor.hpp>
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

std::vector<domain::Point2D> obstaclePoints(
    const domain::WorldModel& world) {
  std::vector<domain::Point2D> points;
  if (!world.robot.laser) return points;
  const auto& laser = *world.robot.laser;
  points.reserve(laser.ranges_m.size());
  double angle = world.robot.pose.heading.radians() +
                 laser.angle_min.radians();
  for (const double range : laser.ranges_m) {
    const bool obstacle_return =
        std::isfinite(range) && range >= laser.minimum_range.meters() &&
        range < laser.maximum_range.meters() - domain::geometry_tolerance_m;
    if (obstacle_return)
      points.push_back(
          {world.robot.pose.position.x_m + range * std::cos(angle),
           world.robot.pose.position.y_m + range * std::sin(angle)});
    angle += laser.angle_increment.radians();
  }
  return points;
}

std::vector<std::vector<domain::Point2D>> trailPolylines(
    const domain::SpatialModel& spatial) {
  if (spatial.learned_trails.empty()) return spatial.trails;
  std::vector<std::vector<domain::Point2D>> result;
  result.reserve(spatial.learned_trails.size());
  for (const auto& learned : spatial.learned_trails) {
    std::vector<domain::Point2D> trail;
    trail.reserve(learned.markers.size());
    for (const auto& marker : learned.markers)
      trail.push_back(marker.pose.position);
    if (trail.size() >= 2U) result.push_back(std::move(trail));
  }
  return result;
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

double segmentParameter(domain::Point2D point,
                        const domain::Segment2D& segment) {
  const double dx = segment.end.x_m - segment.start.x_m;
  const double dy = segment.end.y_m - segment.start.y_m;
  const double squared = dx * dx + dy * dy;
  if (squared <= domain::geometry_tolerance_m) return 0.0;
  return std::clamp(((point.x_m - segment.start.x_m) * dx +
                     (point.y_m - segment.start.y_m) * dy) /
                        squared,
                    0.0, 1.0);
}

domain::Point2D closestPoint(domain::Point2D point,
                             const domain::Segment2D& segment) {
  const double parameter = segmentParameter(point, segment);
  return {segment.start.x_m +
              parameter * (segment.end.x_m - segment.start.x_m),
          segment.start.y_m +
              parameter * (segment.end.y_m - segment.start.y_m)};
}

double distanceToSegment(domain::Point2D point,
                         const domain::Segment2D& segment) {
  return domain::distance(point, closestPoint(point, segment)).meters();
}

double signedDistanceToRegion(domain::Point2D point,
                              const domain::Circle& region) {
  return domain::distance(point, region.center).meters() -
         region.radius.meters();
}

const domain::LearnedRegion* regionById(const domain::SpatialModel& spatial,
                                        domain::RegionId id) {
  const auto found = std::find_if(
      spatial.regions.begin(), spatial.regions.end(),
      [id](const auto& region) { return region.id == id; });
  return found == spatial.regions.end() ? nullptr : &*found;
}

std::size_t learnedDoorCount(const domain::SpatialModel& spatial,
                             domain::RegionId region) {
  return static_cast<std::size_t>(std::count_if(
      spatial.doors.begin(), spatial.doors.end(),
      [region](const auto& door) { return door.region == region; }));
}

std::size_t skeletonDegree(const domain::SpatialModel& spatial,
                           std::size_t node) {
  return static_cast<std::size_t>(std::count_if(
      spatial.region_skeleton_edges.begin(),
      spatial.region_skeleton_edges.end(), [node](const auto& edge) {
        return edge.from == node || edge.to == node;
      }));
}

std::vector<domain::Segment2D> hallwaySegments(
    const domain::SpatialModel& spatial) {
  if (!spatial.hallway_entities.empty()) {
    std::vector<domain::Segment2D> result;
    result.reserve(spatial.hallway_entities.size());
    for (const auto& hallway : spatial.hallway_entities)
      result.push_back(hallway.centerline);
    return result;
  }
  return spatial.hallways;
}

bool insideHallway(domain::Point2D point,
                   const domain::LearnedHallway& hallway) {
  const double half_width = std::max(0.0, hallway.width_m * 0.5);
  return half_width > 0.0 &&
         distanceToSegment(point, hallway.centerline) <= half_width;
}

bool overlaps(const domain::Segment2D& first,
              const domain::Segment2D& second,
              double tolerance_m = 0.75) {
  return domain::intersects(first, second, tolerance_m) ||
         distanceToSegment(first.start, second) <= tolerance_m ||
         distanceToSegment(first.end, second) <= tolerance_m ||
         distanceToSegment(second.start, first) <= tolerance_m ||
         distanceToSegment(second.end, first) <= tolerance_m;
}

template <typename Grid>
std::optional<std::size_t> gridIndex(const Grid& grid,
                                     domain::Point2D point) {
  return grid.extent().index(point);
}

std::optional<domain::Point2D> localObjective(
    const DecisionContext& context) {
  if (context.active_plan_objective)
    return context.active_plan_objective->target;
  if (!context.world.mission.active()) return std::nullopt;
  return context.world.mission.active()->waypoint().value_or(
      context.world.mission.active()->target);
}

double targetProgress(const DecisionContext& context,
                      const domain::Pose2D& expected) {
  const auto target = localObjective(context);
  if (!target) return 0.0;
  const auto& world = context.world;
  return domain::distance(world.robot.pose.position, *target).meters() -
         domain::distance(expected.position, *target).meters();
}

}  // namespace

HeuristicAdvisor::HeuristicAdvisor(
    HeuristicAdvisorConfiguration configuration)
    : configuration_(std::move(configuration)) {
  if (configuration_.name.empty() || !std::isfinite(configuration_.weight) ||
      configuration_.weight < 0.0)
    throw std::invalid_argument("invalid heuristic advisor configuration");
}

std::vector<std::string_view> HeuristicAdvisor::dependencies() const {
  using O = HeuristicObjective;
  switch (configuration_.objective) {
    case O::Novelty:
    case O::Curiosity: return {"navigation_history"};
    case O::ElbowRoom:
    case O::GoAround: return {"laser"};
    case O::Enfilade: return {"navigation_history"};
    case O::VisualScan: return {"laser", "navigation_history"};
    case O::Convey: return {"conveyor_grid"};
    case O::Enter: return {"regions"};
    case O::Exit: return {"regions"};
    case O::Access: return {"regions", "doors"};
    case O::Stay: return {"hallways"};
    case O::Trailer: return {"trails"};
    case O::Unlikely: return {"regions", "region_skeleton"};
    case O::Crossroads: return {"hallways"};
    case O::Follow: return {"hallways"};
    case O::SpatialLearner:
      return {"inclusion_grid", "regions", "conveyor_grid"};
    case O::LeastAngle:
      return {"active_target", "regions", "region_skeleton"};
    case O::BigStep:
    case O::Greedy: return {"active_target"};
  }
  return {};
}

AdvisorMetadata HeuristicAdvisor::metadata() const {
  using A = domain::ActionType;
  using O = HeuristicObjective;
  std::vector<A> actions{A::Pause, A::Forward, A::TurnLeft, A::TurnRight};
  if (configuration_.objective == O::VisualScan ||
      configuration_.objective == O::GoAround)
    actions = {A::TurnLeft, A::TurnRight};
  const bool target_free =
      configuration_.objective == O::ElbowRoom ||
      configuration_.objective == O::Curiosity ||
      configuration_.objective == O::Enfilade ||
      configuration_.objective == O::VisualScan ||
      configuration_.objective == O::SpatialLearner;
  std::string_view rationale =
      "normalized Tier-3 action preference";
  switch (configuration_.objective) {
    case O::BigStep: rationale = "prefer the longest safe prospective step"; break;
    case O::ElbowRoom: rationale = "maximize predicted obstacle clearance"; break;
    case O::Novelty: rationale = "avoid locations visited for the active target"; break;
    case O::GoAround: rationale = "turn away from the nearest obstacle"; break;
    case O::Greedy: rationale = "reduce distance to the active plan step or mission target"; break;
    case O::Curiosity: rationale = "avoid every location visited in the experiment"; break;
    case O::Enfilade: rationale = "return toward recently visited locations"; break;
    case O::VisualScan: rationale = "rotate toward previously unseen orientations"; break;
    case O::Convey: rationale = "approach frequent, useful conveyor-grid cells"; break;
    case O::Enter: rationale = "enter the region containing the local plan objective"; break;
    case O::Exit: rationale = "leave the current region when it lacks the local plan objective"; break;
    case O::Trailer: rationale = "join a trail segment that approaches the local plan objective"; break;
    case O::Unlikely: rationale = "avoid non-target regions with poor skeleton connectivity"; break;
    case O::Access: rationale = "approach regions with many doors"; break;
    case O::Crossroads: rationale = "approach highly overlapping hallways"; break;
    case O::Follow: rationale = "follow a hallway relevant to the local plan objective"; break;
    case O::LeastAngle: rationale = "take the skeleton branch best aligned with the local plan objective"; break;
    case O::SpatialLearner: rationale = "approach locations absent from the spatial model"; break;
    case O::Stay: rationale = "remain within the current hallway"; break;
    default: break;
  }
  return {dependencies(), std::move(actions), target_free,
          ScoreNormalization::TenPoint, rationale};
}

bool HeuristicAdvisor::accepts(domain::ActionType type) const noexcept {
  const auto actions = metadata().scored_action_types;
  return std::find(actions.begin(), actions.end(), type) != actions.end();
}

bool HeuristicAdvisor::applicable(
    const DecisionContext& context) const {
  const auto& world = context.world;
  const auto objective = localObjective(context);
  using O = HeuristicObjective;
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
    case O::Convey:
      return world.spatial.conveyor_grid.geometry.valid() &&
             !world.spatial.conveyor_grid.cells.empty();
    case O::Enter:
      return objective &&
             std::any_of(world.spatial.learned_regions.begin(),
                         world.spatial.learned_regions.end(),
                         [&](const auto& region) {
                           return region.contains(*objective) &&
                                  !region.contains(world.robot.pose.position);
                         });
    case O::Exit:
      return objective &&
             std::any_of(world.spatial.learned_regions.begin(),
                         world.spatial.learned_regions.end(),
                         [&](const auto& region) {
                           return region.contains(world.robot.pose.position) &&
                                  !region.contains(*objective);
                         });
    case O::Trailer: {
      const auto trails = trailPolylines(world.spatial);
      return std::any_of(trails.begin(), trails.end(),
                         [](const auto& trail) { return trail.size() >= 2U; });
    }
    case O::Unlikely:
      return !world.spatial.regions.empty() &&
             !world.spatial.region_skeleton_nodes.empty() &&
             !world.spatial.region_skeleton_edges.empty();
    case O::Access:
      return !world.spatial.regions.empty() && !world.spatial.doors.empty();
    case O::Crossroads:
      return world.spatial.hallway_entities.size() >= 2U ||
             world.spatial.hallways.size() >= 2U;
    case O::Follow:
      return objective &&
             (!world.spatial.hallway_entities.empty() ||
              !world.spatial.hallways.empty());
    case O::LeastAngle:
      return objective &&
             !world.spatial.regions.empty() &&
             !world.spatial.region_skeleton_nodes.empty() &&
             !world.spatial.region_skeleton_edges.empty();
    case O::SpatialLearner:
      return world.spatial.inclusion_grid.observedCellCount() != 0U ||
             !world.spatial.learned_regions.empty() ||
             !world.spatial.conveyor_grid.cells.empty();
    case O::Stay:
      if (!world.spatial.hallway_entities.empty())
        return std::any_of(world.spatial.hallway_entities.begin(),
                           world.spatial.hallway_entities.end(),
                           [&](const auto& hallway) {
                             return insideHallway(world.robot.pose.position,
                                                  hallway);
                           });
      return std::any_of(world.spatial.hallways.begin(),
                         world.spatial.hallways.end(), [&](const auto& hallway) {
                           return distanceToSegment(world.robot.pose.position,
                                                    hallway) <= 0.75;
                         });
    default: return true;
  }
}

double HeuristicAdvisor::score(const DecisionContext& context,
                               const domain::Action& action) const {
  const auto& world = context.world;
  using O = HeuristicObjective;
  const auto expected = anticipatedPose(world, action,
                                        configuration_.action_space);
  const double progress = targetProgress(context, expected);
  const auto objective = localObjective(context);
  switch (configuration_.objective) {
    case O::BigStep:
      if (action.type() == domain::ActionType::Pause) return 0.0;
      if (action.type() == domain::ActionType::Forward)
        return configuration_.action_space.move_distances_m().at(
            action.magnitude_index() - 1U);
      return configuration_.action_space.move_distances_m().back() * 0.5;
    case O::Greedy: return progress;
    case O::LeastAngle: {
      const auto current_region = std::find_if(
          world.spatial.regions.begin(), world.spatial.regions.end(),
          [&](const auto& region) {
            return region.boundary.contains(world.robot.pose.position);
          });
      if (current_region == world.spatial.regions.end()) return 0.0;
      const auto current = std::find_if(
          world.spatial.region_skeleton_nodes.begin(),
          world.spatial.region_skeleton_nodes.end(), [&](const auto& node) {
            return node.region == current_region->id;
          });
      if (current == world.spatial.region_skeleton_nodes.end()) return 0.0;
      const auto origin = current->center;
      const double target_angle = std::atan2(
          objective->y_m - origin.y_m, objective->x_m - origin.x_m);
      double smallest_angle = std::numeric_limits<double>::infinity();
      std::optional<domain::Point2D> selected;
      for (const auto& edge : world.spatial.region_skeleton_edges) {
        std::optional<std::size_t> neighbor;
        if (edge.from == current->id) neighbor = edge.to;
        if (edge.to == current->id) neighbor = edge.from;
        if (!neighbor)
          continue;
        const auto neighbor_node = std::find_if(
            world.spatial.region_skeleton_nodes.begin(),
            world.spatial.region_skeleton_nodes.end(), [&](const auto& node) {
              return node.id == *neighbor;
            });
        if (neighbor_node == world.spatial.region_skeleton_nodes.end())
          continue;
        const auto point = neighbor_node->center;
        const double branch_angle =
            std::atan2(point.y_m - origin.y_m, point.x_m - origin.x_m);
        const double angle = std::abs(
            domain::Angle::normalize(branch_angle - target_angle));
        if (angle < smallest_angle) {
          smallest_angle = angle;
          selected = point;
        }
      }
      if (!selected) return 0.0;
      const double branch_heading = std::atan2(
          selected->y_m - origin.y_m, selected->x_m - origin.x_m);
      const double action_alignment = std::cos(domain::Angle::normalize(
          expected.heading.radians() - branch_heading));
      return selected ? action_alignment -
                            0.25 * domain::distance(expected.position,
                                                    *selected).meters()
                      : 0.0;
    }
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
      double best = -std::numeric_limits<double>::infinity();
      for (const auto& cell : world.spatial.conveyor_grid.cells) {
        if (cell.traversal_frequency == 0U) continue;
        const auto location =
            world.spatial.conveyor_grid.geometry.center(cell.index);
        const double before = domain::distance(
                                  world.robot.pose.position, location).meters();
        const double after =
            domain::distance(expected.position, location).meters();
        const double frequency =
            std::log1p(static_cast<double>(cell.traversal_frequency));
        // Distance makes remote, strong conveyors useful; the difference
        // term ensures an action must actually approach that evidence.
        best = std::max(best, frequency + 0.1 * before + before - after);
      }
      return std::isfinite(best) ? best : 0.0;
    }
    case O::Enter: {
      double best = -std::numeric_limits<double>::infinity();
      for (const auto& region : world.spatial.learned_regions)
        if (region.contains(*objective))
          best = std::max(best,
                          -signedDistanceToRegion(expected.position, region));
      return best;
    }
    case O::Exit: {
      double result = -std::numeric_limits<double>::infinity();
      for (const auto& region : world.spatial.learned_regions)
        if (region.contains(world.robot.pose.position) &&
            !region.contains(*objective))
          result = std::max(
              result, signedDistanceToRegion(expected.position, region));
      return result;
    }
    case O::Trailer: {
      const domain::Point2D target =
          objective.value_or(world.robot.pose.position);
      double best_utility = -std::numeric_limits<double>::infinity();
      std::optional<domain::Segment2D> selected;
      domain::Point2D preferred;
      for (const auto& trail : trailPolylines(world.spatial)) {
        for (std::size_t index = 1U; index < trail.size(); ++index) {
          domain::Segment2D segment{trail[index - 1U], trail[index]};
          const double first_target =
              domain::distance(segment.start, target).meters();
          const double second_target =
              domain::distance(segment.end, target).meters();
          const domain::Point2D endpoint =
              first_target < second_target ? segment.start : segment.end;
          const double target_gain = std::abs(first_target - second_target);
          const double utility =
              target_gain - distanceToSegment(
                                world.robot.pose.position, segment);
          if (utility > best_utility) {
            best_utility = utility;
            selected = segment;
            preferred = endpoint;
          }
        }
      }
      return selected
                 ? -distanceToSegment(expected.position, *selected) -
                       0.5 * domain::distance(expected.position,
                                              preferred).meters()
                 : 0.0;
    }
    case O::Unlikely: {
      double risk = 0.0;
      for (std::size_t node = 0U;
           node < world.spatial.region_skeleton_nodes.size(); ++node) {
        const auto& skeleton_node = world.spatial.region_skeleton_nodes[node];
        const auto* region = regionById(world.spatial, skeleton_node.region);
        if (!region) continue;
        if (objective && region->boundary.contains(*objective))
          continue;
        const auto degree = skeletonDegree(world.spatial, skeleton_node.id);
        if (degree > 1U) continue;
        const double influence = std::max(
            0.0, 1.0 - signedDistanceToRegion(expected.position,
                                               region->boundary));
        risk += static_cast<double>(2U - degree) * influence;
      }
      return -risk;
    }
    case O::Access: {
      double best = -std::numeric_limits<double>::infinity();
      for (const auto& region : world.spatial.regions) {
        const auto doors = learnedDoorCount(world.spatial, region.id);
        best = std::max(
            best, static_cast<double>(doors) -
                      std::max(0.0,
                               signedDistanceToRegion(expected.position,
                                                      region.boundary)));
      }
      return std::isfinite(best) ? best : 0.0;
    }
    case O::Crossroads: {
      const auto hallways = hallwaySegments(world.spatial);
      double best = -std::numeric_limits<double>::infinity();
      for (std::size_t index = 0U; index < hallways.size(); ++index) {
        std::size_t degree = 0U;
        for (std::size_t other = 0U; other < hallways.size(); ++other)
          if (other != index &&
              overlaps(hallways[index], hallways[other]))
            ++degree;
        best = std::max(
            best, static_cast<double>(degree) -
                      distanceToSegment(expected.position, hallways[index]));
      }
      return best;
    }
    case O::Follow: {
      const auto target = *objective;
      const auto hallways = hallwaySegments(world.spatial);
      const auto selected = std::min_element(
          hallways.begin(), hallways.end(),
          [&](const auto& left, const auto& right) {
            return distanceToSegment(target, left) <
                   distanceToSegment(target, right);
          });
      const auto preferred =
          domain::distance(selected->start, target).meters() <
                  domain::distance(selected->end, target).meters()
              ? selected->start
              : selected->end;
      const double robot_parameter =
          segmentParameter(world.robot.pose.position, *selected);
      const double expected_parameter =
          segmentParameter(expected.position, *selected);
      const bool toward_end = preferred == selected->end;
      const double advance = toward_end
                                 ? expected_parameter - robot_parameter
                                 : robot_parameter - expected_parameter;
      return 2.0 * advance - distanceToSegment(expected.position, *selected);
    }
    case O::SpatialLearner: {
      const auto& grid = world.spatial.inclusion_grid;
      double unknown = 0.0;
      const auto index = gridIndex(grid, expected.position);
      const auto inclusion = index ? grid.valueAt(*index) : 0U;
      if (inclusion == 0U)
        unknown += 1.0;
      else
        unknown -= std::log1p(static_cast<double>(inclusion));
      if (std::any_of(world.spatial.learned_regions.begin(),
                      world.spatial.learned_regions.end(),
                      [&](const auto& region) {
                        return region.contains(expected.position);
                      }))
        unknown -= 1.0;
      if (const auto* conveyor = world.spatial.conveyor_grid.at(
              expected.position))
        unknown -= std::log1p(
            static_cast<double>(conveyor->traversal_frequency));
      return unknown;
    }
    case O::Stay: {
      if (!world.spatial.hallway_entities.empty()) {
        const auto current = std::find_if(
            world.spatial.hallway_entities.begin(),
            world.spatial.hallway_entities.end(), [&](const auto& hallway) {
              return insideHallway(world.robot.pose.position, hallway);
            });
        if (current == world.spatial.hallway_entities.end()) return 0.0;
        const double distance =
            distanceToSegment(expected.position, current->centerline);
        return (insideHallway(expected.position, *current) ? 1.0 : -1.0) -
               distance;
      }
      const auto current = std::min_element(
          world.spatial.hallways.begin(), world.spatial.hallways.end(),
          [&](const auto& left, const auto& right) {
            return distanceToSegment(world.robot.pose.position, left) <
                   distanceToSegment(world.robot.pose.position, right);
          });
      const double distance = distanceToSegment(expected.position, *current);
      return (distance <= 0.75 ? 1.0 : -1.0) - distance;
    }
  }
  return 0.0;
}

AdvisorEvaluation HeuristicAdvisor::evaluate(
    const DecisionContext& context,
    std::span<const domain::Action> candidates) const {
  AdvisorEvaluation result;
  const auto contract = metadata();
  if (!contract.participates_without_target &&
      !context.world.mission.active())
    return result;
  if (!applicable(context)) return result;
  result.weight = configuration_.weight;
  result.explanation = std::string(contract.rationale);
  using O = HeuristicObjective;
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
      result.scores.push_back({action, score(context, action)});
  result.participated = !result.scores.empty();
  return result;
}

}  // namespace semaforr::decision
