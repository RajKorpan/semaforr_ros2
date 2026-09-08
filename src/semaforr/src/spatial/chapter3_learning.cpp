/**
 * @file chapter3_learning.cpp
 * @brief Chapter3 learning responsibilities.
 *
 * @details This file implements chapter3 learning behavior for learned spatial
 * representations and their lifecycle. It centers on `Candidate`,
 * `ExitAccumulator`, `TravelSegment`, `PairScore`, `Evidence`. Its
 * package-relative location is `src/spatial/chapter3_learning.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <semaforr/spatial/chapter3_learning.hpp>
#include <set>
#include <stdexcept>
#include <unordered_map>

#include "learning_geometry.hpp"

namespace semaforr::spatial {
namespace {

constexpr double pi = 3.14159265358979323846;

/**
 * @brief Performs the normalize operation for this subsystem.
 *
 * Arguments:
 * - @p angle: Supplies angle input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double normalize(double angle) {
  while (angle <= -pi) angle += 2.0 * pi;
  while (angle > pi) angle -= 2.0 * pi;
  return angle;
}

/**
 * @brief Performs the positive angle operation for this subsystem.
 *
 * Arguments:
 * - @p angle: Supplies angle input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double positiveAngle(double angle) {
  angle = std::fmod(angle, 2.0 * pi);
  return angle < 0.0 ? angle + 2.0 * pi : angle;
}

/**
 * @brief Performs the point segment distance operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 * - @p segment: Supplies segment input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double pointSegmentDistance(domain::Point2D point,
                            const domain::Segment2D& segment) {
  const double dx = segment.end.x_m - segment.start.x_m;
  const double dy = segment.end.y_m - segment.start.y_m;
  const double squared = dx * dx + dy * dy;
  const double t = squared <= domain::geometry_tolerance_m
                       ? 0.0
                       : std::clamp(((point.x_m - segment.start.x_m) * dx +
                                     (point.y_m - segment.start.y_m) * dy) /
                                        squared,
                                    0.0, 1.0);
  return std::hypot(point.x_m - segment.start.x_m - t * dx,
                    point.y_m - segment.start.y_m - t * dy);
}

/**
 * @brief Performs the ray endpoints operation for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 *
 * Returns:
 * - `std::vector<domain::Point2D>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<domain::Point2D> rayEndpoints(
    const domain::RobotObservation& observation) {
  std::vector<domain::Point2D> result;
  const auto& laser = observation.laser;
  result.reserve(laser.ranges_m.size());
  for (std::size_t index = 0U; index < laser.ranges_m.size(); ++index) {
    double range = laser.ranges_m[index];
    if (!std::isfinite(range) || range < laser.minimum_range.meters()) continue;
    range = std::min(range, laser.maximum_range.meters());
    const double angle =
        observation.pose.heading.radians() + laser.angle_min.radians() +
        static_cast<double>(index) * laser.angle_increment.radians();
    result.push_back({observation.pose.position.x_m + range * std::cos(angle),
                      observation.pose.position.y_m + range * std::sin(angle)});
  }
  return result;
}

/**
 * @brief Performs the containing region operation for this subsystem.
 *
 * Arguments:
 * - @p regions: Supplies regions input to the operation.
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `std::optional<std::size_t>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<std::size_t> containingRegion(
    const std::vector<domain::LearnedRegion>& regions, domain::Point2D point) {
  std::optional<std::size_t> result;
  double best_radius = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < regions.size(); ++index) {
    if (regions[index].boundary.contains(point) &&
        regions[index].boundary.radius.meters() < best_radius) {
      result = index;
      best_radius = regions[index].boundary.radius.meters();
    }
  }
  return result;
}

/**
 * @brief Performs the rasterize operation for this subsystem.
 *
 * Arguments:
 * - @p segment: Supplies segment input to the operation.
 * - @p resolution: Supplies resolution input to the operation.
 *
 * Returns:
 * - `std::vector<domain::Point2D>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<domain::Point2D> rasterize(domain::Segment2D segment,
                                       double resolution) {
  const double length = segment.length().meters();
  const std::size_t count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / resolution)));
  std::vector<domain::Point2D> result;
  result.reserve(count + 1U);
  for (std::size_t index = 0U; index <= count; ++index) {
    const double t = static_cast<double>(index) / static_cast<double>(count);
    result.push_back(
        {segment.start.x_m + t * (segment.end.x_m - segment.start.x_m),
         segment.start.y_m + t * (segment.end.y_m - segment.start.y_m)});
  }
  return result;
}

/**
 * @brief Performs the direction for operation for this subsystem.
 *
 * Arguments:
 * - @p segment: Supplies segment input to the operation.
 *
 * Returns:
 * - `domain::HallwayDirection` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::HallwayDirection directionFor(domain::Segment2D segment) {
  double angle = positiveAngle(std::atan2(segment.end.y_m - segment.start.y_m,
                                          segment.end.x_m - segment.start.x_m));
  if (angle >= pi) angle -= pi;
  const int bin =
      static_cast<int>(std::floor((angle + pi / 8.0) / (pi / 4.0))) % 4;
  switch (bin) {
    case 0:
      return domain::HallwayDirection::Horizontal;
    case 1:
      return domain::HallwayDirection::MajorDiagonal;
    case 2:
      return domain::HallwayDirection::Vertical;
    default:
      return domain::HallwayDirection::MinorDiagonal;
  }
}

/**
 * @brief Performs the direction angle operation for this subsystem.
 *
 * Arguments:
 * - @p direction: Supplies direction input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double directionAngle(domain::HallwayDirection direction) {
  switch (direction) {
    case domain::HallwayDirection::Horizontal:
      return 0.0;
    case domain::HallwayDirection::MajorDiagonal:
      return pi / 4.0;
    case domain::HallwayDirection::Vertical:
      return pi / 2.0;
    case domain::HallwayDirection::MinorDiagonal:
      return 3.0 * pi / 4.0;
  }
  return 0.0;
}

/**
 * @brief Performs the stable cell id operation for this subsystem.
 *
 * Arguments:
 * - @p argument_1: Supplies argument 1 input to the operation.
 * - @p argument_2: Supplies argument 2 input to the operation.
 * - @p label: Supplies label input to the operation.
 *
 * Returns:
 * - `std::uint64_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::uint64_t stableCellId(long long x, long long y, std::uint64_t label) {
  std::uint64_t seed = label + 0x9e3779b97f4a7c15ULL;
  seed ^= static_cast<std::uint64_t>(x) + 0x9e3779b97f4a7c15ULL + (seed << 6U) +
          (seed >> 2U);
  seed ^= static_cast<std::uint64_t>(y) + 0x9e3779b97f4a7c15ULL + (seed << 6U) +
          (seed >> 2U);
  return seed;
}

}  // namespace

/**
 * @brief Performs the completed paths from episodes operation for this
 * subsystem.
 *
 * Arguments:
 * - @p episodes: Supplies episodes input to the operation.
 *
 * Returns:
 * - `std::vector<domain::CompletedPath>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<domain::CompletedPath> completedPathsFromEpisodes(
    const std::vector<NavigationEpisode>& episodes) {
  std::vector<domain::CompletedPath> result;
  std::map<domain::TaskId, std::size_t> path_for_task;
  for (const auto& episode : episodes) {
    if (!episode.execution_result || !episode.active_task) continue;
    auto found = path_for_task.find(*episode.active_task);
    if (found == path_for_task.end()) {
      domain::CompletedPath path;
      path.id = static_cast<domain::PathId>(*episode.active_task + 1U);
      path.task_id = episode.active_task;
      path.target = episode.active_target;
      result.push_back(std::move(path));
      found =
          path_for_task.emplace(*episode.active_task, result.size() - 1U).first;
    }
    auto& path = result[found->second];
    domain::PathDecisionPoint point;
    if (episode.selection) {
      point.selection = *episode.selection;
    } else {
      point.selection.decision_id = episode.sequence;
      point.selection.action_id = episode.execution_result->action_id;
      point.selection.task_id = episode.active_task;
      point.selection.expected_start = episode.execution_result->start_pose;
      point.selection.action =
          episode.selected_action.value_or(domain::Action::pause());
    }
    point.execution = *episode.execution_result;
    point.decision_observation = episode.observation;
    if (episode.action_started) point.executed_action = point.selection.action;
    point.target = episode.active_target;
    point.task_started = episode.task_started;
    point.task_finished = episode.task_finished;
    point.interrupted = !point.execution.successful();
    if (path.decision_points.empty())
      path.started_at = point.execution.started_at;
    path.finished_at = point.execution.finished_at;
    path.decision_points.push_back(std::move(point));
    path.target_reached = path.target_reached || episode.target_reached;
    path.task_skipped = path.task_skipped || episode.task_skipped;
  }
  for (auto& path : result) {
    if (path.decision_points.empty()) continue;
    if (!path.decision_points.back().task_finished)
      path.decision_points.back().task_finished = true;
  }
  return result;
}

/**
 * @brief Performs the historically visible operation for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 * - @p marker: Supplies marker input to the operation.
 * - @p tolerance_m: Supplies tolerance m input to the operation.
 * - @p evidence: Supplies evidence input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool historicallyVisible(const domain::RobotObservation& observation,
                         domain::Point2D marker, double tolerance_m,
                         domain::VisibilityEvidence* evidence) {
  if (!marker.finite() || !observation.pose.position.finite() ||
      observation.laser.ranges_m.empty())
    return false;
  const double dx = marker.x_m - observation.pose.position.x_m;
  const double dy = marker.y_m - observation.pose.position.y_m;
  const double distance = std::hypot(dx, dy);
  if (distance <= tolerance_m) return true;
  if (distance > observation.laser.maximum_range.meters() + tolerance_m)
    return false;
  const double relative =
      normalize(std::atan2(dy, dx) - observation.pose.heading.radians());
  const double increment = observation.laser.angle_increment.radians();
  if (std::abs(increment) <= domain::geometry_tolerance_m) return false;
  double angle_min = observation.laser.angle_min.radians();
  // Angle intentionally normalizes -pi to +pi.  Restore the equivalent
  // unwrapped scan origin when a positive-increment scan spans forward from
  // that boundary; otherwise full-circle laser histories lose every ray.
  if (increment > 0.0 && angle_min > 0.0 &&
      angle_min + increment * static_cast<double>(
                                  observation.laser.ranges_m.size() - 1U) >
          pi + domain::geometry_tolerance_m)
    angle_min -= 2.0 * pi;
  const double raw = (relative - angle_min) / increment;
  const auto nearest = static_cast<long long>(std::llround(raw));
  if (nearest < 0 ||
      nearest >= static_cast<long long>(observation.laser.ranges_m.size()))
    return false;
  double visible =
      observation.laser.ranges_m[static_cast<std::size_t>(nearest)];
  if (!std::isfinite(visible))
    visible = observation.laser.maximum_range.meters();
  visible = std::min(visible, observation.laser.maximum_range.meters());
  if (visible + tolerance_m < distance) return false;
  if (evidence) {
    evidence->observer = observation.pose.position;
    evidence->endpoint = marker;
    evidence->visible_distance = domain::Distance(visible);
    evidence->ray_index = static_cast<std::size_t>(nearest);
  }
  return true;
}

/**
 * @brief Performs the learn visibility trail operation for this subsystem.
 *
 * Arguments:
 * - @p path: Supplies path input to the operation.
 * - @p id: Supplies id input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `domain::LearnedTrail` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::LearnedTrail learnVisibilityTrail(
    const domain::CompletedPath& path, domain::TrailId id,
    const TrailLearningConfiguration& configuration) {
  domain::LearnedTrail trail;
  trail.id = id;
  trail.source_path = path.id;
  trail.task_id = path.task_id;
  trail.target = path.target;
  if (path.decision_points.empty()) return trail;

  struct Candidate {
    std::size_t path_index;
    domain::Pose2D pose;
    domain::LaserObservation view;
  };
  std::vector<Candidate> candidates;
  for (std::size_t index = 0U; index < path.decision_points.size(); ++index) {
    const auto& point = path.decision_points[index];
    const bool traversed =
        point.successfulTraversal() ||
        (configuration.include_partial_movement && point.partialTraversal());
    if (!traversed) continue;
    const auto start = point.execution.start_pose;
    if (!candidates.empty() &&
        domain::distance(candidates.back().pose.position, start.position)
                .meters() > configuration.visibility_tolerance_m) {
      // Do not invent a connecting segment across a controller restart,
      // localization jump, preemption, or other discontinuity. The latest
      // contiguous suffix is the target-relevant traversal.
      candidates.clear();
    }
    if (candidates.empty() || candidates.back().pose.position != start.position)
      candidates.push_back({index, start, point.decision_observation.laser});
    const auto finish = point.actualReachedPose();
    if (candidates.back().pose.position != finish.position)
      candidates.push_back({index, finish, point.decision_observation.laser});
  }
  if (candidates.size() < 2U) return trail;

  std::vector<std::size_t> reversed{candidates.size() - 1U};
  std::size_t current = candidates.size() - 1U;
  while (current > 0U) {
    std::size_t predecessor = current - 1U;
    domain::VisibilityEvidence selected_evidence;
    bool found = false;
    for (std::size_t candidate = 0U; candidate < current; ++candidate) {
      domain::RobotObservation observation;
      observation.pose = candidates[candidate].pose;
      observation.laser = candidates[candidate].view;
      domain::VisibilityEvidence visibility;
      if (historicallyVisible(observation, candidates[current].pose.position,
                              configuration.visibility_tolerance_m,
                              &visibility)) {
        predecessor = candidate;
        visibility.decision_id =
            path.decision_points[candidates[candidate].path_index]
                .selection.decision_id;
        selected_evidence = visibility;
        found = true;
        break;
      }
    }
    reversed.push_back(predecessor);
    current = predecessor;
    if (!found && current == 0U) break;
  }
  std::reverse(reversed.begin(), reversed.end());
  for (std::size_t position = 0U; position < reversed.size(); ++position) {
    const auto candidate = reversed[position];
    domain::TrailMarker marker;
    marker.path_point_index = candidates[candidate].path_index;
    marker.pose = candidates[candidate].pose;
    marker.view = candidates[candidate].view;
    if (position + 1U < reversed.size()) {
      domain::RobotObservation observation;
      observation.pose = marker.pose;
      observation.laser = marker.view;
      domain::VisibilityEvidence visibility;
      if (historicallyVisible(
              observation, candidates[reversed[position + 1U]].pose.position,
              configuration.visibility_tolerance_m, &visibility)) {
        visibility.decision_id =
            path.decision_points[marker.path_point_index].selection.decision_id;
        marker.visibility_to_next = visibility;
      }
    }
    trail.markers.push_back(std::move(marker));
  }

  if (configuration.simplification_tolerance_m > 0.0 &&
      trail.markers.size() > 2U) {
    std::vector<domain::TrailMarker> simplified{trail.markers.front()};
    for (std::size_t index = 1U; index + 1U < trail.markers.size(); ++index) {
      const domain::Segment2D shortcut{simplified.back().pose.position,
                                       trail.markers[index + 1U].pose.position};
      if (pointSegmentDistance(trail.markers[index].pose.position, shortcut) >
          configuration.simplification_tolerance_m)
        simplified.push_back(trail.markers[index]);
    }
    simplified.push_back(trail.markers.back());
    trail.markers = std::move(simplified);
  }
  for (std::size_t index = 1U; index < trail.markers.size(); ++index) {
    trail.subtrail_geometry.push_back({trail.markers[index - 1U].pose.position,
                                       trail.markers[index].pose.position});
    trail.length_m += domain::distance(trail.markers[index - 1U].pose.position,
                                       trail.markers[index].pose.position)
                          .meters();
  }
  return trail;
}

/**
 * @brief Performs the learn decision regions operation for this subsystem.
 *
 * Arguments:
 * - @p decision_episodes: Supplies decision episodes input to the
 * operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `RegionModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
RegionModel learnDecisionRegions(
    const std::vector<NavigationEpisode>& decision_episodes,
    const RegionLearningConfiguration& configuration) {
  std::vector<domain::LearnedRegion> raw;
  for (const auto& episode : decision_episodes) {
    double radius = std::numeric_limits<double>::infinity();
    for (double range : episode.observation.laser.ranges_m)
      if (std::isfinite(range) &&
          range >= episode.observation.laser.minimum_range.meters())
        radius = std::min(
            radius,
            std::min(range, episode.observation.laser.maximum_range.meters()));
    if (!std::isfinite(radius)) continue;
    domain::LearnedRegion region;
    region.id =
        episode.selection ? episode.selection->decision_id : episode.sequence;
    region.boundary = {
        episode.observation.pose.position,
        domain::Distance(std::max(configuration.minimum_radius_m, radius))};
    region.supporting_observation = episode.observation;
    region.supporting_decision = region.id;
    region.contributing_decisions.push_back(region.id);
    raw.push_back(std::move(region));
  }

  std::vector<domain::LearnedRegion> retained;
  for (auto candidate : raw) {
    bool discarded = false;
    for (std::size_t index = 0U; index < retained.size();) {
      auto& existing = retained[index];
      const double separation =
          domain::distance(existing.boundary.center, candidate.boundary.center)
              .meters();
      const bool overlaps =
          separation <= existing.boundary.radius.meters() +
                            candidate.boundary.radius.meters() +
                            configuration.overlap_tolerance_m;
      if (!overlaps) {
        ++index;
        continue;
      }
      if (existing.boundary.contains(candidate.boundary.center) &&
          existing.boundary.radius.meters() >=
              candidate.boundary.radius.meters()) {
        existing.contributing_decisions.push_back(
            candidate.supporting_decision);
        discarded = true;
        break;
      }
      if (candidate.boundary.contains(existing.boundary.center) &&
          candidate.boundary.radius.meters() >
              existing.boundary.radius.meters()) {
        candidate.id = std::min(candidate.id, existing.id);
        candidate.contributing_decisions.insert(
            candidate.contributing_decisions.end(),
            existing.contributing_decisions.begin(),
            existing.contributing_decisions.end());
        retained.erase(retained.begin() + static_cast<std::ptrdiff_t>(index));
        continue;
      }
      ++index;
    }
    if (!discarded) retained.push_back(std::move(candidate));
  }
  std::sort(
      retained.begin(), retained.end(),
      [](const auto& left, const auto& right) { return left.id < right.id; });

  for (auto& region : retained) {
    double reconciled_radius = std::numeric_limits<double>::infinity();
    std::vector<const domain::LearnedRegion*> contributors;
    for (const auto& evidence : raw) {
      if (!region.boundary.contains(evidence.boundary.center)) continue;
      contributors.push_back(&evidence);
      for (const auto endpoint : rayEndpoints(evidence.supporting_observation))
        reconciled_radius = std::min(
            reconciled_radius,
            domain::distance(region.boundary.center, endpoint).meters());
    }
    if (std::isfinite(reconciled_radius))
      region.boundary.radius = domain::Distance(
          std::max(configuration.minimum_radius_m, reconciled_radius));
    for (const auto* evidence : contributors) {
      const auto endpoints = rayEndpoints(evidence->supporting_observation);
      for (const auto endpoint : endpoints) {
        const double dx = endpoint.x_m - region.boundary.center.x_m;
        const double dy = endpoint.y_m - region.boundary.center.y_m;
        const double distance = std::hypot(dx, dy);
        const auto bin = static_cast<std::size_t>(std::floor(
                             positiveAngle(std::atan2(dy, dx)) * 180.0 / pi)) %
                         360U;
        if (!region.visibility[bin].known ||
            distance > region.visibility[bin].maximum_distance_m) {
          region.visibility[bin] = {
              true, distance, evidence->supporting_observation.pose.position,
              endpoint, evidence->supporting_decision};
        }
      }
    }
    for (const auto decision : region.contributing_decisions)
      region.visibility_revision = std::max(region.visibility_revision,
                                            static_cast<std::size_t>(decision));
  }
  RegionModel result;
  result.learned_regions = std::move(retained);
  for (const auto& region : result.learned_regions)
    result.regions.push_back(region.boundary);
  return result;
}

/**
 * @brief Performs the learn region exits and doors operation for this
 * subsystem.
 *
 * Arguments:
 * - @p regions: Supplies regions input to the operation.
 * - @p paths: Supplies paths input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `DoorExitModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
DoorExitModel learnRegionExitsAndDoors(
    const RegionModel& regions, const std::vector<domain::CompletedPath>& paths,
    const DoorLearningConfiguration& configuration) {
  DoorExitModel result;
  struct ExitAccumulator {
    domain::RegionExit exit;
    double angle{0.0};
  };
  std::vector<ExitAccumulator> exits;
  for (const auto& path : paths) {
    for (const auto& point : path.decision_points) {
      if (!(point.successfulTraversal() || point.partialTraversal())) continue;
      const auto start = point.execution.start_pose.position;
      const auto end = point.execution.final_pose.position;
      const double dx = end.x_m - start.x_m, dy = end.y_m - start.y_m;
      const double a = dx * dx + dy * dy;
      if (a <= domain::geometry_tolerance_m) continue;
      for (const auto& region : regions.learned_regions) {
        const double ox = start.x_m - region.boundary.center.x_m;
        const double oy = start.y_m - region.boundary.center.y_m;
        const double b = 2.0 * (ox * dx + oy * dy);
        const double c =
            ox * ox + oy * oy - std::pow(region.boundary.radius.meters(), 2.0);
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) continue;
        for (double sign : {-1.0, 1.0}) {
          const double t = (-b + sign * std::sqrt(discriminant)) / (2.0 * a);
          if (t < 0.0 || t > 1.0) continue;
          const domain::Point2D crossing{start.x_m + t * dx,
                                         start.y_m + t * dy};
          const double angle = positiveAngle(
              std::atan2(crossing.y_m - region.boundary.center.y_m,
                         crossing.x_m - region.boundary.center.x_m));
          auto existing =
              std::find_if(exits.begin(), exits.end(), [&](const auto& item) {
                return item.exit.region == region.id &&
                       std::abs(normalize(item.angle - angle)) <=
                           configuration.exit_merge_angle_rad;
              });
          if (existing == exits.end()) {
            domain::RegionExit exit;
            exit.id = stableCellId(
                static_cast<long long>(region.id),
                static_cast<long long>(std::llround(angle * 1000.0)), 17U);
            exit.region = region.id;
            exit.point = crossing;
            exit.outward_heading = domain::Angle(std::atan2(dy, dx));
            exit.traversal_count = 1U;
            exit.supporting_paths.push_back(path.id);
            exit.confidence = 0.5;
            exits.push_back({std::move(exit), angle});
          } else {
            const double count =
                static_cast<double>(existing->exit.traversal_count);
            existing->exit.point.x_m =
                (existing->exit.point.x_m * count + crossing.x_m) /
                (count + 1.0);
            existing->exit.point.y_m =
                (existing->exit.point.y_m * count + crossing.y_m) /
                (count + 1.0);
            ++existing->exit.traversal_count;
            existing->exit.confidence = std::min(
                1.0, static_cast<double>(existing->exit.traversal_count) / 2.0);
            if (std::find(existing->exit.supporting_paths.begin(),
                          existing->exit.supporting_paths.end(),
                          path.id) == existing->exit.supporting_paths.end())
              existing->exit.supporting_paths.push_back(path.id);
          }
        }
      }

      const auto& ranges = point.decision_observation.laser.ranges_m;
      const auto endpoints = detail::laserEndpoints(point.decision_observation);
      for (std::size_t index = 1U; index < ranges.size(); ++index) {
        if (!std::isfinite(ranges[index]) ||
            !std::isfinite(ranges[index - 1U]) ||
            std::abs(ranges[index] - ranges[index - 1U]) <
                configuration.sensor_opening_minimum_jump_m)
          continue;
        domain::Segment2D opening{endpoints[index - 1U], endpoints[index]};
        if (opening.length().meters() <=
            configuration.sensor_opening_maximum_width_m)
          result.sensor_openings.push_back(
              {opening, point.selection.decision_id, 0.5});
      }
    }
  }

  std::sort(exits.begin(), exits.end(),
            [](const auto& left, const auto& right) {
              return left.exit.region != right.exit.region
                         ? left.exit.region < right.exit.region
                         : left.angle < right.angle;
            });
  for (const auto& exit : exits) result.exits.push_back(exit.exit);

  for (const auto& region : regions.learned_regions) {
    std::vector<const ExitAccumulator*> around;
    for (const auto& exit : exits)
      if (exit.exit.region == region.id) around.push_back(&exit);
    if (around.size() < 2U) continue;
    const double count = static_cast<double>(around.size());
    const double nearby = (-1.3 * count) / (count + 2.0 * pi) + 1.35;
    std::vector<bool> joined(around.size(), false);
    for (std::size_t index = 0U; index < around.size(); ++index) {
      const std::size_t next = (index + 1U) % around.size();
      const double gap =
          positiveAngle(around[next]->angle - around[index]->angle);
      joined[index] = gap <= nearby;
    }
    std::size_t start = 0U;
    while (start < joined.size() && joined[start]) ++start;
    start = start == joined.size() ? 0U : (start + 1U) % around.size();
    std::size_t consumed = 0U;
    while (consumed < around.size()) {
      std::vector<const ExitAccumulator*> group{around[start]};
      std::size_t current = start;
      while (joined[current] && group.size() < around.size()) {
        current = (current + 1U) % around.size();
        group.push_back(around[current]);
      }
      if (group.size() > 1U) {
        domain::LearnedDoor door;
        door.id = stableCellId(
            static_cast<long long>(region.id),
            static_cast<long long>(std::llround(group.front()->angle * 1000.0)),
            23U);
        door.region = region.id;
        door.clockwise_start_rad = group.front()->angle;
        door.clockwise_end_rad = group.back()->angle;
        for (const auto* exit : group) {
          door.exits.push_back(exit->exit.id);
          door.supporting_traversals += exit->exit.traversal_count;
        }
        door.confidence =
            std::min(1.0, static_cast<double>(door.supporting_traversals) /
                              (2.0 * static_cast<double>(group.size())));
        result.doors.push_back(std::move(door));
      }
      const std::size_t advanced = group.size();
      consumed += advanced;
      start = (current + 1U) % around.size();
      if (advanced == around.size()) break;
    }
  }
  for (const auto& door : result.doors) {
    const auto region = std::find_if(
        regions.learned_regions.begin(), regions.learned_regions.end(),
        [&](const auto& item) { return item.id == door.region; });
    if (region == regions.learned_regions.end()) continue;
    result.openings.push_back(
        {{region->boundary.center.x_m + region->boundary.radius.meters() *
                                            std::cos(door.clockwise_start_rad),
          region->boundary.center.y_m + region->boundary.radius.meters() *
                                            std::sin(door.clockwise_start_rad)},
         {region->boundary.center.x_m + region->boundary.radius.meters() *
                                            std::cos(door.clockwise_end_rad),
          region->boundary.center.y_m + region->boundary.radius.meters() *
                                            std::sin(door.clockwise_end_rad)}});
  }
  return result;
}

/**
 * @brief Performs the learn compatibility hallways operation for this
 * subsystem.
 *
 * Arguments:
 * - @p paths: Supplies paths input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `HallwayModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
HallwayModel learnCompatibilityHallways(
    const std::vector<domain::CompletedPath>& paths,
    const HallwayLearningConfiguration& configuration) {
  struct TravelSegment {
    domain::Segment2D segment;
    domain::RobotObservation observation;
    domain::HallwayDirection direction;
  };
  std::array<std::vector<TravelSegment>, 4U> groups;
  for (const auto& path : paths)
    for (const auto& point : path.decision_points) {
      if (!(point.successfulTraversal() || point.partialTraversal())) continue;
      domain::Segment2D segment{point.execution.start_pose.position,
                                point.execution.final_pose.position};
      if (segment.length().meters() < configuration.minimum_segment_length_m)
        continue;
      const auto direction = directionFor(segment);
      groups[static_cast<std::size_t>(direction)].push_back(
          {segment, point.decision_observation, direction});
    }

  HallwayModel result;
  for (std::size_t group_index = 0U; group_index < groups.size();
       ++group_index) {
    auto& segments = groups[group_index];
    if (segments.size() < 2U) continue;
    struct PairScore {
      std::size_t first, second;
      double score;
    };
    std::vector<PairScore> scores;
    const double bucket_size = std::max(1.0, configuration.comparison_radius_m);
    std::map<std::pair<long long, long long>, std::vector<std::size_t>> buckets;
    for (std::size_t index = 0U; index < segments.size(); ++index) {
      const auto& segment = segments[index].segment;
      const domain::Point2D midpoint{
          (segment.start.x_m + segment.end.x_m) / 2.0,
          (segment.start.y_m + segment.end.y_m) / 2.0};
      buckets[{static_cast<long long>(std::floor(midpoint.x_m / bucket_size)),
               static_cast<long long>(std::floor(midpoint.y_m / bucket_size))}]
          .push_back(index);
    }
    std::set<std::pair<std::size_t, std::size_t>> compared;
    for (const auto& [bucket, members] : buckets) {
      for (long long dy = -1; dy <= 1; ++dy)
        for (long long dx = -1; dx <= 1; ++dx) {
          const auto found =
              buckets.find({bucket.first + dx, bucket.second + dy});
          if (found == buckets.end()) continue;
          for (auto first : members)
            for (auto second : found->second) {
              if (first == second) continue;
              const auto ordered = std::minmax(first, second);
              if (!compared.insert(ordered).second) continue;
              const auto& a = segments[ordered.first].segment;
              const auto& b = segments[ordered.second].segment;
              const domain::Point2D ma{(a.start.x_m + a.end.x_m) / 2.0,
                                       (a.start.y_m + a.end.y_m) / 2.0};
              const domain::Point2D mb{(b.start.x_m + b.end.x_m) / 2.0,
                                       (b.start.y_m + b.end.y_m) / 2.0};
              const double midpoint_distance =
                  domain::distance(ma, mb).meters();
              if (midpoint_distance > configuration.comparison_radius_m)
                continue;
              const double angle_a =
                  std::atan2(a.end.y_m - a.start.y_m, a.end.x_m - a.start.x_m);
              const double angle_b =
                  std::atan2(b.end.y_m - b.start.y_m, b.end.x_m - b.start.x_m);
              const double angle_difference =
                  std::abs(normalize(angle_a - angle_b));
              const double mean_length =
                  (a.length().meters() + b.length().meters()) / 2.0;
              scores.push_back(
                  {ordered.first, ordered.second,
                   mean_length / (1.0 + midpoint_distance + angle_difference)});
            }
        }
    }
    if (scores.empty()) continue;
    const double mean = std::accumulate(scores.begin(), scores.end(), 0.0,
                                        [](double sum, const auto& value) {
                                          return sum + value.score;
                                        }) /
                        static_cast<double>(scores.size());
    double variance = 0.0;
    for (const auto& value : scores)
      variance += (value.score - mean) * (value.score - mean);
    const double deviation =
        std::sqrt(variance / static_cast<double>(scores.size()));
    double sigma = configuration.initial_sigma;
    std::vector<PairScore> parents;
    while (parents.empty() && sigma >= 0.0) {
      const double threshold = mean + sigma * deviation;
      for (const auto& score : scores)
        if (score.score + domain::geometry_tolerance_m >= threshold)
          parents.push_back(score);
      sigma -= configuration.sigma_decrement;
    }

    std::vector<domain::Segment2D> candidates;
    for (const auto& parent : parents) {
      auto first = segments[parent.first].segment;
      auto second = segments[parent.second].segment;
      const double ux =
          std::cos(directionAngle(segments[parent.first].direction));
      const double uy =
          std::sin(directionAngle(segments[parent.first].direction));
      if ((first.end.x_m - first.start.x_m) * ux +
              (first.end.y_m - first.start.y_m) * uy <
          0.0)
        std::swap(first.start, first.end);
      if ((second.end.x_m - second.start.x_m) * ux +
              (second.end.y_m - second.start.y_m) * uy <
          0.0)
        std::swap(second.start, second.end);
      domain::Segment2D child{{(first.start.x_m + second.start.x_m) / 2.0,
                               (first.start.y_m + second.start.y_m) / 2.0},
                              {(first.end.x_m + second.end.x_m) / 2.0,
                               (first.end.y_m + second.end.y_m) / 2.0}};
      if (directionFor(child) != segments[parent.first].direction) continue;
      const bool visible =
          historicallyVisible(segments[parent.first].observation, child.start,
                              0.05) &&
          historicallyVisible(segments[parent.first].observation, child.end,
                              0.05) &&
          historicallyVisible(segments[parent.second].observation, child.start,
                              0.05) &&
          historicallyVisible(segments[parent.second].observation, child.end,
                              0.05);
      if (!visible) continue;
      candidates.push_back(first);
      candidates.push_back(second);
      candidates.push_back(child);
    }
    if (candidates.empty()) continue;

    using Cell = std::pair<long long, long long>;
    std::map<Cell, std::uint32_t> heat;
    for (const auto& segment : candidates)
      for (const auto point :
           rasterize(segment, configuration.heatmap_resolution_m))
        ++heat[{static_cast<long long>(
                    std::floor(point.x_m / configuration.heatmap_resolution_m)),
                static_cast<long long>(std::floor(
                    point.y_m / configuration.heatmap_resolution_m))}];
    for (int iteration = 0; iteration < 2; ++iteration) {
      if (heat.empty()) break;
      long long min_x = heat.begin()->first.first, max_x = min_x;
      long long min_y = heat.begin()->first.second, max_y = min_y;
      for (const auto& [cell, value] : heat) {
        (void)value;
        min_x = std::min(min_x, cell.first);
        max_x = std::max(max_x, cell.first);
        min_y = std::min(min_y, cell.second);
        max_y = std::max(max_y, cell.second);
      }
      std::vector<Cell> additions;
      for (long long y = min_y; y <= max_y; ++y)
        for (long long x = min_x; x <= max_x; ++x) {
          if (heat.contains({x, y})) continue;
          std::size_t neighbors = 0U, hot = 0U;
          for (long long dy = -1; dy <= 1; ++dy)
            for (long long dx = -1; dx <= 1; ++dx) {
              if (dx == 0 && dy == 0) continue;
              if (x + dx < min_x || x + dx > max_x || y + dy < min_y ||
                  y + dy > max_y)
                continue;
              ++neighbors;
              const auto found = heat.find({x + dx, y + dy});
              if (found != heat.end() &&
                  found->second >= configuration.smoothing_threshold)
                ++hot;
            }
          if (neighbors > 0U &&
              static_cast<double>(hot) / static_cast<double>(neighbors) >=
                  configuration.smoothing_neighbor_fraction)
            additions.push_back({x, y});
        }
      for (const auto& cell : additions)
        heat[cell] = configuration.smoothing_threshold;
      if (additions.empty()) break;
    }

    std::set<Cell> remaining;
    for (const auto& [cell, value] : heat)
      if (value > 0U) remaining.insert(cell);
    std::vector<std::vector<Cell>> components;
    while (!remaining.empty()) {
      std::vector<Cell> component;
      std::queue<Cell> frontier;
      frontier.push(*remaining.begin());
      remaining.erase(remaining.begin());
      while (!frontier.empty()) {
        const auto cell = frontier.front();
        frontier.pop();
        component.push_back(cell);
        for (long long dy = -1; dy <= 1; ++dy)
          for (long long dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const Cell neighbor{cell.first + dx, cell.second + dy};
            const auto found = remaining.find(neighbor);
            if (found != remaining.end()) {
              frontier.push(neighbor);
              remaining.erase(found);
            }
          }
      }
      components.push_back(std::move(component));
    }

    // The connected-component pass separates aggregates that have not yet
    // accumulated adjoining heatmap cells.  Chapter 3's final hallway stage
    // joins such aggregates when travel observations from both areas show
    // that the intervening space was mutually visible.  Use union/find so the
    // result is deterministic and transitive, then retain the connecting
    // cells as explicit supporting area.
    const auto componentCentroid = [&](const std::vector<Cell>& component) {
      domain::Point2D centroid;
      for (const auto& cell : component) {
        centroid.x_m += (static_cast<double>(cell.first) + 0.5) *
                        configuration.heatmap_resolution_m;
        centroid.y_m += (static_cast<double>(cell.second) + 0.5) *
                        configuration.heatmap_resolution_m;
      }
      centroid.x_m /= static_cast<double>(component.size());
      centroid.y_m /= static_cast<double>(component.size());
      return centroid;
    };
    std::vector<domain::Point2D> centroids;
    centroids.reserve(components.size());
    for (const auto& component : components)
      centroids.push_back(componentCentroid(component));
    std::vector<std::size_t> parent(components.size());
    std::iota(parent.begin(), parent.end(), 0U);
    const auto findRoot = [&](std::size_t index) {
      while (parent[index] != index) {
        parent[index] = parent[parent[index]];
        index = parent[index];
      }
      return index;
    };
    const auto nearestObservation =
        [&](const domain::Point2D& point) -> const domain::RobotObservation* {
      const domain::RobotObservation* nearest = nullptr;
      double best = std::numeric_limits<double>::infinity();
      for (const auto& segment : segments) {
        const double candidate =
            domain::distance(segment.observation.pose.position, point).meters();
        if (candidate < best) {
          best = candidate;
          nearest = &segment.observation;
        }
      }
      return nearest;
    };
    for (std::size_t first = 0U; first < components.size(); ++first)
      for (std::size_t second = first + 1U; second < components.size();
           ++second) {
        const auto* first_view = nearestObservation(centroids[first]);
        const auto* second_view = nearestObservation(centroids[second]);
        if (first_view == nullptr || second_view == nullptr ||
            !historicallyVisible(*first_view, centroids[second], 0.05) ||
            !historicallyVisible(*second_view, centroids[first], 0.05))
          continue;
        const auto first_root = findRoot(first);
        const auto second_root = findRoot(second);
        if (first_root != second_root) parent[second_root] = first_root;
      }
    std::map<std::size_t, std::set<Cell>> merged_cells;
    for (std::size_t index = 0U; index < components.size(); ++index) {
      const auto root = findRoot(index);
      merged_cells[root].insert(components[index].begin(),
                                components[index].end());
      if (root != index)
        for (const auto point : rasterize({centroids[root], centroids[index]},
                                          configuration.heatmap_resolution_m)) {
          const Cell cell{static_cast<long long>(std::floor(
                              point.x_m / configuration.heatmap_resolution_m)),
                          static_cast<long long>(std::floor(
                              point.y_m / configuration.heatmap_resolution_m))};
          merged_cells[root].insert(cell);
          heat[cell] = std::max(heat[cell], configuration.smoothing_threshold);
        }
    }
    components.clear();
    for (const auto& [root, cells] : merged_cells) {
      (void)root;
      components.emplace_back(cells.begin(), cells.end());
    }

    for (auto& component : components) {
      const auto direction = static_cast<domain::HallwayDirection>(group_index);
      const double ux = std::cos(directionAngle(direction));
      const double uy = std::sin(directionAngle(direction));
      const double vx = -uy, vy = ux;
      double min_u = std::numeric_limits<double>::infinity();
      double max_u = -min_u, min_v = min_u, max_v = -min_u;
      for (const auto& cell : component) {
        const double x = (static_cast<double>(cell.first) + 0.5) *
                         configuration.heatmap_resolution_m;
        const double y = (static_cast<double>(cell.second) + 0.5) *
                         configuration.heatmap_resolution_m;
        const double along = x * ux + y * uy, across = x * vx + y * vy;
        min_u = std::min(min_u, along);
        max_u = std::max(max_u, along);
        min_v = std::min(min_v, across);
        max_v = std::max(max_v, across);
      }
      const double middle_v = (min_v + max_v) / 2.0;
      domain::LearnedHallway hallway;
      const auto anchor = *std::min_element(component.begin(), component.end());
      hallway.id = stableCellId(anchor.first, anchor.second, group_index + 31U);
      hallway.direction = direction;
      hallway.centerline = {
          {min_u * ux + middle_v * vx, min_u * uy + middle_v * vy},
          {max_u * ux + middle_v * vx, max_u * uy + middle_v * vy}};
      hallway.extent_m = max_u - min_u + configuration.heatmap_resolution_m;
      hallway.width_m = max_v - min_v + configuration.heatmap_resolution_m;
      hallway.supporting_segments = candidates.size();
      for (const auto& cell : component)
        hallway.connected_area.push_back({cell.first, cell.second, heat[cell]});
      result.centerlines.push_back(hallway.centerline);
      result.hallways.push_back(std::move(hallway));
    }
  }
  std::sort(
      result.hallways.begin(), result.hallways.end(),
      [](const auto& left, const auto& right) { return left.id < right.id; });
  result.centerlines.clear();
  for (const auto& hallway : result.hallways)
    result.centerlines.push_back(hallway.centerline);
  return result;
}

/**
 * @brief Performs the learn conveyor grid operation for this subsystem.
 *
 * Arguments:
 * - @p trails: Supplies trails input to the operation.
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - `ConveyorModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
ConveyorModel learnConveyorGrid(
    const std::vector<domain::LearnedTrail>& trails,
    const ConveyorLearningConfiguration& configuration) {
  if (!(configuration.resolution_m > 0.0) ||
      !(configuration.decay_factor > 0.0 && configuration.decay_factor <= 1.0))
    throw std::invalid_argument("conveyor resolution and decay are invalid");
  struct Evidence {
    double frequency{0.0}, dx{0.0}, dy{0.0};
  };
  using WorldCell = std::pair<long long, long long>;
  std::map<WorldCell, Evidence> evidence;
  ConveyorModel result;
  for (const auto& trail : trails) {
    if (configuration.decay_factor < 1.0)
      for (auto& [cell, value] : evidence) {
        (void)cell;
        value.frequency *= configuration.decay_factor;
        value.dx *= configuration.decay_factor;
        value.dy *= configuration.decay_factor;
      }
    std::map<WorldCell, std::pair<double, double>> touched;
    for (std::size_t index = 1U; index < trail.markers.size(); ++index) {
      domain::Segment2D segment{trail.markers[index - 1U].pose.position,
                                trail.markers[index].pose.position};
      const double length = segment.length().meters();
      if (length <= domain::geometry_tolerance_m) continue;
      const double dx = (segment.end.x_m - segment.start.x_m) / length;
      const double dy = (segment.end.y_m - segment.start.y_m) / length;
      for (const auto point :
           rasterize(segment, configuration.resolution_m / 2.0))
        touched[{static_cast<long long>(
                     std::floor(point.x_m / configuration.resolution_m)),
                 static_cast<long long>(std::floor(
                     point.y_m / configuration.resolution_m))}] = {dx, dy};
      bool merged = false;
      for (auto& flow : result.flows)
        if (detail::equivalent(flow.axis, segment,
                               configuration.resolution_m / 2.0)) {
          ++flow.traversals;
          merged = true;
          break;
        }
      if (!merged) result.flows.push_back({segment, 1U});
    }
    for (const auto& [cell, direction] : touched) {
      auto& value = evidence[cell];
      value.frequency += 1.0;
      if (configuration.directional) {
        value.dx += direction.first;
        value.dy += direction.second;
      }
    }
  }
  if (evidence.empty()) return result;
  long long min_x = evidence.begin()->first.first, max_x = min_x;
  long long min_y = evidence.begin()->first.second, max_y = min_y;
  double maximum = 0.0;
  for (const auto& [cell, value] : evidence) {
    min_x = std::min(min_x, cell.first);
    max_x = std::max(max_x, cell.first);
    min_y = std::min(min_y, cell.second);
    max_y = std::max(max_y, cell.second);
    maximum = std::max(maximum, value.frequency);
  }
  const auto padding = static_cast<long long>(
      std::ceil(configuration.bounds_padding_m / configuration.resolution_m));
  min_x -= padding;
  min_y -= padding;
  max_x += padding;
  max_y += padding;
  result.grid.geometry = domain::GridGeometry::fromBounds(
      "map",
      {static_cast<double>(min_x) * configuration.resolution_m,
       static_cast<double>(min_y) * configuration.resolution_m},
      {static_cast<double>(max_x + 1) * configuration.resolution_m,
       static_cast<double>(max_y + 1) * configuration.resolution_m},
      configuration.resolution_m, domain::GridExtentMode::Expandable,
      domain::GridExtentSource::RepresentationLocalBounds,
      domain::GridOutOfBoundsBehavior::ExpandBeforeInsert, 1U);
  result.grid.decay_factor = configuration.decay_factor;
  result.grid.maximum_frequency =
      static_cast<std::uint32_t>(std::max(0.0, std::round(maximum)));
  for (const auto& [cell, value] : evidence) {
    const domain::Point2D center{
        (static_cast<double>(cell.first) + 0.5) * configuration.resolution_m,
        (static_cast<double>(cell.second) + 0.5) * configuration.resolution_m};
    const auto index = result.grid.geometry.index(center);
    if (!index) continue;
    const double norm = std::hypot(value.dx, value.dy);
    result.grid.cells.push_back(
        {*index,
         static_cast<std::uint32_t>(std::max(0.0, std::round(value.frequency))),
         norm > 0.0 ? value.dx / norm : 0.0, norm > 0.0 ? value.dy / norm : 0.0,
         maximum > 0.0 ? value.frequency / maximum : 0.0});
  }
  std::sort(result.grid.cells.begin(), result.grid.cells.end(),
            [](const auto& left, const auto& right) {
              return left.index < right.index;
            });
  return result;
}

/**
 * @brief Performs the learn region skeleton operation for this subsystem.
 *
 * Arguments:
 * - @p regions: Supplies regions input to the operation.
 * - @p trails: Supplies trails input to the operation.
 * - @p paths: Supplies paths input to the operation.
 *
 * Returns:
 * - `PassageSkeletonModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PassageSkeletonModel learnRegionSkeleton(
    const RegionModel& regions, const std::vector<domain::LearnedTrail>& trails,
    const std::vector<domain::CompletedPath>& paths) {
  PassageSkeletonModel result;
  for (std::size_t index = 0U; index < regions.learned_regions.size();
       ++index) {
    const auto& region = regions.learned_regions[index];
    result.region_nodes.push_back(
        {index, region.id, region.boundary.center, region.visibility});
    result.nodes.push_back(region.boundary.center);
  }
  std::map<std::pair<std::size_t, std::size_t>, domain::RegionSkeletonEdge>
      edges;
  domain::TrailId next_edge_trail_id = 1U;
  for (const auto& trail : trails)
    next_edge_trail_id = std::max(next_edge_trail_id, trail.id + 1U);
  for (const auto& path : paths) {
    std::optional<std::size_t> origin_region;
    std::optional<domain::Point2D> previous_finish;
    std::vector<domain::PathDecisionPoint> raw_segment;
    for (const auto& decision : path.decision_points) {
      // A skeleton transition is successful-traversal evidence. Partial,
      // rejected, interrupted, and failed actions remain in path history but
      // cannot label a traversable region edge.
      if (!decision.successfulTraversal()) {
        origin_region.reset();
        previous_finish.reset();
        raw_segment.clear();
        continue;
      }
      const auto start = decision.execution.start_pose.position;
      const auto finish = decision.execution.final_pose.position;
      if (!previous_finish ||
          domain::distance(*previous_finish, start).meters() > 0.05) {
        origin_region = containingRegion(regions.learned_regions, start);
        raw_segment.clear();
      }
      raw_segment.push_back(decision);
      const auto destination_region =
          containingRegion(regions.learned_regions, finish);
      if (!origin_region) {
        if (destination_region) {
          origin_region = destination_region;
          raw_segment.clear();
        }
        previous_finish = finish;
        continue;
      }
      if (!destination_region || *destination_region == *origin_region) {
        previous_finish = finish;
        continue;
      }

      domain::CompletedPath transition;
      transition.id = path.id;
      transition.task_id = path.task_id;
      transition.target = path.target;
      transition.decision_points = raw_segment;
      transition.target_reached = true;
      if (!raw_segment.empty()) {
        transition.started_at = raw_segment.front().execution.started_at;
        transition.finished_at = raw_segment.back().execution.finished_at;
      }
      auto learned =
          learnVisibilityTrail(transition, next_edge_trail_id++,
                               TrailLearningConfiguration{0.05, 0.0, false});
      if (learned.markers.size() >= 2U) {
        std::vector<domain::Point2D> compressed;
        compressed.reserve(learned.markers.size());
        for (const auto& marker : learned.markers)
          compressed.push_back(marker.pose.position);
        const auto key = std::minmax(*origin_region, *destination_region);
        auto [found, inserted] = edges.try_emplace(key);
        auto& edge = found->second;
        if (inserted) {
          edge.from = key.first;
          edge.to = key.second;
          edge.length_m = std::numeric_limits<double>::infinity();
        }
        edge.supporting_trails.push_back(
            {*origin_region, *destination_region, learned});
        if (learned.length_m < edge.length_m) {
          if (*origin_region != edge.from)
            std::reverse(compressed.begin(), compressed.end());
          edge.supporting_subtrail = std::move(compressed);
          edge.length_m = learned.length_m;
          edge.source_path = path.id;
          edge.operational_trail_id = learned.id;
        }
      }
      origin_region = destination_region;
      previous_finish = finish;
      raw_segment.clear();
    }
  }
  for (auto& [key, edge] : edges) {
    result.edges.push_back({key.first, key.second});
    result.region_edges.push_back(std::move(edge));
  }
  result.component_by_node.resize(result.nodes.size());
  std::iota(result.component_by_node.begin(), result.component_by_node.end(),
            0U);
  const auto find = [&](std::size_t value, auto&& self) -> std::size_t {
    return result.component_by_node[value] == value
               ? value
               : result.component_by_node[value] =
                     self(result.component_by_node[value], self);
  };
  for (const auto& edge : result.edges) {
    const auto a = find(edge.from, find), b = find(edge.to, find);
    if (a != b) result.component_by_node[b] = a;
  }
  for (auto& component : result.component_by_node)
    component = find(component, find);
  result.connectivity_revision = result.edges.empty() ? 0U : 1U;
  return result;
}

}  // namespace semaforr::spatial
