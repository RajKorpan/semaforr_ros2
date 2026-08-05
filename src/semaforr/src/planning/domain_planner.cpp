#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <semaforr/planning/domain_planner.hpp>
#include <stdexcept>

namespace semaforr::planning {
namespace {

double pointSegmentDistance(domain::Point2D p, const domain::Segment2D& s) {
  const double dx = s.end.x_m - s.start.x_m, dy = s.end.y_m - s.start.y_m;
  const double n = dx * dx + dy * dy;
  const double t =
      n == 0.0
          ? 0.0
          : std::clamp(
                ((p.x_m - s.start.x_m) * dx + (p.y_m - s.start.y_m) * dy) / n,
                0.0, 1.0);
  return std::hypot(p.x_m - (s.start.x_m + t * dx),
                    p.y_m - (s.start.y_m + t * dy));
}

std::size_t nearSegments(domain::Point2D p,
                         const std::vector<domain::Segment2D>& segments,
                         double radius = 0.75) {
  return static_cast<std::size_t>(std::count_if(
      segments.begin(), segments.end(),
      [=](const auto& s) { return pointSegmentDistance(p, s) <= radius; }));
}

std::size_t nearTrailMarkers(
    domain::Point2D p,
    const std::vector<std::vector<domain::Point2D>>& trails) {
  std::size_t count = 0;
  for (const auto& trail : trails)
    count += static_cast<std::size_t>(std::count_if(
        trail.begin(), trail.end(),
        [=](auto q) { return domain::distance(p, q).meters() <= 0.75; }));
  return count;
}

double socialPenalty(const PlanningRequest& request, PlanObjective objective,
                     domain::Point2D midpoint, double length,
                     domain::Point2D from, domain::Point2D to) {
  if (!request.crowd_model) return length;
  const auto sample = request.crowd_model->learnedAt(midpoint);
  if (!sample || sample->stale) return length;
  if (objective == PlanObjective::CrowdDensity)
    return length * (1.0 + std::max(0.0, sample->cell.density));
  if (objective == PlanObjective::EncounterRisk)
    return length * (1.0 + std::max(0.0, sample->cell.learned_encounter_risk));
  const domain::Angle heading(std::atan2(to.y_m - from.y_m, to.x_m - from.x_m));
  return length *
         (2.0 - request.crowd_model->flowAlignmentAt(midpoint, heading));
}

double edgeCost(PlanObjective objective, const PlanningRequest& request,
                domain::Point2D a, domain::Point2D b) {
  const double w = domain::distance(a, b).meters();
  const auto* s = request.spatial_model;
  const domain::Point2D m{(a.x_m + b.x_m) / 2.0, (a.y_m + b.y_m) / 2.0};
  switch (objective) {
    case PlanObjective::Distance:
    case PlanObjective::SkeletonDistance:
    case PlanObjective::HighwayDistance:
      return w;
    case PlanObjective::CrowdDensity:
    case PlanObjective::EncounterRisk:
    case PlanObjective::FlowOpposition:
      return socialPenalty(request, objective, m, w, a, b);
    case PlanObjective::RegionPreference: {
      if (!s) return 10.0 * w;
      const bool ia =
          std::any_of(s->learned_regions.begin(), s->learned_regions.end(),
                      [&](const auto& r) { return r.contains(a); });
      const bool ib =
          std::any_of(s->learned_regions.begin(), s->learned_regions.end(),
                      [&](const auto& r) { return r.contains(b); });
      if (ia && ib) return .25 * w;
      if (ia != ib) {
        const bool door = nearSegments(m, s->doorways) > 0U;
        return (door ? .5 : .75) * w;
      }
      return 10.0 * w;
    }
    case PlanObjective::HallwayPreference: {
      if (!s) return 10.0 * w;
      const auto fa = nearSegments(a, s->hallways),
                 fb = nearSegments(b, s->hallways);
      return fa && fb ? 2.0 * w / static_cast<double>(fa + fb) : 10.0 * w;
    }
    case PlanObjective::TrailPreference: {
      if (!s) return 10.0 * w;
      const auto fa = nearTrailMarkers(a, s->trails),
                 fb = nearTrailMarkers(b, s->trails);
      return fa && fb ? 2.0 * w / static_cast<double>(fa + fb) : 10.0 * w;
    }
    case PlanObjective::ConveyorPreference: {
      if (!s) return 10.0 * w;
      const auto fa = nearSegments(a, s->conveyor_flows),
                 fb = nearSegments(b, s->conveyor_flows);
      if (!fa || !fb) return 10.0 * w;
      double value = static_cast<double>(fa + fb);
      for (auto visits : s->conveyor_traversals)
        value += static_cast<double>(visits);
      return 2.0 * w / std::max(2.0, value);
    }
  }
  return w;
}

std::vector<domain::Point2D> completePath(
    const PlanningRequest& request, const std::vector<domain::Point2D>& p) {
  std::vector<domain::Point2D> result{request.start.position};
  result.insert(result.end(), p.begin(), p.end());
  return result;
}
}  // namespace

ObjectiveCosts evaluatePathObjectives(
    const PlanningRequest& request, const std::vector<domain::Point2D>& path) {
  ObjectiveCosts result;
  const auto points = completePath(request, path);
  for (int raw = static_cast<int>(PlanObjective::Distance);
       raw <= static_cast<int>(PlanObjective::HighwayDistance); ++raw) {
    const auto objective = static_cast<PlanObjective>(raw);
    double total = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i)
      total += edgeCost(objective, request, points[i - 1], points[i]);
    result[objective] = total;
  }
  return result;
}

DomainPlanner::DomainPlanner(std::string name, PlannerObjective objective)
    : DomainPlanner(std::move(name), objective,
                    OccupancySourceMode::StaticMapWithSensors) {}

DomainPlanner::DomainPlanner(std::string name, PlannerObjective objective,
                             OccupancySourceMode source_mode)
    : name_(std::move(name)), objective_(objective), source_mode_(source_mode) {
  if (name_.empty())
    throw std::invalid_argument("planner name must not be empty");
}

PlanResult DomainPlanner::plan(const PlanningRequest& request) {
  if (!request.start.position.finite() || !request.goal.finite())
    return {
        PlanStatus::InvalidRequest, {}, 0.0, "start and goal must be finite"};
  std::optional<TraversabilityBuildResult> traversability;
  if (source_mode_ !=
      OccupancySourceMode::LearnedFreespaceWithOptionalOccupancy) {
    auto traversal_configuration = request.traversability;
    if (source_mode_ == OccupancySourceMode::SensorDerivedPartial)
      traversal_configuration.unknown_policy =
          traversal_configuration.sensor_unknown_policy;
    traversability = deriveTraversability(
        source_mode_, request.static_map,
        request.spatial_model ? &request.spatial_model->sensed_occupancy
                              : nullptr,
        traversal_configuration);
    if (!traversability->grid.valid())
      return {PlanStatus::PlannerUnavailable, {}, 0.0,
              traversability->diagnostic};
    if (source_mode_ == OccupancySourceMode::StaticMapWithSensors &&
        (!request.static_map->bounds.contains(request.start.position) ||
         !request.static_map->bounds.contains(request.goal)))
      return {PlanStatus::InvalidRequest, {}, 0.0,
              "start or goal lies outside static-map bounds"};
    const auto start_cell =
        traversability->grid.geometry.index(request.start.position);
    const auto goal_cell = traversability->grid.geometry.index(request.goal);
    if (!start_cell || !goal_cell)
      return {PlanStatus::InvalidRequest, {}, 0.0,
              "start or goal lies outside the planning extent"};
    if (!traversability->grid.cells[*start_cell].permitsTraversal() ||
        !traversability->grid.cells[*goal_cell].permitsTraversal())
      return {PlanStatus::NoPath, {}, 0.0,
              "start or goal is not traversable under the selected occupancy "
              "policy"};
  }
  std::vector<domain::Point2D> nodes;
  std::vector<float> node_costs;
  std::vector<std::pair<std::size_t, std::size_t>> edges;
  if (traversability) {
    const auto& grid = traversability->grid;
    if (grid.valid()) {
      std::vector<std::size_t> node_for_cell(grid.cells.size(),
                                             grid.cells.size());
      for (std::size_t row = 0; row < grid.geometry.rows; ++row)
        for (std::size_t column = 0; column < grid.geometry.columns; ++column) {
          const std::size_t cell = row * grid.geometry.columns + column;
          if (!grid.cells[cell].permitsTraversal()) continue;
          const domain::Point2D point = grid.geometry.center(cell);
          node_for_cell[cell] = nodes.size();
          nodes.push_back(point);
          node_costs.push_back(grid.cells[cell].cost_multiplier);
        }
      for (std::size_t row = 0; row < grid.geometry.rows; ++row)
        for (std::size_t column = 0; column < grid.geometry.columns; ++column) {
          const std::size_t cell = row * grid.geometry.columns + column;
          if (node_for_cell[cell] >= nodes.size()) continue;
          if (column + 1U < grid.geometry.columns &&
              node_for_cell[cell + 1U] < nodes.size())
            edges.emplace_back(node_for_cell[cell], node_for_cell[cell + 1U]);
          if (row + 1U < grid.geometry.rows &&
              node_for_cell[cell + grid.geometry.columns] < nodes.size())
            edges.emplace_back(node_for_cell[cell],
                               node_for_cell[cell + grid.geometry.columns]);
        }
    }
  } else if (request.spatial_model) {
    nodes = request.spatial_model->skeleton_nodes;
    node_costs.assign(nodes.size(), 1.0F);
    edges = request.spatial_model->skeleton_edges;
  }
  if (nodes.empty()) {
    return {PlanStatus::PlannerUnavailable, {}, 0.0,
            traversability
                ? "derived traversability contains no permitted cells"
                : "learned freespace graph is unavailable"};
  }
  const std::size_t original = nodes.size(), start = nodes.size();
  nodes.push_back(request.start.position);
  node_costs.push_back(1.0F);
  const std::size_t goal = nodes.size();
  nodes.push_back(request.goal);
  node_costs.push_back(1.0F);
  auto attach = [&](std::size_t id) {
    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < original; ++i) {
      double d = domain::distance(nodes[id], nodes[i]).meters();
      if (d < best) {
        best = d;
        nearest = i;
      }
    }
    edges.emplace_back(id, nearest);
  };
  attach(start);
  attach(goal);
  std::vector<std::vector<std::pair<std::size_t, double>>> adjacency(
      nodes.size());
  for (const auto& [a, b] : edges) {
    if (a >= nodes.size() || b >= nodes.size())
      return {PlanStatus::InvalidRequest,
              {},
              0.0,
              "planning graph contains an invalid edge"};
    const double c = edgeCost(objective_, request, nodes[a], nodes[b]) *
                     (static_cast<double>(node_costs[a]) +
                      static_cast<double>(node_costs[b])) /
                     2.0;
    adjacency[a].push_back({b, c});
    adjacency[b].push_back({a, c});
  }
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<>> queue;
  std::vector<double> distance(nodes.size(),
                               std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(nodes.size(), nodes.size());
  distance[start] = 0;
  queue.push({0, start});
  while (!queue.empty()) {
    auto [cost, u] = queue.top();
    queue.pop();
    if (cost != distance[u]) continue;
    if (u == goal) break;
    for (auto [v, w] : adjacency[u])
      if (cost + w < distance[v]) {
        distance[v] = cost + w;
        parent[v] = u;
        queue.push({distance[v], v});
      }
  }
  if (!std::isfinite(distance[goal]))
    return {PlanStatus::NoPath, {}, 0.0, "no path in shared planning graph"};
  std::vector<std::size_t> ids;
  for (std::size_t u = goal; u != start; u = parent[u]) {
    if (u >= nodes.size() || parent[u] >= nodes.size())
      return {PlanStatus::NoPath, {}, 0.0, "broken predecessor chain"};
    ids.push_back(u);
  }
  std::reverse(ids.begin(), ids.end());
  PlanResult result;
  result.status = PlanStatus::Success;
  for (auto id : ids) result.path.push_back(nodes[id]);
  result.cost_m = distance[goal];
  result.primary_objective = objective_;
  result.objective_costs = evaluatePathObjectives(request, result.path);
  result.explanation = "Dijkstra over " +
                       (traversability ? traversability->diagnostic
                                       : "learned freespace representation") +
                       " using the " + std::string(toString(objective_)) +
                       " objective";
  HierarchicalPlan hierarchy;
  hierarchy.planner = name_;
  hierarchy.objective = objective_;
  hierarchy.provenance = result.explanation;
  hierarchy.estimated_objective_costs = result.objective_costs;
  if (request.spatial_model)
    hierarchy.source_model_revisions["spatial"] =
        request.spatial_model->revision;
  if (request.static_map)
    hierarchy.source_model_revisions["static_map"] =
        request.static_map->revision;
  if (request.spatial_model)
    hierarchy.source_model_revisions["sensed_occupancy"] =
        request.spatial_model->sensed_occupancy.revision;
  for (auto p : result.path) hierarchy.steps.emplace_back(WaypointStep{p});
  result.hierarchical = std::move(hierarchy);
  return result;
}
}  // namespace semaforr::planning
