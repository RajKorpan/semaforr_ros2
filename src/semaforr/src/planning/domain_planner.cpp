/**
 * @file domain_planner.cpp
 * @brief Domain planner responsibilities.
 *
 * @details This file implements domain planner behavior for path planning and
 * hierarchical plan construction. It records the declarations, settings,
 * fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/planning/domain_planner.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <queue>
#include <semaforr/planning/astar.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <stdexcept>

namespace semaforr::planning {
namespace {

/**
 * @brief Performs the point segment distance operation for this subsystem.
 *
 * Arguments:
 * - @p p: Supplies p input to the operation.
 * - @p s: Supplies s input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Performs the near segments operation for this subsystem.
 *
 * Arguments:
 * - @p p: Supplies p input to the operation.
 * - @p segments: Supplies segments input to the operation.
 * - @p radius: Supplies radius input to the operation.
 *
 * Returns:
 * - `std::size_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::size_t nearSegments(domain::Point2D p,
                         const std::vector<domain::Segment2D>& segments,
                         double radius = 0.75) {
  return static_cast<std::size_t>(std::count_if(
      segments.begin(), segments.end(),
      [=](const auto& s) { return pointSegmentDistance(p, s) <= radius; }));
}

/**
 * @brief Performs the near trail markers operation for this subsystem.
 *
 * Arguments:
 * - @p p: Supplies p input to the operation.
 * - @p trails: Supplies trails input to the operation.
 *
 * Returns:
 * - `std::size_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Performs the near exits operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 * - @p exits: Supplies exits input to the operation.
 * - @p radius: Supplies radius input to the operation.
 *
 * Returns:
 * - `std::size_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::size_t nearExits(domain::Point2D point,
                      const std::vector<domain::RegionExit>& exits,
                      double radius = 0.5) {
  return static_cast<std::size_t>(std::count_if(
      exits.begin(), exits.end(), [&](const auto& exit) {
        return domain::distance(point, exit.point).meters() <= radius;
      }));
}

/**
 * @brief Performs the hallway contains operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 * - @p hallway: Supplies hallway input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool hallwayContains(domain::Point2D point,
                     const domain::LearnedHallway& hallway) {
  return pointSegmentDistance(point, hallway.centerline) <=
         std::max(0.5, hallway.width_m / 2.0);
}

/**
 * @brief Performs the social penalty operation for this subsystem.
 *
 * Arguments:
 * - @p request: Supplies request input to the operation.
 * - @p objective: Supplies objective input to the operation.
 * - @p midpoint: Supplies midpoint input to the operation.
 * - @p length: Supplies length input to the operation.
 * - @p from: Supplies from input to the operation.
 * - @p to: Supplies to input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Performs the edge cost operation for this subsystem.
 *
 * Arguments:
 * - @p objective: Supplies objective input to the operation.
 * - @p request: Supplies request input to the operation.
 * - @p a: Supplies a input to the operation.
 * - @p b: Supplies b input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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
        const bool exit = nearExits(m, s->exits) > 0U;
        if (door && exit) return .5 * w;
        return (door || exit ? .75 : 1.0) * w;
      }
      return 10.0 * w;
    }
    case PlanObjective::HallwayPreference: {
      if (!s) return 10.0 * w;
      const auto fa = s->hallway_entities.empty()
                          ? nearSegments(a, s->hallways)
                          : static_cast<std::size_t>(std::count_if(
                                s->hallway_entities.begin(),
                                s->hallway_entities.end(),
                                [&](const auto& hallway) {
                                  return hallwayContains(a, hallway);
                                }));
      const auto fb = s->hallway_entities.empty()
                          ? nearSegments(b, s->hallways)
                          : static_cast<std::size_t>(std::count_if(
                                s->hallway_entities.begin(),
                                s->hallway_entities.end(),
                                [&](const auto& hallway) {
                                  return hallwayContains(b, hallway);
                                }));
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
      if (s->conveyor_grid.geometry.valid()) {
        const auto* first = s->conveyor_grid.at(a);
        const auto* second = s->conveyor_grid.at(b);
        if (!first || !second || first->traversal_frequency == 0U ||
            second->traversal_frequency == 0U)
          return 10.0 * w;
        return 2.0 * w /
               static_cast<double>(first->traversal_frequency +
                                   second->traversal_frequency);
      }
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

/**
 * @brief Performs the complete path operation for this subsystem.
 *
 * Arguments:
 * - @p request: Supplies request input to the operation.
 * - @p p: Supplies p input to the operation.
 *
 * Returns:
 * - `std::vector<domain::Point2D>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<domain::Point2D> completePath(
    const PlanningRequest& request, const std::vector<domain::Point2D>& p) {
  std::vector<domain::Point2D> result{request.start.position};
  result.insert(result.end(), p.begin(), p.end());
  return result;
}
}  // namespace

/**
 * @brief Performs the dependencies operation for this subsystem.
 *
 * Arguments:
 * - @p request: Supplies request input to the operation.
 *
 * Returns:
 * - `std::vector<domain::ModelDependency>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<domain::ModelDependency> DomainPlanner::dependencies(
    const PlanningRequest& request) const {
  using D = domain::ModelDependency;
  std::vector<D> result;
  if (source_mode_ == OccupancySourceMode::StaticMapWithSensors) {
    result = {D::StaticMapGeometry, D::StaticOccupancy,
              D::SensedOccupancy};
  } else if (source_mode_ == OccupancySourceMode::SensorDerivedPartial) {
    result = {D::SensedOccupancy};
  } else if (request.static_map && request.static_map->occupancyAvailable()) {
    result = {D::StaticMapGeometry, D::StaticOccupancy,
              D::SensedOccupancy};
  } else {
    result = {D::SensedOccupancy};
  }
  switch (objective_) {
    case PlanObjective::CrowdDensity:
      result.push_back(D::CrowdDensity);
      break;
    case PlanObjective::EncounterRisk:
      result.push_back(D::CrowdRisk);
      break;
    case PlanObjective::FlowOpposition:
      result.push_back(D::CrowdFlow);
      break;
    case PlanObjective::RegionPreference:
      result.push_back(D::Regions);
      result.push_back(D::DoorsAndExits);
      break;
    case PlanObjective::HallwayPreference:
      result.push_back(D::Hallways);
      break;
    case PlanObjective::TrailPreference:
      result.push_back(D::Trails);
      break;
    case PlanObjective::ConveyorPreference:
      result.push_back(D::Conveyors);
      break;
    default:
      break;
  }
  return result;
}

/**
 * @brief Evaluates path objectives for this subsystem.
 *
 * Arguments:
 * - @p request: Supplies request input to the operation.
 * - @p path: Supplies path input to the operation.
 *
 * Returns:
 * - `ObjectiveCosts` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Performs the domain planner operation for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 * - @p objective: Supplies objective input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
DomainPlanner::DomainPlanner(std::string name, PlannerObjective objective)
    : DomainPlanner(std::move(name), objective,
                    OccupancySourceMode::StaticMapWithSensors) {}

/**
 * @brief Performs the domain planner operation for this subsystem.
 *
 * Arguments:
 * - @p name: Supplies name input to the operation.
 * - @p objective: Supplies objective input to the operation.
 * - @p source_mode: Supplies source mode input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
DomainPlanner::DomainPlanner(std::string name, PlannerObjective objective,
                             OccupancySourceMode source_mode)
    : name_(std::move(name)), objective_(objective), source_mode_(source_mode) {
  if (name_.empty())
    throw std::invalid_argument("planner name must not be empty");
}

/**
 * @brief Constructs package content for this subsystem.
 *
 * Arguments:
 * - @p request: Supplies request input to the operation.
 *
 * Returns:
 * - `PlanResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PlanResult DomainPlanner::plan(const PlanningRequest& request) {
  if (!request.start.position.finite() || !request.goal.finite())
    return {
        PlanStatus::InvalidRequest, {}, 0.0, "start and goal must be finite"};
  std::optional<TraversabilityBuildResult> traversability;
  {
    auto traversal_configuration = request.traversability;
    auto effective_source = source_mode_;
    if (source_mode_ == OccupancySourceMode::StaticOrSensorDerived)
      effective_source = request.static_map && request.static_map->occupancyAvailable()
                             ? OccupancySourceMode::StaticMapWithSensors
                             : OccupancySourceMode::SensorDerivedPartial;
    if (effective_source == OccupancySourceMode::SensorDerivedPartial)
      traversal_configuration.unknown_policy =
          traversal_configuration.sensor_unknown_policy;
    traversability = deriveTraversability(
        effective_source, request.static_map,
        request.spatial_model ? &request.spatial_model->sensed_occupancy
                              : nullptr,
        traversal_configuration);
    if (!traversability->grid.valid())
      return {PlanStatus::PlannerUnavailable, {}, 0.0,
              traversability->diagnostic};
    if (effective_source == OccupancySourceMode::StaticMapWithSensors &&
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
  }
  if (nodes.empty()) {
    return {PlanStatus::PlannerUnavailable, {}, 0.0,
            "derived traversability contains no permitted cells"};
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
  Graph distance_graph;
  if (objective_ == PlanObjective::Distance)
    for (const auto& node : nodes) distance_graph.addVertex(node);
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
    if (objective_ == PlanObjective::Distance)
      distance_graph.addUndirectedEdge(a, b, {c, 0.0, 0.0});
    adjacency[a].push_back({b, c});
    adjacency[b].push_back({a, c});
  }
  std::vector<std::size_t> ids;
  double selected_cost = 0.0;
  std::string search_algorithm;
  if (objective_ == PlanObjective::Distance) {
    const auto path = AStar{}.search(distance_graph, start, goal);
    if (!path.succeeded())
      return {PlanStatus::NoPath, {}, 0.0,
              "no path in shared planning graph"};
    ids.assign(std::next(path.vertices.begin()), path.vertices.end());
    selected_cost = path.cost;
    search_algorithm = "A*";
  } else {
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
      return {PlanStatus::NoPath, {}, 0.0,
              "no path in shared planning graph"};
    for (std::size_t u = goal; u != start; u = parent[u]) {
      if (u >= nodes.size() || parent[u] >= nodes.size())
        return {PlanStatus::NoPath, {}, 0.0, "broken predecessor chain"};
      ids.push_back(u);
    }
    std::reverse(ids.begin(), ids.end());
    selected_cost = distance[goal];
    search_algorithm = "Dijkstra";
  }
  PlanResult result;
  result.status = PlanStatus::Success;
  for (auto id : ids) result.path.push_back(nodes[id]);
  result.cost_m = selected_cost;
  result.primary_objective = objective_;
  result.objective_costs = evaluatePathObjectives(request, result.path);
  result.explanation = search_algorithm + " over " +
                       traversability->diagnostic +
                       " using the " + std::string(toString(objective_)) +
                       " objective";
  HierarchicalPlan hierarchy;
  hierarchy.family = PlanFamily::Grid;
  hierarchy.planner = name_;
  hierarchy.objective = objective_;
  hierarchy.provenance = result.explanation;
  hierarchy.estimated_objective_costs = result.objective_costs;
  hierarchy.geometric_path = result.path;
  for (auto p : result.path) hierarchy.steps.emplace_back(WaypointStep{p});
  result.hierarchical = std::move(hierarchy);
  result.family = PlanFamily::Grid;
  attachDependencySnapshot(result, request, dependencies(request));
  return result;
}
}  // namespace semaforr::planning
semaforr::planning::PlannerMetadata
/**
 * @brief Performs the metadata operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::planning::DomainPlanner::metadata() const {
  const bool mapless = source_mode_ != OccupancySourceMode::StaticMapWithSensors;
  return {name_, PlanFamily::Grid, objective_, std::string(toString(objective_)),
          std::string(objectiveDescription(objective_)),
          {mapless ? "sensed_occupancy" : "static_occupancy"}, !mapless,
          mapless};
}
