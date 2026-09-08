/**
 * @file hierarchical_plan.cpp
 * @brief Hierarchical plan responsibilities.
 *
 * @details This file implements hierarchical plan behavior for path planning
 * and hierarchical plan construction. It centers on `RegionSurrogate`,
 * `Selection`, `VisibleCandidate`, `SkeletonRoute`, `RegionAccess`,
 * `HighwayAttachment`, `HighwayRoute`. Its package-relative location is
 * `src/planning/hierarchical_plan.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <sstream>
#include <tuple>

namespace semaforr::planning {
namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double tie_tolerance = 1e-9;

/**
 * @brief Performs the polyline length operation for this subsystem.
 *
 * Arguments:
 * - @p points: Supplies points input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double polylineLength(const std::vector<domain::Point2D>& points) {
  double length = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index)
    length += domain::distance(points[index - 1U], points[index]).meters();
  return length;
}

/**
 * @brief Performs the region boundary operation for this subsystem.
 *
 * Arguments:
 * - @p spatial: Supplies spatial input to the operation.
 * - @p node_index: Supplies node index input to the operation.
 *
 * Returns:
 * - `std::optional<domain::Circle>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<domain::Circle> regionBoundary(
    const domain::SpatialModel& spatial, std::size_t node_index) {
  if (node_index >= spatial.region_skeleton_nodes.size()) return std::nullopt;
  const auto region_id = spatial.region_skeleton_nodes[node_index].region;
  const auto learned = std::find_if(
      spatial.regions.begin(), spatial.regions.end(),
      [region_id](const auto& region) { return region.id == region_id; });
  if (learned != spatial.regions.end()) return learned->boundary;
  if (node_index < spatial.learned_regions.size())
    return spatial.learned_regions[node_index];
  return std::nullopt;
}

/**
 * @brief Performs the skeleton degrees operation for this subsystem.
 *
 * Arguments:
 * - @p spatial: Supplies spatial input to the operation.
 *
 * Returns:
 * - `std::vector<std::size_t>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<std::size_t> skeletonDegrees(const domain::SpatialModel& spatial) {
  std::vector<std::size_t> result(spatial.region_skeleton_nodes.size(), 0U);
  for (const auto& edge : spatial.region_skeleton_edges) {
    if (edge.from < result.size()) ++result[edge.from];
    if (edge.to < result.size()) ++result[edge.to];
  }
  return result;
}

/**
 * @brief Performs the visibility bin operation for this subsystem.
 *
 * Arguments:
 * - @p origin: Supplies origin input to the operation.
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `std::size_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::size_t visibilityBin(domain::Point2D origin, domain::Point2D point) {
  double angle = std::atan2(point.y_m - origin.y_m, point.x_m - origin.x_m);
  if (angle < 0.0) angle += 2.0 * pi;
  return static_cast<std::size_t>(std::floor(angle * 180.0 / pi)) % 360U;
}

/**
 * @brief Encapsulates region surrogate state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct RegionSurrogate {
  /**
   * @brief Enumerates the supported selection values used by this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - Not applicable to this declaration.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  enum class Selection { Contained, Visible, DegreeDistance } selection;
  std::size_t node{0U};
  std::optional<VisibilityConnectionStep> connection;
  std::vector<std::string> diagnostics;
};

/**
 * @brief Performs the select region surrogate operation for this subsystem.
 *
 * Arguments:
 * - @p spatial: Supplies spatial input to the operation.
 * - @p point: Supplies point input to the operation.
 * - @p start_side: Supplies start side input to the operation.
 *
 * Returns:
 * - `std::optional<RegionSurrogate>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<RegionSurrogate> selectRegionSurrogate(
    const domain::SpatialModel& spatial, domain::Point2D point,
    bool start_side) {
  if (spatial.region_skeleton_nodes.empty()) return std::nullopt;
  const auto degrees = skeletonDegrees(spatial);
  std::optional<std::size_t> contained;
  double retained_radius = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < spatial.region_skeleton_nodes.size();
       ++index) {
    const auto boundary = regionBoundary(spatial, index);
    if (!boundary || !boundary->contains(point)) continue;
    const double radius = boundary->radius.meters();
    if (radius < retained_radius - tie_tolerance ||
        (std::abs(radius - retained_radius) <= tie_tolerance &&
         (!contained || index < *contained))) {
      contained = index;
      retained_radius = radius;
    }
  }
  if (contained) {
    RegionSurrogate result{
        RegionSurrogate::Selection::Contained, *contained, std::nullopt, {}};
    result.diagnostics.push_back("surrogate:contained:region=" +
                                 std::to_string(*contained));
    return result;
  }

  struct VisibleCandidate {
    std::size_t node;
    double distance;
    const domain::RegionVisibilityBin* ray;
  };
  std::vector<VisibleCandidate> visible;
  for (std::size_t index = 0U; index < spatial.region_skeleton_nodes.size();
       ++index) {
    const auto& node = spatial.region_skeleton_nodes[index];
    const double distance = domain::distance(node.center, point).meters();
    const auto& ray = node.visibility[visibilityBin(node.center, point)];
    if (ray.known && distance <= ray.maximum_distance_m + 0.05)
      visible.push_back({index, distance, &ray});
  }
  if (!visible.empty()) {
    std::sort(visible.begin(), visible.end(),
              [&](const auto& left, const auto& right) {
                if (std::abs(left.distance - right.distance) > tie_tolerance)
                  return left.distance < right.distance;
                if (degrees[left.node] != degrees[right.node])
                  return degrees[left.node] > degrees[right.node];
                return left.node < right.node;
              });
    const auto& selected = visible.front();
    const auto center = spatial.region_skeleton_nodes[selected.node].center;
    VisibilityConnectionStep connection;
    connection.region_id = selected.node;
    connection.from = start_side ? point : center;
    connection.to = start_side ? center : point;
    connection.evidence_ray_start = selected.ray->ray_start;
    connection.evidence_ray_end = selected.ray->ray_end;
    connection.supporting_decision = selected.ray->decision_id;
    connection.toward_region = start_side;
    RegionSurrogate result{
        RegionSurrogate::Selection::Visible, selected.node, connection, {}};
    for (const auto& candidate : visible) {
      std::ostringstream trace;
      trace << "surrogate:visible:candidate=" << candidate.node
            << ",distance=" << candidate.distance
            << ",degree=" << degrees[candidate.node];
      result.diagnostics.push_back(trace.str());
    }
    result.diagnostics.push_back("surrogate:visible:selected=" +
                                 std::to_string(selected.node));
    return result;
  }

  std::size_t selected = 0U;
  double best_score = std::numeric_limits<double>::infinity();
  RegionSurrogate result{
      RegionSurrogate::Selection::DegreeDistance, 0U, std::nullopt, {}};
  for (std::size_t index = 0U; index < spatial.region_skeleton_nodes.size();
       ++index) {
    const double distance =
        domain::distance(point, spatial.region_skeleton_nodes[index].center)
            .meters();
    const double score = distance / static_cast<double>(degrees[index] + 1U);
    std::ostringstream trace;
    trace << "surrogate:degree_distance:candidate=" << index
          << ",distance=" << distance << ",degree=" << degrees[index]
          << ",score=" << score;
    result.diagnostics.push_back(trace.str());
    if (score < best_score - tie_tolerance ||
        (std::abs(score - best_score) <= tie_tolerance && index < selected)) {
      best_score = score;
      selected = index;
    }
  }
  result.node = selected;
  result.diagnostics.push_back("surrogate:degree_distance:selected=" +
                               std::to_string(selected));
  return result;
}

/**
 * @brief Encapsulates skeleton route state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct SkeletonRoute {
  std::vector<std::size_t> nodes;
  double cost{0.0};
};

/**
 * @brief Performs the shortest skeleton route operation for this subsystem.
 *
 * Arguments:
 * - @p spatial: Supplies spatial input to the operation.
 * - @p start: Supplies start input to the operation.
 * - @p goal: Supplies goal input to the operation.
 *
 * Returns:
 * - `std::optional<SkeletonRoute>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<SkeletonRoute> shortestSkeletonRoute(
    const domain::SpatialModel& spatial, std::size_t start, std::size_t goal) {
  const std::size_t count = spatial.region_skeleton_nodes.size();
  if (start >= count || goal >= count) return std::nullopt;
  std::vector<std::vector<std::pair<std::size_t, double>>> adjacency(count);
  for (const auto& edge : spatial.region_skeleton_edges) {
    if (edge.from >= count || edge.to >= count ||
        edge.supporting_subtrail.empty())
      continue;
    const double cost = edge.length_m > 0.0
                            ? edge.length_m
                            : polylineLength(edge.supporting_subtrail);
    adjacency[edge.from].push_back({edge.to, cost});
    adjacency[edge.to].push_back({edge.from, cost});
  }
  using Item = std::pair<double, std::size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
  std::vector<double> distance(count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> predecessor(count, count);
  distance[start] = 0.0;
  queue.push({0.0, start});
  while (!queue.empty()) {
    const auto [cost, node] = queue.top();
    queue.pop();
    if (cost > distance[node] + tie_tolerance) continue;
    if (node == goal) break;
    for (const auto& [neighbor, edge_cost] : adjacency[node]) {
      const double candidate = cost + edge_cost;
      if (candidate < distance[neighbor] - tie_tolerance ||
          (std::abs(candidate - distance[neighbor]) <= tie_tolerance &&
           node < predecessor[neighbor])) {
        distance[neighbor] = candidate;
        predecessor[neighbor] = node;
        queue.push({candidate, neighbor});
      }
    }
  }
  if (!std::isfinite(distance[goal])) return std::nullopt;
  std::vector<std::size_t> nodes;
  for (std::size_t node = goal;; node = predecessor[node]) {
    nodes.push_back(node);
    if (node == start) break;
    if (predecessor[node] >= count) return std::nullopt;
  }
  std::reverse(nodes.begin(), nodes.end());
  return SkeletonRoute{std::move(nodes), distance[goal]};
}

/**
 * @brief Performs the skeleton edge operation for this subsystem.
 *
 * Arguments:
 * - @p spatial: Supplies spatial input to the operation.
 * - @p from: Supplies from input to the operation.
 * - @p to: Supplies to input to the operation.
 *
 * Returns:
 * - `const domain::RegionSkeletonEdge*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const domain::RegionSkeletonEdge* skeletonEdge(
    const domain::SpatialModel& spatial, std::size_t from, std::size_t to) {
  const auto found =
      std::find_if(spatial.region_skeleton_edges.begin(),
                   spatial.region_skeleton_edges.end(), [&](const auto& edge) {
                     return (edge.from == from && edge.to == to) ||
                            (edge.from == to && edge.to == from);
                   });
  return found == spatial.region_skeleton_edges.end() ? nullptr : &*found;
}

/**
 * @brief Performs the append skeleton route operation for this subsystem.
 *
 * Arguments:
 * - @p spatial: Supplies spatial input to the operation.
 * - @p route: Supplies route input to the operation.
 * - @p steps: Supplies steps input to the operation.
 * - @p include_first_region: Supplies include first region input to the
 * operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void appendSkeletonRoute(const domain::SpatialModel& spatial,
                         const SkeletonRoute& route,
                         std::vector<PlanStep>& steps,
                         bool include_first_region = true) {
  if (route.nodes.empty()) return;
  if (include_first_region) {
    const auto node = route.nodes.front();
    steps.emplace_back(
        RegionStep{node, spatial.region_skeleton_nodes[node].center});
  }
  for (std::size_t index = 1U; index < route.nodes.size(); ++index) {
    const auto from = route.nodes[index - 1U];
    const auto to = route.nodes[index];
    const auto* edge = skeletonEdge(spatial, from, to);
    if (!edge) continue;
    auto subtrail = edge->supporting_subtrail;
    if (edge->from != from) std::reverse(subtrail.begin(), subtrail.end());
    steps.emplace_back(SkeletonTransitionStep{from, to, std::move(subtrail)});
    steps.emplace_back(
        RegionStep{to, spatial.region_skeleton_nodes[to].center});
  }
}

/**
 * @brief Performs the geometry for operation for this subsystem.
 *
 * Arguments:
 * - @p steps: Supplies steps input to the operation.
 *
 * Returns:
 * - `std::vector<domain::Point2D>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<domain::Point2D> geometryFor(const std::vector<PlanStep>& steps) {
  std::vector<domain::Point2D> result;
  for (const auto& step : steps)
    if (const auto target = stepTarget(step);
        target && (result.empty() || result.back() != *target))
      result.push_back(*target);
  return result;
}

/**
 * @brief Builds skeleton plan for this subsystem.
 *
 * Arguments:
 * - @p request: Supplies request input to the operation.
 * - @p planner_name: Supplies planner name input to the operation.
 *
 * Returns:
 * - `PlanResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PlanResult buildSkeletonPlan(const PlanningRequest& request,
                             std::string planner_name) {
  if (!request.spatial_model)
    return {PlanStatus::PlannerUnavailable,
            {},
            0.0,
            "spatial model is unavailable"};
  const auto& spatial = *request.spatial_model;
  if (spatial.region_skeleton_nodes.empty())
    return {
        PlanStatus::PlannerUnavailable,
        {},
        0.0,
        "region skeleton is unavailable; sampled path graphs are not accepted"};
  const auto start =
      selectRegionSurrogate(spatial, request.start.position, true);
  const auto goal = selectRegionSurrogate(spatial, request.goal, false);
  if (!start || !goal)
    return {PlanStatus::PlannerUnavailable,
            {},
            0.0,
            "region surrogates are unavailable"};
  const auto route = shortestSkeletonRoute(spatial, start->node, goal->node);
  if (!route)
    return {PlanStatus::NoPath,
            {},
            0.0,
            "selected region surrogates are disconnected"};

  HierarchicalPlan plan;
  plan.family = PlanFamily::Model;
  plan.planner = std::move(planner_name);
  plan.objective = PlanObjective::SkeletonDistance;
  plan.provenance =
      "region surrogate hierarchy and learned skeleton edge trails";
  plan.diagnostics = start->diagnostics;
  plan.diagnostics.insert(plan.diagnostics.end(), goal->diagnostics.begin(),
                          goal->diagnostics.end());
  if (start->connection) plan.steps.emplace_back(*start->connection);
  appendSkeletonRoute(spatial, *route, plan.steps);
  if (goal->connection) plan.steps.emplace_back(*goal->connection);
  plan.steps.emplace_back(FinalTargetStep{request.goal});
  plan.geometric_path = geometryFor(plan.steps);
  const double connection_cost =
      domain::distance(request.start.position,
                       spatial.region_skeleton_nodes[start->node].center)
          .meters() +
      domain::distance(spatial.region_skeleton_nodes[goal->node].center,
                       request.goal)
          .meters();
  const double cost = route->cost + connection_cost;
  plan.estimated_objective_costs[PlanObjective::SkeletonDistance] = cost;
  PlanResult result{
      PlanStatus::Success, plan.geometric_path, cost,
      "hierarchical region-skeleton route with operational learned trails",
      std::move(plan)};
  result.family = PlanFamily::Model;
  result.primary_objective = PlanObjective::SkeletonDistance;
  result.objective_costs[PlanObjective::SkeletonDistance] = cost;
  return result;
}

/**
 * @brief Performs the intersection by id operation for this subsystem.
 *
 * Arguments:
 * - @p graph: Supplies graph input to the operation.
 * - @p id: Supplies id input to the operation.
 *
 * Returns:
 * - `const domain::Intersection*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const domain::Intersection* intersectionById(const domain::HighwayGraph& graph,
                                             domain::IntersectionId id) {
  const auto found =
      std::find_if(graph.graph.vertices.begin(), graph.graph.vertices.end(),
                   [id](const auto& item) { return item.id == id; });
  return found == graph.graph.vertices.end() ? nullptr : &*found;
}

/**
 * @brief Performs the point in cell operation for this subsystem.
 *
 * Arguments:
 * - @p graph: Supplies graph input to the operation.
 * - @p point: Supplies point input to the operation.
 * - @p cell: Supplies cell input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool pointInCell(const domain::HighwayGraph& graph, domain::Point2D point,
                 domain::GridCell cell) {
  if (!graph.geometry.valid()) return false;
  const auto query = graph.geometry.cell(point);
  return query && static_cast<int>(query->first) == cell.column &&
         static_cast<int>(query->second) == cell.row;
}

/**
 * @brief Performs the point intersection operation for this subsystem.
 *
 * Arguments:
 * - @p graph: Supplies graph input to the operation.
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `std::optional<domain::IntersectionId>` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<domain::IntersectionId> pointIntersection(
    const domain::HighwayGraph& graph, domain::Point2D point) {
  for (const auto& intersection : graph.graph.vertices)
    if (pointInCell(graph, point, intersection.cell) ||
        (!graph.geometry.valid() &&
         domain::distance(point, intersection.position).meters() <= 0.25))
      return intersection.id;
  return std::nullopt;
}

/**
 * @brief Performs the point highway operation for this subsystem.
 *
 * Arguments:
 * - @p graph: Supplies graph input to the operation.
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `const domain::Highway*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const domain::Highway* pointHighway(const domain::HighwayGraph& graph,
                                    domain::Point2D point) {
  for (const auto& highway : graph.highways)
    if (std::any_of(
            highway.cells.begin(), highway.cells.end(),
            [&](const auto cell) { return pointInCell(graph, point, cell); }))
      return &highway;
  return nullptr;
}

/**
 * @brief Performs the closer endpoint operation for this subsystem.
 *
 * Arguments:
 * - @p graph: Supplies graph input to the operation.
 * - @p highway: Supplies highway input to the operation.
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `std::optional<domain::IntersectionId>` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<domain::IntersectionId> closerEndpoint(
    const domain::HighwayGraph& graph, const domain::Highway& highway,
    domain::Point2D point) {
  std::optional<domain::IntersectionId> selected;
  double best = std::numeric_limits<double>::infinity();
  for (const auto id : highway.endpoints) {
    const auto* endpoint = intersectionById(graph, id);
    if (!endpoint) continue;
    const double candidate =
        domain::distance(point, endpoint->position).meters();
    if (candidate < best - tie_tolerance ||
        (std::abs(candidate - best) <= tie_tolerance &&
         (!selected || id < *selected))) {
      best = candidate;
      selected = id;
    }
  }
  return selected;
}

/**
 * @brief Encapsulates region access state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct RegionAccess {
  domain::IntersectionId intersection{0U};
  std::optional<domain::HighwayId> highway;
};

/**
 * @brief Performs the region highway access operation for this subsystem.
 *
 * Arguments:
 * - @p spatial: Supplies spatial input to the operation.
 * - @p node: Supplies node input to the operation.
 *
 * Returns:
 * - `std::optional<RegionAccess>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<RegionAccess> regionHighwayAccess(
    const domain::SpatialModel& spatial, std::size_t node) {
  const auto boundary = regionBoundary(spatial, node);
  if (!boundary) return std::nullopt;
  for (const auto& intersection : spatial.highways.graph.vertices)
    if (boundary->contains(intersection.position))
      return RegionAccess{intersection.id, std::nullopt};
  for (const auto& highway : spatial.highways.highways) {
    bool overlaps = false;
    for (const auto cell : highway.cells) {
      if (!spatial.highways.geometry.valid()) continue;
      const auto center = spatial.highways.geometry.center(
          static_cast<std::size_t>(cell.column),
          static_cast<std::size_t>(cell.row));
      const double half_diagonal =
          spatial.highways.geometry.resolution_m * std::sqrt(0.5);
      if (domain::distance(boundary->center, center).meters() <=
          boundary->radius.meters() + half_diagonal) {
        overlaps = true;
        break;
      }
    }
    if (!overlaps) continue;
    const auto endpoint =
        closerEndpoint(spatial.highways, highway, boundary->center);
    if (endpoint) return RegionAccess{*endpoint, highway.id};
  }
  return std::nullopt;
}

/**
 * @brief Encapsulates highway attachment state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct HighwayAttachment {
  domain::IntersectionId intersection{0U};
  std::vector<PlanStep> point_to_intersection;
  double cost{0.0};
  std::vector<std::string> diagnostics;
};

/**
 * @brief Performs the attach to highway operation for this subsystem.
 *
 * Arguments:
 * - @p spatial: Supplies spatial input to the operation.
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `std::optional<HighwayAttachment>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<HighwayAttachment> attachToHighway(
    const domain::SpatialModel& spatial, domain::Point2D point) {
  if (const auto direct = pointIntersection(spatial.highways, point)) {
    const auto* intersection = intersectionById(spatial.highways, *direct);
    HighwayAttachment result{*direct, {}, 0.0, {}};
    result.point_to_intersection.emplace_back(
        IntersectionStep{*direct, intersection->position});
    result.diagnostics.push_back("highway_attachment:inside_intersection=" +
                                 std::to_string(*direct));
    return result;
  }
  if (const auto* highway = pointHighway(spatial.highways, point)) {
    const auto endpoint = closerEndpoint(spatial.highways, *highway, point);
    if (!endpoint) return std::nullopt;
    const auto* intersection = intersectionById(spatial.highways, *endpoint);
    std::vector<domain::Point2D> connection{point, intersection->position};
    HighwayAttachment result{*endpoint, {}, 0.0, {}};
    result.cost = polylineLength(connection);
    result.point_to_intersection.emplace_back(HighwayEntryStep{
        highway->id, intersection->position, std::move(connection)});
    result.point_to_intersection.emplace_back(
        IntersectionStep{*endpoint, intersection->position});
    result.diagnostics.push_back(
        "highway_attachment:inside_highway=" + std::to_string(highway->id) +
        ",endpoint=" + std::to_string(*endpoint));
    return result;
  }

  const auto surrogate = selectRegionSurrogate(spatial, point, true);
  if (!surrogate) return std::nullopt;
  std::optional<SkeletonRoute> best_route;
  std::optional<RegionAccess> best_access;
  std::size_t best_node = 0U;
  for (std::size_t node = 0U; node < spatial.region_skeleton_nodes.size();
       ++node) {
    const auto access = regionHighwayAccess(spatial, node);
    if (!access) continue;
    const auto route = shortestSkeletonRoute(spatial, surrogate->node, node);
    if (!route) continue;
    const auto current_key = std::pair{node, access->intersection};
    const auto best_key = std::pair{
        best_node, best_access
                       ? best_access->intersection
                       : std::numeric_limits<domain::IntersectionId>::max()};
    if (!best_route || route->cost < best_route->cost - tie_tolerance ||
        (std::abs(route->cost - best_route->cost) <= tie_tolerance &&
         current_key < best_key)) {
      best_route = route;
      best_access = access;
      best_node = node;
    }
  }
  if (!best_route || !best_access) return std::nullopt;
  HighwayAttachment result{best_access->intersection, {}, 0.0, {}};
  result.diagnostics = surrogate->diagnostics;
  if (surrogate->connection)
    result.point_to_intersection.emplace_back(*surrogate->connection);
  appendSkeletonRoute(spatial, *best_route, result.point_to_intersection);
  const auto* intersection =
      intersectionById(spatial.highways, best_access->intersection);
  if (!intersection) return std::nullopt;
  if (best_access->highway) {
    std::vector<domain::Point2D> connection{
        spatial.region_skeleton_nodes[best_node].center,
        intersection->position};
    result.point_to_intersection.emplace_back(HighwayEntryStep{
        *best_access->highway, intersection->position, std::move(connection)});
  }
  result.point_to_intersection.emplace_back(
      IntersectionStep{best_access->intersection, intersection->position});
  result.cost =
      best_route->cost +
      domain::distance(point,
                       spatial.region_skeleton_nodes[surrogate->node].center)
          .meters() +
      domain::distance(spatial.region_skeleton_nodes[best_node].center,
                       intersection->position)
          .meters();
  result.diagnostics.push_back(
      "highway_attachment:skeleton_access_region=" + std::to_string(best_node) +
      ",intersection=" + std::to_string(best_access->intersection));
  return result;
}

/**
 * @brief Performs the reverse attachment operation for this subsystem.
 *
 * Arguments:
 * - @p forward: Supplies forward input to the operation.
 *
 * Returns:
 * - `std::vector<PlanStep>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<PlanStep> reverseAttachment(std::vector<PlanStep> forward) {
  std::vector<PlanStep> result;
  result.reserve(forward.size());
  for (auto iterator = forward.rbegin(); iterator != forward.rend();
       ++iterator) {
    std::visit(
        [&](auto step) {
          using T = std::decay_t<decltype(step)>;
          if constexpr (std::is_same_v<T, VisibilityConnectionStep>) {
            std::swap(step.from, step.to);
            step.toward_region = false;
            result.emplace_back(std::move(step));
          } else if constexpr (std::is_same_v<T, SkeletonTransitionStep>) {
            std::swap(step.from_region, step.to_region);
            std::reverse(step.supporting_subtrail.begin(),
                         step.supporting_subtrail.end());
            result.emplace_back(std::move(step));
          } else if constexpr (std::is_same_v<T, HighwayEntryStep>) {
            std::reverse(step.supporting_subtrail.begin(),
                         step.supporting_subtrail.end());
            const auto exit = step.supporting_subtrail.empty()
                                  ? step.entry
                                  : step.supporting_subtrail.back();
            result.emplace_back(HighwayExitStep{
                step.highway_id, exit, std::move(step.supporting_subtrail)});
          } else {
            result.emplace_back(std::move(step));
          }
        },
        *iterator);
  }
  return result;
}

/**
 * @brief Encapsulates highway route state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct HighwayRoute {
  std::vector<domain::IntersectionId> vertices;
  std::vector<const domain::HighwayEdge*> edges;
  double cost{0.0};
};

/**
 * @brief Performs the shortest highway route operation for this subsystem.
 *
 * Arguments:
 * - @p highway: Supplies highway input to the operation.
 * - @p start: Supplies start input to the operation.
 * - @p goal: Supplies goal input to the operation.
 *
 * Returns:
 * - `std::optional<HighwayRoute>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<HighwayRoute> shortestHighwayRoute(
    const domain::HighwayGraph& highway, domain::IntersectionId start,
    domain::IntersectionId goal) {
  std::map<domain::IntersectionId, std::vector<const domain::HighwayEdge*>>
      adjacency;
  for (const auto& edge : highway.graph.edges) {
    adjacency[edge.from].push_back(&edge);
    adjacency[edge.to].push_back(&edge);
  }
  using Item = std::pair<double, domain::IntersectionId>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
  std::map<domain::IntersectionId, double> distance;
  std::map<domain::IntersectionId,
           std::pair<domain::IntersectionId, const domain::HighwayEdge*>>
      predecessor;
  for (const auto& vertex : highway.graph.vertices)
    distance[vertex.id] = std::numeric_limits<double>::infinity();
  if (!distance.contains(start) || !distance.contains(goal))
    return std::nullopt;
  distance[start] = 0.0;
  queue.push({0.0, start});
  while (!queue.empty()) {
    const auto [cost, node] = queue.top();
    queue.pop();
    if (cost > distance[node] + tie_tolerance) continue;
    if (node == goal) break;
    for (const auto* edge : adjacency[node]) {
      const auto neighbor = edge->from == node ? edge->to : edge->from;
      const double edge_cost = edge->length_m > 0.0
                                   ? edge->length_m
                                   : polylineLength(edge->operational_subtrail);
      const double candidate = cost + edge_cost;
      if (candidate < distance[neighbor] - tie_tolerance) {
        distance[neighbor] = candidate;
        predecessor[neighbor] = {node, edge};
        queue.push({candidate, neighbor});
      }
    }
  }
  if (!std::isfinite(distance[goal])) return std::nullopt;
  HighwayRoute result;
  result.cost = distance[goal];
  for (auto node = goal;;) {
    result.vertices.push_back(node);
    if (node == start) break;
    const auto found = predecessor.find(node);
    if (found == predecessor.end()) return std::nullopt;
    result.edges.push_back(found->second.second);
    node = found->second.first;
  }
  std::reverse(result.vertices.begin(), result.vertices.end());
  std::reverse(result.edges.begin(), result.edges.end());
  return result;
}

/**
 * @brief Builds highway assisted plan for this subsystem.
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
PlanResult buildHighwayAssistedPlan(const PlanningRequest& request) {
  const auto& spatial = *request.spatial_model;
  if (spatial.highways.graph.vertices.empty())
    return {PlanStatus::PlannerUnavailable,
            {},
            0.0,
            "highway graph is unavailable"};
  const auto start = attachToHighway(spatial, request.start.position);
  const auto goal = attachToHighway(spatial, request.goal);
  if (!start || !goal)
    return {PlanStatus::NoPath,
            {},
            0.0,
            "no hierarchical skeleton-to-highway attachment exists"};
  const auto highway = shortestHighwayRoute(
      spatial.highways, start->intersection, goal->intersection);
  if (!highway)
    return {PlanStatus::NoPath,
            {},
            0.0,
            "selected highway attachments are disconnected"};

  HierarchicalPlan plan;
  plan.family = PlanFamily::Model;
  plan.planner = "highway_plan";
  plan.objective = PlanObjective::HighwayDistance;
  plan.provenance =
      "hierarchical region surrogate, skeleton access, and highway graph route";
  plan.diagnostics = start->diagnostics;
  plan.diagnostics.insert(plan.diagnostics.end(), goal->diagnostics.begin(),
                          goal->diagnostics.end());
  plan.steps = start->point_to_intersection;
  for (std::size_t index = 0U; index < highway->edges.size(); ++index) {
    const auto from = highway->vertices[index];
    const auto to = highway->vertices[index + 1U];
    const auto* edge = highway->edges[index];
    auto operational = edge->operational_subtrail;
    if (edge->from != from)
      std::reverse(operational.begin(), operational.end());
    plan.steps.emplace_back(
        HighwayStep{edge->highway, from, to, std::move(operational)});
    const auto* intersection = intersectionById(spatial.highways, to);
    plan.steps.emplace_back(IntersectionStep{to, intersection->position});
  }
  auto goal_steps = reverseAttachment(goal->point_to_intersection);
  if (!plan.steps.empty() && !goal_steps.empty() &&
      std::holds_alternative<IntersectionStep>(plan.steps.back()) &&
      std::holds_alternative<IntersectionStep>(goal_steps.front()) &&
      std::get<IntersectionStep>(plan.steps.back()).intersection_id ==
          std::get<IntersectionStep>(goal_steps.front()).intersection_id)
    goal_steps.erase(goal_steps.begin());
  plan.steps.insert(plan.steps.end(), goal_steps.begin(), goal_steps.end());
  plan.steps.emplace_back(FinalTargetStep{request.goal});
  plan.geometric_path = geometryFor(plan.steps);
  const double cost = start->cost + highway->cost + goal->cost;
  plan.estimated_objective_costs[PlanObjective::HighwayDistance] = cost;
  PlanResult result{
      PlanStatus::Success, plan.geometric_path, cost,
      "hybrid region/skeleton/highway route with complete attachments",
      std::move(plan)};
  result.family = PlanFamily::Model;
  result.primary_objective = PlanObjective::HighwayDistance;
  result.objective_costs[PlanObjective::HighwayDistance] = cost;
  return result;
}

}  // namespace

/**
 * @brief Performs the dependencies operation for this subsystem.
 *
 * Arguments:
 * - @p argument_1: Supplies argument 1 input to the operation.
 *
 * Returns:
 * - `std::vector<domain::ModelDependency>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<domain::ModelDependency> SkeletonPlan::dependencies(
    const PlanningRequest&) const {
  return {domain::ModelDependency::Skeleton, domain::ModelDependency::Regions,
          domain::ModelDependency::Trails,
          domain::ModelDependency::VisibilityGeometry};
}

/**
 * @brief Performs the dependencies operation for this subsystem.
 *
 * Arguments:
 * - @p argument_1: Supplies argument 1 input to the operation.
 *
 * Returns:
 * - `std::vector<domain::ModelDependency>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<domain::ModelDependency> HighwayPlan::dependencies(
    const PlanningRequest&) const {
  using D = domain::ModelDependency;
  return {D::Skeleton, D::Highways, D::HighwayGraph,
          D::Regions,  D::Trails,   D::VisibilityGeometry};
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
PlanResult SkeletonPlan::plan(const PlanningRequest& request) {
  auto result = buildSkeletonPlan(request, std::string(name()));
  attachDependencySnapshot(result, request, dependencies(request));
  return result;
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
PlanResult HighwayPlan::plan(const PlanningRequest& request) {
  if (!request.spatial_model)
    return {PlanStatus::PlannerUnavailable,
            {},
            0.0,
            "spatial model is unavailable"};
  auto skeleton = buildSkeletonPlan(request, std::string(name()));
  auto highway = buildHighwayAssistedPlan(request);
  PlanResult result;
  if (!highway.succeeded()) {
    result = std::move(skeleton);
    if (result.succeeded()) {
      result.explanation =
          "skeleton-only route selected because hierarchical highway "
          "attachment failed";
      result.hierarchical->diagnostics.push_back(highway.explanation);
    } else {
      result = std::move(highway);
    }
  } else if (!skeleton.succeeded() || highway.cost_m < skeleton.cost_m) {
    result = std::move(highway);
  } else {
    result = std::move(skeleton);
    result.explanation =
        "skeleton-only route selected over valid hierarchical highway route";
  }
  attachDependencySnapshot(result, request, dependencies(request));
  return result;
}

}  // namespace semaforr::planning

semaforr::planning::
    PlannerMetadata
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
    semaforr::planning::SkeletonPlan::metadata() const {
  return {std::string(name()),
          PlanFamily::Model,
          objective(),
          std::string(toString(objective())),
          std::string(objectiveDescription(objective())),
          {"regions", "region_visibility", "region_skeleton",
           "learned_edge_trails"},
          false,
          true};
}

semaforr::planning::
    PlannerMetadata
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
    semaforr::planning::HighwayPlan::metadata() const {
  return {std::string(name()),
          PlanFamily::Model,
          objective(),
          std::string(toString(objective())),
          std::string(objectiveDescription(objective())),
          {"region_visibility", "region_skeleton", "highways", "highway_graph",
           "learned_edge_trails"},
          false,
          true};
}
