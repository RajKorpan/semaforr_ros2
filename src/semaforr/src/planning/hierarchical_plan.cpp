#include <algorithm>
#include <limits>
#include <semaforr/planning/astar.hpp>
#include <semaforr/planning/graph.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>

namespace semaforr::planning {
namespace {

PlanResult buildNetworkPlan(
    const PlanningRequest& request, const std::vector<domain::Point2D>& nodes,
    const std::vector<std::pair<std::size_t, std::size_t>>& edges,
    std::string strategy, bool highway,
    const std::vector<domain::HighwayIntersection>& intersections = {},
    std::size_t attachment_node_count = 0U,
    std::size_t highway_node_offset = 0U) {
  if (!request.start.position.finite() || !request.goal.finite())
    return {
        PlanStatus::InvalidRequest, {}, 0.0, "start and goal must be finite"};
  if (nodes.empty())
    return {PlanStatus::PlannerUnavailable,
            {},
            0.0,
            strategy + " network is unavailable"};
  Graph graph;
  for (const auto& node : nodes) graph.addVertex(node);
  for (const auto& edge : edges) {
    if (edge.first >= nodes.size() || edge.second >= nodes.size())
      return {PlanStatus::InvalidRequest,
              {},
              0.0,
              strategy + " contains an invalid edge"};
    graph.addUndirectedEdge(
        edge.first, edge.second,
        {domain::distance(nodes[edge.first], nodes[edge.second]).meters(), 0.0,
         0.0});
  }
  const VertexId start = graph.addVertex(request.start.position);
  const VertexId goal = graph.addVertex(request.goal);
  const auto connectNearest = [&](VertexId vertex) {
    VertexId nearest = 0U;
    double best = std::numeric_limits<double>::infinity();
    const std::size_t limit =
        attachment_node_count == 0U ? nodes.size() : attachment_node_count;
    for (VertexId candidate = 0U; candidate < limit; ++candidate) {
      const double distance =
          domain::distance(graph.position(vertex), nodes[candidate]).meters();
      if (distance < best) {
        best = distance;
        nearest = candidate;
      }
    }
    graph.addUndirectedEdge(vertex, nearest, {best, 0.0, 0.0});
  };
  connectNearest(start);
  connectNearest(goal);
  const PathResult path = AStar{}.search(graph, start, goal);
  if (!path.succeeded()) return {PlanStatus::NoPath, {}, 0.0, path.explanation};

  HierarchicalPlan hierarchy;
  hierarchy.planner = strategy;
  hierarchy.objective = highway ? PlanObjective::HighwayDistance
                                : PlanObjective::SkeletonDistance;
  hierarchy.provenance = "Dijkstra/A* connection through " + strategy;
  std::vector<domain::Point2D> waypoints;
  for (std::size_t index = 1U; index < path.vertices.size(); ++index) {
    const VertexId vertex = path.vertices[index];
    const domain::Point2D point = graph.position(vertex);
    waypoints.push_back(point);
    const auto intersection = std::find_if(
        intersections.begin(), intersections.end(),
        [vertex](const auto& item) { return item.node == vertex; });
    if (highway && intersection != intersections.end()) {
      hierarchy.steps.emplace_back(IntersectionStep{
          vertex >= highway_node_offset ? vertex - highway_node_offset : vertex,
          point});
    } else if (request.spatial_model) {
      const auto region = std::find_if(
          request.spatial_model->learned_regions.begin(),
          request.spatial_model->learned_regions.end(),
          [&point](const auto& item) { return item.contains(point); });
      if (region != request.spatial_model->learned_regions.end()) {
        hierarchy.steps.emplace_back(RegionStep{
            static_cast<std::size_t>(std::distance(
                request.spatial_model->learned_regions.begin(), region)),
            region->center});
      } else {
        hierarchy.steps.emplace_back(WaypointStep{point});
      }
    } else {
      hierarchy.steps.emplace_back(WaypointStep{point});
    }
  }
  if (highway && request.spatial_model &&
      !request.spatial_model->highways.graph.edges.empty()) {
    std::vector<PlanStep> typed;
    for (const auto& step : hierarchy.steps) {
      if (!typed.empty()) {
        const auto* from = std::get_if<IntersectionStep>(&typed.back());
        const auto* to = std::get_if<IntersectionStep>(&step);
        if (from && to) {
          const auto edge =
              std::find_if(request.spatial_model->highways.graph.edges.begin(),
                           request.spatial_model->highways.graph.edges.end(),
                           [&](const auto& candidate) {
                             return (candidate.from == from->intersection_id &&
                                     candidate.to == to->intersection_id) ||
                                    (candidate.to == from->intersection_id &&
                                     candidate.from == to->intersection_id);
                           });
          if (edge != request.spatial_model->highways.graph.edges.end()) {
            auto fallback = edge->operational_subtrail;
            if (edge->to == from->intersection_id)
              std::reverse(fallback.begin(), fallback.end());
            if (fallback.empty())
              for (const auto trail_id : edge->trail_labels)
                if (trail_id < request.spatial_model->trails.size())
                  fallback.insert(fallback.end(),
                                  request.spatial_model->trails[trail_id].begin(),
                                  request.spatial_model->trails[trail_id].end());
            typed.emplace_back(HighwayStep{edge->highway, from->intersection_id,
                                           to->intersection_id,
                                           std::move(fallback)});
          }
        }
      }
      typed.push_back(step);
    }
    hierarchy.steps = std::move(typed);
  }
  hierarchy.estimated_objective_costs[hierarchy.objective] = path.cost;
  return {PlanStatus::Success, std::move(waypoints), path.cost,
          "hierarchical " + hierarchy.planner, std::move(hierarchy)};
}

}  // namespace

std::vector<domain::ModelDependency> SkeletonPlan::dependencies(
    const PlanningRequest&) const {
  return {domain::ModelDependency::Skeleton, domain::ModelDependency::Regions};
}

std::vector<domain::ModelDependency> HighwayPlan::dependencies(
    const PlanningRequest&) const {
  using D = domain::ModelDependency;
  return {D::Skeleton, D::Highways, D::HighwayGraph, D::Regions, D::Trails};
}

PlanResult SkeletonPlan::plan(const PlanningRequest& request) {
  if (!request.spatial_model)
    return {PlanStatus::PlannerUnavailable,
            {},
            0.0,
            "spatial model is unavailable"};
  auto result = buildNetworkPlan(request, request.spatial_model->skeleton_nodes,
                                 request.spatial_model->skeleton_edges,
                                 "skeleton", false);
  if (result.hierarchical &&
      !request.spatial_model->region_skeleton_edges.empty()) {
    std::vector<PlanStep> operational;
    std::optional<std::size_t> previous_region;
    for (const auto& step : result.hierarchical->steps) {
      if (const auto* region = std::get_if<RegionStep>(&step)) {
        if (previous_region && *previous_region != region->region_id) {
          const auto edge = std::find_if(
              request.spatial_model->region_skeleton_edges.begin(),
              request.spatial_model->region_skeleton_edges.end(),
              [&](const auto& candidate) {
                return (candidate.from == *previous_region &&
                        candidate.to == region->region_id) ||
                       (candidate.to == *previous_region &&
                        candidate.from == region->region_id);
              });
          if (edge != request.spatial_model->region_skeleton_edges.end()) {
            auto subtrail = edge->supporting_subtrail;
            if (edge->to == *previous_region) std::reverse(subtrail.begin(), subtrail.end());
            operational.emplace_back(
                SubtrailStep{std::move(subtrail), std::nullopt, 0U});
          }
        }
        previous_region = region->region_id;
      }
      operational.push_back(step);
    }
    result.hierarchical->steps = std::move(operational);
    result.hierarchical->provenance =
        "region skeleton with shortest execution-confirmed subtrail labels";
    result.explanation =
        "hierarchical region-skeleton route with operationalizable subtrails";
  }
  attachDependencySnapshot(result, request, dependencies(request));
  return result;
}

PlanResult HighwayPlan::plan(const PlanningRequest& request) {
  if (!request.spatial_model)
    return {PlanStatus::PlannerUnavailable,
            {},
            0.0,
            "spatial model is unavailable"};
  const auto& spatial = *request.spatial_model;
  const auto finish = [&](PlanResult result) {
    attachDependencySnapshot(result, request, dependencies(request));
    return result;
  };
  std::vector<domain::Point2D> highway_nodes = spatial.highways.nodes;
  std::vector<std::pair<std::size_t, std::size_t>> highway_edges =
      spatial.highways.edges;
  std::vector<domain::HighwayIntersection> intersections =
      spatial.highways.intersections;
  if (!spatial.highways.graph.vertices.empty()) {
    highway_nodes.clear();
    highway_edges.clear();
    intersections.clear();
    for (const auto& intersection : spatial.highways.graph.vertices) {
      highway_nodes.push_back(intersection.position);
      intersections.push_back(
          {intersection.id, intersection.terminal_access ? 1U : 3U});
    }
    for (const auto& edge : spatial.highways.graph.edges)
      highway_edges.emplace_back(edge.from, edge.to);
  }

  PlanResult skeleton =
      buildNetworkPlan(request, spatial.skeleton_nodes, spatial.skeleton_edges,
                       "skeleton", false);
  if (highway_nodes.empty()) return finish(std::move(skeleton));

  if (spatial.skeleton_nodes.empty()) {
    PlanResult highway = buildNetworkPlan(request, highway_nodes, highway_edges,
                                          "highway", true, intersections);
    return finish(highway.succeeded() ? std::move(highway)
                                      : std::move(skeleton));
  }

  std::vector<domain::Point2D> combined = spatial.skeleton_nodes;
  const std::size_t highway_offset = combined.size();
  combined.insert(combined.end(), highway_nodes.begin(), highway_nodes.end());
  std::vector<std::pair<std::size_t, std::size_t>> edges =
      spatial.skeleton_edges;
  for (const auto& edge : highway_edges)
    edges.emplace_back(edge.first + highway_offset,
                       edge.second + highway_offset);
  // Skeleton access links are derived once per highway vertex. A* then chooses
  // where to enter and leave the highway rather than receiving direct
  // start/goal-to-highway shortcuts.
  for (std::size_t highway = 0U; highway < highway_nodes.size(); ++highway) {
    std::size_t nearest = 0U;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t skeleton_node = 0U;
         skeleton_node < spatial.skeleton_nodes.size(); ++skeleton_node) {
      const double candidate =
          domain::distance(highway_nodes[highway],
                           spatial.skeleton_nodes[skeleton_node])
              .meters();
      if (candidate < best) {
        best = candidate;
        nearest = skeleton_node;
      }
    }
    edges.emplace_back(nearest, highway_offset + highway);
  }
  for (auto& intersection : intersections) intersection.node += highway_offset;
  PlanResult assisted = buildNetworkPlan(
      request, combined, edges, "highway_assisted", true, intersections,
      spatial.skeleton_nodes.size(), highway_offset);
  if (!assisted.succeeded()) return finish(std::move(skeleton));
  if (!skeleton.succeeded() || assisted.cost_m < skeleton.cost_m)
    return finish(std::move(assisted));
  skeleton.explanation =
      "skeleton-only route selected over valid highway-assisted route";
  return finish(std::move(skeleton));
}

}  // namespace semaforr::planning
