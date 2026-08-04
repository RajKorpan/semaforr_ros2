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
    return {PlanStatus::InvalidRequest, {}, 0.0,
            "start and goal must be finite"};
  if (nodes.empty())
    return {PlanStatus::PlannerUnavailable, {}, 0.0,
            strategy + " network is unavailable"};
  Graph graph;
  for (const auto& node : nodes) graph.addVertex(node);
  for (const auto& edge : edges) {
    if (edge.first >= nodes.size() || edge.second >= nodes.size())
      return {PlanStatus::InvalidRequest, {}, 0.0,
              strategy + " contains an invalid edge"};
    graph.addUndirectedEdge(
        edge.first, edge.second,
        {domain::distance(nodes[edge.first], nodes[edge.second]).meters(),
         0.0, 0.0});
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
  if (!path.succeeded())
    return {PlanStatus::NoPath, {}, 0.0, path.explanation};

  HierarchicalPlan hierarchy;
  hierarchy.strategy = std::move(strategy);
  hierarchy.spatial_revision =
      request.spatial_model ? request.spatial_model->revision : 0U;
  std::vector<domain::Point2D> waypoints;
  for (std::size_t index = 1U; index < path.vertices.size(); ++index) {
    const VertexId vertex = path.vertices[index];
    const domain::Point2D point = graph.position(vertex);
    waypoints.push_back(point);
    PlanStepKind kind = PlanStepKind::ReachGoal;
    if (vertex != goal) {
      const bool on_highway =
          highway && vertex >= highway_node_offset && vertex < nodes.size();
      if (index == 1U)
        kind = on_highway ? PlanStepKind::EnterHighway
                          : PlanStepKind::ApproachNetwork;
      else
        kind = on_highway ? PlanStepKind::FollowHighway
                          : PlanStepKind::TraverseSkeleton;
      if (on_highway &&
          std::any_of(intersections.begin(), intersections.end(),
                      [vertex](const auto& item) { return item.node == vertex; }))
        kind = PlanStepKind::CrossIntersection;
    }
    hierarchy.steps.push_back(
        {kind, point, vertex < nodes.size()
                          ? std::optional<std::size_t>(vertex)
                          : std::nullopt});
  }
  return {PlanStatus::Success, std::move(waypoints), path.cost,
          "hierarchical " + hierarchy.strategy, std::move(hierarchy)};
}

}  // namespace

std::string_view toString(PlanStepKind kind) noexcept {
  switch (kind) {
    case PlanStepKind::ApproachNetwork: return "approach_network";
    case PlanStepKind::TraverseSkeleton: return "traverse_skeleton";
    case PlanStepKind::EnterHighway: return "enter_highway";
    case PlanStepKind::FollowHighway: return "follow_highway";
    case PlanStepKind::CrossIntersection: return "cross_intersection";
    case PlanStepKind::LeaveNetwork: return "leave_network";
    case PlanStepKind::ReachGoal: return "reach_goal";
  }
  return "reach_goal";
}

PlanResult SkeletonPlan::plan(const PlanningRequest& request) {
  if (!request.spatial_model)
    return {PlanStatus::PlannerUnavailable, {}, 0.0,
            "spatial model is unavailable"};
  return buildNetworkPlan(request, request.spatial_model->skeleton_nodes,
                          request.spatial_model->skeleton_edges,
                          "skeleton", false);
}

PlanResult HighwayPlan::plan(const PlanningRequest& request) {
  if (!request.spatial_model)
    return {PlanStatus::PlannerUnavailable, {}, 0.0,
            "spatial model is unavailable"};
  const auto& spatial = *request.spatial_model;
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
      buildNetworkPlan(request, spatial.skeleton_nodes,
                       spatial.skeleton_edges, "skeleton", false);
  if (highway_nodes.empty()) return skeleton;

  if (spatial.skeleton_nodes.empty()) {
    PlanResult highway =
        buildNetworkPlan(request, highway_nodes, highway_edges, "highway",
                         true, intersections);
    return highway.succeeded() ? highway : skeleton;
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
  for (auto& intersection : intersections)
    intersection.node += highway_offset;
  PlanResult assisted = buildNetworkPlan(
      request, combined, edges, "highway_assisted", true, intersections,
      spatial.skeleton_nodes.size(), highway_offset);
  if (!assisted.succeeded()) return skeleton;
  if (!skeleton.succeeded() || assisted.cost_m < skeleton.cost_m)
    return assisted;
  skeleton.explanation =
      "skeleton-only route selected over valid highway-assisted route";
  return skeleton;
}

}  // namespace semaforr::planning
