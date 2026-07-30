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
    const std::vector<domain::HighwayIntersection>& intersections = {}) {
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
    for (VertexId candidate = 0U; candidate < nodes.size(); ++candidate) {
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
      if (index == 1U)
        kind = highway ? PlanStepKind::EnterHighway
                       : PlanStepKind::ApproachNetwork;
      else
        kind = highway ? PlanStepKind::FollowHighway
                       : PlanStepKind::TraverseSkeleton;
      if (highway &&
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
  return buildNetworkPlan(request, request.spatial_model->highways.nodes,
                          request.spatial_model->highways.edges,
                          "highway", true,
                          request.spatial_model->highways.intersections);
}

}  // namespace semaforr::planning
