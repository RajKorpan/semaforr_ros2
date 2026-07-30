#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/planning/domain_planner.hpp>
#include <stdexcept>
#include <utility>

namespace semaforr::planning {

DomainPlanner::DomainPlanner(std::string name, PlannerObjective objective)
    : name_(std::move(name)), objective_(objective) {
  if (name_.empty()) {
    throw std::invalid_argument("planner name must not be empty");
  }
}

PlanResult DomainPlanner::direct(const PlanningRequest& request) const {
  if (!request.start.position.finite() || !request.goal.finite()) {
    return {
        PlanStatus::InvalidRequest, {}, 0.0, "start and goal must be finite"};
  }
  return {PlanStatus::Success,
          {request.goal},
          domain::distance(request.start.position, request.goal).meters(),
          "direct metric path"};
}

double DomainPlanner::socialPenalty(const PlanningRequest& request,
                                    domain::Point2D from,
                                    domain::Point2D to) const {
  if (objective_ == PlannerObjective::Distance ||
      objective_ == PlannerObjective::SkeletonDistance ||
      !request.crowd_model) {
    return 0.0;
  }
  const domain::Point2D midpoint{(from.x_m + to.x_m) * 0.5,
                                 (from.y_m + to.y_m) * 0.5};
  const auto sample = request.crowd_model->learnedAt(midpoint);
  if (!sample || sample->stale) {
    return 0.0;
  }
  const double length = domain::distance(from, to).meters();
  switch (objective_) {
    case PlannerObjective::Distance:
    case PlannerObjective::SkeletonDistance:
      return 0.0;
    case PlannerObjective::CrowdDensity:
      return length * std::max(0.0, sample->cell.density);
    case PlannerObjective::EncounterRisk:
      return length * std::max(0.0, sample->cell.learned_encounter_risk);
    case PlannerObjective::FlowAlignment: {
      const domain::Angle heading(
          std::atan2(to.y_m - from.y_m, to.x_m - from.x_m));
      return length *
             (1.0 - request.crowd_model->flowAlignmentAt(midpoint, heading));
    }
  }
  return 0.0;
}

PlanResult DomainPlanner::plan(const PlanningRequest& request) {
  if (objective_ == PlannerObjective::Distance) {
    return direct(request);
  }
  if (!request.spatial_model || request.spatial_model->skeleton_nodes.empty()) {
    return {PlanStatus::PlannerUnavailable,
            {},
            0.0,
            "spatial skeleton is not available"};
  }

  Graph graph;
  for (const auto& point : request.spatial_model->skeleton_nodes) {
    graph.addVertex(point);
  }
  for (const auto& [from, to] : request.spatial_model->skeleton_edges) {
    if (from >= graph.size() || to >= graph.size()) {
      return {PlanStatus::InvalidRequest,
              {},
              0.0,
              "spatial skeleton contains an invalid edge"};
    }
    const double length =
        domain::distance(graph.position(from), graph.position(to)).meters();
    graph.addUndirectedEdge(
        from, to,
        {length,
         socialPenalty(request, graph.position(from), graph.position(to)),
         0.0});
  }

  const VertexId start = graph.addVertex(request.start.position);
  const VertexId goal = graph.addVertex(request.goal);
  const auto connectNearest = [&](VertexId vertex) {
    VertexId nearest = 0U;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (VertexId candidate = 0U; candidate + 2U < graph.size(); ++candidate) {
      const double candidate_distance =
          domain::distance(graph.position(vertex), graph.position(candidate))
              .meters();
      if (candidate_distance < nearest_distance) {
        nearest = candidate;
        nearest_distance = candidate_distance;
      }
    }
    graph.addUndirectedEdge(vertex, nearest,
                            {nearest_distance,
                             socialPenalty(request, graph.position(vertex),
                                           graph.position(nearest)),
                             0.0});
  };
  connectNearest(start);
  connectNearest(goal);

  const PathResult path = astar_.search(graph, start, goal);
  if (!path.succeeded()) {
    return {PlanStatus::NoPath, {}, 0.0, path.explanation};
  }
  std::vector<domain::Point2D> points;
  points.reserve(path.vertices.size());
  for (const VertexId vertex : path.vertices) {
    if (vertex != start) {
      points.push_back(graph.position(vertex));
    }
  }
  return {PlanStatus::Success, std::move(points), path.cost,
          "A* over immutable domain skeleton"};
}

}  // namespace semaforr::planning
