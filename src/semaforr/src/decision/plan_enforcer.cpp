#include <algorithm>
#include <semaforr/decision/enforcer.hpp>

namespace semaforr::decision {
namespace {
bool reached(const domain::Pose2D& pose, domain::Point2D point,
             domain::Distance tolerance) {
  return domain::distance(pose.position, point).meters() <=
         tolerance.meters() + domain::geometry_tolerance_m;
}
bool visible(const domain::Pose2D& pose, domain::Point2D point,
             const domain::SpatialModel& spatial) {
  if (domain::distance(pose.position, point).meters() > 5.0) return false;
  const domain::Segment2D sight{pose.position, point};
  for (const auto& obstacle : spatial.obstacle_polygons) {
    if (obstacle.contains(point)) return false;
    const auto& vertices = obstacle.vertices();
    for (std::size_t i = 0; i < vertices.size(); ++i)
      if (domain::intersects(
              sight, {vertices[i], vertices[(i + 1U) % vertices.size()]}))
        return false;
  }
  return true;
}
}  // namespace

std::vector<domain::Point2D> Enforcer::operationalize(
    const planning::HierarchicalPlan& plan) const {
  if (plan.validity != planning::PlanValidity::Valid || plan.exhausted())
    return {};
  const auto target = planning::stepTarget(plan.steps[plan.cursor]);
  return target ? std::vector<domain::Point2D>{*target}
                : std::vector<domain::Point2D>{};
}

std::size_t Enforcer::activeStep(const planning::HierarchicalPlan& plan,
                                 const domain::Pose2D& pose,
                                 domain::Distance tolerance) const noexcept {
  std::size_t cursor = plan.cursor;
  while (cursor < plan.steps.size()) {
    const auto target = planning::stepTarget(plan.steps[cursor]);
    if (!target || !reached(pose, *target, tolerance)) break;
    ++cursor;
  }
  return cursor;
}

std::optional<domain::Point2D> Enforcer::operationalizeNext(
    planning::HierarchicalPlan& plan, const domain::SpatialModel& spatial,
    const domain::Pose2D& pose, domain::Distance tolerance) const {
  const auto spatial_revision = plan.source_model_revisions.find("spatial");
  if (spatial_revision != plan.source_model_revisions.end() &&
      spatial_revision->second != spatial.revision) {
    plan.validity = planning::PlanValidity::Stale;
    plan.diagnostics.push_back("stale_plan:spatial_revision_changed");
    return std::nullopt;
  }
  plan.cursor = activeStep(plan, pose, tolerance);
  if (plan.exhausted()) {
    plan.validity = planning::PlanValidity::Complete;
    return std::nullopt;
  }

  auto& current = plan.steps[plan.cursor];
  if (plan.cursor + 1U < plan.steps.size()) {
    const auto second = planning::stepTarget(plan.steps[plan.cursor + 1U]);
    if (second && visible(pose, *second, spatial)) {
      ++plan.cursor;
      plan.diagnostics.push_back("visible_second_step_shortcut");
      return second;
    }
  }
  if (auto* trail = std::get_if<planning::SubtrailStep>(&current)) {
    while (trail->cursor < trail->waypoints.size() &&
           reached(pose, trail->waypoints[trail->cursor], tolerance))
      ++trail->cursor;
    if (trail->cursor >= trail->waypoints.size()) {
      plan.diagnostics.push_back("obsolete_subtrail_skipped");
      ++plan.cursor;
      return operationalizeNext(plan, spatial, pose, tolerance);
    }
    for (std::size_t i = trail->waypoints.size(); i > trail->cursor; --i)
      if (visible(pose, trail->waypoints[i - 1], spatial)) {
        trail->cursor = i - 1;
        plan.diagnostics.push_back("subtrail_lookahead_shortcut");
        break;
      }
    return trail->waypoints[trail->cursor];
  }
  if (auto* region = std::get_if<planning::RegionStep>(&current)) {
    if (region->region_id >= spatial.learned_regions.size()) {
      plan.validity = planning::PlanValidity::Invalid;
      plan.diagnostics.push_back("invalid_region_step");
      return std::nullopt;
    }
    region->center = spatial.learned_regions[region->region_id].center;
    if (spatial.learned_regions[region->region_id].contains(pose.position) &&
        plan.cursor + 1U < plan.steps.size()) {
      const auto next = planning::stepTarget(plan.steps[plan.cursor + 1U]);
      if (next && visible(pose, *next, spatial)) {
        ++plan.cursor;
        plan.diagnostics.push_back("visible_later_step_shortcut");
        return next;
      }
    }
    if (domain::distance(pose.position, region->center).meters() > 5.0) {
      const auto trail = std::find_if(
          spatial.trails.begin(), spatial.trails.end(),
          [&](const auto& candidate) {
            if (candidate.empty()) return false;
            const bool forward =
                domain::distance(pose.position, candidate.front()).meters() <=
                    2.0 &&
                domain::distance(region->center, candidate.back()).meters() <=
                    2.0;
            const bool reverse =
                domain::distance(pose.position, candidate.back()).meters() <=
                    2.0 &&
                domain::distance(region->center, candidate.front()).meters() <=
                    2.0;
            return forward || reverse;
          });
      if (trail != spatial.trails.end()) {
        auto markers = *trail;
        if (domain::distance(pose.position, markers.back()).meters() <
            domain::distance(pose.position, markers.front()).meters())
          std::reverse(markers.begin(), markers.end());
        current =
            planning::SubtrailStep{std::move(markers),
                                   static_cast<domain::TrailId>(std::distance(
                                       spatial.trails.begin(), trail)),
                                   0U};
        plan.diagnostics.push_back("region_repaired_with_stored_subtrail");
        return operationalizeNext(plan, spatial, pose, tolerance);
      }
    }
    return region->center;
  }
  if (auto* highway = std::get_if<planning::HighwayStep>(&current)) {
    const auto model = std::find_if(
        spatial.highways.highways.begin(), spatial.highways.highways.end(),
        [&](const auto& candidate) {
          return candidate.id == highway->highway_id;
        });
    if (model != spatial.highways.highways.end() &&
        spatial.known_grid.resolution_m > 0.0) {
      std::vector<planning::PlanStep> regions;
      for (std::size_t id = 0; id < spatial.learned_regions.size(); ++id) {
        const bool overlaps = std::any_of(
            model->cells.begin(), model->cells.end(), [&](const auto& cell) {
              const domain::Point2D center{
                  spatial.known_grid.origin.x_m +
                      (static_cast<double>(cell.column) + .5) *
                          spatial.known_grid.resolution_m,
                  spatial.known_grid.origin.y_m +
                      (static_cast<double>(cell.row) + .5) *
                          spatial.known_grid.resolution_m};
              return spatial.learned_regions[id].contains(center);
            });
        if (overlaps)
          regions.emplace_back(
              planning::RegionStep{id, spatial.learned_regions[id].center});
      }
      if (regions.size() >= 2U) {
        plan.steps.erase(plan.steps.begin() +
                         static_cast<std::ptrdiff_t>(plan.cursor));
        plan.steps.insert(
            plan.steps.begin() + static_cast<std::ptrdiff_t>(plan.cursor),
            regions.begin(), regions.end());
        plan.diagnostics.push_back("highway_replaced_with_overlapping_regions");
        return operationalizeNext(plan, spatial, pose, tolerance);
      }
    }
    if (!highway->fallback_subtrail.empty()) {
      current =
          planning::SubtrailStep{highway->fallback_subtrail, std::nullopt, 0U};
      plan.diagnostics.push_back("highway_repaired_with_stored_trail");
      return operationalizeNext(plan, spatial, pose, tolerance);
    }
    plan.validity = planning::PlanValidity::Invalid;
    plan.diagnostics.push_back("highway_step_has_no_operationalization");
    return std::nullopt;
  }
  return planning::stepTarget(current);
}
}  // namespace semaforr::decision
