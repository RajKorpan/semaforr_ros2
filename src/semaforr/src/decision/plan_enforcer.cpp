/**
 * @file plan_enforcer.cpp
 * @brief Plan enforcer responsibilities.
 *
 * @details This file implements plan enforcer behavior for tiered decision making
 * and action arbitration. It records the declarations, settings, fixtures,
 * or guidance needed by that responsibility. Its package-relative location
 * is `src/decision/plan_enforcer.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/decision/enforcer.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <semaforr/planning/traversability.hpp>
#include <sstream>

namespace semaforr::decision {
namespace {
/**
 * @brief Performs the reached operation for this subsystem.
 *
 * Arguments:
 * - @p pose: Supplies pose input to the operation.
 * - @p point: Supplies point input to the operation.
 * - @p tolerance: Supplies tolerance input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool reached(const domain::Pose2D& pose, domain::Point2D point,
             domain::Distance tolerance) {
  return domain::distance(pose.position, point).meters() <=
         tolerance.meters() + domain::geometry_tolerance_m;
}
/**
 * @brief Performs the visible operation for this subsystem.
 *
 * Arguments:
 * - @p pose: Supplies pose input to the operation.
 * - @p point: Supplies point input to the operation.
 * - @p spatial: Supplies spatial input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool visible(const domain::Pose2D& pose, domain::Point2D point,
             const domain::SpatialModel& spatial) {
  bool learned_visibility = false;
  for (const auto& region : spatial.regions) {
    if (!region.boundary.contains(pose.position)) continue;
    const double angle = std::atan2(point.y_m - region.boundary.center.y_m,
                                    point.x_m - region.boundary.center.x_m);
    constexpr double pi = 3.14159265358979323846;
    double positive = std::fmod(angle, 2.0 * pi);
    if (positive < 0.0) positive += 2.0 * pi;
    const auto bin = static_cast<std::size_t>(
                         std::floor(positive * 180.0 / pi)) %
                     360U;
    const auto& evidence = region.visibility[bin];
    learned_visibility = evidence.known &&
                         evidence.maximum_distance_m +
                                 domain::geometry_tolerance_m >=
                             domain::distance(region.boundary.center, point)
                                 .meters();
    if (learned_visibility) break;
  }
  if (!learned_visibility &&
      domain::distance(pose.position, point).meters() > 5.0)
    return false;
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
/**
 * @brief Reports whether spatial dependency for this subsystem.
 *
 * Arguments:
 * - @p dependency: Supplies dependency input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool isSpatialDependency(domain::ModelDependency dependency) {
  using D = domain::ModelDependency;
  return dependency != D::StaticMapGeometry &&
         dependency != D::StaticOccupancy && dependency != D::CrowdDensity &&
         dependency != D::CrowdRisk && dependency != D::CrowdFlow &&
         dependency != D::PlannerConfiguration;
}
/**
 * @brief Records package content for this subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 * - @p operation: Supplies operation input to the operation.
 * - @p spatial: Supplies spatial input to the operation.
 * - @p dependencies: Supplies dependencies input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void record(planning::HierarchicalPlan& plan, std::string operation,
            const domain::SpatialModel& spatial,
            std::initializer_list<domain::ModelDependency> dependencies) {
  planning::HierarchicalPlan::OperationalizationRecord item;
  item.step_index = plan.cursor;
  item.operation = std::move(operation);
  for (const auto dependency : dependencies)
    item.dependency_revisions[dependency] = spatial.revisionOf(dependency);
  plan.operationalizations.push_back(std::move(item));
}

/**
 * @brief Performs the request for operation for this subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 * - @p context: Supplies context input to the operation.
 *
 * Returns:
 * - `planning::PlanningRequest` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
planning::PlanningRequest requestFor(
    const planning::HierarchicalPlan& plan,
    const PlanEnforcementContext& context) {
  return {context.pose, plan.planned_goal, &context.spatial, context.crowd,
          context.static_map, context.traversability, context.task_id,
          plan.planner_configuration_revision};
}

/**
 * @brief Performs the dependency changes operation for this subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 * - @p context: Supplies context input to the operation.
 *
 * Returns:
 * - `std::vector<std::string>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<std::string> dependencyChanges(
    const planning::HierarchicalPlan& plan,
    const PlanEnforcementContext& context) {
  return planning::dependencyChangeReasons(plan.dependency_revisions,
                                           requestFor(plan, context));
}

/**
 * @brief Performs the segment traversable operation for this subsystem.
 *
 * Arguments:
 * - @p grid: Supplies grid input to the operation.
 * - @p from: Supplies from input to the operation.
 * - @p to: Supplies to input to the operation.
 * - @p evidence: Supplies evidence input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool segmentTraversable(const domain::TraversabilityGrid& grid,
                        domain::Point2D from, domain::Point2D to,
                        std::string& evidence) {
  const double length = domain::distance(from, to).meters();
  const double stride = std::max(0.02, grid.geometry.resolution_m * 0.5);
  const std::size_t samples =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(length / stride)));
  for (std::size_t i = 0U; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const domain::Point2D point{from.x_m + (to.x_m - from.x_m) * t,
                                from.y_m + (to.y_m - from.y_m) * t};
    const auto cell = grid.geometry.index(point);
    if (!cell || !grid.cells[*cell].permitsTraversal()) {
      evidence = !cell ? "outside_planning_extent"
                       : "prohibited_or_inflated_cell";
      return false;
    }
  }
  evidence = "segment_cells_traversable";
  return true;
}

/**
 * @brief Performs the step name operation for this subsystem.
 *
 * Arguments:
 * - @p step: Supplies step input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string stepName(const planning::PlanStep& step) {
  return std::visit(
      [](const auto& value) -> std::string {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, planning::WaypointStep>) return "waypoint";
        if constexpr (std::is_same_v<T, planning::SubtrailStep>) return "subtrail";
        if constexpr (std::is_same_v<T, planning::RegionStep>) return "region";
        if constexpr (std::is_same_v<T, planning::VisibilityConnectionStep>) return "visibility_connection";
        if constexpr (std::is_same_v<T, planning::HighwayStep>) return "highway";
        if constexpr (std::is_same_v<T, planning::IntersectionStep>) return "intersection";
        if constexpr (std::is_same_v<T, planning::HighwayEntryStep>) return "highway_entry";
        if constexpr (std::is_same_v<T, planning::HighwayExitStep>) return "highway_exit";
        if constexpr (std::is_same_v<T, planning::SkeletonTransitionStep>) return "skeleton_transition";
        return "final_target";
      }, step);
}
}  // namespace

/**
 * @brief Evaluates package content for this subsystem.
 *
 * Arguments:
 * - @p result: Supplies result input to the operation.
 * - @p context: Supplies context input to the operation.
 *
 * Returns:
 * - `PlanEnforcementResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PlanEnforcementResult LocalActionEvaluator::evaluate(
    PlanEnforcementResult result,
    const PlanEnforcementContext& context) const {
  if (!result.operational_target) return result;
  const auto target = *result.operational_target;
  const auto score = [&](const domain::Pose2D& pose) {
    const double heading = std::atan2(target.y_m - pose.position.y_m,
                                      target.x_m - pose.position.x_m);
    return domain::distance(pose.position, target).meters() +
           0.25 * std::abs(domain::Angle::normalize(
                      heading - pose.heading.radians()));
  };
  const double before = score(context.pose);
  std::optional<std::size_t> best;
  double best_progress = 1e-6;
  for (const auto action : context.viable_actions) {
    const auto predicted = domain::expectedPoseAfterAction(
        context.pose, action, context.action_space);
    const double progress = before - score(predicted);
    result.candidates.push_back({action, predicted, progress, false});
    if (progress > best_progress) {
      best_progress = progress;
      best = result.candidates.size() - 1U;
    }
  }
  if (!best) {
    result.status = EnforcementStatus::CannotOperationalize;
    result.reason_code = "enforcer:no_local_action_makes_valid_progress";
    return result;
  }
  result.candidates[*best].selected = true;
  result.action = result.candidates[*best].action;
  result.status = EnforcementStatus::Mandated;
  result.reason_code = result.mode == EnforcerMode::Grid
                           ? "enforcer:grid_path_progress"
                           : "enforcer:model_step_progress";
  return result;
}

/**
 * @brief Performs the enforce operation for this subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 * - @p context: Supplies context input to the operation.
 *
 * Returns:
 * - `PlanEnforcementResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PlanEnforcementResult GridPlanEnforcer::enforce(
    planning::HierarchicalPlan& plan,
    const PlanEnforcementContext& context) const {
  PlanEnforcementResult result;
  result.mode = EnforcerMode::Grid;
  result.dependency_revisions = plan.dependency_revisions;
  if (plan.family != planning::PlanFamily::Grid) {
    result.status = EnforcementStatus::Invalid;
    result.reason_code = "enforcer:grid_family_mismatch";
    return result;
  }
  const auto changed = dependencyChanges(plan, context);
  if (!changed.empty()) {
    plan.validity = planning::PlanValidity::Stale;
    plan.diagnostics.insert(plan.diagnostics.end(), changed.begin(), changed.end());
    result.status = EnforcementStatus::Stale;
    result.reason_code = "enforcer:grid_dependency_changed:" + changed.front();
    return result;
  }
  const auto& path = plan.geometric_path;
  if (path.empty()) {
    plan.validity = planning::PlanValidity::Invalid;
    result.status = EnforcementStatus::Invalid;
    result.reason_code = "enforcer:grid_path_empty";
    return result;
  }
  const double deviation_limit = std::max(2.0, 4.0 * context.tolerance.meters());
  double nearest = std::numeric_limits<double>::infinity();
  std::size_t nearest_index = plan.cursor;
  for (std::size_t i = plan.cursor; i < path.size(); ++i) {
    const double candidate =
        domain::distance(context.pose.position, path[i]).meters();
    if (candidate < nearest) {
      nearest = candidate;
      nearest_index = i;
    }
  }
  if (nearest > deviation_limit) {
    plan.validity = planning::PlanValidity::Invalid;
    result.status = EnforcementStatus::Invalid;
    result.reason_code = "enforcer:grid_path_deviation";
    return result;
  }
  const std::size_t original = plan.cursor;
  plan.cursor = nearest_index;
  while (plan.cursor < path.size() && reached(context.pose, path[plan.cursor],
                                               context.tolerance))
    ++plan.cursor;
  if (plan.cursor >= path.size()) {
    plan.validity = planning::PlanValidity::Complete;
    result.status = EnforcementStatus::Complete;
    result.reason_code = "enforcer:grid_plan_complete";
    return result;
  }
  auto source = plan.static_map_contributed
                    ? planning::OccupancySourceMode::StaticMapWithSensors
                    : planning::OccupancySourceMode::SensorDerivedPartial;
  auto config = context.traversability;
  if (source == planning::OccupancySourceMode::SensorDerivedPartial)
    config.unknown_policy = config.sensor_unknown_policy;
  const auto traversal = planning::deriveTraversability(
      source, context.static_map, &context.spatial.sensed_occupancy, config);
  if (!traversal.grid.valid()) {
    result.status = EnforcementStatus::CannotOperationalize;
    result.reason_code = "enforcer:grid_traversability_unavailable";
    result.validation_evidence = traversal.diagnostic;
    return result;
  }
  std::size_t selected = plan.cursor;
  std::string evidence;
  for (std::size_t i = plan.cursor; i < path.size(); ++i) {
    std::string candidate_evidence;
    if (!segmentTraversable(traversal.grid, context.pose.position, path[i],
                            candidate_evidence))
      break;
    selected = i;
    evidence = std::move(candidate_evidence);
  }
  if (selected > plan.cursor) {
    result.shortcut = "grid_path_lookahead";
    result.skipped_elements = selected - plan.cursor;
    plan.operationalizations.push_back(
        {plan.cursor, "grid_path_lookahead_shortcut",
         plan.dependency_revisions});
    plan.cursor = selected;
  }
  result.step_index = plan.cursor;
  if (!result.shortcut) result.skipped_elements = plan.cursor - original;
  result.operational_target = path[plan.cursor];
  result.lookahead_m =
      domain::distance(context.pose.position, *result.operational_target).meters();
  result.step_type = "grid_waypoint";
  result.validation_evidence = evidence.empty() ? "next_path_cell_traversable"
                                                : evidence;
  return LocalActionEvaluator{}.evaluate(std::move(result), context);
}

/**
 * @brief Performs the enforce operation for this subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 * - @p context: Supplies context input to the operation.
 *
 * Returns:
 * - `PlanEnforcementResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PlanEnforcementResult ModelPlanEnforcer::enforce(
    planning::HierarchicalPlan& plan,
    const PlanEnforcementContext& context) const {
  PlanEnforcementResult result;
  result.mode = EnforcerMode::Model;
  result.dependency_revisions = plan.dependency_revisions;
  if (plan.family != planning::PlanFamily::Model) {
    result.status = EnforcementStatus::Invalid;
    result.reason_code = "enforcer:model_family_mismatch";
    return result;
  }
  const auto changed = dependencyChanges(plan, context);
  if (!changed.empty()) {
    plan.validity = planning::PlanValidity::Stale;
    plan.diagnostics.insert(plan.diagnostics.end(), changed.begin(), changed.end());
    result.status = EnforcementStatus::Stale;
    result.reason_code = "enforcer:model_dependency_changed:" + changed.front();
    return result;
  }
  const std::size_t before = plan.cursor;
  const auto target = Enforcer{}.operationalizeNext(
      plan, context.spatial, context.pose, context.tolerance);
  if (!target) {
    result.status = plan.validity == planning::PlanValidity::Complete
                        ? EnforcementStatus::Complete
                    : plan.validity == planning::PlanValidity::Stale
                        ? EnforcementStatus::Stale
                    : plan.validity == planning::PlanValidity::Invalid
                        ? EnforcementStatus::Invalid
                        : EnforcementStatus::CannotOperationalize;
    result.reason_code = "enforcer:model_step_not_operationalizable";
    return result;
  }
  result.step_index = plan.cursor;
  result.skipped_elements = plan.cursor > before ? plan.cursor - before : 0U;
  result.operational_target = *target;
  result.lookahead_m = domain::distance(context.pose.position, *target).meters();
  result.step_type = plan.cursor < plan.steps.size()
                         ? stepName(plan.steps[plan.cursor])
                         : "complete";
  result.validation_evidence = "typed_step_and_visibility_validated";
  if (!plan.operationalizations.empty()) {
    const auto& operation = plan.operationalizations.back().operation;
    if (operation.find("shortcut") != std::string::npos)
      result.shortcut = operation;
    if (operation.find("repair") != std::string::npos ||
        operation.find("replaced") != std::string::npos)
      result.repair = operation;
  }
  return LocalActionEvaluator{}.evaluate(std::move(result), context);
}

/**
 * @brief Performs the enforce operation for this subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 * - @p context: Supplies context input to the operation.
 *
 * Returns:
 * - `PlanEnforcementResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PlanEnforcementResult Enforcer::enforce(
    planning::HierarchicalPlan& plan,
    const PlanEnforcementContext& context) const {
  const auto cursor = plan.cursor;
  const auto validity = plan.validity;
  const auto operations = plan.operationalizations.size();
  const auto steps = plan.steps.size();
  auto result = plan.family == planning::PlanFamily::Grid
                    ? grid_.enforce(plan, context)
                    : model_.enforce(plan, context);
  if (cursor != plan.cursor || validity != plan.validity ||
      operations != plan.operationalizations.size() || steps != plan.steps.size())
    ++plan.execution_revision;
  return result;
}

/**
 * @brief Performs the operationalize operation for this subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 *
 * Returns:
 * - `std::vector<domain::Point2D>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<domain::Point2D> Enforcer::operationalize(
    const planning::HierarchicalPlan& plan) const {
  if (plan.validity != planning::PlanValidity::Valid || plan.exhausted())
    return {};
  const auto target = planning::stepTarget(plan.steps[plan.cursor]);
  return target ? std::vector<domain::Point2D>{*target}
                : std::vector<domain::Point2D>{};
}

/**
 * @brief Performs the active step operation for this subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 * - @p pose: Supplies pose input to the operation.
 * - @p tolerance: Supplies tolerance input to the operation.
 *
 * Returns:
 * - `std::size_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::size_t Enforcer::activeStep(const planning::HierarchicalPlan& plan,
                                 const domain::Pose2D& pose,
                                 domain::Distance tolerance) const noexcept {
  std::size_t cursor = plan.cursor;
  while (cursor < plan.steps.size()) {
    if (const auto* subtrail =
            std::get_if<planning::SubtrailStep>(&plan.steps[cursor])) {
      // A subtrail is a compound step. Its current marker may already be the
      // robot pose, but that advances only the subtrail cursor in
      // operationalizeNext; the plan step completes at the final marker.
      if (!subtrail->waypoints.empty() &&
          !reached(pose, subtrail->waypoints.back(), tolerance))
        break;
      ++cursor;
      continue;
    }
    const auto target = planning::stepTarget(plan.steps[cursor]);
    if (!target || !reached(pose, *target, tolerance)) break;
    ++cursor;
  }
  return cursor;
}

/**
 * @brief Performs the operationalize next operation for this subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 * - @p spatial: Supplies spatial input to the operation.
 * - @p pose: Supplies pose input to the operation.
 * - @p tolerance: Supplies tolerance input to the operation.
 *
 * Returns:
 * - `std::optional<domain::Point2D>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<domain::Point2D> Enforcer::operationalizeNext(
    planning::HierarchicalPlan& plan, const domain::SpatialModel& spatial,
    const domain::Pose2D& pose, domain::Distance tolerance) const {
  if (plan.validity != planning::PlanValidity::Valid) return std::nullopt;
  for (const auto& [dependency, consumed] : plan.dependency_revisions) {
    if (!isSpatialDependency(dependency)) continue;
    const auto current = spatial.revisionOf(dependency);
    if (current != consumed) {
      plan.validity = planning::PlanValidity::Stale;
      std::ostringstream diagnostic;
      diagnostic << "stale_plan:dependency_changed:"
                 << domain::toString(dependency) << ':' << consumed << "->"
                 << current;
      plan.diagnostics.push_back(diagnostic.str());
      return std::nullopt;
    }
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
      record(plan, "visible_second_step_shortcut", spatial,
             {domain::ModelDependency::VisibilityGeometry});
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
        record(plan, "subtrail_lookahead_shortcut", spatial,
               {domain::ModelDependency::VisibilityGeometry,
                domain::ModelDependency::Trails});
        plan.diagnostics.push_back("subtrail_lookahead_shortcut");
        break;
      }
    return trail->waypoints[trail->cursor];
  }
  if (auto* transition =
          std::get_if<planning::SkeletonTransitionStep>(&current)) {
    if (transition->supporting_subtrail.empty()) {
      plan.validity = planning::PlanValidity::Invalid;
      plan.diagnostics.push_back("invalid_skeleton_transition_no_subtrail");
      return std::nullopt;
    }
    current = planning::SubtrailStep{transition->supporting_subtrail,
                                     std::nullopt, 0U};
    record(plan, "skeleton_transition_operationalized_as_subtrail", spatial,
           {domain::ModelDependency::Skeleton,
            domain::ModelDependency::Trails});
    return operationalizeNext(plan, spatial, pose, tolerance);
  }
  if (auto* entry = std::get_if<planning::HighwayEntryStep>(&current)) {
    if (!entry->supporting_subtrail.empty()) {
      current = planning::SubtrailStep{entry->supporting_subtrail,
                                       std::nullopt, 0U};
      record(plan, "highway_entry_operationalized_as_subtrail", spatial,
             {domain::ModelDependency::Skeleton,
              domain::ModelDependency::Highways});
      return operationalizeNext(plan, spatial, pose, tolerance);
    }
    return entry->entry;
  }
  if (auto* exit = std::get_if<planning::HighwayExitStep>(&current)) {
    if (!exit->supporting_subtrail.empty()) {
      current = planning::SubtrailStep{exit->supporting_subtrail,
                                       std::nullopt, 0U};
      record(plan, "highway_exit_operationalized_as_subtrail", spatial,
             {domain::ModelDependency::Skeleton,
              domain::ModelDependency::Highways});
      return operationalizeNext(plan, spatial, pose, tolerance);
    }
    return exit->exit;
  }
  if (auto* region = std::get_if<planning::RegionStep>(&current)) {
    if (region->region_id >= spatial.learned_regions.size()) {
      plan.validity = planning::PlanValidity::Invalid;
      plan.diagnostics.push_back("invalid_region_step");
      return std::nullopt;
    }
    region->center = spatial.learned_regions[region->region_id].center;
    record(plan, "region_center_substitution", spatial,
           {domain::ModelDependency::Regions});
    if (spatial.learned_regions[region->region_id].contains(pose.position) &&
        plan.cursor + 1U < plan.steps.size()) {
      const auto next = planning::stepTarget(plan.steps[plan.cursor + 1U]);
      if (next && visible(pose, *next, spatial)) {
        record(plan, "visible_later_step_shortcut", spatial,
               {domain::ModelDependency::Regions,
                domain::ModelDependency::VisibilityGeometry});
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
        record(plan, "region_repaired_with_stored_subtrail", spatial,
               {domain::ModelDependency::Regions,
                domain::ModelDependency::Trails});
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
        spatial.highways.geometry.valid()) {
      std::vector<planning::PlanStep> regions;
      for (std::size_t id = 0; id < spatial.learned_regions.size(); ++id) {
        const bool overlaps = std::any_of(
            model->cells.begin(), model->cells.end(), [&](const auto& cell) {
              const domain::Point2D center = spatial.highways.geometry.center(
                  static_cast<std::size_t>(cell.column),
                  static_cast<std::size_t>(cell.row));
              return spatial.learned_regions[id].contains(center);
            });
        if (overlaps)
          regions.emplace_back(
              planning::RegionStep{id, spatial.learned_regions[id].center});
      }
      if (regions.size() >= 2U) {
        record(plan, "highway_replaced_with_overlapping_regions", spatial,
               {domain::ModelDependency::Highways,
                domain::ModelDependency::HighwayGraph,
                domain::ModelDependency::Regions,
                domain::ModelDependency::Familiarity});
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
      record(plan, "highway_repaired_with_stored_trail", spatial,
             {domain::ModelDependency::Highways,
              domain::ModelDependency::Trails});
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
