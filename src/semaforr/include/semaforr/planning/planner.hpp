/**
 * @file planner.hpp
 * @brief Planner responsibilities.
 *
 * @details This file defines planner behavior for path planning and hierarchical
 * plan construction. It centers on `PlanFamily`, `PlanningOperatingMode`,
 * `PlanObjective`, `PlannerMetadata`, `WaypointStep`, `SubtrailStep`,
 * `RegionStep`, `VisibilityConnectionStep`. Its package-relative location
 * is `include/semaforr/planning/planner.hpp`.
 */
#ifndef SEMAFORR_PLANNING_PLANNER_HPP
#define SEMAFORR_PLANNING_PLANNER_HPP

#include <chrono>
#include <map>
#include <optional>
#include <semaforr/domain/highway.hpp>
#include <semaforr/domain/world_model.hpp>
#include <semaforr/planning/traversability.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace semaforr::planning {

using PlanId = std::uint64_t;
/**
 * @brief Enumerates the supported plan family values used by this
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
enum class PlanFamily { Grid, Model };
/**
 * @brief Enumerates the supported planning operating mode values used by
 * this subsystem.
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
enum class PlanningOperatingMode { MapEnabled, Mapless };
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p family: Supplies family input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(PlanFamily family) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p mode: Supplies mode input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(PlanningOperatingMode mode) noexcept;

/**
 * @brief Enumerates the supported plan objective values used by this
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
enum class PlanObjective {
  Distance,
  CrowdDensity,
  EncounterRisk,
  FlowOpposition,
  RegionPreference,
  HallwayPreference,
  TrailPreference,
  ConveyorPreference,
  SkeletonDistance,
  HighwayDistance
};

/**
 * @brief Encapsulates planner metadata state and behavior for this
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
struct PlannerMetadata {
  std::string name;
  PlanFamily plan_family{PlanFamily::Grid};
  PlanObjective primary_objective{PlanObjective::Distance};
  std::string objective_name;
  std::string objective_description;
  std::vector<std::string> representation_dependencies;
  bool requires_static_map{false};
  bool supports_mapless_operation{false};

  /**
   * @brief Performs the valid operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool valid() const noexcept {
    return !name.empty() && !objective_name.empty() &&
           !objective_description.empty();
  }
};

using ObjectiveCosts = std::map<PlanObjective, double>;

/**
 * @brief Encapsulates waypoint step state and behavior for this subsystem.
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
struct WaypointStep {
  domain::Point2D target;
};
/**
 * @brief Encapsulates subtrail step state and behavior for this subsystem.
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
struct SubtrailStep {
  std::vector<domain::Point2D> waypoints;
  std::optional<domain::TrailId> trail_id;
  std::size_t cursor = 0U;
};
/**
 * @brief Encapsulates region step state and behavior for this subsystem.
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
struct RegionStep {
  std::size_t region_id = 0U;
  domain::Point2D center;
};
/**
 * @brief Encapsulates visibility connection step state and behavior for
 * this subsystem.
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
struct VisibilityConnectionStep {
  std::size_t region_id = 0U;
  domain::Point2D from;
  domain::Point2D to;
  domain::Point2D evidence_ray_start;
  domain::Point2D evidence_ray_end;
  domain::DecisionId supporting_decision{0U};
  bool toward_region{true};
};
/**
 * @brief Encapsulates highway step state and behavior for this subsystem.
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
struct HighwayStep {
  domain::HighwayId highway_id = 0U;
  domain::IntersectionId from = 0U;
  domain::IntersectionId to = 0U;
  std::vector<domain::Point2D> fallback_subtrail;
};
/**
 * @brief Encapsulates intersection step state and behavior for this
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
struct IntersectionStep {
  domain::IntersectionId intersection_id = 0U;
  domain::Point2D centroid;
};
/**
 * @brief Encapsulates highway entry step state and behavior for this
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
struct HighwayEntryStep {
  domain::HighwayId highway_id = 0U;
  domain::Point2D entry;
  std::vector<domain::Point2D> supporting_subtrail;
};
/**
 * @brief Encapsulates highway exit step state and behavior for this
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
struct HighwayExitStep {
  domain::HighwayId highway_id = 0U;
  domain::Point2D exit;
  std::vector<domain::Point2D> supporting_subtrail;
};
/**
 * @brief Encapsulates skeleton transition step state and behavior for this
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
struct SkeletonTransitionStep {
  std::size_t from_region = 0U;
  std::size_t to_region = 0U;
  std::vector<domain::Point2D> supporting_subtrail;
};
/**
 * @brief Encapsulates final target step state and behavior for this
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
struct FinalTargetStep {
  domain::Point2D target;
};

using PlanStep = std::variant<WaypointStep, SubtrailStep, RegionStep,
                              VisibilityConnectionStep, HighwayStep,
                              IntersectionStep, HighwayEntryStep,
                              HighwayExitStep, SkeletonTransitionStep,
                              FinalTargetStep>;

/**
 * @brief Enumerates the supported plan validity values used by this
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
enum class PlanValidity { Valid, Stale, Invalid, Complete };
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p validity: Supplies validity input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(PlanValidity validity) noexcept;

/**
 * @brief Encapsulates hierarchical plan state and behavior for this
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
struct HierarchicalPlan {
  PlanId id = 0U;
  domain::Revision execution_revision = 1U;
  PlanFamily family = PlanFamily::Grid;
  std::string planner;
  PlanObjective objective = PlanObjective::Distance;
  std::vector<PlanStep> steps;
  ObjectiveCosts estimated_objective_costs;
  domain::DependencyRevisions dependency_revisions;
  domain::Pose2D planned_start;
  domain::Point2D planned_goal;
  std::optional<domain::TaskId> task_id;
  domain::Revision planner_configuration_revision = 0U;
  std::chrono::steady_clock::time_point created_at{};
  PlanningOperatingMode operating_mode{PlanningOperatingMode::Mapless};
  bool static_map_contributed{false};
  std::vector<domain::Point2D> geometric_path;
  /**
   * @brief Encapsulates operationalization record state and behavior for
   * this subsystem.
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
  struct OperationalizationRecord {
    std::size_t step_index = 0U;
    std::string operation;
    domain::DependencyRevisions dependency_revisions;
  };
  std::vector<OperationalizationRecord> operationalizations;
  std::size_t cursor = 0U;
  std::string provenance;
  PlanValidity validity = PlanValidity::Valid;
  std::vector<std::string> diagnostics;

  /**
   * @brief Performs the empty operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool empty() const noexcept { return steps.empty(); }
  /**
   * @brief Performs the exhausted operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool exhausted() const noexcept { return cursor >= steps.size(); }
};

/**
 * @brief Performs the step target operation for this subsystem.
 *
 * Arguments:
 * - @p step: Supplies step input to the operation.
 *
 * Returns:
 * - `std::optional<domain::Point2D>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<domain::Point2D> stepTarget(const PlanStep& step) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p objective: Supplies objective input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(PlanObjective objective) noexcept;
/**
 * @brief Performs the objective description operation for this subsystem.
 *
 * Arguments:
 * - @p objective: Supplies objective input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view objectiveDescription(PlanObjective objective) noexcept;

/**
 * @brief Enumerates the supported plan status values used by this
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
enum class PlanStatus { Success, NoPath, InvalidRequest, PlannerUnavailable };

/**
 * @brief Encapsulates planning request state and behavior for this
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
struct PlanningRequest {
  domain::Pose2D start;
  domain::Point2D goal;
  const domain::SpatialModel* spatial_model{nullptr};
  const domain::CrowdModel* crowd_model{nullptr};
  const domain::StaticMap* static_map{nullptr};
  TraversabilityConfiguration traversability;
  std::optional<domain::TaskId> task_id;
  domain::Revision planner_configuration_revision = 0U;

  /**
   * @brief Constructs ning request for this subsystem.
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
  PlanningRequest() = default;
  /**
   * @brief Constructs ning request for this subsystem.
   *
   * Arguments:
   * - @p request_start: Supplies request start input to the operation.
   * - @p request_goal: Supplies request goal input to the operation.
   * - @p request_spatial: Supplies request spatial input to the operation.
   * - @p request_crowd: Supplies request crowd input to the operation.
   * - @p request_map: Supplies request map input to the operation.
   * - @p traversal_configuration: Supplies traversal configuration input to
   * the operation.
   * - @p request_task: Supplies request task input to the operation.
   * - @p configuration_revision: Supplies configuration revision input to
   * the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PlanningRequest(domain::Pose2D request_start, domain::Point2D request_goal,
                  const domain::SpatialModel* request_spatial = nullptr,
                  const domain::CrowdModel* request_crowd = nullptr,
                  const domain::StaticMap* request_map = nullptr,
                  TraversabilityConfiguration traversal_configuration = {},
                  std::optional<domain::TaskId> request_task = std::nullopt,
                  domain::Revision configuration_revision = 0U)
      : start(request_start),
        goal(request_goal),
        spatial_model(request_spatial),
        crowd_model(request_crowd),
        static_map(request_map),
        traversability(std::move(traversal_configuration)),
        task_id(request_task),
        planner_configuration_revision(configuration_revision) {}
};

/**
 * @brief Encapsulates plan result state and behavior for this subsystem.
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
struct PlanResult {
  PlanId plan_id = 0U;
  PlanFamily family = PlanFamily::Grid;
  PlanStatus status{PlanStatus::PlannerUnavailable};
  std::vector<domain::Point2D> path;
  double cost_m{0.0};
  std::string explanation;
  std::optional<HierarchicalPlan> hierarchical;
  PlanObjective primary_objective = PlanObjective::Distance;
  ObjectiveCosts objective_costs;
  domain::DependencyRevisions dependency_revisions;
  domain::Pose2D planned_start;
  domain::Point2D planned_goal;
  std::optional<domain::TaskId> task_id;
  domain::Revision planner_configuration_revision = 0U;
  std::chrono::steady_clock::time_point created_at{};
  PlanningOperatingMode operating_mode{PlanningOperatingMode::Mapless};
  bool static_map_contributed{false};
  std::vector<std::string> stale_reasons;

  /**
   * @brief Constructs result for this subsystem.
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
  PlanResult() = default;
  /**
   * @brief Constructs result for this subsystem.
   *
   * Arguments:
   * - @p plan_status: Supplies plan status input to the operation.
   * - @p plan_path: Supplies plan path input to the operation.
   * - @p plan_cost_m: Supplies plan cost m input to the operation.
   * - @p plan_explanation: Supplies plan explanation input to the
   * operation.
   * - @p hierarchical_plan: Supplies hierarchical plan input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PlanResult(PlanStatus plan_status, std::vector<domain::Point2D> plan_path,
             double plan_cost_m, std::string plan_explanation,
             std::optional<HierarchicalPlan> hierarchical_plan = std::nullopt)
      : status(plan_status),
        path(std::move(plan_path)),
        cost_m(plan_cost_m),
        explanation(std::move(plan_explanation)),
        hierarchical(std::move(hierarchical_plan)) {}

  /**
   * @brief Performs the succeeded operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool succeeded() const noexcept { return status == PlanStatus::Success; }
};

/**
 * @brief Encapsulates planner state and behavior for this subsystem.
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
class Planner {
 public:
  /**
   * @brief Constructs ner for this subsystem.
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
  virtual ~Planner() = default;
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
  virtual PlanResult plan(const PlanningRequest& request) = 0;
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::string_view name() const noexcept = 0;
  /**
   * @brief Performs the objective operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `PlanObjective` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual PlanObjective objective() const noexcept {
    return PlanObjective::Distance;
  }
  // Custom/test planners default to geometric output; production planners
  // override this declaration explicitly.
  /**
   * @brief Constructs family for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `PlanFamily` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual PlanFamily planFamily() const noexcept { return PlanFamily::Grid; }
  /**
   * @brief Performs the metadata operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `PlannerMetadata` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual PlannerMetadata metadata() const;
  /**
   * @brief Performs the dependencies operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `std::vector<domain::ModelDependency>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual std::vector<domain::ModelDependency> dependencies(
      const PlanningRequest&) const {
    return {};
  }
};

/**
 * @brief Performs the current revision operation for this subsystem.
 *
 * Arguments:
 * - @p request: Supplies request input to the operation.
 * - @p dependency: Supplies dependency input to the operation.
 *
 * Returns:
 * - `domain::Revision` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::Revision currentRevision(const PlanningRequest& request,
                                 domain::ModelDependency dependency) noexcept;
/**
 * @brief Performs the dependency change reasons operation for this
 * subsystem.
 *
 * Arguments:
 * - @p consumed: Supplies consumed input to the operation.
 * - @p request: Supplies request input to the operation.
 *
 * Returns:
 * - `std::vector<std::string>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<std::string> dependencyChangeReasons(
    const domain::DependencyRevisions& consumed,
    const PlanningRequest& request);
/**
 * @brief Performs the attach dependency snapshot operation for this
 * subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 * - @p request: Supplies request input to the operation.
 * - @p dependencies: Supplies dependencies input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void attachDependencySnapshot(
    PlanResult& plan, const PlanningRequest& request,
    std::vector<domain::ModelDependency> dependencies);
/**
 * @brief Performs the stale plan reasons operation for this subsystem.
 *
 * Arguments:
 * - @p plan: Supplies plan input to the operation.
 * - @p request: Supplies request input to the operation.
 * - @p start_tolerance: Supplies start tolerance input to the operation.
 * - @p target_tolerance: Supplies target tolerance input to the operation.
 * - @p execution_invalidated: Supplies execution invalidated input to the
 * operation.
 *
 * Returns:
 * - `std::vector<std::string>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<std::string> stalePlanReasons(
    const PlanResult& plan, const PlanningRequest& request,
    domain::Distance start_tolerance, domain::Distance target_tolerance,
    bool execution_invalidated = false);

}  // namespace semaforr::planning
#endif
