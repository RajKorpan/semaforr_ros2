/**
 * @file tier_two_enforcer_test.cpp
 * @brief Tier two enforcer test responsibilities.
 *
 * @details This file exercises tier two enforcer test behavior for automated
 * verification and regression testing. It centers on `EvidencePlanner`,
 * `CompressesStraightPathToFarthestTraversablePoint`,
 * `RefusesObstacleShortcutAndDetectsRelevantRevision`,
 * `DetectsPathDeviationAndCompletion`,
 * `FollowsAPathTurnWithAHeadingAction`,
 * `OperationalizesRegionsSubtrailsAndSkeletonTransitions`,
 * `OperationalizesHighwayTrailAndRejectsStaleGraph`,
 * `HandlesRegionIntersectionEntryExitAndFinalTargetTypes`. Its
 * package-relative location is `test/unit/tier_two_enforcer_test.cpp`.
 */
#include <gtest/gtest.h>

#include <semaforr/decision/enforcer.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/planner_registry.hpp>
#include <semaforr/planning/planning_coordinator.hpp>

namespace {
using namespace semaforr;

/**
 * @brief Performs the free grid operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `domain::SpatialModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::SpatialModel freeGrid() {
  domain::SpatialModel spatial;
  spatial.sensed_occupancy.geometry = {8U, 3U, 1.0, {0.0, 0.0}};
  spatial.sensed_occupancy.cells.resize(24U);
  for (auto& cell : spatial.sensed_occupancy.cells)
    cell.state = domain::SensedOccupancyState::ObservedFree;
  spatial.revisions[domain::ModelDependency::SensedOccupancy] = 3U;
  return spatial;
}

/**
 * @brief Performs the grid plan operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `planning::HierarchicalPlan` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
planning::HierarchicalPlan gridPlan() {
  planning::HierarchicalPlan plan;
  plan.id = 41U;
  plan.family = planning::PlanFamily::Grid;
  plan.planner = "sensor_distance";
  plan.planned_start = {{0.5, 1.5}, domain::Angle::zero()};
  plan.planned_goal = {5.5, 1.5};
  plan.geometric_path = {{1.5, 1.5}, {2.5, 1.5}, {4.5, 1.5}, {5.5, 1.5}};
  for (const auto point : plan.geometric_path)
    plan.steps.emplace_back(planning::WaypointStep{point});
  plan.dependency_revisions[domain::ModelDependency::SensedOccupancy] = 3U;
  return plan;
}

/**
 * @brief Performs the zero inflation operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `planning::TraversabilityConfiguration` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
planning::TraversabilityConfiguration zeroInflation() {
  planning::TraversabilityConfiguration config;
  config.robot_radius_m = 0.0;
  config.safety_clearance_m = 0.0;
  config.localization_uncertainty_m = 0.0;
  config.dynamic_obstacle_margin_m = 0.0;
  return config;
}

/**
 * @brief Performs the context operation for this subsystem.
 *
 * Arguments:
 * - @p spatial: Supplies spatial input to the operation.
 * - @p pose: Supplies pose input to the operation.
 * - @p actions: Supplies actions input to the operation.
 * - @p viable: Supplies viable input to the operation.
 *
 * Returns:
 * - `decision::PlanEnforcementContext` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
decision::PlanEnforcementContext context(
    const domain::SpatialModel& spatial, const domain::Pose2D& pose,
    const domain::ActionSpace& actions,
    const std::vector<domain::Action>& viable) {
  return {spatial, nullptr, nullptr, pose, actions, viable, zeroInflation(),
          domain::Distance(0.2), std::nullopt};
}

TEST(GridPlanEnforcer, CompressesStraightPathToFarthestTraversablePoint) {
  auto spatial = freeGrid();
  auto plan = gridPlan();
  const domain::Pose2D pose{{0.5, 1.5}, domain::Angle::zero()};
  const domain::ActionSpace actions({0.5, 1.0, 2.0}, {0.5});
  const std::vector<domain::Action> viable = {
      domain::Action::pause(), {domain::ActionType::Forward, 1U},
      {domain::ActionType::Forward, 2U}, {domain::ActionType::Forward, 3U},
      {domain::ActionType::TurnLeft, 1U},
      {domain::ActionType::TurnRight, 1U}};
  const auto result =
      decision::GridPlanEnforcer{}.enforce(plan, context(spatial, pose, actions, viable));
  ASSERT_EQ(result.status, decision::EnforcementStatus::Mandated);
  EXPECT_EQ(result.operational_target, plan.geometric_path.back());
  EXPECT_EQ(result.step_index, 3U);
  EXPECT_EQ(result.skipped_elements, 3U);
  EXPECT_EQ(result.shortcut, "grid_path_lookahead");
  EXPECT_EQ(result.action,
            domain::Action(domain::ActionType::Forward, 3U));
}

TEST(GridPlanEnforcer, RefusesObstacleShortcutAndDetectsRelevantRevision) {
  auto spatial = freeGrid();
  spatial.sensed_occupancy.cells[11U].state =
      domain::SensedOccupancyState::ObservedOccupied;
  auto plan = gridPlan();
  const domain::Pose2D pose{{0.5, 1.5}, domain::Angle::zero()};
  const domain::ActionSpace actions({0.5, 1.0}, {0.5});
  const std::vector<domain::Action> viable = {
      {domain::ActionType::Forward, 1U},
      {domain::ActionType::Forward, 2U},
      {domain::ActionType::TurnLeft, 1U},
      {domain::ActionType::TurnRight, 1U}};
  auto result =
      decision::GridPlanEnforcer{}.enforce(plan, context(spatial, pose, actions, viable));
  ASSERT_EQ(result.status, decision::EnforcementStatus::Mandated);
  ASSERT_TRUE(result.operational_target);
  EXPECT_LT(result.operational_target->x_m, 3.5);

  spatial.revisions[domain::ModelDependency::Regions] = 99U;
  result = decision::GridPlanEnforcer{}.enforce(
      plan, context(spatial, pose, actions, viable));
  EXPECT_NE(result.status, decision::EnforcementStatus::Stale);
  spatial.revisions[domain::ModelDependency::SensedOccupancy] = 4U;
  result = decision::GridPlanEnforcer{}.enforce(
      plan, context(spatial, pose, actions, viable));
  EXPECT_EQ(result.status, decision::EnforcementStatus::Stale);
  EXPECT_NE(result.reason_code.find("sensed_occupancy"), std::string::npos);
}

TEST(GridPlanEnforcer, DetectsPathDeviationAndCompletion) {
  auto spatial = freeGrid();
  auto deviated = gridPlan();
  const domain::ActionSpace actions({1.0}, {0.5});
  const std::vector<domain::Action> viable = {
      {domain::ActionType::Forward, 1U},
      {domain::ActionType::TurnLeft, 1U}};
  domain::Pose2D pose{{0.5, -5.0}, domain::Angle::zero()};
  EXPECT_EQ(decision::GridPlanEnforcer{}
                .enforce(deviated, context(spatial, pose, actions, viable))
                .status,
            decision::EnforcementStatus::Invalid);
  auto complete = gridPlan();
  pose.position = complete.geometric_path.back();
  EXPECT_EQ(decision::GridPlanEnforcer{}
                .enforce(complete, context(spatial, pose, actions, viable))
                .status,
            decision::EnforcementStatus::Complete);
}

TEST(GridPlanEnforcer, FollowsAPathTurnWithAHeadingAction) {
  auto spatial = freeGrid();
  auto plan = gridPlan();
  plan.geometric_path = {{0.5, 2.5}, {0.5, 2.5}};
  plan.steps = {planning::WaypointStep{{0.5, 2.5}}};
  const domain::Pose2D pose{{0.5, 1.5}, domain::Angle::zero()};
  const domain::ActionSpace actions({0.5}, {0.5, 1.0, 1.57});
  const std::vector<domain::Action> viable = {
      {domain::ActionType::Forward, 1U},
      {domain::ActionType::TurnLeft, 1U},
      {domain::ActionType::TurnLeft, 2U},
      {domain::ActionType::TurnLeft, 3U},
      {domain::ActionType::TurnRight, 1U}};
  const auto result = decision::GridPlanEnforcer{}.enforce(
      plan, context(spatial, pose, actions, viable));
  ASSERT_EQ(result.status, decision::EnforcementStatus::Mandated);
  EXPECT_EQ(result.action,
            domain::Action(domain::ActionType::TurnLeft, 3U));
}

TEST(ModelPlanEnforcer, OperationalizesRegionsSubtrailsAndSkeletonTransitions) {
  domain::SpatialModel spatial;
  spatial.learned_regions = {{{2.0, 0.0}, domain::Distance(1.0)}};
  spatial.revisions[domain::ModelDependency::Regions] = 1U;
  const domain::Pose2D pose{{0.0, 0.0}, domain::Angle::zero()};
  const domain::ActionSpace actions({0.5, 1.0}, {0.5});
  const std::vector<domain::Action> viable = {
      {domain::ActionType::Forward, 1U},
      {domain::ActionType::Forward, 2U},
      {domain::ActionType::TurnLeft, 1U}};
  planning::HierarchicalPlan plan;
  plan.family = planning::PlanFamily::Model;
  plan.planner = "skeleton_plan";
  plan.steps.emplace_back(planning::SkeletonTransitionStep{
      0U, 1U, {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}});
  const auto result = decision::ModelPlanEnforcer{}.enforce(
      plan, context(spatial, pose, actions, viable));
  ASSERT_EQ(result.status, decision::EnforcementStatus::Mandated);
  EXPECT_EQ(result.step_type, "subtrail");
  EXPECT_TRUE(std::holds_alternative<planning::SubtrailStep>(plan.steps[0]));
  EXPECT_EQ(result.action,
            domain::Action(domain::ActionType::Forward, 2U));
}

TEST(ModelPlanEnforcer, OperationalizesHighwayTrailAndRejectsStaleGraph) {
  domain::SpatialModel spatial;
  spatial.revisions[domain::ModelDependency::HighwayGraph] = 7U;
  const domain::Pose2D pose{{0.0, 0.0}, domain::Angle::zero()};
  const domain::ActionSpace actions({1.0}, {0.5});
  const std::vector<domain::Action> viable = {
      {domain::ActionType::Forward, 1U},
      {domain::ActionType::TurnLeft, 1U}};
  planning::HierarchicalPlan plan;
  plan.family = planning::PlanFamily::Model;
  plan.planner = "highway_plan";
  plan.dependency_revisions[domain::ModelDependency::HighwayGraph] = 7U;
  plan.steps.emplace_back(
      planning::HighwayStep{4U, 1U, 2U, {{1.0, 0.0}, {2.0, 0.0}}});
  auto result = decision::ModelPlanEnforcer{}.enforce(
      plan, context(spatial, pose, actions, viable));
  EXPECT_EQ(result.status, decision::EnforcementStatus::Mandated);
  EXPECT_TRUE(std::holds_alternative<planning::SubtrailStep>(plan.steps[0]));
  spatial.revisions[domain::ModelDependency::HighwayGraph] = 8U;
  result = decision::ModelPlanEnforcer{}.enforce(
      plan, context(spatial, pose, actions, viable));
  EXPECT_EQ(result.status, decision::EnforcementStatus::Stale);
}

TEST(ModelPlanEnforcer, HandlesRegionIntersectionEntryExitAndFinalTargetTypes) {
  domain::SpatialModel spatial;
  spatial.learned_regions = {{{1.0, 0.0}, domain::Distance(0.5)}};
  const domain::Pose2D pose{{0.0, 0.0}, domain::Angle::zero()};
  const domain::ActionSpace actions({0.5, 1.0}, {0.5});
  const std::vector<domain::Action> viable = {
      {domain::ActionType::Forward, 1U},
      {domain::ActionType::Forward, 2U},
      {domain::ActionType::TurnLeft, 1U}};
  const auto run = [&](planning::PlanStep step, std::string expected_type) {
    planning::HierarchicalPlan plan;
    plan.family = planning::PlanFamily::Model;
    plan.steps.push_back(std::move(step));
    const auto result = decision::ModelPlanEnforcer{}.enforce(
        plan, context(spatial, pose, actions, viable));
    EXPECT_EQ(result.status, decision::EnforcementStatus::Mandated);
    EXPECT_EQ(result.step_type, expected_type);
    EXPECT_TRUE(result.operational_target.has_value());
  };
  run(planning::RegionStep{0U, {1.0, 0.0}}, "region");
  run(planning::VisibilityConnectionStep{
          0U, {0.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}, {2.0, 0.0}, 9U, true},
      "visibility_connection");
  run(planning::IntersectionStep{2U, {1.0, 0.0}}, "intersection");
  run(planning::HighwayEntryStep{3U, {1.0, 0.0}, {}}, "highway_entry");
  run(planning::HighwayExitStep{3U, {1.0, 0.0}, {}}, "highway_exit");
  run(planning::FinalTargetStep{{1.0, 0.0}}, "final_target");
}

TEST(ModelPlanEnforcer, ReportsInvalidRegionRepairFailurePrecisely) {
  domain::SpatialModel spatial;
  const domain::Pose2D pose{{0.0, 0.0}, domain::Angle::zero()};
  const domain::ActionSpace actions({1.0}, {0.5});
  const std::vector<domain::Action> viable = {
      {domain::ActionType::Forward, 1U}};
  planning::HierarchicalPlan plan;
  plan.family = planning::PlanFamily::Model;
  plan.steps.emplace_back(planning::RegionStep{99U, {1.0, 0.0}});
  const auto result = decision::ModelPlanEnforcer{}.enforce(
      plan, context(spatial, pose, actions, viable));
  EXPECT_EQ(result.status, decision::EnforcementStatus::Invalid);
  ASSERT_FALSE(plan.diagnostics.empty());
  EXPECT_EQ(plan.diagnostics.back(), "invalid_region_step");
}

/**
 * @brief Encapsulates evidence planner state and behavior for this
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
class EvidencePlanner final : public planning::Planner {
 public:
  /**
   * @brief Performs the evidence planner operation for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   * - @p objective: Supplies objective input to the operation.
   * - @p path: Supplies path input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  EvidencePlanner(std::string name, planning::PlanObjective objective,
                  std::vector<domain::Point2D> path)
      : name_(std::move(name)), objective_(objective), path_(std::move(path)) {}
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
  std::string_view name() const noexcept override { return name_; }
  /**
   * @brief Performs the objective operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `planning::PlanObjective` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  planning::PlanObjective objective() const noexcept override { return objective_; }
  /**
   * @brief Constructs family for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `planning::PlanFamily` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  planning::PlanFamily planFamily() const noexcept override {
    return planning::PlanFamily::Grid;
  }
  /**
   * @brief Constructs package content for this subsystem.
   *
   * Arguments:
   * - @p PlanningRequest: Supplies planning request input to the operation.
   *
   * Returns:
   * - `planning::PlanResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  planning::PlanResult plan(const planning::PlanningRequest&) override {
    return {planning::PlanStatus::Success, path_, 1.0, "evidence fixture"};
  }
 private:
  std::string name_;
  planning::PlanObjective objective_;
  std::vector<domain::Point2D> path_;
};

TEST(PlanningCoordinator, RetainsFullCrossObjectiveRangeVoteEvidence) {
  planning::PlanningCoordinator coordinator(planning::PlanSelectionPolicy::RangeVote);
  coordinator.registerPlanner(std::make_unique<EvidencePlanner>(
      "direct", planning::PlanObjective::Distance,
      std::vector<domain::Point2D>{{2.0, 0.0}}));
  coordinator.registerPlanner(std::make_unique<EvidencePlanner>(
      "detour", planning::PlanObjective::RegionPreference,
      std::vector<domain::Point2D>{{0.0, 2.0}, {2.0, 0.0}}));
  domain::SpatialModel spatial;
  const auto selected = coordinator.selectPlan(
      {{{0.0, 0.0}, domain::Angle::zero()}, {2.0, 0.0}, &spatial});
  ASSERT_TRUE(selected);
  ASSERT_EQ(selected->evidence.candidates.size(), 2U);
  EXPECT_NE(selected->evidence.candidates[0].plan_id,
            selected->evidence.candidates[1].plan_id);
  for (const auto& candidate : selected->evidence.candidates) {
    EXPECT_EQ(candidate.raw_costs.size(), 10U);
    EXPECT_EQ(candidate.normalized_costs.size(), 2U);
  }
  EXPECT_FALSE(selected->evidence.tie_break_reason.empty());
}

TEST(PlannerRegistry, DeclaresActualPlanFamilyAndOccupancyContracts) {
  const auto registry = planning::defaultPlannerRegistry();
  EXPECT_EQ(registry.create("distance")->planFamily(),
            planning::PlanFamily::Grid);
  EXPECT_EQ(registry.create("region")->planFamily(), planning::PlanFamily::Grid);
  EXPECT_EQ(registry.create("skeleton")->planFamily(),
            planning::PlanFamily::Model);
  EXPECT_EQ(registry.create("highway")->planFamily(),
            planning::PlanFamily::Model);
  EXPECT_EQ(registry.occupancyRequirement("region"),
            planning::OccupancyRequirement::StaticOrSensedPartial);
  domain::SpatialModel spatial;
  const auto declaration = registry.declaration(
      "region", {{{0.0, 0.0}, domain::Angle::zero()}, {1.0, 0.0}, &spatial});
  EXPECT_TRUE(declaration.supports_partial_sensor_occupancy);
  EXPECT_EQ(declaration.objective, planning::PlanObjective::RegionPreference);
  EXPECT_NE(std::find(declaration.revision_dependencies.begin(),
                      declaration.revision_dependencies.end(),
                      domain::ModelDependency::Regions),
            declaration.revision_dependencies.end());
}

TEST(PlannerRegistry, EveryPlannerPublishesCompleteExplanationMetadata) {
  const auto registry = planning::defaultPlannerRegistry();
  domain::SpatialModel spatial;
  const planning::PlanningRequest request{
      {{0.0, 0.0}, domain::Angle::zero()}, {1.0, 0.0}, &spatial};
  for (const auto input : {planning::PlannerInputModel::Grid,
                           planning::PlannerInputModel::AffordanceModifiedGrid,
                           planning::PlannerInputModel::Freespace}) {
    for (const auto& name : registry.names(input)) {
      const auto declaration = registry.declaration(name, request);
      const auto& metadata = declaration.explanation_metadata;
      EXPECT_TRUE(metadata.valid()) << name;
      EXPECT_EQ(metadata.name, name);
      EXPECT_FALSE(metadata.objective_description.empty()) << name;
      EXPECT_FALSE(metadata.representation_dependencies.empty()) << name;
      if (declaration.static_map == planning::StaticMapRequirement::Required) {
        EXPECT_TRUE(metadata.requires_static_map) << name;
        EXPECT_FALSE(metadata.supports_mapless_operation) << name;
      }
    }
  }
}
}  // namespace
