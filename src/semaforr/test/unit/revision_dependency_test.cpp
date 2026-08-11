#include <gtest/gtest.h>

#include <semaforr/decision/enforcer.hpp>
#include <semaforr/planning/planning_coordinator.hpp>

namespace {

class CountingPlanner final : public semaforr::planning::Planner {
 public:
  CountingPlanner(int& calls,
                  std::vector<semaforr::domain::ModelDependency> dependencies)
      : calls_(calls), dependencies_(std::move(dependencies)) {}

  semaforr::planning::PlanResult plan(
      const semaforr::planning::PlanningRequest& request) override {
    ++calls_;
    semaforr::planning::HierarchicalPlan hierarchy;
    hierarchy.planner = "counting";
    hierarchy.steps.emplace_back(
        semaforr::planning::WaypointStep{request.goal});
    return {semaforr::planning::PlanStatus::Success,
            {request.goal},
            semaforr::domain::distance(request.start.position, request.goal)
                .meters(),
            "counting test plan",
            std::move(hierarchy)};
  }
  std::string_view name() const noexcept override { return "counting"; }
  std::vector<semaforr::domain::ModelDependency> dependencies(
      const semaforr::planning::PlanningRequest&) const override {
    return dependencies_;
  }

 private:
  int& calls_;
  std::vector<semaforr::domain::ModelDependency> dependencies_;
};

semaforr::planning::PlanningRequest request(
    semaforr::domain::SpatialModel& spatial,
    semaforr::domain::CrowdModel* crowd = nullptr) {
  return {{{0.0, 0.0}, semaforr::domain::Angle::zero()},
          {2.0, 0.0}, &spatial, crowd, nullptr, {}, 7U};
}

semaforr::domain::CrowdFieldSnapshot crowdField(double density) {
  semaforr::domain::CrowdFieldSnapshot field;
  field.geometry = {1U, 1U, 1.0, {0.0, 0.0},
                    semaforr::domain::GridExtentMode::Fixed,
                    semaforr::domain::GridExtentSource::RepresentationLocalBounds};
  field.cells.resize(1U);
  field.cells[0].density = density;
  field.cells[0].visibility_exposures = 1.0;
  field.cells[0].confidence = 1.0;
  field.version = 1U;
  return field;
}

TEST(ExactDependencyRevisions, LowerRevisionChangeIsNotHiddenByHigherLayer) {
  using D = semaforr::domain::ModelDependency;
  semaforr::domain::SpatialModel spatial;
  spatial.revisions[D::Trails] = 100U;
  spatial.revisions[D::Skeleton] = 1U;
  int calls = 0;
  semaforr::planning::PlanningCoordinator coordinator;
  coordinator.registerPlanner(
      std::make_unique<CountingPlanner>(calls, std::vector<D>{D::Skeleton}));

  ASSERT_TRUE(coordinator.selectPlan(request(spatial)));
  ASSERT_TRUE(coordinator.selectPlan(request(spatial)));
  EXPECT_EQ(calls, 1);
  spatial.revisions[D::Skeleton] = 2U;
  ASSERT_TRUE(coordinator.selectPlan(request(spatial)));
  EXPECT_EQ(calls, 2);
}

TEST(ExactDependencyRevisions, UnrelatedCrowdChangeKeepsDistancePlanCached) {
  using D = semaforr::domain::ModelDependency;
  semaforr::domain::SpatialModel spatial;
  spatial.revisions[D::Skeleton] = 4U;
  semaforr::domain::CrowdModel crowd;
  crowd.setLearned(crowdField(0.1));
  int calls = 0;
  semaforr::planning::PlanningCoordinator coordinator;
  coordinator.registerPlanner(
      std::make_unique<CountingPlanner>(calls, std::vector<D>{D::Skeleton}));

  ASSERT_TRUE(coordinator.selectPlan(request(spatial, &crowd)));
  crowd.setLearned(crowdField(0.9));
  ASSERT_TRUE(coordinator.selectPlan(request(spatial, &crowd)));
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(coordinator.cacheHits(), 1U);
}

TEST(ExactDependencyRevisions, SameCrowdContentDoesNotAdvanceRevision) {
  using D = semaforr::domain::ModelDependency;
  semaforr::domain::CrowdModel crowd;
  const auto field = crowdField(0.4);
  crowd.setLearned(field);
  const auto revision = crowd.revisionOf(D::CrowdDensity);
  const auto sequence = crowd.mutationSequence();
  auto republished = field;
  republished.version = 99U;
  republished.generated_at += std::chrono::seconds(1);
  crowd.setLearned(std::move(republished));
  EXPECT_EQ(crowd.revisionOf(D::CrowdDensity), revision);
  EXPECT_EQ(crowd.mutationSequence(), sequence);
}

TEST(ExactDependencyRevisions, HighwayChangeStalesHighwayExecutionExactly) {
  using D = semaforr::domain::ModelDependency;
  semaforr::domain::SpatialModel spatial;
  spatial.revisions[D::HighwayGraph] = 2U;
  semaforr::planning::HierarchicalPlan plan;
  plan.steps.emplace_back(semaforr::planning::WaypointStep{{1.0, 0.0}});
  plan.dependency_revisions[D::HighwayGraph] = 1U;
  const auto next = semaforr::decision::Enforcer{}.operationalizeNext(
      plan, spatial, {{0.0, 0.0}, semaforr::domain::Angle::zero()},
      semaforr::domain::Distance(0.1));
  EXPECT_FALSE(next);
  EXPECT_EQ(plan.validity, semaforr::planning::PlanValidity::Stale);
  ASSERT_FALSE(plan.diagnostics.empty());
  EXPECT_EQ(plan.diagnostics.back(),
            "stale_plan:dependency_changed:highway_graph:1->2");
}

TEST(ExactDependencyRevisions, OccupancyChangeStalesRemainingGridRoute) {
  using D = semaforr::domain::ModelDependency;
  semaforr::domain::SpatialModel spatial;
  spatial.revisions[D::SensedOccupancy] = 8U;
  semaforr::planning::HierarchicalPlan plan;
  plan.steps.emplace_back(semaforr::planning::WaypointStep{{1.0, 0.0}});
  plan.dependency_revisions[D::SensedOccupancy] = 7U;
  EXPECT_FALSE(semaforr::decision::Enforcer{}.operationalizeNext(
      plan, spatial, {{0.0, 0.0}, semaforr::domain::Angle::zero()},
      semaforr::domain::Distance(0.1)));
  ASSERT_FALSE(plan.diagnostics.empty());
  EXPECT_NE(plan.diagnostics.back().find("sensed_occupancy:7->8"),
            std::string::npos);
}

TEST(ExactDependencyRevisions, OperationalizationRecordsItsOwnInputs) {
  using D = semaforr::domain::ModelDependency;
  semaforr::domain::SpatialModel spatial;
  spatial.revisions[D::VisibilityGeometry] = 3U;
  semaforr::planning::HierarchicalPlan plan;
  plan.steps.emplace_back(semaforr::planning::WaypointStep{{1.0, 0.0}});
  plan.steps.emplace_back(semaforr::planning::WaypointStep{{2.0, 0.0}});
  const auto next = semaforr::decision::Enforcer{}.operationalizeNext(
      plan, spatial, {{0.0, 0.0}, semaforr::domain::Angle::zero()},
      semaforr::domain::Distance(0.1));
  ASSERT_TRUE(next);
  ASSERT_EQ(plan.operationalizations.size(), 1U);
  EXPECT_EQ(plan.operationalizations.front().operation,
            "visible_second_step_shortcut");
  EXPECT_EQ(plan.operationalizations.front().dependency_revisions.at(
                D::VisibilityGeometry),
            3U);
}

TEST(ExactDependencyRevisions, ValidationNamesTaskPosePolicyAndExecution) {
  using D = semaforr::domain::ModelDependency;
  semaforr::domain::SpatialModel spatial;
  spatial.revisions[D::Skeleton] = 6U;
  semaforr::planning::PlanResult plan;
  plan.dependency_revisions = {{D::Skeleton, 5U},
                               {D::PlannerConfiguration, 2U}};
  plan.planned_start = {{0.0, 0.0}, semaforr::domain::Angle::zero()};
  plan.planned_goal = {2.0, 0.0};
  plan.task_id = 1U;
  auto changed = request(spatial);
  changed.start.position = {1.0, 0.0};
  changed.goal = {3.0, 0.0};
  changed.task_id = 2U;
  changed.planner_configuration_revision = 3U;
  const auto reasons = semaforr::planning::stalePlanReasons(
      plan, changed, semaforr::domain::Distance(0.1),
      semaforr::domain::Distance(0.1), true);
  EXPECT_EQ(reasons.size(), 6U);
}

TEST(ExactDependencyRevisions, WorldMutationJournalIsDiagnosticAndMonotonic) {
  using D = semaforr::domain::ModelDependency;
  semaforr::domain::WorldModel world;
  world.spatial.revisions[D::Skeleton] = 1U;
  world.spatial.mutation_history.push_back(
      {1U, D::Skeleton, 1U, std::chrono::steady_clock::now(),
       "skeleton connectivity changed"});
  world.crowd.setLearned(crowdField(0.2));
  world.synchronizeMutationJournal();
  ASSERT_GE(world.mutation_history.size(), 2U);
  for (std::size_t index = 0U; index < world.mutation_history.size(); ++index)
    EXPECT_EQ(world.mutation_history[index].sequence, index + 1U);
  EXPECT_EQ(world.mutation_sequence, world.mutation_history.size());
  EXPECT_EQ(world.spatial.revisionOf(D::Skeleton), 1U);
  EXPECT_NE(world.spatial.revisionOf(D::Trails), 1U);
}

}  // namespace
