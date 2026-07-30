#include <gtest/gtest.h>
#include <memory>
#include <semaforr/decision/enforcer.hpp>
#include <semaforr/exploration/highway_explorer.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/planning_coordinator.hpp>
#include <semaforr/spatial/highway_learner.hpp>

namespace {

semaforr::domain::RobotObservation observation(
    double x, std::vector<double> ranges = {2.0, 2.0, 2.0, 2.0, 2.0}) {
  semaforr::domain::RobotObservation result;
  result.pose = {{x, 0.0}, semaforr::domain::Angle::zero()};
  result.laser.angle_min = semaforr::domain::Angle(-0.4);
  result.laser.angle_increment = semaforr::domain::Angle(0.2);
  result.laser.minimum_range = semaforr::domain::Distance(0.1);
  result.laser.maximum_range = semaforr::domain::Distance(5.0);
  result.laser.ranges_m = std::move(ranges);
  return result;
}

}  // namespace

TEST(HighwayExplore, PassageSelectionAndStateTransitionsAreDeterministic) {
  semaforr::exploration::HighwayExplorer first;
  semaforr::exploration::HighwayExplorer second;
  const semaforr::domain::ActionSpace actions({0.25, 0.75}, {0.2, 0.5});
  const auto view = observation(0.0, {2.0, 2.0, 0.2, 2.0, 2.0});
  const auto one = first.decide(view, actions);
  const auto two = second.decide(view, actions);
  EXPECT_EQ(one.action, two.action);
  EXPECT_EQ(one.state, two.state);
  ASSERT_EQ(one.candidates.size(), 2U);
  EXPECT_EQ(one.action.type(), semaforr::domain::ActionType::TurnRight);
}

TEST(HighwayLearning, BuildsVersionedGraphIncrementally) {
  semaforr::spatial::HighwayLearner learner(0.5, 0.8);
  for (std::size_t sequence = 1U; sequence <= 3U; ++sequence) {
    learner.observe({sequence, observation(static_cast<double>(sequence)),
                     semaforr::domain::Action::pause(), std::nullopt, false,
                     false, true});
  }
  EXPECT_EQ(learner.snapshot().revision, 0U);
  learner.rebuild();
  const auto update = learner.snapshot();
  ASSERT_TRUE(update.usable());
  EXPECT_EQ(update.revision, 1U);
  const auto& model =
      std::get<semaforr::spatial::HighwayModel>(update.payload);
  EXPECT_EQ(model.nodes.size(), 3U);
  EXPECT_EQ(model.edges.size(), 2U);
  EXPECT_FALSE(model.grid_labels.empty());
  EXPECT_FALSE(model.touched_rows.empty());
  EXPECT_FALSE(model.touched_columns.empty());
}

TEST(HierarchicalPlans, HighwayPlanProducesTypedOperationalSteps) {
  semaforr::domain::SpatialModel spatial;
  spatial.highways.nodes = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  spatial.highways.edges = {{0U, 1U}, {1U, 2U}};
  spatial.highways.intersections = {{1U, 3U}};
  spatial.revision = 7U;
  semaforr::planning::HighwayPlan planner;
  const auto result = planner.plan(
      {{{-1.0, 0.0}, semaforr::domain::Angle::zero()}, {3.0, 0.0},
       &spatial, nullptr});
  ASSERT_TRUE(result.succeeded());
  ASSERT_TRUE(result.hierarchical);
  EXPECT_EQ(result.hierarchical->spatial_revision, 7U);
  EXPECT_TRUE(std::any_of(
      result.hierarchical->steps.begin(), result.hierarchical->steps.end(),
      [](const auto& step) {
        return step.kind ==
               semaforr::planning::PlanStepKind::CrossIntersection;
      }));
  const auto waypoints =
      semaforr::decision::Enforcer{}.operationalize(*result.hierarchical);
  EXPECT_EQ(waypoints, result.path);
}

TEST(PlanCache, ReusesExactRevisionAndInvalidatesOnModelRevision) {
  semaforr::planning::PlanningCoordinator coordinator;
  coordinator.registerPlanner(
      std::make_unique<semaforr::planning::DomainPlanner>(
          "distance", semaforr::planning::PlannerObjective::Distance));
  semaforr::domain::SpatialModel spatial;
  semaforr::planning::PlanningRequest request{
      {{0.0, 0.0}, semaforr::domain::Angle::zero()}, {2.0, 0.0}, &spatial,
      nullptr};
  ASSERT_TRUE(coordinator.selectPlan(request));
  ASSERT_TRUE(coordinator.selectPlan(request));
  EXPECT_EQ(coordinator.cacheHits(), 1U);
  ++spatial.revision;
  ASSERT_TRUE(coordinator.selectPlan(request));
  EXPECT_EQ(coordinator.cacheHits(), 1U);
}
