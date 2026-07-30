#include <gtest/gtest.h>
#include <memory>
#include <semaforr/decision/enforcer.hpp>
#include <semaforr/exploration/exploration_coordinator.hpp>
#include <semaforr/exploration/highway_explorer.hpp>
#include <semaforr/exploration/high_level_explorer.hpp>
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

TEST(HighLevelExplore, OwnsDeterministicCandidateLifecycleAndSparsePassageGrid) {
  semaforr::exploration::HighLevelExplorationConfiguration configuration;
  configuration.candidate_completion_distance =
      semaforr::domain::Distance(0.1);
  semaforr::exploration::HighLevelExplorer explorer(configuration);
  const semaforr::domain::ActionSpace actions({0.1, 0.2}, {0.25, 0.5});
  auto view = observation(0.0);

  EXPECT_EQ(explorer.update({view, actions, {}}).state,
            semaforr::exploration::HleState::DiscoverCandidate);
  const auto selected = explorer.update({view, actions, {}});
  ASSERT_EQ(selected.state,
            semaforr::exploration::HleState::ReturnToCandidateStart);
  ASSERT_TRUE(selected.candidate_id);
  ASSERT_FALSE(selected.discovered.empty());
  EXPECT_EQ(*selected.candidate_id, selected.discovered.front().id);
  EXPECT_EQ(*selected.candidate_id, 1U);

  const auto pursuing = explorer.update({view, actions, {}});
  EXPECT_EQ(pursuing.state,
            semaforr::exploration::HleState::PursueCandidate);
  EXPECT_EQ(pursuing.event,
            semaforr::exploration::CandidateLifecycleEvent::PursuitStarted);
  view.pose.position.x_m = 0.2;
  const auto completed = explorer.update({view, actions, {}});
  EXPECT_EQ(completed.event,
            semaforr::exploration::CandidateLifecycleEvent::Completed);
  EXPECT_GT(completed.passage_grid_revision, 0U);
  EXPECT_FALSE(explorer.passageGrid().cells.empty());
}

TEST(HighLevelExplore, ReportsBudgetCompletionAndFinalizesExactlyOnce) {
  semaforr::exploration::HighLevelExplorationConfiguration configuration;
  configuration.decision_budget = 1U;
  semaforr::exploration::HighLevelExplorer explorer(configuration);
  const semaforr::domain::ActionSpace actions({0.1}, {0.25});
  const auto view = observation(0.0);
  static_cast<void>(explorer.update({view, actions, {}}));
  const auto finalizing = explorer.update({view, actions, {}});
  EXPECT_EQ(finalizing.state,
            semaforr::exploration::HleState::FinalizeModel);
  EXPECT_EQ(finalizing.completion_reason,
            semaforr::exploration::ExplorationCompletionReason::
                DecisionBudgetExceeded);
  EXPECT_EQ(explorer.update({view, actions, {}}).state,
            semaforr::exploration::HleState::Complete);

  semaforr::exploration::ExplorationCoordinator coordinator(configuration);
  std::size_t finalizations = 0U;
  coordinator.setModelFinalizer([&finalizations] { ++finalizations; });
  coordinator.finish();
  coordinator.finish();
  EXPECT_EQ(finalizations, 1U);
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
