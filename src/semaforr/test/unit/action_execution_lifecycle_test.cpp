#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <semaforr/decision/navigation_engine.hpp>
#include <semaforr/spatial/learners/conveyor_learner.hpp>
#include <semaforr/spatial/learners/trail_learner.hpp>

namespace {

semaforr::domain::RobotObservation observation(double x_m = 0.0) {
  semaforr::domain::RobotObservation result;
  result.pose = {{x_m, 0.0}, semaforr::domain::Angle::zero()};
  result.laser.angle_min = semaforr::domain::Angle(-0.1);
  result.laser.angle_increment = semaforr::domain::Angle(0.1);
  result.laser.minimum_range = semaforr::domain::Distance(0.05);
  result.laser.maximum_range = semaforr::domain::Distance(5.0);
  result.laser.ranges_m = {2.0, 2.0, 2.0};
  result.observed_at = std::chrono::steady_clock::now();
  return result;
}

semaforr::domain::WorldModel configuredWorld() {
  semaforr::domain::WorldModel result;
  result.mission = semaforr::domain::Mission({{7U, {5.0, 0.0}}}, 20U);
  return result;
}

class LifecycleFixture : public ::testing::Test {
 protected:
  LifecycleFixture()
      : action_space({0.2}, {0.5}),
        decisions({1.0e-9,
                   semaforr::decision::UnscoredActionPolicy::Exclude, 0.0,
                   semaforr::domain::Action(
                       semaforr::domain::ActionType::Forward, 1U),
                   0U}),
        mission(world.mission),
        learning(100U),
        engine(world, action_space, decisions, mission, planning, learning) {
    learning.addLearner(std::make_unique<semaforr::spatial::TrailLearner>());
  }

  semaforr::decision::DecisionResult select() {
    engine.observe(observation());
    return engine.decide();
  }

  semaforr::domain::ActionExecutionResult resultFor(
      const semaforr::decision::DecisionResult& decision,
      semaforr::domain::ExecutionCompletionStatus status,
      double final_x = 0.0) {
    semaforr::domain::ActionExecutionResult result;
    result.decision_id = decision.decision_id;
    result.action_id = decision.action_id;
    result.task_id = semaforr::domain::TaskId{7U};
    result.finished_at = std::chrono::steady_clock::now();
    result.status = status;
    result.start_pose = observation().pose;
    result.final_pose = observation(final_x).pose;
    result.distance_achieved_m = final_x;
    result.timed_out =
        status == semaforr::domain::ExecutionCompletionStatus::TimedOut;
    result.safety_interruption =
        status ==
        semaforr::domain::ExecutionCompletionStatus::SafetyInterrupted;
    result.controller_failure =
        status == semaforr::domain::ExecutionCompletionStatus::ControllerFailure;
    return result;
  }

  void start(const semaforr::decision::DecisionResult& decision) {
    ASSERT_EQ(engine.onActionStarted(
                  {decision.decision_id, decision.action_id,
                   std::chrono::steady_clock::now(), observation().pose}),
              semaforr::domain::FeedbackDisposition::Accepted);
  }

  semaforr::domain::WorldModel world = configuredWorld();
  semaforr::domain::ActionSpace action_space;
  semaforr::decision::DecisionCoordinator decisions;
  semaforr::decision::MissionManager mission;
  semaforr::planning::PlanningCoordinator planning;
  semaforr::spatial::SpatialLearningCoordinator learning;
  semaforr::decision::NavigationEngine engine;
};

TEST_F(LifecycleFixture, SelectionIsNotExecutionAndStableIdsCorrelateFeedback) {
  const auto decision = select();
  ASSERT_NE(decision.decision_id, 0U);
  ASSERT_NE(decision.action_id, 0U);
  ASSERT_EQ(world.decision_history.entries().size(), 1U);
  EXPECT_TRUE(world.command_history.entries().empty());
  EXPECT_TRUE(world.execution_history.entries().empty());
  EXPECT_TRUE(world.navigation_history.entries().empty());
  EXPECT_THROW(engine.decide(), std::logic_error);

  start(decision);
  EXPECT_EQ(world.command_history.entries().size(), 1U);
  auto completed = resultFor(
      decision, semaforr::domain::ExecutionCompletionStatus::Succeeded, 0.2);
  EXPECT_EQ(engine.onActionCompleted(completed),
            semaforr::domain::FeedbackDisposition::Accepted);
  ASSERT_EQ(world.execution_history.entries().size(), 1U);
  EXPECT_EQ(world.execution_history.entries().front().decision_id,
            decision.decision_id);
  EXPECT_EQ(world.execution_history.entries().front().action_id,
            decision.action_id);
  EXPECT_EQ(world.navigation_history.entries().size(), 1U);
  EXPECT_EQ(world.completed_path_history.entries().size(), 1U);
}

TEST_F(LifecycleFixture, DuplicateUnknownAndStaleFeedbackAreRejected) {
  const auto decision = select();
  start(decision);
  auto completed = resultFor(
      decision, semaforr::domain::ExecutionCompletionStatus::Succeeded, 0.2);
  EXPECT_EQ(engine.onActionCompleted(completed),
            semaforr::domain::FeedbackDisposition::Accepted);
  EXPECT_EQ(engine.onActionCompleted(completed),
            semaforr::domain::FeedbackDisposition::Duplicate);
  completed.action_id += 99U;
  EXPECT_EQ(engine.onActionFailed(completed),
            semaforr::domain::FeedbackDisposition::UnknownAction);

  const auto next = select();
  EXPECT_GT(next.action_id, decision.action_id);
  EXPECT_EQ(engine.onActionStarted(
                {next.decision_id + 1U, next.action_id,
                 std::chrono::steady_clock::now(), observation().pose}),
            semaforr::domain::FeedbackDisposition::StaleDecision);
}

class TerminalStatusTest
    : public LifecycleFixture,
      public ::testing::WithParamInterface<
          semaforr::domain::ExecutionCompletionStatus> {};

TEST_P(TerminalStatusTest, FailuresNeverBecomeSuccessfulTraversal) {
  const auto decision = select();
  start(decision);
  auto failed = resultFor(decision, GetParam(), 0.1);
  EXPECT_EQ(engine.onActionFailed(failed),
            semaforr::domain::FeedbackDisposition::Accepted);
  ASSERT_EQ(world.execution_history.entries().size(), 1U);
  EXPECT_EQ(world.execution_history.entries().front().status, GetParam());
  EXPECT_TRUE(world.completed_path_history.entries().empty());
  const auto trail = learning.snapshot(
      semaforr::spatial::SpatialRepresentation::Trails);
  ASSERT_TRUE(trail);
  EXPECT_EQ(trail->status, semaforr::spatial::ModelStatus::Empty);
}

INSTANTIATE_TEST_SUITE_P(
    UnsuccessfulOutcomes, TerminalStatusTest,
    ::testing::Values(
        semaforr::domain::ExecutionCompletionStatus::PartialMovement,
        semaforr::domain::ExecutionCompletionStatus::NoMovement,
        semaforr::domain::ExecutionCompletionStatus::TimedOut,
        semaforr::domain::ExecutionCompletionStatus::Cancelled,
        semaforr::domain::ExecutionCompletionStatus::SafetyInterrupted,
        semaforr::domain::ExecutionCompletionStatus::ControllerRejected,
        semaforr::domain::ExecutionCompletionStatus::ControllerFailure,
        semaforr::domain::ExecutionCompletionStatus::GoalPreempted,
        semaforr::domain::ExecutionCompletionStatus::NavigationModeTransition,
        semaforr::domain::ExecutionCompletionStatus::SensorLost,
        semaforr::domain::ExecutionCompletionStatus::Shutdown,
        semaforr::domain::ExecutionCompletionStatus::ClockReset,
        semaforr::domain::ExecutionCompletionStatus::OdometryReset));

TEST_F(LifecycleFixture, ControllerRejectionCanTerminateBeforeStart) {
  const auto decision = select();
  auto rejected = resultFor(
      decision,
      semaforr::domain::ExecutionCompletionStatus::ControllerRejected);
  EXPECT_EQ(engine.onActionFailed(rejected),
            semaforr::domain::FeedbackDisposition::Accepted);
  EXPECT_TRUE(world.command_history.entries().empty());
  ASSERT_EQ(world.execution_history.entries().size(), 1U);
  EXPECT_EQ(world.execution_history.entries().front().status,
            semaforr::domain::ExecutionCompletionStatus::ControllerRejected);
}

TEST_F(LifecycleFixture, TaskMismatchAndPreStartSuccessAreRejected) {
  const auto decision = select();
  auto completed = resultFor(
      decision, semaforr::domain::ExecutionCompletionStatus::Succeeded, 0.2);
  EXPECT_EQ(engine.onActionCompleted(completed),
            semaforr::domain::FeedbackDisposition::NotStarted);
  start(decision);
  completed.task_id = semaforr::domain::TaskId{999U};
  EXPECT_EQ(engine.onActionCompleted(completed),
            semaforr::domain::FeedbackDisposition::TaskMismatch);
}

TEST_F(LifecycleFixture, ControllerRestartTerminatesThePendingAction) {
  const auto decision = select();
  start(decision);
  EXPECT_EQ(engine.onControllerRestart(std::chrono::steady_clock::now(),
                                      observation(0.05).pose),
            semaforr::domain::FeedbackDisposition::Accepted);
  ASSERT_EQ(world.execution_history.entries().size(), 1U);
  EXPECT_EQ(world.execution_history.entries().front().status,
            semaforr::domain::ExecutionCompletionStatus::ControllerFailure);
}

TEST(CompletedActionLearning, RotationDoesNotCreateAConveyor) {
  semaforr::spatial::SpatialLearningCoordinator learning(100U);
  learning.addLearner(
      std::make_unique<semaforr::spatial::ConveyorLearner>());
  semaforr::spatial::NavigationEpisode episode;
  episode.sequence = 1U;
  episode.observation = observation();
  episode.selected_action = semaforr::domain::Action(
      semaforr::domain::ActionType::TurnLeft, 1U);
  semaforr::domain::ActionExecutionResult result;
  result.status = semaforr::domain::ExecutionCompletionStatus::Succeeded;
  result.start_pose = observation().pose;
  result.final_pose = {{0.0, 0.0}, semaforr::domain::Angle(0.5)};
  result.rotation_achieved_rad = 0.5;
  episode.execution_result = result;
  learning.observeActionTerminal(episode);
  const auto conveyor = learning.snapshot(
      semaforr::spatial::SpatialRepresentation::Conveyors);
  ASSERT_TRUE(conveyor);
  EXPECT_EQ(conveyor->status, semaforr::spatial::ModelStatus::Incomplete);
}

}  // namespace
