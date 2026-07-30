#include <gtest/gtest.h>

#include <limits>
#include <numbers>
#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/decision/navigation_advisor.hpp>
#include <semaforr/decision/obstacle_veto_rule.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <stdexcept>
#include <vector>

namespace {

using semaforr::domain::Action;
using semaforr::domain::ActionSpace;
using semaforr::domain::ActionType;
using semaforr::domain::Angle;
using semaforr::domain::Distance;
using semaforr::domain::LaserObservation;
using semaforr::domain::Pose2D;
using semaforr::domain::WorldModel;

LaserObservation scan(std::vector<double> ranges) {
  return {Angle(-std::numbers::pi / 4.0), Angle(std::numbers::pi / 4.0),
          Distance(0.05), Distance(10.0), std::move(ranges)};
}

TEST(MotionModel, ComputesExpectedPoseWithoutChangingFramesOrUnits) {
  const ActionSpace actions(std::vector<double>{0.25, 1.0},
                            std::vector<double>{std::numbers::pi / 4.0});
  const Pose2D start{{2.0, 3.0}, Angle(std::numbers::pi / 2.0)};

  const auto forward = semaforr::domain::expectedPoseAfterAction(
      start, Action(ActionType::Forward, 2U), actions);
  EXPECT_NEAR(forward.position.x_m, 2.0, 1.0e-12);
  EXPECT_NEAR(forward.position.y_m, 4.0, 1.0e-12);
  EXPECT_NEAR(forward.heading.radians(), std::numbers::pi / 2.0, 1.0e-12);

  const auto right = semaforr::domain::expectedPoseAfterAction(
      start, Action(ActionType::TurnRight, 1U), actions);
  EXPECT_EQ(right.position, start.position);
  EXPECT_NEAR(right.heading.radians(), std::numbers::pi / 4.0, 1.0e-12);

  const auto left = semaforr::domain::expectedPoseAfterAction(
      start, Action(ActionType::TurnLeft, 1U), actions);
  EXPECT_EQ(left.position, start.position);
  EXPECT_NEAR(left.heading.radians(), 3.0 * std::numbers::pi / 4.0, 1.0e-12);

  EXPECT_EQ(semaforr::domain::expectedPoseAfterAction(start, Action::pause(),
                                                      actions),
            start);
  EXPECT_THROW(semaforr::domain::expectedPoseAfterAction(
                   start, Action(ActionType::Forward, 3U), actions),
               std::out_of_range);
}

TEST(MotionModel, TransformsLaserEndpointsIntoTheMapFrame) {
  const Pose2D pose{{1.0, 2.0}, Angle(std::numbers::pi / 2.0)};
  const auto endpoints =
      semaforr::domain::laserEndpoints(pose, scan({2.0, 1.0, 2.0}));

  ASSERT_EQ(endpoints.size(), 3U);
  EXPECT_NEAR(endpoints[0].x_m, 1.0 + std::sqrt(2.0), 1.0e-12);
  EXPECT_NEAR(endpoints[0].y_m, 2.0 + std::sqrt(2.0), 1.0e-12);
  EXPECT_NEAR(endpoints[1].x_m, 1.0, 1.0e-12);
  EXPECT_NEAR(endpoints[1].y_m, 3.0, 1.0e-12);
  EXPECT_NEAR(endpoints[2].x_m, 1.0 - std::sqrt(2.0), 1.0e-12);
  EXPECT_NEAR(endpoints[2].y_m, 2.0 + std::sqrt(2.0), 1.0e-12);

  auto no_return_scan = scan({std::numeric_limits<double>::infinity()});
  no_return_scan.angle_min = Angle::zero();
  const auto no_return = semaforr::domain::laserEndpoints(pose, no_return_scan);
  ASSERT_EQ(no_return.size(), 1U);
  EXPECT_NEAR(no_return[0].x_m, 1.0, 1.0e-12);
  EXPECT_NEAR(no_return[0].y_m, 12.0, 1.0e-12);
}

TEST(MotionModel, GoalCompletionUsesAnExplicitMetricTolerance) {
  const Pose2D pose{{1.0, 1.0}, Angle::zero()};
  EXPECT_TRUE(
      semaforr::domain::goalReached(pose, {1.03, 1.04}, Distance(0.05)));
  EXPECT_FALSE(
      semaforr::domain::goalReached(pose, {1.03, 1.04}, Distance(0.049)));
}

TEST(ObstacleVetoRule, VetoesOnlyForwardActionsThatReachTheObstacle) {
  WorldModel world;
  world.robot.laser = scan({5.0, 0.7, 5.0});
  semaforr::decision::DecisionCoordinator coordinator;
  coordinator.addVetoRule(
      std::make_unique<semaforr::decision::ObstacleVetoRule>(
          std::vector<double>{0.2, 0.5, 1.0}, 0.2, 0.05));

  const std::vector<Action> candidates{Action(ActionType::Forward, 1U),
                                       Action(ActionType::Forward, 2U),
                                       Action(ActionType::Forward, 3U)};
  const auto result = coordinator.decide(
      semaforr::decision::DecisionContext{world}, candidates);

  ASSERT_EQ(result.vetoes.size(), 2U);
  EXPECT_EQ(result.vetoes[0].action, Action(ActionType::Forward, 2U));
  EXPECT_EQ(result.vetoes[1].action, Action(ActionType::Forward, 3U));
  EXPECT_EQ(result.action, Action::pause());
}

TEST(SpatialRelationships, RegionsAndDoorsUseTheSameMetricGeometry) {
  const semaforr::domain::Circle first{{0.0, 0.0}, Distance(2.0)};
  const semaforr::domain::Circle second{{3.0, 0.0}, Distance(2.0)};
  const semaforr::domain::Segment2D doorway{{1.5, -0.5}, {1.5, 0.5}};

  EXPECT_LT(semaforr::domain::distance(first.center, second.center).meters(),
            first.radius.meters() + second.radius.meters());
  EXPECT_TRUE(first.contains(doorway.start));
  EXPECT_TRUE(second.contains(doorway.end));
}

TEST(NavigationAdvisor, ScoresGoalClearanceAndExplorationObjectives) {
  const semaforr::domain::ActionSpace action_space{{1.0}, {0.5}};
  WorldModel world;
  world.robot.pose = {{0.0, 0.0}, Angle::zero()};
  world.mission = semaforr::domain::Mission({{1U, {3.0, 0.0}}}, 10U);
  ASSERT_TRUE(world.mission.activate_next());
  world.navigation_history.record(
      {world.robot.pose, LaserObservation{}, Action::pause()});
  const std::vector<Action> actions{Action::pause(),
                                    Action(ActionType::Forward, 1U),
                                    Action(ActionType::TurnLeft, 1U)};

  const semaforr::decision::NavigationAdvisor goal(
      {"goal_progress",
       semaforr::decision::NavigationAdvisorObjective::GoalProgress,
       semaforr::decision::ActionSelection::All, action_space, 2.0});
  const auto goal_scores =
      goal.evaluate(semaforr::decision::DecisionContext{world}, actions);
  ASSERT_TRUE(goal_scores.participated);
  EXPECT_EQ(goal_scores.weight, 2.0);
  EXPECT_GT(goal_scores.scores[1].raw_score, goal_scores.scores[0].raw_score);

  const semaforr::decision::NavigationAdvisor exploration(
      {"exploration",
       semaforr::decision::NavigationAdvisorObjective::Exploration,
       semaforr::decision::ActionSelection::Linear, action_space, 1.0});
  const auto exploration_scores =
      exploration.evaluate(semaforr::decision::DecisionContext{world}, actions);
  ASSERT_EQ(exploration_scores.scores.size(), 2U);
  EXPECT_GT(exploration_scores.scores[1].raw_score,
            exploration_scores.scores[0].raw_score);

  EXPECT_THROW(
      semaforr::decision::NavigationAdvisor(
          {"", semaforr::decision::NavigationAdvisorObjective::Clearance,
           semaforr::decision::ActionSelection::Rotation, action_space, 1.0}),
      std::invalid_argument);
}

TEST(NavigationAdvisor, FiltersActionsAndHandlesMissingOrInfiniteSensorData) {
  const semaforr::domain::ActionSpace action_space{{1.0}, {0.5}};
  WorldModel world;
  world.robot.pose = {{0.0, 0.0}, Angle::zero()};
  const std::vector<Action> actions{
      Action::pause(), Action(ActionType::Forward, 1U),
      Action(ActionType::TurnLeft, 1U), Action(ActionType::TurnRight, 1U)};

  const semaforr::decision::NavigationAdvisor goal(
      {"goal_progress",
       semaforr::decision::NavigationAdvisorObjective::GoalProgress,
       semaforr::decision::ActionSelection::All, action_space, 1.0});
  const auto no_goal =
      goal.evaluate(semaforr::decision::DecisionContext{world}, actions);
  ASSERT_EQ(no_goal.scores.size(), actions.size());
  EXPECT_EQ(no_goal.scores.front().raw_score, 0.0);

  const semaforr::decision::NavigationAdvisor clearance(
      {"clearance", semaforr::decision::NavigationAdvisorObjective::Clearance,
       semaforr::decision::ActionSelection::Rotation, action_space, 1.0});
  const auto no_scan =
      clearance.evaluate(semaforr::decision::DecisionContext{world}, actions);
  ASSERT_EQ(no_scan.scores.size(), 3U);
  EXPECT_EQ(no_scan.scores.front().raw_score, 0.0);

  world.robot.laser = scan({0.5, std::numeric_limits<double>::infinity(), 2.0});
  const auto infinite_scan =
      clearance.evaluate(semaforr::decision::DecisionContext{world}, actions);
  ASSERT_EQ(infinite_scan.scores.size(), 3U);
  EXPECT_NEAR(infinite_scan.scores.front().raw_score, 0.1, 1.0e-12);

  const semaforr::decision::NavigationAdvisor linear_clearance(
      {"clearance_linear",
       semaforr::decision::NavigationAdvisorObjective::Clearance,
       semaforr::decision::ActionSelection::Linear, action_space, 1.0});
  const auto linear = linear_clearance.evaluate(
      semaforr::decision::DecisionContext{world}, actions);
  ASSERT_EQ(linear.scores.size(), 2U);
  EXPECT_EQ(linear.scores[1].raw_score, 1.0);

  EXPECT_THROW(semaforr::decision::NavigationAdvisor(
                   {"clearance",
                    semaforr::decision::NavigationAdvisorObjective::Clearance,
                    semaforr::decision::ActionSelection::All, action_space,
                    std::numeric_limits<double>::infinity()}),
               std::invalid_argument);
}

}  // namespace
