#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

#include <rcl/time.h>
#include <rclcpp/time.hpp>
#include <social_context_msgs/msg/social_observation.hpp>

#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/decision/social_navigation_advisor.hpp>
#include <semaforr/domain/social.hpp>
#include <semaforr/navigation/PathPlanner.hpp>
#include <semaforr/ros/social_observation_buffer.hpp>

namespace {

using semaforr::domain::Action;
using semaforr::domain::ActionType;
using semaforr::domain::CrowdObservation;
using semaforr::domain::PedestrianObservation;
using semaforr::domain::PredictedPosition;
using semaforr::domain::SocialTimestamp;

constexpr auto observed_at = std::chrono::seconds(10);

PedestrianObservation pedestrian(
  std::string id,
  double x,
  double y,
  double velocity_x,
  double velocity_y,
  std::vector<PredictedPosition> predictions = {})
{
  return {
    std::move(id),
    {x, y},
    {velocity_x, velocity_y},
    std::move(predictions),
    1.0,
    {0.04, 0.0, 0.0, 0.04}};
}

CrowdObservation crowd(PedestrianObservation person)
{
  CrowdObservation observation;
  observation.frame_id = "map";
  observation.observed_at =
    std::chrono::duration_cast<SocialTimestamp>(observed_at);
  observation.data_age = std::chrono::milliseconds(100);
  observation.pedestrians.push_back(std::move(person));
  observation.validate();
  return observation;
}

semaforr::decision::SocialNavigationAdvisor advisor(double weight = 1.0)
{
  return semaforr::decision::SocialNavigationAdvisor({
    {0.2, 1.0},
    {0.5},
    std::chrono::milliseconds(750),
    0.25,
    2.0,
    1.2,
    0.65,
    weight});
}

semaforr::domain::WorldModel worldWith(CrowdObservation observation)
{
  semaforr::domain::WorldModel world;
  world.robot.pose = {{0.0, 0.0}, semaforr::domain::Angle::zero()};
  world.crowd.update(std::move(observation));
  return world;
}

double scoreFor(
  const semaforr::decision::AdvisorEvaluation& evaluation,
  const Action& action)
{
  const auto found = std::find_if(
    evaluation.scores.begin(),
    evaluation.scores.end(),
    [&](const auto& score) { return score.action == action; });
  EXPECT_NE(found, evaluation.scores.end());
  return found == evaluation.scores.end() ? 0.0 : found->raw_score;
}

class ForwardGoalAdvisor final : public semaforr::decision::Advisor {
public:
  std::string_view name() const noexcept override { return "forward_goal"; }

  semaforr::decision::AdvisorEvaluation evaluate(
    const semaforr::decision::DecisionContext&,
    std::span<const Action> candidates) const override
  {
    semaforr::decision::AdvisorEvaluation result;
    result.participated = true;
    for (const auto& action : candidates) {
      result.scores.push_back({
        action,
        action.type() == ActionType::Forward ? 4.0 : 0.0});
    }
    return result;
  }
};

}  // namespace

TEST(SocialNavigation, InterpersonalDistancePenalizesCloseApproach)
{
  auto world = worldWith(crowd(
    pedestrian("person", 0.8, 0.0, 0.0, 0.0)));
  ASSERT_TRUE(world.crowd.current());
  ASSERT_TRUE(world.crowd.current()->usable(
    std::chrono::milliseconds(750), 0.25));
  const std::vector<Action> actions{
    Action::pause(), Action(ActionType::Forward, 2U)};
  const auto result = advisor().evaluate(
    {world}, actions);
  ASSERT_TRUE(result.participated);
  EXPECT_LT(scoreFor(result, actions[1]), scoreFor(result, actions[0]));
}

TEST(SocialNavigation, CrossingPredictionPenalizesTemporalIntersection)
{
  auto world = worldWith(crowd(pedestrian(
    "crossing",
    0.5,
    -1.0,
    0.0,
    1.0,
    {
      {{0.5, 0.0}, observed_at + std::chrono::seconds(1)},
      {{0.5, 1.0}, observed_at + std::chrono::seconds(2)},
    })));
  const std::vector<Action> actions{
    Action::pause(), Action(ActionType::Forward, 2U)};
  const auto result = advisor().evaluate({world}, actions);
  EXPECT_LT(scoreFor(result, actions[1]), scoreFor(result, actions[0]));
}

TEST(SocialNavigation, FollowingPenalizesClosingOnSlowerPedestrian)
{
  auto world = worldWith(crowd(
    pedestrian("following", 0.8, 0.0, 0.1, 0.0)));
  const std::vector<Action> actions{
    Action(ActionType::Forward, 1U),
    Action(ActionType::Forward, 2U)};
  const auto result = advisor().evaluate({world}, actions);
  EXPECT_LT(scoreFor(result, actions[1]), scoreFor(result, actions[0]));
}

TEST(SocialNavigation, OpposingFlowPenalizesForwardMotion)
{
  auto world = worldWith(crowd(
    pedestrian("opposing", 1.2, 0.0, -0.5, 0.0)));
  const std::vector<Action> actions{
    Action::pause(), Action(ActionType::Forward, 2U)};
  const auto result = advisor().evaluate({world}, actions);
  EXPECT_LT(scoreFor(result, actions[1]), scoreFor(result, actions[0]));
}

TEST(SocialNavigation, StalePredictionDisablesAdvisor)
{
  auto stale = crowd(
    pedestrian("stale", 1.0, 0.0, -0.5, 0.0));
  stale.data_age = std::chrono::seconds(2);
  auto world = worldWith(std::move(stale));
  const std::vector<Action> actions{
    Action::pause(), Action(ActionType::Forward, 2U)};
  const auto result = advisor().evaluate({world}, actions);
  EXPECT_FALSE(result.participated);
  EXPECT_TRUE(result.scores.empty());
}

TEST(SocialNavigation, RecordedTrajectoryChangesDeterministicAction)
{
  const std::vector<Action> actions{
    Action::pause(), Action(ActionType::Forward, 2U)};
  semaforr::domain::WorldModel empty_world;
  empty_world.robot.pose = {
    {0.0, 0.0}, semaforr::domain::Angle::zero()};

  semaforr::decision::ArbitrationConfiguration arbitration;
  arbitration.random_seed = 7U;
  arbitration.tie_tolerance = 1.0e-9;
  arbitration.unscored_policy =
    semaforr::decision::UnscoredActionPolicy::Zero;
  arbitration.unscored_baseline = 0.0;
  arbitration.fallback = Action::pause();
  semaforr::decision::DecisionCoordinator without_social(arbitration);
  without_social.addAdvisor(std::make_unique<ForwardGoalAdvisor>());
  const auto baseline = without_social.decide({empty_world}, actions);
  EXPECT_EQ(baseline.action, actions[1]);

  auto social_world = worldWith(crowd(
    pedestrian("opposing", 1.0, 0.0, -0.6, 0.0)));
  semaforr::decision::DecisionCoordinator with_social(arbitration);
  with_social.addAdvisor(std::make_unique<ForwardGoalAdvisor>());
  with_social.addAdvisor(
    std::make_unique<semaforr::decision::SocialNavigationAdvisor>(
      advisor(5.0)));
  const auto social = with_social.decide({social_world}, actions);
  EXPECT_EQ(social.action, Action::pause());
  EXPECT_NE(social.action, baseline.action);
}

TEST(SocialPlanning, DensityAndPredictionCostsConsumeCrowdState)
{
  auto observation = crowd(pedestrian(
    "planned",
    1.0,
    0.0,
    0.5,
    0.0,
    {{{2.0, 0.0}, observed_at + std::chrono::seconds(2)}}));
  semaforr::domain::CrowdState state;
  state.update(std::move(observation));
  PathPlanner planner(Node{}, Node{}, "risk");
  planner.setCrowdState(state);

  EXPECT_GT(planner.cellCost(100, 0, 0), planner.cellCost(500, 500, 0));
  EXPECT_GT(planner.riskCost(200, 0, 0), planner.cellCost(200, 0, 0));

  planner.setCrowdState({});
  EXPECT_DOUBLE_EQ(planner.cellCost(100, 0, 0), 0.0);
  EXPECT_DOUBLE_EQ(planner.riskCost(200, 0, 0), 0.0);
}

TEST(SocialDomain, RejectsDuplicateIdentityAndInvalidCovariance)
{
  auto duplicate = crowd(
    pedestrian("same", 0.0, 0.0, 0.0, 0.0));
  duplicate.pedestrians.push_back(
    pedestrian("same", 1.0, 0.0, 0.0, 0.0));
  EXPECT_THROW(duplicate.validate(), std::invalid_argument);

  auto invalid = pedestrian("covariance", 0.0, 0.0, 0.0, 0.0);
  invalid.position_covariance = {1.0, 2.0, 0.0, 1.0};
  EXPECT_THROW(invalid.validate(observed_at), std::invalid_argument);
}

TEST(SocialObservationBuffer, ValidatesAgeFrameAndConfidence)
{
  semaforr::ros::SocialObservationBuffer buffer({
    "map", 0.75, 0.5});
  social_context_msgs::msg::SocialObservation message;
  message.header.frame_id = "map";
  message.header.stamp.sec = 10;
  message.pedestrians.resize(1);
  message.pedestrians[0].id = "person";
  message.pedestrians[0].confidence = 0.9;
  message.pedestrians[0].position_covariance =
    {0.04, 0.0, 0.0, 0.04};

  const rclcpp::Time received(
    10'100'000'000LL, RCL_ROS_TIME);
  ASSERT_TRUE(buffer.accept(message, received));
  EXPECT_TRUE(buffer.snapshot(received));
  EXPECT_EQ(
    buffer.status(rclcpp::Time(11'000'000'000LL, RCL_ROS_TIME)),
    semaforr::ros::SocialObservationStatus::Stale);

  message.header.frame_id = "odom";
  EXPECT_FALSE(buffer.accept(message, received));
  EXPECT_EQ(
    buffer.status(received),
    semaforr::ros::SocialObservationStatus::FrameMismatch);

  message.header.frame_id = "map";
  message.pedestrians[0].predicted_positions.resize(1);
  EXPECT_FALSE(buffer.accept(message, received));
  EXPECT_EQ(
    buffer.status(received),
    semaforr::ros::SocialObservationStatus::Invalid);
}
