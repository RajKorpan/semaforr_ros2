/**
 * @file social_navigation_test.cpp
 * @brief Social navigation test responsibilities.
 *
 * @details This file exercises social navigation test behavior for automated
 * verification and regression testing. It centers on `ForwardGoalAdvisor`,
 * `InterpersonalDistancePenalizesCloseApproach`,
 * `CrossingPredictionPenalizesTemporalIntersection`,
 * `FollowingPenalizesClosingOnSlowerPedestrian`,
 * `OpposingFlowPenalizesForwardMotion`, `StalePredictionDisablesAdvisor`,
 * `PredictionStartsAtObservationAge`,
 * `RecordedTrajectoryChangesDeterministicAction`. Its package-relative
 * location is `test/unit/social_navigation_test.cpp`.
 */
#include <gtest/gtest.h>
#include <rcl/time.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <hunav_msgs/msg/agents.hpp>
#include <rclcpp/time.hpp>
#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/decision/advisors/social/social_navigation_advisor.hpp>
#include <semaforr/domain/crowd_model.hpp>
#include <semaforr/domain/social.hpp>
#include <semaforr/ros/social_observation_buffer.hpp>
#include <social_context_msgs/msg/formation_group_array.hpp>
#include <social_context_msgs/msg/tracked_person_array.hpp>
#include <span>
#include <string_view>
#include <vector>

namespace {

using semaforr::domain::Action;
using semaforr::domain::ActionType;
using semaforr::domain::CrowdObservation;
using semaforr::domain::PedestrianObservation;
using semaforr::domain::PredictedPosition;
using semaforr::domain::SocialTimestamp;

constexpr auto observed_at = std::chrono::seconds(10);

/**
 * @brief Performs the pedestrian operation for this subsystem.
 *
 * Arguments:
 * - @p id: Supplies id input to the operation.
 * - @p x: Supplies x input to the operation.
 * - @p y: Supplies y input to the operation.
 * - @p velocity_x: Supplies velocity x input to the operation.
 * - @p velocity_y: Supplies velocity y input to the operation.
 * - @p predictions: Supplies predictions input to the operation.
 *
 * Returns:
 * - `PedestrianObservation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PedestrianObservation pedestrian(
    std::string id, double x, double y, double velocity_x, double velocity_y,
    std::vector<PredictedPosition> predictions = {}) {
  return {std::move(id),          {x, y}, {velocity_x, velocity_y},
          std::move(predictions), 1.0,    {0.04, 0.0, 0.0, 0.04},
          "none",                std::nullopt};
}

/**
 * @brief Performs the crowd operation for this subsystem.
 *
 * Arguments:
 * - @p person: Supplies person input to the operation.
 *
 * Returns:
 * - `CrowdObservation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
CrowdObservation crowd(PedestrianObservation person) {
  CrowdObservation observation;
  observation.frame_id = "map";
  observation.observed_at =
      std::chrono::duration_cast<SocialTimestamp>(observed_at);
  observation.data_age = std::chrono::milliseconds(100);
  observation.pedestrians.push_back(std::move(person));
  observation.validate();
  return observation;
}

/**
 * @brief Performs the advisor operation for this subsystem.
 *
 * Arguments:
 * - @p weight: Supplies weight input to the operation.
 *
 * Returns:
 * - `semaforr::decision::SocialNavigationAdvisor` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::decision::SocialNavigationAdvisor advisor(double weight = 1.0) {
  return semaforr::decision::SocialNavigationAdvisor(
      {{0.2, 1.0},
       {0.5},
       std::chrono::milliseconds(750),
       0.25,
       2.0,
       1.2,
       0.65,
       weight});
}

/**
 * @brief Performs the world with operation for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 *
 * Returns:
 * - `semaforr::domain::WorldModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::WorldModel worldWith(CrowdObservation observation) {
  semaforr::domain::WorldModel world;
  world.robot.pose = {{0.0, 0.0}, semaforr::domain::Angle::zero()};
  world.crowd.update(std::move(observation));
  return world;
}

/**
 * @brief Performs the score for operation for this subsystem.
 *
 * Arguments:
 * - @p evaluation: Supplies evaluation input to the operation.
 * - @p action: Supplies action input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double scoreFor(const semaforr::decision::AdvisorEvaluation& evaluation,
                const Action& action) {
  const auto found =
      std::find_if(evaluation.scores.begin(), evaluation.scores.end(),
                   [&](const auto& score) { return score.action == action; });
  EXPECT_NE(found, evaluation.scores.end());
  return found == evaluation.scores.end() ? 0.0 : found->raw_score;
}

/**
 * @brief Encapsulates forward goal advisor state and behavior for this
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
class ForwardGoalAdvisor final : public semaforr::decision::Advisor {
 public:
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
  std::string_view name() const noexcept override { return "forward_goal"; }

  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - `semaforr::decision::AdvisorEvaluation` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  semaforr::decision::AdvisorEvaluation evaluate(
      const semaforr::decision::DecisionContext&,
      std::span<const Action> candidates) const override {
    semaforr::decision::AdvisorEvaluation result;
    result.participated = true;
    for (const auto& action : candidates) {
      result.scores.push_back(
          {action, action.type() == ActionType::Forward ? 4.0 : 0.0});
    }
    return result;
  }
};

}  // namespace

TEST(SocialNavigation, InterpersonalDistancePenalizesCloseApproach) {
  auto world = worldWith(crowd(pedestrian("person", 0.8, 0.0, 0.0, 0.0)));
  ASSERT_TRUE(world.crowd.current());
  ASSERT_TRUE(
      world.crowd.current()->usable(std::chrono::milliseconds(750), 0.25));
  const std::vector<Action> actions{Action::pause(),
                                    Action(ActionType::Forward, 2U)};
  const auto result = advisor().evaluate({world}, actions);
  ASSERT_TRUE(result.participated);
  EXPECT_LT(scoreFor(result, actions[1]), scoreFor(result, actions[0]));
}

TEST(SocialNavigation, CrossingPredictionPenalizesTemporalIntersection) {
  auto world = worldWith(
      crowd(pedestrian("crossing", 0.5, -1.0, 0.0, 1.0,
                       {
                           {{0.5, 0.0}, observed_at + std::chrono::seconds(1)},
                           {{0.5, 1.0}, observed_at + std::chrono::seconds(2)},
                       })));
  const std::vector<Action> actions{Action::pause(),
                                    Action(ActionType::Forward, 2U)};
  const auto result = advisor().evaluate({world}, actions);
  EXPECT_LT(scoreFor(result, actions[1]), scoreFor(result, actions[0]));
}

TEST(SocialNavigation, FollowingPenalizesClosingOnSlowerPedestrian) {
  auto world = worldWith(crowd(pedestrian("following", 0.8, 0.0, 0.1, 0.0)));
  const std::vector<Action> actions{Action(ActionType::Forward, 1U),
                                    Action(ActionType::Forward, 2U)};
  const auto result = advisor().evaluate({world}, actions);
  EXPECT_LT(scoreFor(result, actions[1]), scoreFor(result, actions[0]));
}

TEST(SocialNavigation, OpposingFlowPenalizesForwardMotion) {
  auto world = worldWith(crowd(pedestrian("opposing", 1.2, 0.0, -0.5, 0.0)));
  const std::vector<Action> actions{Action::pause(),
                                    Action(ActionType::Forward, 2U)};
  const auto result = advisor().evaluate({world}, actions);
  EXPECT_LT(scoreFor(result, actions[1]), scoreFor(result, actions[0]));
}

TEST(SocialNavigation, StalePredictionDisablesAdvisor) {
  auto stale = crowd(pedestrian("stale", 1.0, 0.0, -0.5, 0.0));
  stale.data_age = std::chrono::seconds(2);
  auto world = worldWith(std::move(stale));
  const std::vector<Action> actions{Action::pause(),
                                    Action(ActionType::Forward, 2U)};
  const auto result = advisor().evaluate({world}, actions);
  EXPECT_FALSE(result.participated);
  EXPECT_TRUE(result.scores.empty());
}

TEST(SocialNavigation, PredictionStartsAtObservationAge) {
  auto fresh = crowd(pedestrian("crossing", 1.0, -1.0, 0.0, 1.0));
  fresh.data_age = std::chrono::nanoseconds::zero();
  auto aged = fresh;
  aged.data_age = std::chrono::seconds(1);
  const semaforr::decision::SocialNavigationAdvisor age_aware(
      {{0.2, 1.0},
       {0.5},
       std::chrono::seconds(2),
       0.25,
       2.0,
       1.2,
       0.65,
       1.0});
  const std::vector<Action> actions{Action(ActionType::Forward, 2U)};

  const auto fresh_result = age_aware.evaluate({worldWith(fresh)}, actions);
  const auto aged_result = age_aware.evaluate({worldWith(aged)}, actions);
  ASSERT_TRUE(fresh_result.participated);
  ASSERT_TRUE(aged_result.participated);
  EXPECT_GT(scoreFor(aged_result, actions.front()),
            scoreFor(fresh_result, actions.front()));
}

TEST(SocialNavigation, RecordedTrajectoryChangesDeterministicAction) {
  const std::vector<Action> actions{Action::pause(),
                                    Action(ActionType::Forward, 2U)};
  semaforr::domain::WorldModel empty_world;
  empty_world.robot.pose = {{0.0, 0.0}, semaforr::domain::Angle::zero()};

  semaforr::decision::ArbitrationConfiguration arbitration;
  arbitration.random_seed = 7U;
  arbitration.tie_tolerance = 1.0e-9;
  arbitration.unscored_policy = semaforr::decision::UnscoredActionPolicy::Zero;
  arbitration.unscored_baseline = 0.0;
  arbitration.fallback = Action::pause();
  semaforr::decision::DecisionCoordinator without_social(arbitration);
  without_social.addAdvisor(std::make_unique<ForwardGoalAdvisor>());
  const auto baseline = without_social.decide({empty_world}, actions);
  EXPECT_EQ(baseline.action, actions[1]);

  auto social_world =
      worldWith(crowd(pedestrian("opposing", 1.0, 0.0, -0.6, 0.0)));
  semaforr::decision::DecisionCoordinator with_social(arbitration);
  with_social.addAdvisor(std::make_unique<ForwardGoalAdvisor>());
  with_social.addAdvisor(
      std::make_unique<semaforr::decision::SocialNavigationAdvisor>(
          advisor(5.0)));
  const auto social = with_social.decide({social_world}, actions);
  EXPECT_EQ(social.action, Action::pause());
  EXPECT_NE(social.action, baseline.action);
}

TEST(SocialPlanning, DensityAndRiskCostsConsumeLearnedCrowdField) {
  semaforr::domain::CrowdFieldSnapshot field;
  field.geometry = {"map", 10.0, 10.0, 1.0, 0.0, 0.0};
  field.cells.resize(field.geometry.cellCount());
  field.estimator = "count";
  field.version = 1;
  field.generated_at = std::chrono::seconds(1);

  auto& density_cell = field.cells.at(*field.geometry.index({1.0, 0.0}));
  density_cell.visibility_exposures = 10.0;
  density_cell.pedestrian_hits = 8.0;
  density_cell.density = 0.8;
  density_cell.confidence = 1.0;
  density_cell.last_updated = std::chrono::seconds(1);

  auto& risk_cell = field.cells.at(*field.geometry.index({2.0, 0.0}));
  risk_cell.visibility_exposures = 10.0;
  risk_cell.pedestrian_hits = 2.0;
  risk_cell.risk_experiences = 10.0;
  risk_cell.risk_encounters = 9.0;
  risk_cell.density = 0.2;
  risk_cell.learned_encounter_risk = 0.9;
  risk_cell.confidence = 1.0;
  risk_cell.last_updated = std::chrono::seconds(1);
  field.validate();

  semaforr::domain::CrowdModel model;
  model.setLearned(std::move(field));
  EXPECT_GT(model.densityAt({1.0, 0.0}), model.densityAt({5.0, 5.0}));
  EXPECT_GT(model.learnedEncounterRiskAt({2.0, 0.0}),
            model.densityAt({2.0, 0.0}));

  model = semaforr::domain::CrowdModel{};
  EXPECT_DOUBLE_EQ(model.densityAt({1.0, 0.0}), 0.0);
  EXPECT_DOUBLE_EQ(model.learnedEncounterRiskAt({2.0, 0.0}), 0.0);
}

TEST(SocialDomain, RejectsDuplicateIdentityAndInvalidCovariance) {
  auto duplicate = crowd(pedestrian("same", 0.0, 0.0, 0.0, 0.0));
  duplicate.pedestrians.push_back(pedestrian("same", 1.0, 0.0, 0.0, 0.0));
  EXPECT_THROW(duplicate.validate(), std::invalid_argument);

  auto invalid = pedestrian("covariance", 0.0, 0.0, 0.0, 0.0);
  invalid.position_covariance = {1.0, 2.0, 0.0, 1.0};
  EXPECT_THROW(invalid.validate(observed_at), std::invalid_argument);
}

/**
 * @brief Performs the tracked configuration operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `semaforr::ros::SocialObservationConfiguration` containing the
 * operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::ros::SocialObservationConfiguration trackedConfiguration() {
  semaforr::ros::SocialObservationConfiguration configuration;
  configuration.frame = "map";
  configuration.input_mode = semaforr::ros::SocialInputMode::Tracked;
  configuration.minimum_confidence = 0.5;
  configuration.prediction_steps = 2U;
  configuration.prediction_step_s = 0.5;
  return configuration;
}

/**
 * @brief Performs the tracked message operation for this subsystem.
 *
 * Arguments:
 * - @p id: Supplies id input to the operation.
 *
 * Returns:
 * - `social_context_msgs::msg::TrackedPersonArray` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
social_context_msgs::msg::TrackedPersonArray trackedMessage(int id = 7) {
  social_context_msgs::msg::TrackedPersonArray message;
  message.header.frame_id = "map";
  message.header.stamp.sec = 10;
  message.people.resize(1);
  message.people[0].id = id;
  message.people[0].x = 2.0F;
  message.people[0].y = 1.0F;
  message.people[0].confidence = 0.9F;
  message.people[0].history_x = {1.8F, 2.0F};
  message.people[0].history_y = {1.0F, 1.0F};
  return message;
}

TEST(SocialObservationBuffer, ConvertsTrackedStateAndSeparatesFreshness) {
  semaforr::ros::SocialObservationBuffer buffer(trackedConfiguration());
  auto message = trackedMessage();
  const rclcpp::Time received(10'100'000'000LL, RCL_ROS_TIME);
  ASSERT_TRUE(buffer.acceptTracked(message, received));
  const auto snapshot = buffer.snapshot(received);
  ASSERT_TRUE(snapshot);
  ASSERT_EQ(snapshot->pedestrians.size(), 1U);
  EXPECT_EQ(snapshot->pedestrians[0].id, "7");
  EXPECT_NEAR(snapshot->pedestrians[0].velocity_mps.x_m, 2.0, 1.0e-5);
  EXPECT_EQ(snapshot->pedestrians[0].prediction_source,
            "constant_velocity");
  EXPECT_EQ(buffer.status(rclcpp::Time(11'000'000'000LL, RCL_ROS_TIME)),
            semaforr::ros::SocialObservationStatus::Stale);

  message.header.frame_id = "odom";
  EXPECT_FALSE(buffer.acceptTracked(message, received));
  EXPECT_EQ(buffer.status(received),
            semaforr::ros::SocialObservationStatus::FrameMismatch);
}

TEST(SocialObservationBuffer, AccumulatesCompleteGstPredictionCycle) {
  auto configuration = trackedConfiguration();
  semaforr::ros::SocialObservationBuffer buffer(configuration);
  const rclcpp::Time received(10'100'000'000LL, RCL_ROS_TIME);
  ASSERT_TRUE(buffer.acceptTracked(trackedMessage(), received));

  geometry_msgs::msg::PoseStamped prediction;
  prediction.header.frame_id = "7_pred_0";
  prediction.header.stamp.sec = 10;
  prediction.pose.position.x = 2.5;
  prediction.pose.position.y = 1.0;
  ASSERT_TRUE(buffer.acceptPrediction(prediction, received));
  prediction.header.frame_id = "7_pred_1";
  prediction.pose.position.x = 3.0;
  ASSERT_TRUE(buffer.acceptPrediction(prediction, received));
  EXPECT_FALSE(buffer.acceptPrediction(prediction, received));

  const auto snapshot = buffer.snapshot(received);
  ASSERT_TRUE(snapshot);
  ASSERT_EQ(snapshot->pedestrians[0].predicted_trajectory.size(), 2U);
  EXPECT_EQ(snapshot->pedestrians[0].prediction_source, "gst");
  EXPECT_DOUBLE_EQ(
      snapshot->pedestrians[0].predicted_trajectory[1].position.x_m, 3.0);
}

TEST(SocialObservationBuffer, RejectsInvalidAndUnassociatedPredictions) {
  auto configuration = trackedConfiguration();
  semaforr::ros::SocialObservationBuffer buffer(configuration);
  const rclcpp::Time received(10'100'000'000LL, RCL_ROS_TIME);
  ASSERT_TRUE(buffer.acceptTracked(trackedMessage(), received));

  geometry_msgs::msg::PoseStamped prediction;
  prediction.header.stamp.sec = 10;
  prediction.pose.position.x = 2.5;
  prediction.pose.position.y = 1.0;
  prediction.header.frame_id = "malformed";
  EXPECT_FALSE(buffer.acceptPrediction(prediction, received));
  prediction.header.frame_id = "99_pred_0";
  EXPECT_FALSE(buffer.acceptPrediction(prediction, received));
  prediction.header.frame_id = "7_pred_2";
  EXPECT_FALSE(buffer.acceptPrediction(prediction, received));
  prediction.header.frame_id = "7_pred_0";
  prediction.header.stamp.sec = 1;
  EXPECT_FALSE(buffer.acceptPrediction(prediction, received));
}

TEST(SocialObservationBuffer, IncompleteGstUsesFallbackAndLaterCycleRecovers) {
  auto configuration = trackedConfiguration();
  semaforr::ros::SocialObservationBuffer buffer(configuration);
  const rclcpp::Time received(10'100'000'000LL, RCL_ROS_TIME);
  ASSERT_TRUE(buffer.acceptTracked(trackedMessage(), received));

  geometry_msgs::msg::PoseStamped prediction;
  prediction.header.frame_id = "7_pred_0";
  prediction.header.stamp.sec = 10;
  prediction.pose.position.x = 99.0;
  prediction.pose.position.y = 1.0;
  ASSERT_TRUE(buffer.acceptPrediction(prediction, received));
  auto snapshot = buffer.snapshot(received);
  ASSERT_TRUE(snapshot);
  EXPECT_EQ(snapshot->pedestrians[0].prediction_source,
            "constant_velocity");
  EXPECT_NE(snapshot->pedestrians[0].predicted_trajectory[0].position.x_m,
            99.0);

  const rclcpp::Time next_cycle(11'100'000'000LL, RCL_ROS_TIME);
  prediction.header.stamp.sec = 11;
  prediction.pose.position.x = 2.5;
  ASSERT_TRUE(buffer.acceptPrediction(prediction, next_cycle));
  prediction.header.frame_id = "7_pred_1";
  prediction.pose.position.x = 3.0;
  ASSERT_TRUE(buffer.acceptPrediction(prediction, next_cycle));
  auto current = trackedMessage();
  current.header.stamp.sec = 11;
  ASSERT_TRUE(buffer.acceptTracked(current, next_cycle));
  snapshot = buffer.snapshot(next_cycle);
  ASSERT_TRUE(snapshot);
  EXPECT_EQ(snapshot->pedestrians[0].prediction_source, "gst");
}

TEST(SocialObservationBuffer, AttachesOnlyKnownFreshFormationMembers) {
  semaforr::ros::SocialObservationBuffer buffer(trackedConfiguration());
  const rclcpp::Time received(10'100'000'000LL, RCL_ROS_TIME);
  ASSERT_TRUE(buffer.acceptTracked(trackedMessage(), received));
  social_context_msgs::msg::FormationGroupArray formations;
  formations.header.frame_id = "map";
  formations.header.stamp.sec = 10;
  formations.groups.resize(1);
  formations.groups[0].member_ids = {7};
  formations.groups[0].formation_type = "face_to_face";
  formations.groups[0].confidence = 0.9F;
  formations.groups[0].center_x = 2.0F;
  formations.groups[0].center_y = 1.0F;
  ASSERT_TRUE(buffer.acceptFormations(formations, received));
  const auto snapshot = buffer.snapshot(received);
  ASSERT_TRUE(snapshot);
  ASSERT_EQ(snapshot->formations.size(), 1U);
  ASSERT_TRUE(snapshot->pedestrians[0].formation_index);
  EXPECT_EQ(*snapshot->pedestrians[0].formation_index, 0U);

  formations.groups[0].member_ids = {99};
  ASSERT_TRUE(buffer.acceptFormations(formations, received));
  const auto unknown = buffer.snapshot(received);
  ASSERT_TRUE(unknown);
  EXPECT_TRUE(unknown->formations.empty());
}

TEST(SocialObservationBuffer, HuNavModeUsesVelocityAndSharedDomainType) {
  auto configuration = trackedConfiguration();
  configuration.input_mode = semaforr::ros::SocialInputMode::Hunav;
  semaforr::ros::SocialObservationBuffer buffer(configuration);
  hunav_msgs::msg::Agents message;
  message.header.frame_id = "map";
  message.header.stamp.sec = 10;
  message.agents.resize(2);
  message.agents[0].id = 3;
  message.agents[0].type = hunav_msgs::msg::Agent::PERSON;
  message.agents[0].position.position.x = 1.0;
  message.agents[0].velocity.linear.y = 0.4;
  message.agents[1].id = 4;
  message.agents[1].type = hunav_msgs::msg::Agent::ROBOT;
  const rclcpp::Time received(10'100'000'000LL, RCL_ROS_TIME);
  ASSERT_TRUE(buffer.acceptHunav(message, received));
  const auto snapshot = buffer.snapshot(received);
  ASSERT_TRUE(snapshot);
  ASSERT_EQ(snapshot->pedestrians.size(), 1U);
  EXPECT_EQ(snapshot->pedestrians[0].id, "3");
  EXPECT_NEAR(snapshot->pedestrians[0].velocity_mps.y_m, 0.4, 1.0e-6);
  EXPECT_EQ(snapshot->provenance, "hunav_agents");
}

TEST(SocialObservationBuffer, DetectsDisappearanceAndReappearance) {
  semaforr::ros::SocialObservationBuffer buffer(trackedConfiguration());
  const rclcpp::Time received(10'100'000'000LL, RCL_ROS_TIME);
  ASSERT_TRUE(buffer.acceptTracked(trackedMessage(), received));
  auto events = buffer.takeLifecycleEvents();
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].type,
            semaforr::ros::TrackLifecycleEvent::Type::Appeared);

  auto empty = trackedMessage();
  empty.people.clear();
  ASSERT_TRUE(buffer.acceptTracked(empty, received));
  events = buffer.takeLifecycleEvents();
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].type,
            semaforr::ros::TrackLifecycleEvent::Type::Disappeared);

  ASSERT_TRUE(buffer.acceptTracked(trackedMessage(), received));
  events = buffer.takeLifecycleEvents();
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].type,
            semaforr::ros::TrackLifecycleEvent::Type::Reappeared);
}
