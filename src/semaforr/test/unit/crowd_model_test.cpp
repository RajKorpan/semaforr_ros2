#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <semaforr/decision/learned_crowd_advisor.hpp>
#include <semaforr/decision/social_advisor_registry.hpp>
#include <semaforr/domain/crowd_model.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/social/crowd_field_learner.hpp>
#include <sstream>
#include <vector>

namespace {

using namespace std::chrono_literals;

semaforr::domain::LaserObservation laser() {
  semaforr::domain::LaserObservation result;
  result.angle_min = semaforr::domain::Angle::zero();
  result.angle_increment = semaforr::domain::Angle(0.1);
  result.minimum_range = semaforr::domain::Distance(0.0);
  result.maximum_range = semaforr::domain::Distance(5.0);
  result.ranges_m = {3.0};
  return result;
}

semaforr::domain::CrowdObservation crowd(std::chrono::nanoseconds stamp,
                                         bool include_person) {
  semaforr::domain::CrowdObservation result;
  result.frame_id = "map";
  result.observed_at = stamp;
  if (include_person) {
    semaforr::domain::PedestrianObservation person;
    person.id = "person-1";
    person.position = {2.2, 1.5};
    person.velocity_mps = {1.0, 0.0};
    person.confidence = 1.0;
    result.pedestrians.push_back(person);
  }
  result.validate();
  return result;
}

semaforr::social::CrowdFieldLearnerConfiguration configuration(
    semaforr::social::CrowdEstimatorStrategy strategy =
        semaforr::social::CrowdEstimatorStrategy::CountExposure) {
  semaforr::social::CrowdFieldLearnerConfiguration result;
  result.geometry = {"map", 5.0, 5.0, 1.0, 0.0, 0.0};
  result.strategy = strategy;
  result.minimum_update_period_s = 0.0;
  result.random_seed = 42U;
  return result;
}

TEST(CrowdFieldLearner, UsesVisibilityAsDensityDenominator) {
  semaforr::social::CrowdFieldLearner learner(configuration());
  const semaforr::domain::Pose2D robot{{1.5, 1.5},
                                       semaforr::domain::Angle::zero()};

  EXPECT_TRUE(learner.observe(robot, laser(), crowd(1s, true)));
  EXPECT_TRUE(learner.observe(robot, laser(), crowd(2s, false)));
  const auto& snapshot = learner.snapshot();
  const auto occupied = snapshot.sample({2.2, 1.5});
  const auto visible_empty = snapshot.sample({3.2, 1.5});
  ASSERT_TRUE(occupied);
  ASSERT_TRUE(visible_empty);
  EXPECT_DOUBLE_EQ(occupied->cell.visibility_exposures, 2.0);
  EXPECT_DOUBLE_EQ(occupied->cell.pedestrian_hits, 1.0);
  EXPECT_DOUBLE_EQ(occupied->cell.density, 0.5);
  EXPECT_DOUBLE_EQ(occupied->cell.directional_flow[static_cast<std::size_t>(
                       semaforr::domain::CrowdFlowDirection::Right)],
                   0.5);
  EXPECT_DOUBLE_EQ(visible_empty->cell.density, 0.0);

  const auto robot_cell = snapshot.sample({1.5, 1.5});
  ASSERT_TRUE(robot_cell);
  EXPECT_DOUBLE_EQ(robot_cell->cell.risk_experiences, 2.0);
  EXPECT_DOUBLE_EQ(robot_cell->cell.learned_encounter_risk, 0.5);
}

TEST(CrowdFieldLearner, EmptyObservationIsNegativeEvidence) {
  const auto empty = crowd(1s, false);
  EXPECT_TRUE(empty.fresh(1s));
  EXPECT_FALSE(empty.usable(1s));

  semaforr::social::CrowdFieldLearner learner(configuration());
  EXPECT_TRUE(learner.observe({{1.5, 1.5}, semaforr::domain::Angle::zero()},
                              laser(), empty));
  EXPECT_TRUE(learner.snapshot().available());
}

TEST(CrowdFieldLearner, NoReturnLaserBeamExposesCellsToMaximumRange) {
  semaforr::social::CrowdFieldLearner learner(configuration());
  auto no_return = laser();
  no_return.ranges_m = {std::numeric_limits<double>::infinity()};
  EXPECT_NO_THROW(no_return.validate());
  EXPECT_TRUE(learner.observe({{0.5, 0.5}, semaforr::domain::Angle::zero()},
                              no_return, crowd(1s, false)));
  const auto far_cell = learner.snapshot().sample({4.5, 0.5});
  ASSERT_TRUE(far_cell);
  EXPECT_DOUBLE_EQ(far_cell->cell.visibility_exposures, 1.0);
}

TEST(CrowdFieldLearner, SeededThompsonSnapshotsAreDeterministic) {
  semaforr::social::CrowdFieldLearner first(
      configuration(semaforr::social::CrowdEstimatorStrategy::Thompson));
  semaforr::social::CrowdFieldLearner second(
      configuration(semaforr::social::CrowdEstimatorStrategy::Thompson));
  const semaforr::domain::Pose2D robot{{1.5, 1.5},
                                       semaforr::domain::Angle::zero()};

  ASSERT_TRUE(first.observe(robot, laser(), crowd(1s, true)));
  ASSERT_TRUE(second.observe(robot, laser(), crowd(1s, true)));
  EXPECT_EQ(first.snapshot().cells, second.snapshot().cells);
}

TEST(CrowdModel, SerializesAndRemainsUsefulWithoutLivePeople) {
  semaforr::social::CrowdFieldLearner learner(configuration());
  ASSERT_TRUE(learner.observe({{1.5, 1.5}, semaforr::domain::Angle::zero()},
                              laser(), crowd(1s, true)));

  std::stringstream stream;
  learner.snapshot().save(stream);
  const auto restored = semaforr::domain::CrowdFieldSnapshot::load(stream);
  EXPECT_EQ(restored, learner.snapshot());

  semaforr::domain::CrowdModel model;
  model.setLearned(restored);
  model.clearCurrent();
  EXPECT_EQ(model.status(), semaforr::domain::CrowdModelStatus::LearnedOnly);
  EXPECT_GT(model.densityAt({2.2, 1.5}), 0.0);
  EXPECT_GT(model.flowAlignmentAt({2.2, 1.5}, semaforr::domain::Angle::zero()),
            0.0);
}

TEST(CrowdModel, CanonicalPredictionsProduceDeterministicCollisionRisk) {
  auto observation = crowd(1s, true);
  observation.pedestrians.front().predicted_trajectory = {{{4.0, 4.0}, 2s},
                                                          {{4.5, 4.0}, 3s}};
  observation.validate();

  semaforr::domain::CrowdModel model;
  model.update(std::move(observation));
  const double on_prediction = model.predictiveCollisionRiskAt({4.0, 4.0});
  const double away_from_trajectory =
      model.predictiveCollisionRiskAt({0.0, 4.0});
  EXPECT_DOUBLE_EQ(on_prediction, 1.0);
  EXPECT_GT(on_prediction, away_from_trajectory);
}

TEST(CrowdConsumers, RiskAdvisorAndPlannerShareLivePredictionRisk) {
  auto observation = crowd(1s, true);
  observation.pedestrians.front().predicted_trajectory = {{{4.0, 4.0}, 2s}};
  observation.validate();
  semaforr::domain::CrowdModel model;
  model.update(std::move(observation));

  semaforr::domain::WorldModel world;
  world.robot.pose = {{3.0, 4.0}, semaforr::domain::Angle::zero()};
  world.crowd = model;
  semaforr::decision::LearnedCrowdAdvisor advisor(
      {semaforr::decision::LearnedCrowdObjective::AvoidEncounterRisk,
       {1.0},
       {},
       1.0,
       0.0,
       "risk_avoid"});
  const std::vector<semaforr::domain::Action> actions{
      semaforr::domain::Action(semaforr::domain::ActionType::Forward, 1U)};
  const auto evaluation =
      advisor.evaluate(semaforr::decision::DecisionContext{world}, actions);
  ASSERT_TRUE(evaluation.participated);
  ASSERT_EQ(evaluation.scores.size(), 1U);

  EXPECT_DOUBLE_EQ(evaluation.scores.front().raw_score,
                   -model.navigationRiskAt({4.0, 4.0}));
}

TEST(CrowdConsumers, AdvisorAndPlannerReadTheSameLearnedCell) {
  semaforr::social::CrowdFieldLearner learner(configuration());
  ASSERT_TRUE(learner.observe({{1.5, 1.5}, semaforr::domain::Angle::zero()},
                              laser(), crowd(1s, true)));
  semaforr::domain::CrowdModel model;
  model.setLearned(learner.snapshot());

  semaforr::domain::WorldModel world;
  world.robot.pose = {{1.2, 1.5}, semaforr::domain::Angle::zero()};
  world.crowd = model;
  semaforr::decision::LearnedCrowdAdvisor advisor(
      {semaforr::decision::LearnedCrowdObjective::AvoidDensity,
       {1.0},
       {},
       1.0,
       0.0,
       "crowd_avoid"});
  const std::vector<semaforr::domain::Action> actions{
      semaforr::domain::Action(semaforr::domain::ActionType::Forward, 1U)};
  const auto evaluation =
      advisor.evaluate(semaforr::decision::DecisionContext{world}, actions);
  ASSERT_TRUE(evaluation.participated);
  ASSERT_EQ(evaluation.scores.size(), 1U);

  EXPECT_DOUBLE_EQ(evaluation.scores.front().raw_score,
                   -model.densityAt({2.2, 1.5}));
}

TEST(CrowdConsumers, TypedPlannerIncludesLearnedCrowdCost) {
  semaforr::social::CrowdFieldLearner learner(configuration());
  ASSERT_TRUE(learner.observe({{1.5, 1.5}, semaforr::domain::Angle::zero()},
                              laser(), crowd(1s, true)));
  semaforr::domain::CrowdModel model;
  model.setLearned(learner.snapshot());

  semaforr::domain::SpatialModel spatial;
  spatial.skeleton_nodes = {{1.7, 1.5}, {2.7, 1.5}};
  spatial.skeleton_edges = {{0U, 1U}};
  const semaforr::planning::PlanningRequest request{
      {{1.7, 1.5}, semaforr::domain::Angle::zero()},
      {2.7, 1.5},
      &spatial,
      &model};
  semaforr::planning::DomainPlanner planner(
      "density", semaforr::planning::PlannerObjective::CrowdDensity);
  const auto result = planner.plan(request);
  ASSERT_TRUE(result.succeeded());
  EXPECT_GT(result.cost_m, 1.0);
}

TEST(CrowdConsumers, StableRegistryConstructsAllSocialAdvisors) {
  semaforr::decision::SocialAdvisorRegistryConfiguration configuration;
  configuration.live.move_distances_m = {1.0};
  configuration.live.rotation_angles_rad = {0.5};
  configuration.density.move_distances_m = {1.0};
  configuration.density.rotation_angles_rad = {0.5};
  configuration.risk = configuration.density;
  configuration.flow = configuration.density;
  configuration.density.advisor_name = "crowd_avoid";
  configuration.risk.advisor_name = "risk_avoid";
  configuration.flow.advisor_name = "flow_follow";

  semaforr::decision::AdvisorRegistry registry;
  semaforr::decision::registerSocialAdvisorFactories(registry, configuration);
  EXPECT_EQ(registry.create("social_navigation")->name(), "social_navigation");
  EXPECT_EQ(registry.create("crowd_avoid")->name(), "crowd_avoid");
  EXPECT_EQ(registry.create("risk_avoid")->name(), "risk_avoid");
  EXPECT_EQ(registry.create("flow_follow")->name(), "flow_follow");
  EXPECT_THROW(registry.create("unknown_social"), std::invalid_argument);
}

}  // namespace
