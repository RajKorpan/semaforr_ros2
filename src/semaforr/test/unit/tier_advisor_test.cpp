/**
 * @file tier_advisor_test.cpp
 * @brief Tier advisor test responsibilities.
 *
 * @details This file exercises tier advisor test behavior for automated
 * verification and regression testing. It centers on
 * `RestoresEveryHeuristicAdvisorWithMetadata`,
 * `SpatialAdvisorReportsSourceRevision`,
 * `ProductionAdvisorsNormalizeWholeScoreSetToTenPoint`,
 * `IdenticalRawCommentsBecomeNeutralWithoutDivisionByZero`,
 * `BigStepAndGreedyUseMetricLookahead`,
 * `ElbowRoomAndGoAroundRespondToObstacleSide`,
 * `GoAroundIgnoresMaximumRangeAndScoresOnlyRotations`,
 * `NoveltyAndCuriosityUseDifferentHistoryScopes`. Its package-relative
 * location is `test/unit/tier_advisor_test.cpp`.
 */
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <semaforr/decision/advisors/catalog_registry.hpp>
#include <semaforr/decision/advisors/heuristic_advisor.hpp>
#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/decision/tier_registry.hpp>
#include <semaforr/planning/reactive_planner.hpp>
#include <semaforr/spatial/learners/circumstance_learner.hpp>

namespace {

/**
 * @brief Performs the world with target operation for this subsystem.
 *
 * Arguments:
 * - @p target: Supplies target input to the operation.
 *
 * Returns:
 * - `semaforr::domain::WorldModel` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::WorldModel worldWithTarget(
    semaforr::domain::Point2D target = {2.0, 0.0}) {
  semaforr::domain::WorldModel world;
  world.mission =
      semaforr::domain::Mission({{0U, target}}, 20U);
  world.mission.activate_next();
  world.mission.install_active_plan({target});
  return world;
}

/**
 * @brief Performs the laser operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `semaforr::domain::LaserObservation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::LaserObservation laser() {
  semaforr::domain::LaserObservation result;
  result.minimum_range = semaforr::domain::Distance(0.1);
  result.maximum_range = semaforr::domain::Distance(5.0);
  result.angle_increment = semaforr::domain::Angle(0.1);
  result.ranges_m = {2.0, 2.0, 2.0};
  return result;
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
                const semaforr::domain::Action& action) {
  const auto found = std::find_if(
      evaluation.scores.begin(), evaluation.scores.end(),
      [&](const auto& score) { return score.action == action; });
  EXPECT_NE(found, evaluation.scores.end());
  return found == evaluation.scores.end() ? 0.0 : found->raw_score;
}

/**
 * @brief Performs the spatial evaluation operation for this subsystem.
 *
 * Arguments:
 * - @p objective: Supplies objective input to the operation.
 * - @p world: Supplies world input to the operation.
 * - @p actions: Supplies actions input to the operation.
 * - @p candidates: Supplies candidates input to the operation.
 *
 * Returns:
 * - `semaforr::decision::AdvisorEvaluation` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::decision::AdvisorEvaluation spatialEvaluation(
    semaforr::decision::HeuristicObjective objective,
    const semaforr::domain::WorldModel& world,
    const semaforr::domain::ActionSpace& actions,
    const std::vector<semaforr::domain::Action>& candidates) {
  semaforr::decision::HeuristicAdvisor advisor(
      {"spatial_test", objective, actions, 1.0});
  return advisor.evaluate({world}, candidates);
}

/**
 * @brief Performs the learned region operation for this subsystem.
 *
 * Arguments:
 * - @p id: Supplies id input to the operation.
 * - @p center: Supplies center input to the operation.
 * - @p radius_m: Supplies radius m input to the operation.
 *
 * Returns:
 * - `semaforr::domain::LearnedRegion` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::LearnedRegion learnedRegion(
    semaforr::domain::RegionId id, semaforr::domain::Point2D center,
    double radius_m) {
  semaforr::domain::LearnedRegion result;
  result.id = id;
  result.boundary = {center, semaforr::domain::Distance(radius_m)};
  return result;
}

/**
 * @brief Performs the skeleton node operation for this subsystem.
 *
 * Arguments:
 * - @p id: Supplies id input to the operation.
 * - @p region: Supplies region input to the operation.
 * - @p center: Supplies center input to the operation.
 *
 * Returns:
 * - `semaforr::domain::RegionSkeletonNode` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::RegionSkeletonNode skeletonNode(
    std::size_t id, semaforr::domain::RegionId region,
    semaforr::domain::Point2D center) {
  semaforr::domain::RegionSkeletonNode result;
  result.id = id;
  result.region = region;
  result.center = center;
  return result;
}

/**
 * @brief Performs the learned door operation for this subsystem.
 *
 * Arguments:
 * - @p id: Supplies id input to the operation.
 * - @p region: Supplies region input to the operation.
 *
 * Returns:
 * - `semaforr::domain::LearnedDoor` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::LearnedDoor learnedDoor(
    semaforr::domain::DoorId id, semaforr::domain::RegionId region) {
  semaforr::domain::LearnedDoor result;
  result.id = id;
  result.region = region;
  return result;
}

}  // namespace

TEST(TierThreeCatalog, RestoresEveryHeuristicAdvisorWithMetadata) {
  const semaforr::domain::ActionSpace actions({0.25, 0.5}, {0.2, 0.5});
  const std::vector<std::string> names{
      "big_step",       "elbow_room", "novelty",     "go_around",
      "greedy",         "curiosity",  "enfilade",    "visual_scan",
      "convey",         "enter",      "exit",        "trailer",
      "unlikely",       "access",     "crossroads",  "follow",
      "least_angle",    "spatial_learner", "stay",   "social_navigation",
      "crowd_avoid",    "risk_avoid", "flow_follow"};
  std::vector<semaforr::config::AdvisorConfiguration> configured;
  for (const auto& name : names)
    configured.push_back({name, name, true, 1.0, {}});

  semaforr::decision::AdvisorRegistry registry;
  semaforr::decision::registerAdvisorCatalog(registry, actions, configured);
  for (const auto& name : names) {
    const auto advisor = registry.create(name);
    ASSERT_NE(advisor, nullptr) << name;
    EXPECT_EQ(advisor->name(), name);
    const auto metadata = advisor->metadata();
    EXPECT_FALSE(metadata.scored_action_types.empty()) << name;
    EXPECT_FALSE(metadata.rationale.empty()) << name;
    EXPECT_EQ(metadata.normalization,
              semaforr::decision::ScoreNormalization::TenPoint) << name;
  }
}

TEST(TierThreeCatalog, SpatialAdvisorReportsSourceRevision) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  std::vector<semaforr::config::AdvisorConfiguration> configured{
      {"crossroads", "crossroads", true, 1.0, {}}};
  semaforr::decision::AdvisorRegistry registry;
  semaforr::decision::registerAdvisorCatalog(registry, actions, configured);
  const auto advisor = registry.create("crossroads");
  auto world = worldWithTarget();
  world.spatial.revision = 17U;
  world.spatial.hallways = {{{0.0, 0.0}, {2.0, 0.0}},
                            {{1.0, -1.0}, {1.0, 1.0}}};
  const std::vector<semaforr::domain::Action> candidates{
      semaforr::domain::Action::pause(),
      {semaforr::domain::ActionType::Forward, 1U},
      {semaforr::domain::ActionType::TurnLeft, 1U},
      {semaforr::domain::ActionType::TurnRight, 1U}};
  const auto evaluation = advisor->evaluate({world}, candidates);
  EXPECT_EQ(evaluation.model_revision_used, 17U);
  const auto dependencies = advisor->dependencies();
  EXPECT_NE(std::find(dependencies.begin(), dependencies.end(), "hallways"),
            dependencies.end());
}

TEST(TierThreeNormalization, ProductionAdvisorsNormalizeWholeScoreSetToTenPoint) {
  using namespace semaforr;
  const domain::ActionSpace actions({1.0}, {0.5});
  auto world = worldWithTarget({5.0, 0.0});
  const std::vector<domain::Action> candidates{
      domain::Action::pause(),
      {domain::ActionType::Forward, 1U}};
  decision::DecisionCoordinator coordinator;
  coordinator.addAdvisor(std::make_unique<decision::HeuristicAdvisor>(
      decision::HeuristicAdvisorConfiguration{
          "big_step", decision::HeuristicObjective::BigStep, actions, 2.0}));
  const auto result = coordinator.decideTierThree({world}, candidates);
  ASSERT_EQ(result.contributions.size(), 2U);
  const auto pause = std::find_if(
      result.contributions.begin(), result.contributions.end(),
      [](const auto& value) { return value.action == domain::Action::pause(); });
  const auto forward = std::find_if(
      result.contributions.begin(), result.contributions.end(),
      [](const auto& value) {
        return value.action ==
               domain::Action(domain::ActionType::Forward, 1U);
      });
  ASSERT_NE(pause, result.contributions.end());
  ASSERT_NE(forward, result.contributions.end());
  EXPECT_DOUBLE_EQ(pause->raw_score, 0.0);
  EXPECT_DOUBLE_EQ(forward->raw_score, 1.0);
  EXPECT_DOUBLE_EQ(pause->normalized_score, 0.0);
  EXPECT_DOUBLE_EQ(forward->normalized_score, 10.0);
  EXPECT_DOUBLE_EQ(forward->weighted_score, 20.0);
}

TEST(TierThreeNormalization, IdenticalRawCommentsBecomeNeutralWithoutDivisionByZero) {
  using namespace semaforr;
  const domain::ActionSpace actions({1.0}, {0.5});
  auto world = worldWithTarget({5.0, 0.0});
  const std::vector<domain::Action> candidates{
      {domain::ActionType::TurnLeft, 1U},
      {domain::ActionType::TurnRight, 1U}};
  decision::DecisionCoordinator coordinator;
  coordinator.addAdvisor(std::make_unique<decision::HeuristicAdvisor>(
      decision::HeuristicAdvisorConfiguration{
          "big_step", decision::HeuristicObjective::BigStep, actions, 1.0}));
  const auto result = coordinator.decideTierThree({world}, candidates);
  ASSERT_EQ(result.contributions.size(), 2U);
  for (const auto& contribution : result.contributions) {
    EXPECT_DOUBLE_EQ(contribution.raw_score, 0.5);
    EXPECT_DOUBLE_EQ(contribution.normalized_score, 5.0);
    EXPECT_TRUE(std::isfinite(contribution.normalized_score));
  }
}

TEST(CommonsenseAdvisors, BigStepAndGreedyUseMetricLookahead) {
  using semaforr::decision::HeuristicAdvisor;
  using semaforr::decision::HeuristicObjective;
  using semaforr::domain::Action;
  using semaforr::domain::ActionType;
  const semaforr::domain::ActionSpace actions({1.0, 2.0}, {0.5});
  auto world = worldWithTarget({5.0, 0.0});
  auto view = laser();
  view.angle_min = semaforr::domain::Angle(-0.5);
  view.maximum_range = semaforr::domain::Distance(5.0);
  view.ranges_m = {5.0, 5.0, 5.0};
  world.robot.laser = view;
  const std::vector<Action> candidates{
      Action::pause(), {ActionType::Forward, 1U},
      {ActionType::Forward, 2U}, {ActionType::TurnLeft, 1U}};

  HeuristicAdvisor big_step({"big_step",
      HeuristicObjective::BigStep, actions, 1.0});
  const auto big = big_step.evaluate({world}, candidates);
  EXPECT_GT(scoreFor(big, {ActionType::Forward, 2U}),
            scoreFor(big, {ActionType::Forward, 1U}));
  EXPECT_GT(scoreFor(big, {ActionType::Forward, 1U}),
            scoreFor(big, Action::pause()));

  HeuristicAdvisor greedy({"greedy",
      HeuristicObjective::Greedy, actions, 1.0});
  const auto goal = greedy.evaluate({world}, candidates);
  EXPECT_GT(scoreFor(goal, {ActionType::Forward, 2U}),
            scoreFor(goal, Action::pause()));
}

TEST(CommonsenseAdvisors, ElbowRoomAndGoAroundRespondToObstacleSide) {
  using semaforr::decision::HeuristicAdvisor;
  using semaforr::decision::HeuristicObjective;
  using semaforr::domain::Action;
  using semaforr::domain::ActionType;
  const semaforr::domain::ActionSpace actions({1.0}, {0.5});
  auto world = worldWithTarget();
  auto view = laser();
  view.angle_min = semaforr::domain::Angle(0.3);
  view.maximum_range = semaforr::domain::Distance(5.0);
  view.ranges_m = {std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity(), 1.0};
  world.robot.laser = view;
  const std::vector<Action> candidates{
      {ActionType::TurnLeft, 1U}, {ActionType::TurnRight, 1U}};

  HeuristicAdvisor elbow({"elbow_room",
      HeuristicObjective::ElbowRoom, actions, 1.0});
  const auto clearance = elbow.evaluate({world}, candidates);
  EXPECT_GT(scoreFor(clearance, {ActionType::TurnRight, 1U}),
            scoreFor(clearance, {ActionType::TurnLeft, 1U}));

  HeuristicAdvisor around({"go_around",
      HeuristicObjective::GoAround, actions, 1.0});
  const auto avoidance = around.evaluate({world}, candidates);
  EXPECT_GT(scoreFor(avoidance, {ActionType::TurnRight, 1U}),
            scoreFor(avoidance, {ActionType::TurnLeft, 1U}));
}

TEST(CommonsenseAdvisors, GoAroundIgnoresMaximumRangeAndScoresOnlyRotations) {
  using namespace semaforr;
  const domain::ActionSpace actions({1.0}, {0.5});
  auto world = worldWithTarget();
  auto view = laser();
  view.ranges_m.assign(3U, view.maximum_range.meters());
  world.robot.laser = view;
  decision::HeuristicAdvisor advisor(
      {"go_around", decision::HeuristicObjective::GoAround, actions, 1.0});
  const auto metadata = advisor.metadata();
  EXPECT_EQ(metadata.scored_action_types,
            (std::vector<domain::ActionType>{domain::ActionType::TurnLeft,
                                             domain::ActionType::TurnRight}));
  const std::vector<domain::Action> candidates{
      domain::Action::pause(), {domain::ActionType::Forward, 1U},
      {domain::ActionType::TurnLeft, 1U},
      {domain::ActionType::TurnRight, 1U}};
  const auto evaluation = advisor.evaluate({world}, candidates);
  ASSERT_EQ(evaluation.scores.size(), 2U);
  EXPECT_DOUBLE_EQ(evaluation.scores[0].raw_score,
                   evaluation.scores[1].raw_score);
}

TEST(CommonsenseAdvisors, NoveltyAndCuriosityUseDifferentHistoryScopes) {
  using semaforr::decision::HeuristicAdvisor;
  using semaforr::decision::HeuristicObjective;
  using semaforr::domain::Action;
  using semaforr::domain::ActionType;
  const semaforr::domain::ActionSpace actions({2.0}, {0.5});
  auto world = worldWithTarget({5.0, 0.0});
  world.navigation_history.record(
      {{{0.0, 0.0}, semaforr::domain::Angle::zero()}, laser(),
       Action::pause(), 0U});
  world.navigation_history.record(
      {{{2.0, 0.0}, semaforr::domain::Angle::zero()}, laser(),
       Action::pause(), 99U});
  const std::vector<Action> candidates{
      Action::pause(), {ActionType::Forward, 1U}};

  HeuristicAdvisor novelty({"novelty",
      HeuristicObjective::Novelty, actions, 1.0});
  const auto current_target = novelty.evaluate({world}, candidates);
  EXPECT_GT(scoreFor(current_target, {ActionType::Forward, 1U}),
            scoreFor(current_target, Action::pause()));

  HeuristicAdvisor curiosity({"curiosity",
      HeuristicObjective::Curiosity, actions, 1.0});
  const auto lifetime = curiosity.evaluate({world}, candidates);
  EXPECT_DOUBLE_EQ(scoreFor(lifetime, {ActionType::Forward, 1U}),
                   scoreFor(lifetime, Action::pause()));
}

TEST(CommonsenseAdvisors, EnfiladeReturnsAndVisualScanAvoidsSeenHeadings) {
  using semaforr::decision::HeuristicAdvisor;
  using semaforr::decision::HeuristicObjective;
  using semaforr::domain::Action;
  using semaforr::domain::ActionType;
  const semaforr::domain::ActionSpace actions(
      {2.0}, {1.5707963267948966});
  auto world = worldWithTarget();
  auto view = laser();
  view.angle_min = semaforr::domain::Angle(-0.5);
  view.ranges_m = std::vector<double>(11U, 5.0);
  world.robot.laser = view;
  auto historical_view = view;
  world.navigation_history.record(
      {{{1.0, 0.0}, semaforr::domain::Angle::zero()}, historical_view,
       Action::pause(), 0U});
  world.navigation_history.record(
      {{{1.25, 0.0}, semaforr::domain::Angle(1.5707963267948966)},
       historical_view, Action::pause(), 0U});

  HeuristicAdvisor enfilade({"enfilade",
      HeuristicObjective::Enfilade, actions, 1.0});
  const std::vector<Action> movement{
      Action::pause(), {ActionType::Forward, 1U}};
  const auto returning = enfilade.evaluate({world}, movement);
  EXPECT_GT(scoreFor(returning, {ActionType::Forward, 1U}),
            scoreFor(returning, Action::pause()));

  HeuristicAdvisor scan({"visual_scan",
      HeuristicObjective::VisualScan, actions, 1.0});
  EXPECT_EQ(scan.metadata().scored_action_types,
            (std::vector<ActionType>{ActionType::TurnLeft,
                                     ActionType::TurnRight}));
  const std::vector<Action> rotations{
      {ActionType::TurnLeft, 1U}, {ActionType::TurnRight, 1U}};
  const auto scanning = scan.evaluate({world}, rotations);
  EXPECT_GT(scoreFor(scanning, {ActionType::TurnRight, 1U}),
            scoreFor(scanning, {ActionType::TurnLeft, 1U}));
}

TEST(SpatialAdvisors, EnterAbstainsOnceRobotIsInsideObjectiveRegion) {
  using namespace semaforr;
  const domain::ActionSpace actions({1.0}, {0.5});
  auto world = worldWithTarget({0.5, 0.0});
  world.spatial.learned_regions.push_back(
      {{0.0, 0.0}, domain::Distance(1.0)});
  decision::HeuristicAdvisor enter(
      {"enter", decision::HeuristicObjective::Enter, actions, 1.0});
  const std::vector<domain::Action> candidates{
      domain::Action::pause(), {domain::ActionType::Forward, 1U}};
  EXPECT_FALSE(enter.evaluate({world}, candidates).participated);
}

TEST(SpatialAdvisors, AccessRequiresLearnedDoorsNotSensorOpenings) {
  using namespace semaforr;
  const domain::ActionSpace actions({1.0}, {0.5});
  auto world = worldWithTarget();
  world.spatial.regions.push_back(learnedRegion(1U, {1.0, 0.0}, 1.0));
  world.spatial.doorways.push_back({{1.0, -0.5}, {1.0, 0.5}});
  decision::HeuristicAdvisor access(
      {"access", decision::HeuristicObjective::Access, actions, 1.0});
  const std::vector<domain::Action> candidates{
      domain::Action::pause(), {domain::ActionType::Forward, 1U}};
  EXPECT_FALSE(access.evaluate({world}, candidates).participated);
  world.spatial.doors.push_back(learnedDoor(1U, 1U));
  EXPECT_TRUE(access.evaluate({world}, candidates).participated);
}

TEST(SpatialAdvisors, ConveyEnterExitAndTrailerUseTheirStructures) {
  using O = semaforr::decision::HeuristicObjective;
  using semaforr::domain::Action;
  using semaforr::domain::ActionType;
  const semaforr::domain::ActionSpace actions({1.0}, {1.5707963267948966});
  const std::vector<Action> candidates{
      Action::pause(), {ActionType::Forward, 1U}};

  auto convey_world = worldWithTarget({5.0, 0.0});
  convey_world.spatial.conveyor_grid.geometry = semaforr::domain::GridGeometry(
      5U, 1U, 1.0, {0.0, -0.5});
  convey_world.spatial.conveyor_grid.cells = {{2U, 10U, 0.0, 0.0, 1.0}};
  auto evaluation = spatialEvaluation(O::Convey, convey_world,
                                      actions, candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::Forward, 1U}),
            scoreFor(evaluation, Action::pause()));

  auto enter_world = worldWithTarget({2.5, 0.0});
  enter_world.spatial.learned_regions.push_back(
      {{2.0, 0.0}, semaforr::domain::Distance(1.0)});
  evaluation = spatialEvaluation(O::Enter, enter_world, actions, candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::Forward, 1U}),
            scoreFor(evaluation, Action::pause()));

  auto exit_world = worldWithTarget({5.0, 0.0});
  exit_world.spatial.learned_regions.push_back(
      {{0.0, 0.0}, semaforr::domain::Distance(1.0)});
  evaluation = spatialEvaluation(O::Exit, exit_world, actions, candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::Forward, 1U}),
            scoreFor(evaluation, Action::pause()));

  auto trail_world = worldWithTarget({5.0, 0.0});
  trail_world.spatial.trails = {{{1.0, 0.0}, {4.0, 0.0}}};
  evaluation = spatialEvaluation(O::Trailer, trail_world, actions,
                                 candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::Forward, 1U}),
            scoreFor(evaluation, Action::pause()));
}

TEST(SpatialAdvisors, UnlikelyAccessAndCrossroadsUseConnectivity) {
  using O = semaforr::decision::HeuristicObjective;
  using semaforr::domain::Action;
  using semaforr::domain::ActionType;
  const semaforr::domain::ActionSpace actions({1.0}, {1.5707963267948966});
  const std::vector<Action> candidates{
      Action::pause(), {ActionType::Forward, 1U}};

  auto unlikely_world = worldWithTarget({5.0, 0.0});
  unlikely_world.spatial.regions.push_back(
      learnedRegion(1U, {1.0, 0.0}, 1.0));
  unlikely_world.spatial.region_skeleton_nodes = {
      skeletonNode(10U, 1U, {1.0, 0.0}),
      skeletonNode(11U, 2U, {3.0, 0.0})};
  unlikely_world.spatial.region_skeleton_edges = {
      {10U, 11U, {{1.0, 0.0}, {3.0, 0.0}}, 2.0, 1U}};
  auto evaluation = spatialEvaluation(O::Unlikely, unlikely_world,
                                      actions, candidates);
  EXPECT_GT(scoreFor(evaluation, Action::pause()),
            scoreFor(evaluation, {ActionType::Forward, 1U}));

  auto access_world = worldWithTarget({5.0, 0.0});
  access_world.spatial.regions = {
      learnedRegion(1U, {2.0, 0.0}, 1.0),
      learnedRegion(2U, {-2.0, 0.0}, 1.0)};
  access_world.spatial.doors = {
      learnedDoor(1U, 1U), learnedDoor(2U, 1U),
      learnedDoor(3U, 1U), learnedDoor(4U, 2U)};
  evaluation = spatialEvaluation(O::Access, access_world, actions,
                                 candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::Forward, 1U}),
            scoreFor(evaluation, Action::pause()));

  auto crossroads_world = worldWithTarget({5.0, 0.0});
  crossroads_world.spatial.hallways = {
      {{1.0, 0.0}, {3.0, 0.0}}, {{2.0, -2.0}, {2.0, 2.0}},
      {{2.0, -2.0}, {3.0, -1.0}}};
  evaluation = spatialEvaluation(O::Crossroads, crossroads_world, actions,
                                 candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::Forward, 1U}),
            scoreFor(evaluation, Action::pause()));
}

TEST(SpatialAdvisors, FollowLeastAngleSpatialLearnerAndStayAreDirectional) {
  using O = semaforr::decision::HeuristicObjective;
  using semaforr::domain::Action;
  using semaforr::domain::ActionType;
  const semaforr::domain::ActionSpace actions({1.0}, {1.5707963267948966});
  const std::vector<Action> candidates{
      Action::pause(), {ActionType::Forward, 1U},
      {ActionType::TurnLeft, 1U}};

  auto hallway_world = worldWithTarget({5.0, 0.0});
  hallway_world.spatial.hallways = {{{0.0, 0.0}, {4.0, 0.0}}};
  auto evaluation = spatialEvaluation(O::Follow, hallway_world, actions,
                                      candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::Forward, 1U}),
            scoreFor(evaluation, {ActionType::TurnLeft, 1U}));

  auto skeleton_world = worldWithTarget({5.0, 0.0});
  skeleton_world.spatial.regions = {
      learnedRegion(1U, {0.0, 0.0}, 0.5),
      learnedRegion(2U, {1.0, 0.0}, 0.5),
      learnedRegion(3U, {0.0, 1.0}, 0.5)};
  skeleton_world.spatial.region_skeleton_nodes = {
      skeletonNode(0U, 1U, {0.0, 0.0}),
      skeletonNode(1U, 2U, {1.0, 0.0}),
      skeletonNode(2U, 3U, {0.0, 1.0})};
  skeleton_world.spatial.region_skeleton_edges = {
      {0U, 1U, {{0.0, 0.0}, {1.0, 0.0}}, 1.0, 1U},
      {0U, 2U, {{0.0, 0.0}, {0.0, 1.0}}, 1.0, 1U}};
  evaluation = spatialEvaluation(O::LeastAngle, skeleton_world, actions,
                                 candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::Forward, 1U}),
            scoreFor(evaluation, {ActionType::TurnLeft, 1U}));

  auto learner_world = worldWithTarget({5.0, 0.0});
  learner_world.robot.pose.position = {0.5, 0.5};
  learner_world.spatial.inclusion_grid =
      {3U, 1U, 1.0, {}, {2U, 0U, 0U}, 1U};
  learner_world.spatial.learned_regions.push_back(
      {{0.5, 0.5}, semaforr::domain::Distance(0.4)});
  learner_world.spatial.conveyor_grid.geometry =
      semaforr::domain::GridGeometry(3U, 1U, 1.0, {0.0, 0.0});
  learner_world.spatial.conveyor_grid.cells = {
      {0U, 5U, 0.0, 0.0, 1.0}};
  evaluation = spatialEvaluation(O::SpatialLearner, learner_world, actions,
                                 candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::Forward, 1U}),
            scoreFor(evaluation, Action::pause()));

  evaluation = spatialEvaluation(O::Stay, hallway_world, actions, candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::Forward, 1U}),
            scoreFor(evaluation, {ActionType::TurnLeft, 1U}));
}

TEST(TierThreeAdvisors, PlanSensitiveAdvisorsUseActiveLocalObjective) {
  using semaforr::decision::ActivePlanObjective;
  using semaforr::decision::HeuristicAdvisor;
  using semaforr::decision::HeuristicObjective;
  using semaforr::domain::Action;
  using semaforr::domain::ActionType;
  const semaforr::domain::ActionSpace actions({1.0},
                                               {1.5707963267948966});
  const std::vector<Action> candidates{{ActionType::Forward, 1U},
                                       {ActionType::TurnLeft, 1U},
                                       {ActionType::TurnRight, 1U}};
  auto world = worldWithTarget({10.0, 0.0});
  const ActivePlanObjective local{{0.0, 5.0}, "region", 7U, 2U};

  HeuristicAdvisor greedy(
      {"greedy", HeuristicObjective::Greedy, actions, 1.0});
  auto evaluation = greedy.evaluate({world, &actions, candidates, local},
                                    candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::TurnLeft, 1U}),
            scoreFor(evaluation, {ActionType::TurnRight, 1U}));

  world.spatial.learned_regions = {
      {{0.0, 5.0}, semaforr::domain::Distance(1.0)}};
  HeuristicAdvisor enter(
      {"enter", HeuristicObjective::Enter, actions, 1.0});
  EXPECT_TRUE(enter.evaluate({world, &actions, candidates, local}, candidates)
                  .participated);
  EXPECT_FALSE(enter.evaluate({world}, candidates).participated);

  world.spatial.learned_regions = {
      {{0.0, 0.0}, semaforr::domain::Distance(1.0)}};
  const ActivePlanObjective local_inside{{0.0, 0.5}, "region", 7U, 2U};
  HeuristicAdvisor exit({"exit", HeuristicObjective::Exit, actions, 1.0});
  EXPECT_FALSE(
      exit.evaluate({world, &actions, candidates, local_inside}, candidates)
          .participated);
  EXPECT_TRUE(exit.evaluate({world}, candidates).participated);

  world.spatial.trails = {{{0.0, 0.0}, {0.0, 5.0}},
                          {{0.0, 0.0}, {2.0, 0.0}}};
  HeuristicAdvisor trailer(
      {"trailer", HeuristicObjective::Trailer, actions, 1.0});
  evaluation = trailer.evaluate({world, &actions, candidates, local},
                                candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::TurnLeft, 1U}),
            scoreFor(evaluation, {ActionType::TurnRight, 1U}));

  world.spatial.hallways = {{{0.0, 0.0}, {0.0, 5.0}},
                            {{0.0, 0.0}, {10.0, 0.0}}};
  HeuristicAdvisor follow(
      {"follow", HeuristicObjective::Follow, actions, 1.0});
  evaluation = follow.evaluate({world, &actions, candidates, local},
                               candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::TurnLeft, 1U}),
            scoreFor(evaluation, {ActionType::TurnRight, 1U}));

  world.spatial.regions = {
      learnedRegion(1U, {0.0, 0.0}, 1.0),
      learnedRegion(2U, {1.0, 0.0}, 0.5),
      learnedRegion(3U, {0.0, 1.0}, 0.5)};
  world.spatial.region_skeleton_nodes = {
      skeletonNode(0U, 1U, {0.0, 0.0}),
      skeletonNode(1U, 2U, {1.0, 0.0}),
      skeletonNode(2U, 3U, {0.0, 1.0})};
  world.spatial.region_skeleton_edges = {
      {0U, 1U, {{0.0, 0.0}, {1.0, 0.0}}, 1.0, 1U},
      {0U, 2U, {{0.0, 0.0}, {0.0, 1.0}}, 1.0, 1U}};
  HeuristicAdvisor least_angle(
      {"least_angle", HeuristicObjective::LeastAngle, actions, 1.0});
  evaluation = least_angle.evaluate({world, &actions, candidates, local},
                                    candidates);
  EXPECT_GT(scoreFor(evaluation, {ActionType::TurnLeft, 1U}),
            scoreFor(evaluation, {ActionType::Forward, 1U}));
}

TEST(ReactivePlanners, ThruBehindAndOutHaveExplicitDependencies) {
  const semaforr::domain::ActionSpace actions(
      {0.25, 0.8}, {0.2, 1.5707963267948966});
  auto world = worldWithTarget({0.4, 0.0});
  auto tight_view = laser();
  tight_view.angle_min = semaforr::domain::Angle(-0.3);
  tight_view.ranges_m = {2.0, 2.0, 0.5, 0.5, 0.5, 2.0, 2.0};
  world.robot.laser = tight_view;
  semaforr::planning::Thru thru;
  auto result = thru.evaluate({world, actions});
  ASSERT_EQ(result.status, semaforr::planning::ReactiveStatus::Action);
  EXPECT_TRUE(result.action.has_value());
  EXPECT_FALSE(thru.dependencies().empty());

  world = worldWithTarget({-1.0, 0.0});
  auto forward_view = laser();
  forward_view.angle_min = semaforr::domain::Angle(-0.5);
  world.robot.laser = forward_view;
  world.navigation_history.record(
      {world.robot.pose, forward_view, semaforr::domain::Action::pause()});
  semaforr::planning::Behind behind;
  result = behind.evaluate({world, actions});
  ASSERT_EQ(result.status, semaforr::planning::ReactiveStatus::Action);
  EXPECT_EQ(result.action->type(), semaforr::domain::ActionType::TurnRight);
  EXPECT_EQ(result.action->magnitude_index(), 2U);

  world.recovery.confined = true;
  world.spatial.known_grid = {3U, 1U, 1.0, {}, {5U, 4U, 0U}, 1U};
  semaforr::planning::Out out;
  result = out.evaluate({world, actions});
  EXPECT_EQ(result.status, semaforr::planning::ReactiveStatus::Action);
  EXPECT_EQ(result.action->type(), semaforr::domain::ActionType::TurnRight);
  EXPECT_EQ(result.action->magnitude_index(), 2U);
}

TEST(VictoryRule, RequiresThreeOfFiveTargetRaysToBeClear) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({2.0, 0.0});
  auto view = laser();
  view.angle_min = semaforr::domain::Angle(-0.2);
  view.ranges_m = {0.5, 0.5, 2.5, 0.5, 0.5};
  world.robot.laser = view;
  semaforr::decision::VictoryRule victory(semaforr::domain::Distance(0.2),
                                           actions);
  EXPECT_FALSE(victory.evaluate({world}).has_value());
  view.ranges_m = {0.5, 2.5, 2.5, 2.5, 0.5};
  world.robot.laser = view;
  EXPECT_TRUE(victory.evaluate({world}).has_value());
}

TEST(NotOppositeRule, VetoesRotationsBackToEitherRecentOrientation) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.5});
  auto world = worldWithTarget();
  world.navigation_history.record(
      {{{0.0, 0.0}, semaforr::domain::Angle::zero()}, laser(),
       semaforr::domain::Action::pause()});
  world.navigation_history.record(
      {{{0.0, 0.0}, semaforr::domain::Angle(0.5)}, laser(),
       semaforr::domain::Action(
           semaforr::domain::ActionType::TurnLeft, 1U)});
  world.robot.pose.heading = semaforr::domain::Angle(1.0);
  const semaforr::decision::NotOppositeRule rule(actions);
  const auto vetoes = rule.evaluate({world});
  ASSERT_EQ(vetoes.size(), 1U);
  EXPECT_EQ(vetoes.front().action.type(),
            semaforr::domain::ActionType::TurnRight);
}

TEST(Behind, DoesNotRepeatAQuarterTurn) {
  const semaforr::domain::ActionSpace actions(
      {0.25}, {1.5707963267948966});
  auto world = worldWithTarget({-1.0, 0.0});
  auto view = laser();
  view.angle_min = semaforr::domain::Angle(-0.5);
  world.robot.laser = view;
  semaforr::domain::NavigationHistoryEntry quarter_turn{
      world.robot.pose, view,
      semaforr::domain::Action(
          semaforr::domain::ActionType::TurnRight, 1U)};
  quarter_turn.execution_status =
      semaforr::domain::ExecutionCompletionStatus::Succeeded;
  quarter_turn.rotation_achieved_rad = 1.5707963267948966;
  world.navigation_history.record(std::move(quarter_turn));
  semaforr::planning::Behind behind;
  EXPECT_EQ(behind.evaluate({world, actions}).status,
            semaforr::planning::ReactiveStatus::NotApplicable);
}

TEST(Thru, RequiresAVisibleCueAndBlockedForwardMove) {
  const semaforr::domain::ActionSpace actions(
      {0.25, 0.8}, {0.2, 1.5707963267948966});
  auto world = worldWithTarget({0.4, 0.0});
  auto open = laser();
  open.angle_min = semaforr::domain::Angle(-0.3);
  open.ranges_m = std::vector<double>(7U, 2.0);
  world.robot.laser = open;
  semaforr::planning::Thru thru;
  EXPECT_EQ(thru.evaluate({world, actions}).status,
            semaforr::planning::ReactiveStatus::NotApplicable);
}

TEST(Thru, SensedObjectiveRequiresBeamEvidenceAndNarrowCorridor) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.25, 0.8}, {0.2, 0.5});
  const std::vector<domain::Action> rotations{
      domain::Action::pause(), {domain::ActionType::TurnLeft, 1U},
      {domain::ActionType::TurnRight, 1U}};

  // The target is inside sensor range and four nearby beams extend beyond it,
  // but its location is almost one metre off the closest ray's narrow
  // corridor. A range-membership-only implementation would trigger here.
  auto world = worldWithTarget(
      {4.0 * std::cos(0.25), 4.0 * std::sin(0.25)});
  auto view = laser();
  view.angle_min = domain::Angle(-1.5);
  view.angle_increment = domain::Angle(0.5);
  view.ranges_m = std::vector<double>(7U, 5.0);
  view.ranges_m[3] = 0.4;
  world.robot.laser = view;
  planning::Thru corridor;
  const auto corridor_trigger = corridor.evaluateTrigger(
      {world, &actions, rotations});
  EXPECT_FALSE(corridor_trigger.triggered);
  EXPECT_EQ(corridor_trigger.rationale,
            "thru:target_and_waypoint_not_sensed");

  // A geometrically aligned objective still requires at least three clear
  // rays in the configured local neighborhood.
  world = worldWithTarget({2.0, 0.0});
  view.angle_min = domain::Angle(-0.3);
  view.angle_increment = domain::Angle(0.1);
  view.ranges_m = {0.4, 0.4, 3.0, 0.4, 3.0, 0.4, 0.4};
  world.robot.laser = view;
  planning::Thru beam_evidence;
  const auto evidence_trigger = beam_evidence.evaluateTrigger(
      {world, &actions, rotations});
  EXPECT_FALSE(evidence_trigger.triggered);
  EXPECT_EQ(evidence_trigger.rationale,
            "thru:target_and_waypoint_not_sensed");
}

TEST(Thru, RequiresObstacleBlockedAndPostVetoForwardUnavailable) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.25, 0.8}, {0.2, 0.5});
  auto world = worldWithTarget({0.4, 0.0});
  auto view = laser();
  view.angle_min = domain::Angle(-0.5);
  view.angle_increment = domain::Angle(0.1);
  view.ranges_m = std::vector<double>(11U, 2.0);
  view.ranges_m[5] = 0.5;
  world.robot.laser = view;
  const std::vector<domain::Action> with_forward{
      domain::Action::pause(), {domain::ActionType::Forward, 1U},
      {domain::ActionType::TurnLeft, 1U},
      {domain::ActionType::TurnRight, 1U}};
  const std::vector<domain::Action> without_forward{
      domain::Action::pause(), {domain::ActionType::TurnLeft, 1U},
      {domain::ActionType::TurnRight, 1U}};
  planning::Thru thru;
  const auto still_viable =
      thru.evaluateTrigger({world, &actions, with_forward});
  EXPECT_FALSE(still_viable.triggered);
  EXPECT_EQ(still_viable.rationale,
            "thru:forward_action_still_viable");
  const auto vetoed =
      thru.evaluateTrigger({world, &actions, without_forward});
  EXPECT_TRUE(vetoed.triggered);
  EXPECT_EQ(vetoed.rationale,
            "thru:sensed_target_forward_obstacle_blocked");

  view.ranges_m = std::vector<double>(11U, 2.0);
  world.robot.laser = view;
  const auto open = thru.evaluateTrigger({world, &actions, without_forward});
  EXPECT_FALSE(open.triggered);
  EXPECT_EQ(open.rationale, "thru:forward_not_obstacle_blocked");
}

TEST(Thru, UsesTargetThenWaypointAndChoosesLongerAverageRayBundle) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.25, 0.8}, {0.2, 0.5});
  auto asymmetric_view = laser();
  asymmetric_view.angle_min = domain::Angle(-0.5);
  asymmetric_view.angle_increment = domain::Angle(0.1);
  asymmetric_view.ranges_m = {1.0, 1.0, 1.0, 1.0, 1.0, 0.5,
                              4.0, 4.0, 4.0, 4.0, 4.0};

  auto world = worldWithTarget({0.4, 0.0});
  world.mission.install_active_plan({{0.0, 0.4}});
  world.robot.laser = asymmetric_view;
  planning::Thru target_thru;
  auto result = target_thru.evaluate({world, actions});
  ASSERT_EQ(result.status, planning::ReactiveStatus::Action);
  ASSERT_TRUE(result.action);
  EXPECT_EQ(result.action->type(), domain::ActionType::TurnLeft);
  EXPECT_NE(result.explanation.find("pursue_left_opening_for_target"),
            std::string::npos);

  world = worldWithTarget({0.0, 4.0});
  world.mission.install_active_plan({{0.4, 0.0}});
  world.robot.laser = asymmetric_view;
  planning::Thru waypoint_thru;
  result = waypoint_thru.evaluate({world, actions});
  ASSERT_EQ(result.status, planning::ReactiveStatus::Action);
  EXPECT_NE(result.explanation.find("pursue_left_opening_for_waypoint"),
            std::string::npos);

  std::reverse(asymmetric_view.ranges_m.begin(),
               asymmetric_view.ranges_m.end());
  world.robot.laser = asymmetric_view;
  planning::Thru right_thru;
  result = right_thru.evaluate({world, actions});
  ASSERT_EQ(result.status, planning::ReactiveStatus::Action);
  ASSERT_TRUE(result.action);
  EXPECT_EQ(result.action->type(), domain::ActionType::TurnRight);
  EXPECT_NE(result.explanation.find("pursue_right_opening_for_waypoint"),
            std::string::npos);
}

TEST(Thru, StopsOnDecisionLimitInvalidPursuitAndCancellation) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.25, 0.8}, {0.2, 0.5});
  auto world = worldWithTarget({0.4, 0.0});
  auto view = laser();
  view.angle_min = domain::Angle(-0.5);
  view.angle_increment = domain::Angle(0.1);
  view.ranges_m = {1.0, 1.0, 1.0, 1.0, 1.0, 0.5,
                   4.0, 4.0, 4.0, 4.0, 4.0};
  world.robot.laser = view;

  planning::Thru reached(20U, 0.8, 10.0);
  const auto reached_result = reached.evaluate({world, actions});
  EXPECT_EQ(reached_result.status, planning::ReactiveStatus::NotApplicable);
  EXPECT_EQ(reached_result.completion_reason,
            planning::ReactiveCompletionReason::CandidateExhausted);
  EXPECT_EQ(reached_result.explanation, "thru:endpoint_reached");

  planning::Thru budgeted(1U);
  ASSERT_EQ(budgeted.evaluate({world, actions}).status,
            planning::ReactiveStatus::Action);
  const auto budget = budgeted.evaluate({world, actions});
  EXPECT_EQ(budget.status, planning::ReactiveStatus::NotApplicable);
  EXPECT_EQ(budget.completion_reason,
            planning::ReactiveCompletionReason::BudgetExceeded);
  EXPECT_EQ(budget.explanation, "thru:decision_limit_reached");

  planning::Thru invalid;
  ASSERT_EQ(invalid.evaluate({world, actions}).status,
            planning::ReactiveStatus::Action);
  const std::vector<domain::Action> incompatible{
      domain::Action::pause(), {domain::ActionType::TurnRight, 1U}};
  const auto invalid_result = invalid.evaluate({world, actions, incompatible});
  EXPECT_EQ(invalid_result.status, planning::ReactiveStatus::NotApplicable);
  EXPECT_EQ(invalid_result.completion_reason,
            planning::ReactiveCompletionReason::CandidateExhausted);
  EXPECT_EQ(invalid_result.explanation,
            "thru:pursuit_action_not_viable");

  planning::Thru cancelled;
  ASSERT_EQ(cancelled.evaluate({world, actions}).status,
            planning::ReactiveStatus::Action);
  cancelled.cancel(planning::InterruptionReason::TargetSensed);
  view.ranges_m = std::vector<double>(11U, 4.0);
  world.robot.laser = view;
  EXPECT_EQ(cancelled.evaluate({world, actions}).status,
            planning::ReactiveStatus::NotApplicable);
}

TEST(LowLevelExplorer, RequestsTierTwoReplanAfterFailedProgress) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({4.0, 0.0});
  world.robot.laser = laser();
  for (std::size_t index = 0U; index < 4U; ++index)
    world.mission.record_decision();
  for (std::size_t index = 0U; index < 4U; ++index)
    world.navigation_history.record(
        {{{0.01 * static_cast<double>(index), 0.0},
          semaforr::domain::Angle::zero()},
         laser(),
         semaforr::domain::Action(semaforr::domain::ActionType::Forward, 1U)});
  semaforr::planning::LowLevelExplorer explorer(4U, 0.1);
  const auto pursuing = explorer.evaluate({world, actions});
  EXPECT_EQ(pursuing.status, semaforr::planning::ReactiveStatus::Action);
  world.spatial.skeleton_nodes = {{0.0, 0.0}, {1.0, 0.0}};
  world.spatial.skeleton_edges = {{0U, 1U}};
  ++world.spatial.revisions[semaforr::domain::ModelDependency::Skeleton];
  const auto result = explorer.evaluate({world, actions});
  EXPECT_EQ(result.status, semaforr::planning::ReactiveStatus::RequestReplan);
  EXPECT_EQ(result.planner, "LLE");
  EXPECT_EQ(result.completion_reason,
            semaforr::planning::ReactiveCompletionReason::NewPlanAvailable);
}

TEST(LowLevelExplorer, AssemblesValidCueSourcesAndSupportsCancellation) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({10.0, 0.0});
  world.mission.install_active_plan({});
  world.recovery.planning_attempted = true;
  world.robot.laser = laser();
  world.robot.laser->ranges_m = {5.0, 5.0, 5.0};
  world.spatial.unfinished_hle_candidates.push_back(
      {42U, {0.0, 0.0}, {3.5, 0.0}});
  semaforr::domain::LearnedRegion region;
  region.id = 1U;
  region.boundary = {{3.5, 1.0}, semaforr::domain::Distance(0.5)};
  region.visibility[0] = {true, 6.0, {3.5, 1.0}, {9.5, 0.0}, 1U};
  world.spatial.regions.push_back(region);
  world.spatial.inclusion_grid =
      {2U, 1U, 1.0, {}, {0U, 1U}, 1U};
  semaforr::planning::LowLevelExplorer explorer;
  EXPECT_EQ(explorer.evaluate({world, actions}).status,
            semaforr::planning::ReactiveStatus::Action);
  std::vector<semaforr::planning::LLECandidateSource> sources;
  for (const auto& candidate : explorer.candidates())
    sources.push_back(candidate.source);
  EXPECT_NE(std::find(sources.begin(), sources.end(),
                      semaforr::planning::LLECandidateSource::UnfinishedHle),
            sources.end());
  EXPECT_NE(
      std::find(sources.begin(), sources.end(),
                semaforr::planning::LLECandidateSource::
                    CurrentTargetObservation),
      sources.end());
  EXPECT_NE(std::find(sources.begin(), sources.end(),
                      semaforr::planning::LLECandidateSource::RegionVisibility),
            sources.end());
  EXPECT_EQ(std::find(sources.begin(), sources.end(),
                      semaforr::planning::LLECandidateSource::InclusionGap),
            sources.end());
  explorer.cancel(semaforr::planning::InterruptionReason::SensorLost);
  EXPECT_EQ(explorer.state(),
            semaforr::planning::LowLevelExplorationState::Complete);
  EXPECT_EQ(explorer.completionReason(),
            semaforr::planning::ReactiveCompletionReason::SensorLost);
}

TEST(LowLevelExplorer, CompatibilityUsesOnlyPublishedPlanFailureTriggers) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({10.0, 0.0});
  world.robot.laser = laser();
  for (std::size_t index = 0U; index < 4U; ++index)
    world.navigation_history.record(
        {{{0.01 * static_cast<double>(index), 0.0},
          semaforr::domain::Angle::zero()},
         laser(),
         semaforr::domain::Action(semaforr::domain::ActionType::Forward, 1U)});
  semaforr::planning::LowLevelExplorationConfiguration configuration;
  configuration.behavior_policy =
      semaforr::planning::LLEBehaviorPolicy::Compatibility;
  configuration.stalled_history_extension = false;
  semaforr::planning::LowLevelExplorer explorer(configuration);
  EXPECT_FALSE(explorer.evaluateTrigger({world}).triggered);
  EXPECT_EQ(explorer.lastTriggerReasonCode(), "none");

  world.mission.install_active_plan({});
  world.recovery.planning_attempted = true;
  EXPECT_TRUE(explorer.evaluateTrigger({world}).triggered);
  EXPECT_EQ(explorer.lastTriggerReasonCode(), "no_plan_available");

  world.mission.install_active_plan({{1.0, 0.0}});
  world.mission.advance_waypoint(
      {{1.0, 0.0}, semaforr::domain::Angle::zero()},
      semaforr::domain::Distance(0.1));
  world.recovery.plan_available = false;
  world.recovery.completed_plan_failed_target = true;
  EXPECT_TRUE(explorer.evaluateTrigger({world}).triggered);
  EXPECT_EQ(explorer.lastTriggerReasonCode(),
            "completed_plan_failed_target");
}

TEST(LowLevelExplorer, RegionCentersDoNotSubstituteForVisibilityRays) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({10.0, 0.0});
  world.mission.install_active_plan({});
  world.robot.laser = laser();
  world.spatial.learned_regions.push_back(
      {{9.0, 0.0}, semaforr::domain::Distance(0.5)});
  semaforr::planning::LowLevelExplorer explorer;
  ASSERT_EQ(explorer.evaluate({world, actions}).status,
            semaforr::planning::ReactiveStatus::Action);
  EXPECT_TRUE(std::none_of(
      explorer.candidates().begin(), explorer.candidates().end(),
      [](const auto& candidate) {
        return candidate.source ==
               semaforr::planning::LLECandidateSource::RegionVisibility;
      }));
}

TEST(LowLevelExplorer, CompatibilityFallbackIsSeededWithinClosestBin) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({10.0, 0.0});
  world.mission.install_active_plan({});
  world.robot.laser = laser();
  world.robot.laser->angle_min = semaforr::domain::Angle(-0.1);
  semaforr::planning::LowLevelExplorationConfiguration configuration;
  configuration.behavior_policy =
      semaforr::planning::LLEBehaviorPolicy::Compatibility;
  configuration.stalled_history_extension = false;
  configuration.closest_target_bin_m = 1.0;
  configuration.random_seed = 7U;
  semaforr::planning::LowLevelExplorer first(configuration);
  semaforr::planning::LowLevelExplorer second(configuration);
  ASSERT_EQ(first.evaluate({world, actions}).status,
            semaforr::planning::ReactiveStatus::Action);
  ASSERT_EQ(second.evaluate({world, actions}).status,
            semaforr::planning::ReactiveStatus::Action);
  ASSERT_EQ(first.candidates().size(), 1U);
  ASSERT_EQ(second.candidates().size(), 1U);
  EXPECT_EQ(first.candidates().front().target,
            second.candidates().front().target);
  const double selected_distance = semaforr::domain::distance(
                                       first.candidates().front().target,
                                       world.mission.active()->target)
                                       .meters();
  EXPECT_EQ(static_cast<int>(std::floor(selected_distance)), 8);
}

TEST(LowLevelExplorer, PlansToCueStartBeforeInstallingTwentyWaypoints) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.25}, {0.2});
  const auto make_world = [](domain::Point2D start) {
    auto world = worldWithTarget({20.0, 0.0});
    world.mission.install_active_plan({});
    world.recovery.planning_attempted = true;
    world.robot.laser = laser();
    world.spatial.unfinished_hle_candidates.push_back(
        {42U, start, {start.x_m + 3.0, start.y_m}});
    return world;
  };

  auto already = make_world({0.0, 0.0});
  planning::LowLevelExplorer at_start;
  ASSERT_EQ(at_start.evaluate({already, actions}).status,
            planning::ReactiveStatus::Action);
  EXPECT_EQ(at_start.candidateStartPlanOutcome(),
            planning::CandidateStartPlanOutcome::AlreadySatisfied);
  EXPECT_EQ(at_start.cueWaypoints().size(), 20U);

  auto nearby = make_world({1.0, 0.0});
  planning::LowLevelExplorer direct;
  ASSERT_EQ(direct.evaluate({nearby, actions}).status,
            planning::ReactiveStatus::Action);
  EXPECT_EQ(direct.candidateStartPlanOutcome(),
            planning::CandidateStartPlanOutcome::Succeeded);
  EXPECT_EQ(direct.candidateStartPlanReason(),
            "candidate_start_direct_visibility_plan");
  EXPECT_TRUE(direct.cueWaypoints().empty());

  auto distant = make_world({4.5, 0.5});
  distant.spatial.inclusion_grid =
      {6U, 2U, 1.0, {0.0, 0.0}, std::vector<std::uint32_t>(12U, 1U), 3U};
  distant.spatial.revisions[domain::ModelDependency::Inclusion] = 3U;
  planning::LowLevelExplorer routed;
  ASSERT_EQ(routed.evaluate({distant, actions}).status,
            planning::ReactiveStatus::Action);
  EXPECT_EQ(routed.candidateStartPlanReason(),
            "candidate_start_inclusion_plan");
  EXPECT_FALSE(routed.candidateStartPlan().empty());
  EXPECT_TRUE(routed.cueWaypoints().empty());

  auto unreachable = make_world({4.5, 0.5});
  unreachable.spatial.inclusion_grid =
      {6U, 2U, 1.0, {0.0, 0.0}, std::vector<std::uint32_t>(12U, 0U), 1U};
  planning::LowLevelExplorer rejected;
  const auto failed = rejected.evaluate({unreachable, actions});
  EXPECT_EQ(failed.completion_reason,
            planning::ReactiveCompletionReason::CandidateExhausted);
  EXPECT_NE(std::find(rejected.candidateStartDiagnostics().begin(),
                      rejected.candidateStartDiagnostics().end(),
                      "candidate_start_unreachable"),
            rejected.candidateStartDiagnostics().end());
}

TEST(LowLevelExplorer, InvalidatedCueStartPlanDiscardsTheCue) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({20.0, 0.0});
  world.mission.install_active_plan({});
  world.recovery.planning_attempted = true;
  world.robot.laser = laser();
  world.spatial.unfinished_hle_candidates.push_back(
      {42U, {4.5, 0.5}, {7.5, 0.5}});
  world.spatial.inclusion_grid =
      {6U, 2U, 1.0, {0.0, 0.0}, std::vector<std::uint32_t>(12U, 1U), 1U};
  planning::LowLevelExplorer explorer;
  ASSERT_EQ(explorer.evaluate({world, actions}).status,
            planning::ReactiveStatus::Action);
  ASSERT_FALSE(explorer.candidateStartPlan().empty());
  world.spatial.sensed_occupancy.geometry =
      domain::GridGeometry(6U, 2U, 1.0, {0.0, 0.0});
  world.spatial.sensed_occupancy.cells.resize(12U);
  const auto blocked = world.spatial.sensed_occupancy.geometry.index(
      explorer.candidateStartPlan().front());
  ASSERT_TRUE(blocked);
  world.spatial.sensed_occupancy.cells[*blocked].state =
      domain::SensedOccupancyState::ObservedOccupied;
  const auto invalidated = explorer.evaluate({world, actions});
  EXPECT_EQ(invalidated.completion_reason,
            planning::ReactiveCompletionReason::CandidateExhausted);
  EXPECT_NE(std::find(explorer.candidateStartDiagnostics().begin(),
                      explorer.candidateStartDiagnostics().end(),
                      "candidate_start_plan_invalidated"),
            explorer.candidateStartDiagnostics().end());
}

TEST(LowLevelExplorer, AllCoveredRaysRelocateToClosestIncludedTargetCell) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({20.0, 0.0});
  world.mission.install_active_plan({});
  world.recovery.planning_attempted = true;
  world.robot.laser = laser();
  world.spatial.inclusion_grid =
      {10U, 3U, 1.0, {0.0, -1.0}, std::vector<std::uint32_t>(30U, 1U), 1U};
  planning::LowLevelExplorer explorer;
  const auto relocating = explorer.evaluate({world, actions});
  ASSERT_EQ(relocating.status, planning::ReactiveStatus::Action);
  ASSERT_EQ(explorer.candidates().size(), 1U);
  EXPECT_EQ(explorer.candidates().front().source,
            planning::LLECandidateSource::IncludedRelocation);
  EXPECT_NEAR(explorer.candidates().front().target.x_m, 9.5, 1e-9);
  EXPECT_EQ(explorer.candidateStartPlanReason(),
            "candidate_start_inclusion_plan");
  const auto relocation_plan = explorer.candidateStartPlan();
  ASSERT_FALSE(relocation_plan.empty());
  for (const auto point : relocation_plan) {
    world.robot.pose.position = point;
    static_cast<void>(explorer.evaluate({world, actions}));
  }
  EXPECT_NE(std::find(
                explorer.candidateStartDiagnostics().begin(),
                explorer.candidateStartDiagnostics().end(),
                "included_relocation_reached_resume_ray_exploration"),
            explorer.candidateStartDiagnostics().end());
}

TEST(LowLevelExplorer, UnreachableClosestIncludedRelocationFailsExplicitly) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({9.0, 0.0});
  world.mission.install_active_plan({});
  world.recovery.planning_attempted = true;
  world.robot.laser = laser();
  std::vector<std::uint32_t> cells(30U, 0U);
  domain::SparseCountGrid grid{10U, 3U, 1.0, {0.0, -1.0}, cells, 1U};
  const auto robot_cell = grid.extent().index(world.robot.pose.position);
  ASSERT_TRUE(robot_cell);
  grid.cells[*robot_cell] = 1U;
  for (std::size_t beam = 0U; beam < world.robot.laser->ranges_m.size();
       ++beam) {
    const double angle = world.robot.laser->angle_min.radians() +
                         static_cast<double>(beam) *
                             world.robot.laser->angle_increment.radians();
    const domain::Point2D endpoint{
        world.robot.laser->ranges_m[beam] * std::cos(angle),
        world.robot.laser->ranges_m[beam] * std::sin(angle)};
    const auto index = grid.extent().index(endpoint);
    ASSERT_TRUE(index);
    grid.cells[*index] = 1U;
  }
  const auto closest = grid.extent().index({8.5, -0.5});
  ASSERT_TRUE(closest);
  grid.cells[*closest] = 1U;
  world.spatial.inclusion_grid = std::move(grid);
  planning::LowLevelExplorer explorer;
  const auto result = explorer.evaluate({world, actions});
  EXPECT_EQ(result.completion_reason,
            planning::ReactiveCompletionReason::CandidateExhausted);
  EXPECT_NE(std::find(explorer.candidateStartDiagnostics().begin(),
                      explorer.candidateStartDiagnostics().end(),
                      "candidate_start_unreachable"),
            explorer.candidateStartDiagnostics().end());
}

TEST(LowLevelExplorer, UncoveredRayIsUsedBeforeIncludedRelocation) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({12.0, 0.0});
  world.mission.install_active_plan({});
  world.recovery.planning_attempted = true;
  world.robot.laser = laser();
  world.spatial.inclusion_grid =
      {10U, 3U, 1.0, {0.0, -1.0}, std::vector<std::uint32_t>(30U, 1U), 1U};
  const auto endpoint = world.spatial.inclusion_grid.extent().index({2.0, 0.0});
  ASSERT_TRUE(endpoint);
  world.spatial.inclusion_grid.cells[*endpoint] = 0U;
  planning::LowLevelExplorer explorer;
  ASSERT_EQ(explorer.evaluate({world, actions}).status,
            planning::ReactiveStatus::Action);
  ASSERT_EQ(explorer.candidates().size(), 1U);
  EXPECT_EQ(explorer.candidates().front().source,
            planning::LLECandidateSource::CurrentTargetObservation);
}

TEST(LowLevelExplorer, VisibleCueAbandonsCoveredRayRelocation) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({9.0, 0.0});
  world.mission.install_active_plan({});
  world.recovery.planning_attempted = true;
  world.robot.laser = laser();
  world.spatial.inclusion_grid =
      {10U, 3U, 1.0, {0.0, -1.0}, std::vector<std::uint32_t>(30U, 1U), 1U};
  planning::LowLevelExplorer explorer;
  ASSERT_EQ(explorer.evaluate({world, actions}).status,
            planning::ReactiveStatus::Action);
  std::fill(world.robot.laser->ranges_m.begin(),
            world.robot.laser->ranges_m.end(), 5.0);
  ASSERT_EQ(explorer.evaluate({world, actions}).status,
            planning::ReactiveStatus::Action);
  EXPECT_EQ(explorer.candidates().front().source,
            planning::LLECandidateSource::CurrentTargetObservation);
  EXPECT_NE(std::find(explorer.candidateStartDiagnostics().begin(),
                      explorer.candidateStartDiagnostics().end(),
                      "fallback_abandoned_for_visible_cue"),
            explorer.candidateStartDiagnostics().end());
}

TEST(TierOneRules, VictoryForwardAndNotOppositeAreTyped) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({0.1, 0.0});
  semaforr::decision::VictoryRule victory(semaforr::domain::Distance(0.2));
  EXPECT_TRUE(victory.evaluate({world}));
  auto visible = worldWithTarget();
  visible.robot.laser = laser();
  semaforr::decision::VictoryRule direct(
      semaforr::domain::Distance(0.2), actions);
  ASSERT_TRUE(direct.evaluate({visible}));
  EXPECT_EQ(direct.evaluate({visible})->action.type(),
            semaforr::domain::ActionType::Forward);
  semaforr::decision::ForwardRule forward(semaforr::domain::ActionSpace(
      {2.0}, {1.5707963267948966, 3.1415926535897932}));
  EXPECT_TRUE(forward.evaluate({visible}).empty());
  semaforr::domain::SelectedActionRecord enforced;
  enforced.task_id = visible.mission.active()->id;
  enforced.expected_start =
      {{-2.0, 0.0}, semaforr::domain::Angle::zero()};
  enforced.provenance = "mandatory_rule:Enforcer";
  visible.decision_history.record(std::move(enforced));
  EXPECT_FALSE(forward.evaluate({visible}).empty());
  world.navigation_history.record(
      {world.robot.pose, laser(),
       semaforr::domain::Action(semaforr::domain::ActionType::TurnLeft, 1U)});
  world.robot.pose.heading = semaforr::domain::Angle(0.2);
  semaforr::decision::NotOppositeRule not_opposite(actions);
  const auto vetoes = not_opposite.evaluate({world});
  ASSERT_EQ(vetoes.size(), 1U);
  EXPECT_EQ(vetoes.front().action.type(),
            semaforr::domain::ActionType::TurnRight);
}

TEST(Precedent, VetoesOnlyLowConfidenceActionsAfterEvidenceGate) {
  using namespace semaforr;
  const domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({4.0, 0.0});
  world.robot.laser = laser();

  spatial::CircumstanceLearningConfiguration learning;
  learning.setting_radius_m = 5.0;
  learning.minimum_cluster_size = 2U;
  learning.reclustering_threshold = 2U;
  learning.minimum_case_evidence = 10U;
  const domain::SettingNormalizationConfiguration normalization{
      learning.setting_resolution_m,
      learning.setting_radius_m,
      learning.assignment_confidence_threshold,
      learning.similarity_l1_threshold,
      learning.distance_bin_base_m,
      learning.angle_bin_count};
  const auto setting =
      domain::normalizeSetting(*world.robot.laser, normalization);
  auto& model = world.spatial.circumstances;
  model.minimum_cluster_size = 2U;
  model.minimum_case_evidence = 10U;
  model.assignment_confidence_threshold = 0.95;
  model.similarity_l1_threshold = 125.0;
  model.accuracy_threshold = 0.75;
  model.action_confidence_threshold = 0.25;
  model.distance_bin_base_m = 2.0;
  model.angle_bin_count = 8U;
  model.clusters.push_back({0U, setting, 50U, 1.0});
  const auto key = domain::circumstanceCaseKey(
      0U, world.robot.pose, world.mission.active()->target, model);
  const domain::Action forward(domain::ActionType::Forward, 1U);
  const domain::Action left(domain::ActionType::TurnLeft, 1U);
  domain::CircumstanceCaseEvidence case_evidence;
  case_evidence.key = key;
  case_evidence.evidence = 20U;
  case_evidence.accuracy = 0.9;
  domain::ActionCaseEvidence forward_evidence;
  forward_evidence.action = forward;
  forward_evidence.executed = 10U;
  forward_evidence.effective_evidence = 10.0;
  forward_evidence.confidence = 0.9;
  forward_evidence.accuracy = 0.9;
  domain::ActionCaseEvidence left_evidence;
  left_evidence.action = left;
  left_evidence.executed = 10U;
  left_evidence.effective_evidence = 10.0;
  left_evidence.confidence = 0.1;
  left_evidence.accuracy = 0.1;
  case_evidence.actions = {forward_evidence, left_evidence};
  model.cases.push_back(case_evidence);

  decision::PrecedentRule rule(actions, {10U, 5U, 0.95, 0.75, 0.25});
  const auto vetoes = rule.evaluate({world});
  const auto vetoed = [&](domain::Action action) {
    return std::any_of(vetoes.begin(), vetoes.end(),
                       [&](const auto& veto) { return veto.action == action; });
  };
  EXPECT_FALSE(vetoed(forward));
  EXPECT_TRUE(vetoed(left));
  EXPECT_FALSE(vetoed(domain::Action(domain::ActionType::TurnRight, 1U)));

  model.cases.front().evidence = 9U;
  EXPECT_TRUE(rule.evaluate({world}).empty());
}

TEST(TierRegistries, DeclareAndConstructTierDependencies) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  semaforr::decision::TierOneRegistry tier_one;
  semaforr::decision::AdvisorRegistry tier_three;
  semaforr::decision::registerTierFactories(tier_one, tier_three,
                                                     actions);
  EXPECT_EQ(tier_one.createMandatory("victory")->name(), "Victory");
  EXPECT_EQ(tier_one.createVeto("avoid_obstacles")->name(), "AvoidObstacles");
  EXPECT_EQ(tier_one.createVeto("not_opposite")->name(), "NotOpposite");
  EXPECT_EQ(tier_one.createOperationalizer("enforcer")->name(), "enforcer");
  EXPECT_EQ(tier_one.createReactive("thru")->name(), "Thru");
  EXPECT_EQ(tier_one.createReactive("behind")->name(), "Behind");
  EXPECT_EQ(tier_one.createReactive("out")->name(), "Out");
  EXPECT_EQ(tier_one.createReactive("low_level_exploration")->name(), "LLE");
  EXPECT_EQ(tier_one.createVeto("forward")->name(), "Forward");
  EXPECT_EQ(tier_one.createVeto("precedent")->name(), "Precedent");
  const auto highway = tier_three.create("prefer_highways");
  ASSERT_EQ(highway->dependencies().size(), 1U);
  EXPECT_EQ(highway->dependencies().front(), "highways");
}
