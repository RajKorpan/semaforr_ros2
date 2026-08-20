#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/decision/advisors/catalog_registry.hpp>
#include <semaforr/decision/advisors/heuristic_advisor.hpp>
#include <semaforr/decision/tier_registry.hpp>
#include <semaforr/planning/reactive_planner.hpp>
#include <semaforr/spatial/learners/circumstance_learner.hpp>

namespace {

semaforr::domain::WorldModel worldWithTarget(
    semaforr::domain::Point2D target = {2.0, 0.0}) {
  semaforr::domain::WorldModel world;
  world.mission =
      semaforr::domain::Mission({{0U, target}}, 20U);
  world.mission.activate_next();
  world.mission.install_active_plan({target});
  return world;
}

semaforr::domain::LaserObservation laser() {
  semaforr::domain::LaserObservation result;
  result.minimum_range = semaforr::domain::Distance(0.1);
  result.maximum_range = semaforr::domain::Distance(5.0);
  result.angle_increment = semaforr::domain::Angle(0.1);
  result.ranges_m = {2.0, 2.0, 2.0};
  return result;
}

double scoreFor(const semaforr::decision::AdvisorEvaluation& evaluation,
                const semaforr::domain::Action& action) {
  const auto found = std::find_if(
      evaluation.scores.begin(), evaluation.scores.end(),
      [&](const auto& score) { return score.action == action; });
  EXPECT_NE(found, evaluation.scores.end());
  return found == evaluation.scores.end() ? 0.0 : found->raw_score;
}

semaforr::decision::AdvisorEvaluation spatialEvaluation(
    semaforr::decision::HeuristicObjective objective,
    const semaforr::domain::WorldModel& world,
    const semaforr::domain::ActionSpace& actions,
    const std::vector<semaforr::domain::Action>& candidates) {
  semaforr::decision::HeuristicAdvisor advisor(
      {"spatial_test", objective, actions, 1.0});
  return advisor.evaluate({world}, candidates);
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
  const std::vector<Action> rotations{
      {ActionType::TurnLeft, 1U}, {ActionType::TurnRight, 1U}};
  const auto scanning = scan.evaluate({world}, rotations);
  EXPECT_GT(scoreFor(scanning, {ActionType::TurnRight, 1U}),
            scoreFor(scanning, {ActionType::TurnLeft, 1U}));
}

TEST(SpatialAdvisors, ConveyEnterExitAndTrailerUseTheirStructures) {
  using O = semaforr::decision::HeuristicObjective;
  using semaforr::domain::Action;
  using semaforr::domain::ActionType;
  const semaforr::domain::ActionSpace actions({1.0}, {1.5707963267948966});
  const std::vector<Action> candidates{
      Action::pause(), {ActionType::Forward, 1U}};

  auto convey_world = worldWithTarget({5.0, 0.0});
  convey_world.spatial.conveyor_flows = {{{2.0, -1.0}, {2.0, 1.0}}};
  convey_world.spatial.conveyor_traversals = {10U};
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
  unlikely_world.spatial.learned_regions.push_back(
      {{1.0, 0.0}, semaforr::domain::Distance(1.0)});
  auto evaluation = spatialEvaluation(O::Unlikely, unlikely_world,
                                      actions, candidates);
  EXPECT_GT(scoreFor(evaluation, Action::pause()),
            scoreFor(evaluation, {ActionType::Forward, 1U}));

  auto access_world = worldWithTarget({5.0, 0.0});
  access_world.spatial.learned_regions = {
      {{2.0, 0.0}, semaforr::domain::Distance(1.0)},
      {{-2.0, 0.0}, semaforr::domain::Distance(1.0)}};
  access_world.spatial.doorways = {
      {{2.5, -0.2}, {2.5, 0.2}}, {{2.0, 0.8}, {2.0, 1.0}},
      {{1.5, -0.2}, {1.5, 0.2}}, {{-2.5, -0.2}, {-2.5, 0.2}}};
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
  skeleton_world.spatial.skeleton_nodes = {
      {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}};
  skeleton_world.spatial.skeleton_edges = {{0U, 1U}, {0U, 2U}};
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
  learner_world.spatial.conveyor_flows =
      {{{0.0, 0.5}, {1.0, 0.5}}};
  learner_world.spatial.conveyor_traversals = {5U};
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

  world.spatial.skeleton_nodes = {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}};
  world.spatial.skeleton_edges = {{0U, 1U}, {0U, 2U}};
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
