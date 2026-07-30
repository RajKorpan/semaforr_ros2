#include <gtest/gtest.h>
#include <algorithm>
#include <semaforr/decision/advisor_catalog_registry.hpp>
#include <semaforr/decision/restored_tiers.hpp>
#include <semaforr/planning/reactive_planner.hpp>

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

}  // namespace

TEST(TierThreeCatalog, RestoresEveryDissertationAdvisorWithMetadata) {
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
  const std::vector<semaforr::domain::Action> candidates{
      semaforr::domain::Action::pause(),
      {semaforr::domain::ActionType::Forward, 1U},
      {semaforr::domain::ActionType::TurnLeft, 1U},
      {semaforr::domain::ActionType::TurnRight, 1U}};
  const auto evaluation = advisor->evaluate({world}, candidates);
  EXPECT_EQ(evaluation.model_revision_used, 17U);
  const auto dependencies = advisor->dependencies();
  EXPECT_NE(std::find(dependencies.begin(), dependencies.end(), "highways"),
            dependencies.end());
}

TEST(ReactivePlanners, ThruBehindAndOutHaveExplicitDependencies) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2, 1.0});
  auto world = worldWithTarget();
  semaforr::planning::Thru thru;
  auto result = thru.evaluate({world, actions});
  ASSERT_EQ(result.status, semaforr::planning::ReactiveStatus::Action);
  EXPECT_EQ(result.action->type(), semaforr::domain::ActionType::Forward);
  EXPECT_FALSE(thru.dependencies().empty());

  world.robot.pose.heading = semaforr::domain::Angle(3.0);
  semaforr::planning::Behind behind;
  result = behind.evaluate({world, actions});
  ASSERT_EQ(result.status, semaforr::planning::ReactiveStatus::Action);
  EXPECT_NE(result.action->type(), semaforr::domain::ActionType::Forward);

  world.robot.pose.heading = semaforr::domain::Angle::zero();
  world.recovery.confined = true;
  world.spatial.inclusion_grid = {3U, 1U, 1.0, {}, {5U, 4U, 0U}, 1U};
  semaforr::planning::Out out;
  result = out.evaluate({world, actions});
  EXPECT_EQ(result.status, semaforr::planning::ReactiveStatus::Action);
}

TEST(LowLevelExplorer, RequestsTierTwoReplanAfterFailedProgress) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({10.0, 0.0});
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
  ++world.spatial.revision;
  const auto result = explorer.evaluate({world, actions});
  EXPECT_EQ(result.status, semaforr::planning::ReactiveStatus::RequestReplan);
  EXPECT_EQ(result.planner, "LLE");
  EXPECT_EQ(result.completion_reason,
            semaforr::planning::ReactiveCompletionReason::NewPlanAvailable);
}

TEST(LowLevelExplorer, AssemblesEveryCandidateSourceAndSupportsCancellation) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({10.0, 0.0});
  world.robot.laser = laser();
  world.spatial.unfinished_hle_candidates.push_back(
      {42U, {0.0, 0.0}, {1.0, 1.0}});
  world.spatial.learned_regions.push_back({{2.0, 1.0},
                                           semaforr::domain::Distance(0.5)});
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
  EXPECT_NE(std::find(sources.begin(), sources.end(),
                      semaforr::planning::LLECandidateSource::InclusionGap),
            sources.end());
  explorer.cancel(semaforr::planning::InterruptionReason::SensorLost);
  EXPECT_EQ(explorer.state(),
            semaforr::planning::LowLevelExplorationState::Complete);
  EXPECT_EQ(explorer.completionReason(),
            semaforr::planning::ReactiveCompletionReason::SensorLost);
}

TEST(RestoredTierOne, VictoryForwardAndNotOppositeAreTyped) {
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
  semaforr::decision::ForwardRule forward(
      semaforr::domain::ActionSpace({0.25}, {2.0}));
  EXPECT_FALSE(forward.evaluate({visible}).empty());
  world.navigation_history.record(
      {world.robot.pose, laser(),
       semaforr::domain::Action(semaforr::domain::ActionType::TurnLeft, 1U)});
  semaforr::decision::NotOppositeRule not_opposite;
  const auto vetoes = not_opposite.evaluate({world});
  ASSERT_EQ(vetoes.size(), 1U);
  EXPECT_EQ(vetoes.front().action.type(),
            semaforr::domain::ActionType::TurnRight);
}

TEST(RestoredRegistries, DeclareAndConstructTierDependencies) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  semaforr::decision::TierOneRegistry tier_one;
  semaforr::decision::AdvisorRegistry tier_three;
  semaforr::decision::registerRestoredTierFactories(tier_one, tier_three,
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
