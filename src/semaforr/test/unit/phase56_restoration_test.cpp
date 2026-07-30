#include <gtest/gtest.h>
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
  auto world = worldWithTarget();
  for (std::size_t index = 0U; index < 4U; ++index)
    world.mission.record_decision();
  for (std::size_t index = 0U; index < 4U; ++index)
    world.navigation_history.record(
        {{{0.01 * static_cast<double>(index), 0.0},
          semaforr::domain::Angle::zero()},
         laser(),
         semaforr::domain::Action(semaforr::domain::ActionType::Forward, 1U)});
  const auto result =
      semaforr::planning::LowLevelExplorer(4U, 0.1).evaluate({world, actions});
  EXPECT_EQ(result.status, semaforr::planning::ReactiveStatus::RequestReplan);
  EXPECT_EQ(result.planner, "LLE");
}

TEST(RestoredTierOne, VictoryForwardAndNotOppositeAreTyped) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.2});
  auto world = worldWithTarget({0.1, 0.0});
  semaforr::decision::VictoryRule victory(semaforr::domain::Distance(0.2));
  EXPECT_TRUE(victory.evaluate({world}));
  semaforr::decision::ForwardRule forward(actions);
  EXPECT_TRUE(forward.evaluate({world}));
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
  EXPECT_EQ(tier_one.createMandatory("Victory")->name(), "Victory");
  EXPECT_EQ(tier_one.createVeto("NotOpposite")->name(), "NotOpposite");
  const auto highway = tier_three.create("prefer_highways");
  ASSERT_EQ(highway->dependencies().size(), 1U);
  EXPECT_EQ(highway->dependencies().front(), "highways");
}
