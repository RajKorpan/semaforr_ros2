#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/decision/hard_safety_filter.hpp>
#include <semaforr/decision/navigation_engine.hpp>
#include <semaforr/decision/tier_registry.hpp>
#include <semaforr/planning/reactive_planner.hpp>
#include <semaforr/spatial/spatial_learning_coordinator.hpp>
#include <vector>

namespace {

using semaforr::domain::Action;
using semaforr::domain::ActionType;

semaforr::domain::LaserObservation scan(std::size_t beams = 9U,
                                        double range_m = 5.0) {
  semaforr::domain::LaserObservation result;
  result.angle_min = semaforr::domain::Angle(-0.4);
  result.angle_increment = semaforr::domain::Angle(0.1);
  result.minimum_range = semaforr::domain::Distance(0.1);
  result.maximum_range = semaforr::domain::Distance(5.0);
  result.ranges_m.assign(beams, range_m);
  return result;
}

semaforr::domain::WorldModel worldWithTarget(
    semaforr::domain::Point2D target) {
  semaforr::domain::WorldModel world;
  world.mission = semaforr::domain::Mission({{1U, target}}, 100U);
  EXPECT_TRUE(world.mission.activate_next());
  world.mission.install_active_plan({target});
  world.robot.laser = scan();
  world.robot.observed_at = std::chrono::steady_clock::now();
  return world;
}

semaforr::domain::NavigationHistoryEntry executed(
    semaforr::domain::Pose2D pose, Action action,
    double distance_m = 0.0, double rotation_rad = 0.0) {
  semaforr::domain::NavigationHistoryEntry entry{pose, scan(), action, 1U};
  entry.execution_status =
      semaforr::domain::ExecutionCompletionStatus::Succeeded;
  entry.distance_achieved_m = distance_m;
  entry.rotation_achieved_rad = rotation_rad;
  entry.action_id = 1U;
  entry.decision_id = 1U;
  return entry;
}

class InstallRecoveryPlan final : public semaforr::planning::ReactivePlanner {
 public:
  std::string_view name() const noexcept override { return "Out"; }
  std::vector<std::string_view> dependencies() const override { return {}; }
  semaforr::planning::TriggerEvaluation evaluateTrigger(
      const semaforr::decision::DecisionContext&) const override {
    return {true, "out:test_recovery_ready"};
  }
  semaforr::planning::ReactivePlanUpdate update(
      const semaforr::decision::DecisionContext&) override {
    return {semaforr::planning::ReactiveStatus::InstallPlan, std::nullopt, {},
            semaforr::planning::ReactiveCompletionReason::None, std::nullopt,
            "out:prepend_reverse_subtrail_for_enforcer",
            {{2.0, 0.0}, {1.0, 0.0}}};
  }
  void cancel(semaforr::planning::InterruptionReason) override {}
};

TEST(Victory, VerifiesVisibilityTurnMovePauseAndHighestPriority) {
  const semaforr::domain::ActionSpace actions({0.25, 1.0}, {0.2, 0.5});
  semaforr::decision::VictoryRule victory(semaforr::domain::Distance(0.2),
                                           actions);

  auto world = worldWithTarget({0.1, 0.0});
  auto decision = victory.evaluate({world});
  ASSERT_TRUE(decision);
  EXPECT_EQ(decision->action, Action::pause());
  EXPECT_EQ(decision->explanation, "victory:target_within_tolerance");

  world = worldWithTarget({2.0, 0.0});
  decision = victory.evaluate({world});
  ASSERT_TRUE(decision);
  EXPECT_EQ(decision->action.type(), ActionType::Forward);
  EXPECT_EQ(decision->explanation, "victory:move_toward_visible_target");

  world = worldWithTarget({2.0, 0.8});
  decision = victory.evaluate({world});
  ASSERT_TRUE(decision);
  EXPECT_EQ(decision->action.type(), ActionType::TurnLeft);
  EXPECT_EQ(decision->explanation, "victory:turn_toward_visible_target");

  world.robot.laser->ranges_m.assign(9U, 0.5);
  EXPECT_FALSE(victory.evaluate({world}));

  world = worldWithTarget({2.0, 0.0});
  semaforr::decision::DecisionCoordinator coordinator;
  coordinator.addMandatoryRule(std::make_unique<semaforr::decision::VictoryRule>(
      semaforr::domain::Distance(0.2), actions));
  const std::vector<Action> candidates{
      Action::pause(), Action(ActionType::Forward, 1U),
      Action(ActionType::Forward, 2U), Action(ActionType::TurnLeft, 1U)};
  const auto result = coordinator.decide({world}, candidates);
  EXPECT_EQ(result.selected_policy, "mandatory_rule:Victory");
  ASSERT_EQ(result.decision_cycle.size(), 1U);
  EXPECT_EQ(result.decision_cycle.front().reason_code,
            "victory:move_toward_visible_target");
}

TEST(Victory, HardSafetyCanRejectItsCognitiveMandate) {
  const semaforr::domain::ActionSpace actions({0.5}, {0.2});
  auto world = worldWithTarget({1.0, 0.0});
  world.robot.laser->angle_min = semaforr::domain::Angle(-0.2);
  world.robot.laser->ranges_m = {0.3, 1.5, 1.5, 1.5, 0.3};
  semaforr::decision::HardSafetyFilter safety({0.5}, {0.2}, 0.25, 0.05);
  const std::vector<Action> candidates{Action::pause(),
                                       Action(ActionType::Forward, 1U)};
  const auto filtered = safety.filter({world}, candidates);
  ASSERT_EQ(filtered.safe_actions, std::vector<Action>{Action::pause()});
  ASSERT_FALSE(filtered.vetoes.empty());
  EXPECT_EQ(filtered.vetoes.front().rule, "HardSafetyFilter");
  EXPECT_EQ(filtered.vetoes.front().explanation,
            "hard_safety:collision_clearance");

  semaforr::decision::DecisionCoordinator coordinator;
  coordinator.addMandatoryRule(std::make_unique<semaforr::decision::VictoryRule>(
      semaforr::domain::Distance(0.2), actions));
  const auto result = coordinator.decide({world}, filtered.safe_actions);
  EXPECT_EQ(result.action, Action::pause());
  EXPECT_NE(result.selected_policy, "mandatory_rule:Victory");
}

TEST(AvoidObstacles, RemainsCognitiveAndUsesItsOwnReasonCode) {
  auto world = worldWithTarget({2.0, 0.0});
  world.robot.laser->angle_min = semaforr::domain::Angle(-0.1);
  world.robot.laser->ranges_m = {5.0, 0.6, 5.0};
  semaforr::decision::ObstacleVetoRule cognitive({0.2, 0.5}, 0.2, 0.05);
  const auto vetoes = cognitive.evaluate({world});
  ASSERT_FALSE(vetoes.empty());
  EXPECT_EQ(vetoes.front().rule, "AvoidObstacles");
  EXPECT_EQ(vetoes.front().explanation,
            "avoid_obstacles:forward_corridor_obstructed");

  semaforr::decision::HardSafetyFilter safety({0.2, 0.5}, {0.2}, 0.2, 0.05);
  const std::vector<Action> candidates{
      Action::pause(), Action(ActionType::Forward, 1U),
      Action(ActionType::Forward, 2U)};
  const auto filtered = safety.filter({world}, candidates);
  EXPECT_TRUE(std::all_of(filtered.vetoes.begin(), filtered.vetoes.end(),
                          [](const auto& veto) {
                            return veto.rule == "HardSafetyFilter";
                          }));

  world.robot.observed_at = {};
  const std::vector<Action> stale_candidates{
      Action::pause(), Action(ActionType::Forward, 1U),
      Action(ActionType::Forward, 3U)};
  const auto stale = safety.filter({world}, stale_candidates);
  EXPECT_TRUE(std::any_of(stale.vetoes.begin(), stale.vetoes.end(),
                          [](const auto& veto) {
                            return veto.explanation ==
                                   "hard_safety:sensor_stale_or_missing";
                          }));
  EXPECT_TRUE(std::any_of(stale.vetoes.begin(), stale.vetoes.end(),
                          [](const auto& veto) {
                            return veto.explanation ==
                                   "hard_safety:invalid_action_index";
                          }));
}

TEST(NotOpposite, UsesOnlyExecutionConfirmedOrientations) {
  const semaforr::domain::ActionSpace actions({0.25}, {0.5});
  auto world = worldWithTarget({3.0, 0.0});
  world.robot.pose.heading = semaforr::domain::Angle(1.0);
  semaforr::domain::SelectedActionRecord selected;
  selected.action = Action(ActionType::TurnLeft, 1U);
  selected.expected_start.heading = semaforr::domain::Angle::zero();
  world.decision_history.record(selected);
  semaforr::decision::NotOppositeRule rule(actions);
  EXPECT_TRUE(rule.evaluate({world}).empty());

  world.navigation_history.record(executed(
      {{0.0, 0.0}, semaforr::domain::Angle(0.5)},
      Action(ActionType::TurnLeft, 1U), 0.0, 0.5));
  const auto vetoes = rule.evaluate({world});
  ASSERT_EQ(vetoes.size(), 1U);
  EXPECT_EQ(vetoes.front().action, Action(ActionType::TurnRight, 1U));
  EXPECT_EQ(vetoes.front().explanation,
            "not_opposite:predicted_heading_recently_executed");
}

TEST(Behind, IncludesRegionRadiusAndPrefersAvailableRightThenLeft) {
  const semaforr::domain::ActionSpace actions({0.25}, {1.5707963267948966});
  auto world = worldWithTarget({-2.0, 0.0});
  world.robot.laser->angle_min = semaforr::domain::Angle(-0.4);
  semaforr::planning::Behind without_region;
  EXPECT_FALSE(without_region.evaluateTrigger({world, &actions}).triggered);

  world.spatial.learned_regions.push_back(
      {{-2.0, 0.0}, semaforr::domain::Distance(1.0)});
  semaforr::planning::Behind behind;
  const auto trigger = behind.evaluateTrigger({world, &actions});
  EXPECT_TRUE(trigger.triggered);
  EXPECT_EQ(trigger.rationale,
            "behind:nearby_waypoint_outside_recent_views");

  const Action right(ActionType::TurnRight, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  auto update = behind.update({world, &actions, std::vector<Action>{right, left}});
  ASSERT_EQ(update.status, semaforr::planning::ReactiveStatus::Action);
  EXPECT_EQ(update.action, right);
  EXPECT_EQ(update.explanation, "behind:turn_right_to_reveal_waypoint");

  update = behind.update({world, &actions, std::vector<Action>{left}});
  ASSERT_EQ(update.status, semaforr::planning::ReactiveStatus::Action);
  EXPECT_EQ(update.action, left);
  EXPECT_EQ(update.explanation, "behind:turn_left_when_right_unavailable");
}

TEST(Behind, SuppressesOnlyAnExecutionConfirmedQuarterTurn) {
  const semaforr::domain::ActionSpace actions({0.25}, {1.5707963267948966});
  auto world = worldWithTarget({-1.0, 0.0});
  auto failed = executed(world.robot.pose, Action(ActionType::TurnRight, 1U),
                         0.0, 1.5707963267948966);
  failed.execution_status =
      semaforr::domain::ExecutionCompletionStatus::ControllerFailure;
  world.navigation_history.record(failed);
  semaforr::planning::Behind behind;
  EXPECT_TRUE(behind.evaluateTrigger({world, &actions}).triggered);

  world.navigation_history.record(executed(
      world.robot.pose, Action(ActionType::TurnRight, 1U), 0.0,
      1.5707963267948966));
  const auto trigger = behind.evaluateTrigger({world, &actions});
  EXPECT_FALSE(trigger.triggered);
  EXPECT_EQ(trigger.rationale, "behind:quarter_turn_already_executed");
}

TEST(Behind, TestsPreviousVisibilityAtTheScanObservationPose) {
  const semaforr::domain::ActionSpace actions({0.25}, {1.5707963267948966});
  auto world = worldWithTarget({-1.0, 0.0});
  auto previous = executed(
      {{0.0, 0.0}, semaforr::domain::Angle(3.1415926535897932)},
      Action::pause());
  previous.observation_pose =
      {{0.0, 0.0}, semaforr::domain::Angle::zero()};
  world.navigation_history.record(std::move(previous));
  semaforr::planning::Behind behind;
  const auto trigger = behind.evaluateTrigger({world, &actions});
  EXPECT_TRUE(trigger.triggered);
  EXPECT_EQ(trigger.rationale,
            "behind:nearby_waypoint_outside_recent_views");
}

TEST(Out, UsesRecentTenPlusNOverFiftyWindow) {
  auto world = worldWithTarget({20.0, 0.0});
  std::vector<std::uint32_t> familiarity(100U, 0U);
  std::fill_n(familiarity.begin(), 88U, 5U);
  world.spatial.known_grid = {100U, 1U, 1.0, {0.0, 0.0}, familiarity, 1U};
  for (std::size_t index = 0U; index < 100U; ++index) {
    world.navigation_history.record(executed(
        {{static_cast<double>(index), 0.0}, semaforr::domain::Angle::zero()},
        Action(ActionType::Forward, 1U), 1.0, 0.0));
  }
  world.robot.laser.reset();
  semaforr::planning::Out out;
  const auto trigger = out.evaluateTrigger({world});
  EXPECT_FALSE(trigger.triggered);
  EXPECT_EQ(trigger.rationale, "out:recent_window_not_confined");
}

TEST(Out, SurveysThenReturnsReverseSubtrailForEnforcer) {
  const semaforr::domain::ActionSpace actions(
      {0.25, 0.8}, {0.2, 1.5707963267948966});
  auto world = worldWithTarget({20.0, 0.0});
  world.recovery.confined = true;
  for (std::size_t index = 0U; index < 4U; ++index) {
    world.navigation_history.record(executed(
        {{static_cast<double>(index), 0.0}, semaforr::domain::Angle::zero()},
        Action(ActionType::Forward, 1U), 1.0, 0.0));
  }
  semaforr::planning::Out out;
  for (std::size_t survey = 0U; survey < 4U; ++survey) {
    const auto update = out.update({world, &actions});
    ASSERT_EQ(update.status, semaforr::planning::ReactiveStatus::Action);
    EXPECT_EQ(update.explanation, "out:survey_turn_right");
  }
  const auto recovery = out.update({world, &actions});
  EXPECT_EQ(recovery.status, semaforr::planning::ReactiveStatus::InstallPlan);
  EXPECT_FALSE(recovery.action);
  ASSERT_GE(recovery.prepend_waypoints.size(), 2U);
  EXPECT_GT(recovery.prepend_waypoints.front().x_m,
            recovery.prepend_waypoints.back().x_m);
  EXPECT_EQ(recovery.explanation,
            "out:prepend_reverse_subtrail_for_enforcer");
}

TEST(Out, NavigationEnginePrependsRecoveryAndReturnsToEnforcer) {
  using namespace semaforr;
  domain::WorldModel world;
  world.mission = domain::Mission({{1U, {10.0, 0.0}}}, 20U);
  ASSERT_TRUE(world.mission.activate_next());
  world.recovery.plan_abandoned = true;
  const domain::ActionSpace actions({0.25, 1.0}, {0.2, 0.5});
  decision::DecisionCoordinator decisions;
  decision::MissionManager mission(world.mission);
  planning::PlanningCoordinator planning;
  spatial::SpatialLearningCoordinator learning(100U);
  std::vector<std::unique_ptr<planning::ReactivePlanner>> reactive;
  reactive.push_back(std::make_unique<InstallRecoveryPlan>());
  decision::NavigationEngine engine(
      world, actions, decisions, mission, planning, learning, nullptr,
      domain::Distance(0.2), nullptr, nullptr, {}, {}, std::move(reactive),
      false, true);
  domain::RobotObservation observation;
  observation.pose = {{0.0, 0.0}, domain::Angle::zero()};
  observation.laser = scan();
  observation.observed_at = std::chrono::steady_clock::now();

  const auto result = engine.decide(observation);
  EXPECT_EQ(result.selected_policy,
            "mandatory_rule:Enforcer:out_reverse_subtrail");
  EXPECT_EQ(result.tier, decision::DecisionTier::TierOne);
  ASSERT_TRUE(world.mission.active()->waypoint());
  EXPECT_EQ(*world.mission.active()->waypoint(),
            (domain::Point2D{2.0, 0.0}));
  const auto out_event = std::find_if(
      result.decision_cycle.begin(), result.decision_cycle.end(),
      [](const auto& event) { return event.component == "Out"; });
  ASSERT_NE(out_event, result.decision_cycle.end());
  EXPECT_TRUE(out_event->returned_to_earlier_tier);
  EXPECT_EQ(out_event->outcome,
            "reverse_subtrail_installed_return_to_enforcer");
  const auto enforcer_event = std::find_if(
      out_event, result.decision_cycle.end(),
      [](const auto& event) { return event.component == "Enforcer"; });
  ASSERT_NE(enforcer_event, result.decision_cycle.end());
  EXPECT_EQ(enforcer_event->reason_code,
            "enforcer:operationalized_out_reverse_subtrail");
}

TEST(Forward, UsesSuccessfulExecutionConfirmedFootprintCells) {
  const semaforr::domain::ActionSpace actions(
      {2.0}, {1.5707963267948966, 3.1415926535897932});
  auto world = worldWithTarget({10.0, 0.0});
  semaforr::decision::ForwardRule forward(actions, 0.5);
  EXPECT_TRUE(forward.evaluate({world}).empty());

  auto failed = executed(
      {{-2.0, 0.0}, semaforr::domain::Angle::zero()},
      Action(ActionType::Forward, 1U), 2.0, 0.0);
  failed.execution_status =
      semaforr::domain::ExecutionCompletionStatus::ControllerFailure;
  world.navigation_history.record(failed);
  EXPECT_TRUE(forward.evaluate({world}).empty());

  world.navigation_history.record(executed(
      {{-2.0, 0.0}, semaforr::domain::Angle::zero()},
      Action(ActionType::Forward, 1U), 2.0, 0.0));
  const auto vetoes = forward.evaluate({world});
  ASSERT_FALSE(vetoes.empty());
  EXPECT_TRUE(std::all_of(vetoes.begin(), vetoes.end(), [](const auto& veto) {
    return veto.explanation ==
           "forward:projected_footprint_already_visited";
  }));
}

}  // namespace
