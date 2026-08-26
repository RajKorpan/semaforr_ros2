/**
 * @file tier_one_component_test.cpp
 * @brief Tier one component test responsibilities.
 *
 * @details This file exercises tier one component test behavior for automated
 * verification and regression testing. It centers on
 * `InstallRecoveryPlan`,
 * `VerifiesVisibilityTurnMovePauseAndHighestPriority`,
 * `HardSafetyCanRejectItsCognitiveMandate`,
 * `RemainsCognitiveAndUsesItsOwnReasonCode`,
 * `UsesOnlyExecutionConfirmedOrientations`,
 * `IncludesRegionRadiusAndPrefersAvailableRightThenLeft`,
 * `SuppressesOnlyAnExecutionConfirmedQuarterTurn`,
 * `TestsPreviousVisibilityAtTheScanObservationPose`. Its package-relative
 * location is `test/unit/tier_one_component_test.cpp`.
 */
#include <gtest/gtest.h>

#include <chrono>
#include <iterator>
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

/**
 * @brief Performs the scan operation for this subsystem.
 *
 * Arguments:
 * - @p beams: Supplies beams input to the operation.
 * - @p range_m: Supplies range m input to the operation.
 *
 * Returns:
 * - `semaforr::domain::LaserObservation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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
    semaforr::domain::Point2D target) {
  semaforr::domain::WorldModel world;
  world.mission = semaforr::domain::Mission({{1U, target}}, 100U);
  EXPECT_TRUE(world.mission.activate_next());
  world.mission.install_active_plan({target});
  world.robot.laser = scan();
  world.robot.observed_at = std::chrono::steady_clock::now();
  return world;
}

/**
 * @brief Performs the executed operation for this subsystem.
 *
 * Arguments:
 * - @p pose: Supplies pose input to the operation.
 * - @p action: Supplies action input to the operation.
 * - @p distance_m: Supplies distance m input to the operation.
 * - @p rotation_rad: Supplies rotation rad input to the operation.
 *
 * Returns:
 * - `semaforr::domain::NavigationHistoryEntry` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Records selection for this subsystem.
 *
 * Arguments:
 * - @p world: Supplies world input to the operation.
 * - @p position: Supplies position input to the operation.
 * - @p provenance: Supplies provenance input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void recordSelection(semaforr::domain::WorldModel& world,
                     semaforr::domain::Point2D position,
                     std::string provenance = "mandatory_rule:Enforcer") {
  semaforr::domain::SelectedActionRecord selected;
  selected.decision_id = world.decision_history.entries().size() + 1U;
  selected.action_id = selected.decision_id;
  selected.task_id = world.mission.active()->id;
  selected.expected_start = {position, semaforr::domain::Angle::zero()};
  selected.action = Action(ActionType::Forward, 1U);
  selected.provenance = std::move(provenance);
  world.decision_history.record(std::move(selected));
}

/**
 * @brief Performs the add successful path operation for this subsystem.
 *
 * Arguments:
 * - @p world: Supplies world input to the operation.
 * - @p steps: Supplies steps input to the operation.
 * - @p first_x: Supplies first x input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void addSuccessfulPath(semaforr::domain::WorldModel& world,
                       std::size_t steps, double first_x) {
  const auto task = world.mission.active()->id;
  world.path_history.begin(1U, task, world.mission.active()->target);
  for (std::size_t index = 0U; index < steps; ++index) {
    const double start_x = first_x + static_cast<double>(index);
    semaforr::domain::PathDecisionPoint point;
    point.selection.decision_id = index + 1U;
    point.selection.action_id = index + 1U;
    point.selection.task_id = task;
    point.selection.expected_start =
        {{start_x, 0.0}, semaforr::domain::Angle::zero()};
    point.selection.action = Action(ActionType::Forward, 1U);
    point.execution.decision_id = point.selection.decision_id;
    point.execution.action_id = point.selection.action_id;
    point.execution.task_id = task;
    point.execution.status =
        semaforr::domain::ExecutionCompletionStatus::Succeeded;
    point.execution.start_pose = point.selection.expected_start;
    point.execution.final_pose =
        {{start_x + 1.0, 0.0}, semaforr::domain::Angle::zero()};
    point.execution.distance_achieved_m = 1.0;
    point.decision_observation.pose = point.selection.expected_start;
    point.decision_observation.laser = scan();
    point.executed_action = point.selection.action;
    world.path_history.record(std::move(point));
    auto history = executed(
        {{start_x + 1.0, 0.0}, semaforr::domain::Angle::zero()},
        Action(ActionType::Forward, 1U), 1.0, 0.0);
    history.observation_pose =
        {{start_x, 0.0}, semaforr::domain::Angle::zero()};
    world.navigation_history.record(std::move(history));
  }
  world.robot.pose =
      {{first_x + static_cast<double>(steps), 0.0},
       semaforr::domain::Angle::zero()};
  world.robot.laser = scan();
}

/**
 * @brief Encapsulates install recovery plan state and behavior for this
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
class InstallRecoveryPlan final : public semaforr::planning::ReactivePlanner {
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
  std::string_view name() const noexcept override { return "Out"; }
  /**
   * @brief Performs the dependencies operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<std::string_view>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<std::string_view> dependencies() const override { return {}; }
  /**
   * @brief Evaluates trigger for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `semaforr::planning::TriggerEvaluation` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  semaforr::planning::TriggerEvaluation evaluateTrigger(
      const semaforr::decision::DecisionContext&) const override {
    return {true, "out:test_recovery_ready"};
  }
  /**
   * @brief Updates package content for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `semaforr::planning::ReactivePlanUpdate` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  semaforr::planning::ReactivePlanUpdate update(
      const semaforr::decision::DecisionContext&) override {
    return {semaforr::planning::ReactiveStatus::InstallPlan, std::nullopt, {},
            semaforr::planning::ReactiveCompletionReason::None, std::nullopt,
            "out:prepend_reverse_subtrail_for_enforcer",
            {{2.0, 0.0}, {1.0, 0.0}}};
  }
  /**
   * @brief Performs the cancel operation for this subsystem.
   *
   * Arguments:
   * - @p InterruptionReason: Supplies interruption reason input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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

TEST(AvoidObstacles, FootprintWidthIncludesOffAxisObstacleReturns) {
  auto world = worldWithTarget({2.0, 0.0});
  world.robot.laser->angle_min = semaforr::domain::Angle(0.4115);
  world.robot.laser->angle_increment = semaforr::domain::Angle(0.1);
  world.robot.laser->ranges_m = {0.6};
  semaforr::decision::ObstacleVetoRule footprint_aware({0.4}, 0.2, 0.05);
  const auto blocked = footprint_aware.evaluate({world});
  ASSERT_EQ(blocked.size(), 1U);
  EXPECT_EQ(blocked.front().action,
            Action(ActionType::Forward, 1U));

  semaforr::decision::ObstacleVetoRule centerline_only({0.4}, 0.0, 0.05);
  EXPECT_TRUE(centerline_only.evaluate({world}).empty());
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
  world.spatial.known_grid = {100U, 1U, 1.0, {0.0, 0.0},
                              std::vector<std::uint32_t>(100U, 99U), 1U};
  for (std::size_t index = 0U; index < 100U; ++index) {
    auto entry = executed(
        {{static_cast<double>(index), 0.0}, semaforr::domain::Angle::zero()},
        Action(ActionType::Forward, 1U), 1.0, 0.0);
    entry.observation_pose = entry.pose;
    world.navigation_history.record(std::move(entry));
  }
  world.robot.laser.reset();
  semaforr::planning::Out out;
  const auto trigger = out.evaluateTrigger({world});
  EXPECT_FALSE(trigger.triggered);
  EXPECT_EQ(trigger.rationale, "out:recent_window_not_confined");
}

TEST(Out, TriggersFromRecentObservationEvidenceNotCumulativeFamiliarity) {
  auto world = worldWithTarget({20.0, 0.0});
  world.spatial.known_grid = {1U, 1U, 1.0, {0.0, 0.0}, {0U}, 1U};
  for (std::size_t index = 0U; index < 4U; ++index) {
    auto entry = executed({{0.0, 0.0}, semaforr::domain::Angle::zero()},
                          Action::pause());
    entry.observation_pose = entry.pose;
    world.navigation_history.record(std::move(entry));
  }
  world.robot.pose = {{0.0, 0.0}, semaforr::domain::Angle::zero()};
  world.robot.laser = scan();
  semaforr::planning::Out out;
  const auto trigger = out.evaluateTrigger({world});
  EXPECT_TRUE(trigger.triggered);
  EXPECT_EQ(trigger.rationale, "out:recent_window_confined");
}

TEST(Out, SurveyAbortsWheneverAQuarterTurnRevealsNewFreespace) {
  const semaforr::domain::ActionSpace actions(
      {0.25, 0.8}, {0.2, 1.5707963267948966});
  for (std::size_t reveal_after = 1U; reveal_after <= 4U; ++reveal_after) {
    auto world = worldWithTarget({20.0, 0.0});
    world.recovery.confined = true;
    for (std::size_t index = 0U; index < 4U; ++index) {
      auto entry = executed({{0.0, 0.0}, semaforr::domain::Angle::zero()},
                            Action::pause());
      entry.observation_pose = entry.pose;
      world.navigation_history.record(std::move(entry));
    }
    world.robot.pose = {{0.0, 0.0}, semaforr::domain::Angle::zero()};
    world.robot.laser = scan();
    semaforr::planning::Out out;
    for (std::size_t turn_index = 1U; turn_index <= reveal_after; ++turn_index) {
      const auto turn_update = out.update({world, &actions});
      ASSERT_EQ(turn_update.status, semaforr::planning::ReactiveStatus::Action);
      if (turn_index < reveal_after) {
        auto prior = executed(world.robot.pose, Action::pause());
        prior.observation_pose = world.robot.pose;
        world.navigation_history.record(std::move(prior));
      }
    }
    world.robot.pose.position = {100.0, 100.0};
    const auto abandoned = out.update({world, &actions});
    EXPECT_EQ(abandoned.status,
              semaforr::planning::ReactiveStatus::NotApplicable);
    EXPECT_EQ(abandoned.explanation, "out:survey_revealed_new_freespace");
  }
}

TEST(Out, SurveysThenReturnsReverseSubtrailForEnforcer) {
  const semaforr::domain::ActionSpace actions(
      {0.25, 0.8}, {0.2, 1.5707963267948966});
  auto world = worldWithTarget({20.0, 0.0});
  world.recovery.confined = true;
  addSuccessfulPath(world, 20U, -20.0);
  world.robot.laser = scan(1U, 0.1);
  semaforr::planning::Out out;
  for (std::size_t survey = 0U; survey < 4U; ++survey) {
    const auto update = out.update({world, &actions});
    ASSERT_EQ(update.status, semaforr::planning::ReactiveStatus::Action);
    EXPECT_EQ(update.explanation, "out:survey_turn_right");
  }
  const auto recovery = out.update({world, &actions});
  EXPECT_EQ(recovery.status, semaforr::planning::ReactiveStatus::InstallPlan);
  EXPECT_FALSE(recovery.action);
  ASSERT_TRUE(recovery.learned_recovery_trail);
  EXPECT_GE(recovery.learned_recovery_trail->markers.size(), 2U);
  ASSERT_GE(recovery.prepend_waypoints.size(), 2U);
  EXPECT_GT(recovery.prepend_waypoints.front().x_m,
            recovery.prepend_waypoints.back().x_m);
  EXPECT_EQ(recovery.explanation,
            "out:prepend_reverse_subtrail_for_enforcer");
}

TEST(Out, PartialOrFailedSuffixDoesNotCreateRecoveryMarkers) {
  const semaforr::domain::ActionSpace actions(
      {0.25, 0.8}, {0.2, 1.5707963267948966});
  auto world = worldWithTarget({20.0, 0.0});
  world.recovery.confined = true;
  addSuccessfulPath(world, 20U, -20.0);
  world.robot.laser = scan(1U, 0.1);
  semaforr::domain::PathDecisionPoint partial;
  partial.selection.task_id = world.mission.active()->id;
  partial.selection.expected_start = world.robot.pose;
  partial.execution.task_id = world.mission.active()->id;
  partial.execution.start_pose = world.robot.pose;
  partial.execution.final_pose = world.robot.pose;
  partial.execution.status =
      semaforr::domain::ExecutionCompletionStatus::PartialMovement;
  partial.execution.distance_achieved_m = 0.25;
  partial.decision_observation.pose = world.robot.pose;
  partial.decision_observation.laser = scan();
  world.path_history.record(std::move(partial));
  semaforr::planning::Out out;
  for (std::size_t survey = 0U; survey < 4U; ++survey)
    ASSERT_EQ(out.update({world, &actions}).status,
              semaforr::planning::ReactiveStatus::Action);
  const auto recovery = out.update({world, &actions});
  EXPECT_EQ(recovery.status, semaforr::planning::ReactiveStatus::NotApplicable);
  EXPECT_FALSE(recovery.learned_recovery_trail);
}

TEST(Out, NavigationEnginePrependsRecoveryForNextCycleEnforcer) {
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
            "reactive:Out:recovery_plan_installed");
  EXPECT_EQ(result.tier, decision::DecisionTier::TierOne);
  ASSERT_TRUE(world.mission.active()->waypoint());
  EXPECT_EQ(*world.mission.active()->waypoint(),
            (domain::Point2D{2.0, 0.0}));
  const auto out_event = std::find_if(
      result.decision_cycle.begin(), result.decision_cycle.end(),
      [](const auto& event) { return event.component == "Out"; });
  ASSERT_NE(out_event, result.decision_cycle.end());
  EXPECT_FALSE(out_event->returned_to_earlier_tier);
  EXPECT_EQ(out_event->outcome,
            "reverse_subtrail_installed_cycle_end");
  const auto enforcer_event = std::find_if(
      std::next(out_event), result.decision_cycle.end(),
      [](const auto& event) { return event.component == "Enforcer"; });
  EXPECT_EQ(enforcer_event, result.decision_cycle.end());
}

TEST(Forward, UsesOnlyEnforcerSelectionsAndMarksTheThreeByThreeNeighborhood) {
  const semaforr::domain::ActionSpace actions(
      {2.0}, {0.2, 1.5707963267948966});
  auto world = worldWithTarget({10.0, 0.0});
  semaforr::decision::ForwardRule forward(actions);
  EXPECT_TRUE(forward.evaluate({world}).empty());

  recordSelection(world, {0.0, 0.0}, "mandatory_rule:Victory");
  EXPECT_TRUE(forward.evaluate({world}).empty());

  recordSelection(world, {0.0, 0.0});
  const std::vector<Action> viable{
      Action(ActionType::Forward, 1U), Action(ActionType::TurnLeft, 1U),
      Action(ActionType::TurnRight, 1U), Action(ActionType::TurnLeft, 2U),
      Action(ActionType::TurnRight, 2U)};
  const auto vetoes = forward.evaluate({world, &actions, viable});
  ASSERT_EQ(vetoes.size(), 2U);
  EXPECT_TRUE(std::all_of(vetoes.begin(), vetoes.end(), [](const auto& veto) {
    return veto.explanation ==
           "forward:projected_footprint_already_visited";
  }));
  EXPECT_TRUE(std::none_of(vetoes.begin(), vetoes.end(), [](const auto& veto) {
    return veto.action.type() == ActionType::Forward;
  }));
}

TEST(Forward, AllRotationVetoClearsGridAndTargetChangeStartsEmpty) {
  const semaforr::domain::ActionSpace actions(
      {1.0}, {1.5707963267948966});
  auto world = worldWithTarget({10.0, 0.0});
  semaforr::decision::ForwardRule forward(actions);
  recordSelection(world, {-0.2, -0.2});
  world.robot.pose = {{-0.2, -0.2}, semaforr::domain::Angle::zero()};
  EXPECT_TRUE(forward.evaluate({world}).empty());
  EXPECT_TRUE(forward.evaluate({world}).empty());

  auto next_target = worldWithTarget({20.0, 0.0});
  next_target.mission = semaforr::domain::Mission({{2U, {20.0, 0.0}}}, 100U);
  ASSERT_TRUE(next_target.mission.activate_next());
  next_target.mission.install_active_plan({{20.0, 0.0}});
  EXPECT_TRUE(forward.evaluate({next_target}).empty());
}

}  // namespace
