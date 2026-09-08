/**
 * @file navigation_scenario_test.cpp
 * @brief Navigation scenario test responsibilities.
 *
 * @details This file exercises navigation scenario test behavior for automated
 * verification and regression testing. It centers on `ProgressAdvisor`,
 * `EmptyCorridorSelectsForward`, `DoorwayRemainsTraversable`,
 * `ObstacleAheadVetoesForwardMotion`, `DeadEndWithNoSurvivorStopsSafely`,
 * `MultipleTargetsAdvanceInOrder`, `StuckRobotSkipsAtTheDecisionLimit`,
 * `PedestrianCrossingCanChangeTheSelectedAction`. Its package-relative
 * location is `test/integration/navigation_scenario_test.cpp`.
 */
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <numbers>
#include <semaforr/decision/advisors/social/social_navigation_advisor.hpp>
#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/decision/mission_manager.hpp>
#include <semaforr/decision/obstacle_veto_rule.hpp>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {

using semaforr::domain::Action;
using semaforr::domain::ActionType;

/**
 * @brief Encapsulates progress advisor state and behavior for this
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
class ProgressAdvisor final : public semaforr::decision::Advisor {
 public:
  /**
   * @brief Performs the progress advisor operation for this subsystem.
   *
   * Arguments:
   * - @p weight: Supplies weight input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit ProgressAdvisor(double weight = 1.0) : weight_(weight) {}

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
  std::string_view name() const noexcept override { return "progress"; }

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
    result.weight = weight_;
    result.explanation = "prefer forward progress";
    for (const auto& action : candidates) {
      double score = 0.0;
      if (action.type() == ActionType::Forward) {
        score = 4.0;
      } else if (action.type() == ActionType::TurnLeft ||
                 action.type() == ActionType::TurnRight) {
        score = 1.0;
      }
      result.scores.push_back({action, score});
    }
    return result;
  }

 private:
  double weight_;
};

/**
 * @brief Performs the laser operation for this subsystem.
 *
 * Arguments:
 * - @p left: Supplies left input to the operation.
 * - @p ahead: Supplies ahead input to the operation.
 * - @p right: Supplies right input to the operation.
 *
 * Returns:
 * - `semaforr::domain::LaserObservation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::LaserObservation laser(double left, double ahead,
                                         double right) {
  return {semaforr::domain::Angle(-std::numbers::pi / 2.0),
          semaforr::domain::Angle(std::numbers::pi / 2.0),
          semaforr::domain::Distance(0.05),
          semaforr::domain::Distance(10.0),
          {left, ahead, right}};
}

/**
 * @brief Performs the corridor coordinator operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `semaforr::decision::DecisionCoordinator` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::decision::DecisionCoordinator corridorCoordinator() {
  semaforr::decision::ArbitrationConfiguration configuration;
  configuration.random_seed = 19U;
  configuration.fallback = Action::pause();
  semaforr::decision::DecisionCoordinator coordinator(configuration);
  coordinator.addVetoRule(
      std::make_unique<semaforr::decision::ObstacleVetoRule>(
          std::vector<double>{0.2, 0.5}, 0.2, 0.05));
  coordinator.addAdvisor(std::make_unique<ProgressAdvisor>());
  return coordinator;
}

/**
 * @brief Performs the crossing crowd operation for this subsystem.
 *
 * Arguments:
 * - @p count: Supplies count input to the operation.
 *
 * Returns:
 * - `semaforr::domain::CrowdObservation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::CrowdObservation crossingCrowd(std::size_t count) {
  semaforr::domain::CrowdObservation crowd;
  crowd.frame_id = "map";
  crowd.observed_at = std::chrono::seconds(10);
  crowd.data_age = std::chrono::milliseconds(50);
  for (std::size_t index = 0U; index < count; ++index) {
    const double offset = static_cast<double>(index) * 0.15;
    crowd.pedestrians.push_back(
        {"crossing-" + std::to_string(index),
         {0.5 + offset, -0.5},
         {0.0, 1.0},
         {{{0.5 + offset, 0.0}, std::chrono::seconds(11)}},
         1.0,
         {0.04, 0.0, 0.0, 0.04},
         "none",
         std::nullopt});
  }
  crowd.validate();
  return crowd;
}

/**
 * @brief Performs the social advisor operation for this subsystem.
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
semaforr::decision::SocialNavigationAdvisor socialAdvisor(double weight) {
  return semaforr::decision::SocialNavigationAdvisor(
      {{0.2, 0.5},
       {0.5},
       std::chrono::milliseconds(750),
       0.25,
       2.0,
       1.2,
       0.65,
       weight});
}

TEST(ScenarioReplay, EmptyCorridorSelectsForward) {
  semaforr::domain::WorldModel world;
  world.robot.laser = laser(5.0, 5.0, 5.0);
  auto coordinator = corridorCoordinator();
  const std::vector<Action> actions{Action::pause(),
                                    Action(ActionType::Forward, 2U),
                                    Action(ActionType::TurnLeft, 1U)};
  EXPECT_EQ(coordinator.decide({world}, actions).action,
            Action(ActionType::Forward, 2U));
}

TEST(ScenarioReplay, DoorwayRemainsTraversable) {
  semaforr::domain::WorldModel world;
  world.robot.laser = laser(0.7, 3.0, 0.7);
  world.spatial.doorways.push_back({{1.5, -0.5}, {1.5, 0.5}});
  auto coordinator = corridorCoordinator();
  const std::vector<Action> actions{Action(ActionType::Forward, 2U),
                                    Action(ActionType::TurnLeft, 1U)};
  EXPECT_EQ(coordinator.decide({world}, actions).action,
            Action(ActionType::Forward, 2U));
}

TEST(ScenarioReplay, ObstacleAheadVetoesForwardMotion) {
  semaforr::domain::WorldModel world;
  world.robot.laser = laser(5.0, 0.4, 5.0);
  auto coordinator = corridorCoordinator();
  const std::vector<Action> actions{Action(ActionType::Forward, 1U),
                                    Action(ActionType::Forward, 2U),
                                    Action(ActionType::TurnLeft, 1U)};
  const auto result = coordinator.decide({world}, actions);
  EXPECT_EQ(result.action, Action(ActionType::TurnLeft, 1U));
  EXPECT_EQ(result.vetoes.size(), 2U);
}

TEST(ScenarioReplay, DeadEndWithNoSurvivorStopsSafely) {
  semaforr::domain::WorldModel world;
  world.robot.laser = laser(0.3, 0.3, 0.3);
  auto coordinator = corridorCoordinator();
  const std::vector<Action> actions{Action(ActionType::Forward, 1U),
                                    Action(ActionType::Forward, 2U)};
  const auto result = coordinator.decide({world}, actions);
  EXPECT_EQ(result.action, Action::pause());
  EXPECT_EQ(result.source, semaforr::decision::DecisionSource::SafeStop);
}

TEST(ScenarioReplay, MultipleTargetsAdvanceInOrder) {
  semaforr::domain::Mission mission({{10U, {1.0, 0.0}}, {20U, {2.0, 0.0}}}, 3U);
  semaforr::decision::MissionManager manager(mission);
  EXPECT_EQ(manager.prepareDecision(),
            semaforr::decision::MissionStep::ActivatedTask);
  ASSERT_TRUE(mission.active());
  EXPECT_EQ(mission.active()->id, 10U);
  EXPECT_TRUE(manager.completeActiveTask());
  EXPECT_EQ(manager.prepareDecision(),
            semaforr::decision::MissionStep::ActivatedTask);
  ASSERT_TRUE(mission.active());
  EXPECT_EQ(mission.active()->id, 20U);
}

TEST(ScenarioReplay, StuckRobotSkipsAtTheDecisionLimit) {
  semaforr::domain::Mission mission({{1U, {10.0, 0.0}}, {2U, {20.0, 0.0}}}, 2U);
  semaforr::decision::MissionManager manager(mission);
  manager.prepareDecision();
  manager.recordDecision();
  manager.recordDecision();
  EXPECT_EQ(manager.prepareDecision(),
            semaforr::decision::MissionStep::SkippedTask);
  ASSERT_TRUE(mission.active());
  EXPECT_EQ(mission.active()->id, 2U);
  ASSERT_EQ(mission.skipped().size(), 1U);
  EXPECT_EQ(mission.skipped()[0].id, 1U);
}

TEST(ScenarioReplay, PedestrianCrossingCanChangeTheSelectedAction) {
  semaforr::domain::WorldModel world;
  world.robot.pose = {{0.0, 0.0}, semaforr::domain::Angle::zero()};
  world.crowd.update(crossingCrowd(1U));
  semaforr::decision::DecisionCoordinator coordinator;
  coordinator.addAdvisor(std::make_unique<ProgressAdvisor>());
  coordinator.addAdvisor(
      std::make_unique<semaforr::decision::SocialNavigationAdvisor>(
          socialAdvisor(5.0)));
  const std::vector<Action> actions{Action::pause(),
                                    Action(ActionType::Forward, 2U)};
  EXPECT_EQ(coordinator.decide({world}, actions).action, Action::pause());
}

TEST(ScenarioReplay, DenseCrowdProducesSocialContributions) {
  semaforr::domain::WorldModel world;
  world.robot.pose = {{0.0, 0.0}, semaforr::domain::Angle::zero()};
  world.crowd.update(crossingCrowd(5U));
  semaforr::decision::DecisionCoordinator coordinator;
  coordinator.addAdvisor(std::make_unique<ProgressAdvisor>());
  coordinator.addAdvisor(
      std::make_unique<semaforr::decision::SocialNavigationAdvisor>(
          socialAdvisor(6.0)));
  const std::vector<Action> actions{Action::pause(),
                                    Action(ActionType::Forward, 2U)};
  const auto result = coordinator.decide({world}, actions);
  EXPECT_EQ(result.action, Action::pause());
  EXPECT_GE(result.contributions.size(), 4U);
}

}  // namespace
