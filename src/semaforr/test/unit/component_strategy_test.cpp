#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/decision/hard_safety_filter.hpp>
#include <semaforr/decision/mission_manager.hpp>
#include <semaforr/exploration/exploration_coordinator.hpp>
#include <semaforr/navigation/navigation_phase.hpp>
#include <semaforr/planning/planning_coordinator.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using semaforr::decision::ActionScore;
using semaforr::decision::Advisor;
using semaforr::decision::AdvisorEvaluation;
using semaforr::decision::Decision;
using semaforr::decision::DecisionContext;
using semaforr::decision::MandatoryRule;
using semaforr::domain::Action;
using semaforr::domain::ActionType;

class FixedRule final : public MandatoryRule {
 public:
  explicit FixedRule(std::optional<Decision> decision)
      : decision_(std::move(decision)) {}

  std::optional<Decision> evaluate(const DecisionContext&) const override {
    return decision_;
  }

 private:
  std::optional<Decision> decision_;
};

class FixedAdvisor final : public Advisor {
 public:
  FixedAdvisor(std::string name, std::vector<ActionScore> scores, double weight)
      : name_(std::move(name)), scores_(std::move(scores)), weight_(weight) {}

  std::string_view name() const noexcept override { return name_; }

  AdvisorEvaluation evaluate(const DecisionContext&,
                             std::span<const Action>) const override {
    return {true, scores_, weight_, "component fixture"};
  }

 private:
  std::string name_;
  std::vector<ActionScore> scores_;
  double weight_;
};

class FixedPlanner final : public semaforr::planning::Planner {
 public:
  FixedPlanner(std::string name, semaforr::planning::PlanStatus status,
               double cost)
      : name_(std::move(name)), status_(status), cost_(cost) {}

  semaforr::planning::PlanResult plan(
      const semaforr::planning::PlanningRequest&) override {
    return {status_, {{0.0, 0.0}, {1.0, 1.0}}, cost_, "fixture"};
  }

  std::string_view name() const noexcept override { return name_; }

 private:
  std::string name_;
  semaforr::planning::PlanStatus status_;
  double cost_;
};

TEST(MissionManager, ActivatesCompletesSkipsAndFinishesTasks) {
  semaforr::domain::Mission mission({{1U, {1.0, 0.0}}, {2U, {2.0, 0.0}}}, 2U);
  semaforr::decision::MissionManager manager(mission);

  EXPECT_EQ(manager.prepareDecision(),
            semaforr::decision::MissionStep::ActivatedTask);
  manager.recordDecision();
  EXPECT_TRUE(manager.completeActiveTask());
  EXPECT_EQ(manager.prepareDecision(),
            semaforr::decision::MissionStep::ActivatedTask);
  manager.recordDecision();
  manager.recordDecision();
  EXPECT_EQ(manager.prepareDecision(),
            semaforr::decision::MissionStep::Complete);
  EXPECT_EQ(mission.completed().size(), 1U);
  EXPECT_EQ(mission.skipped().size(), 1U);
  EXPECT_TRUE(manager.complete());
}

TEST(TierOneRuleChain, FirstMandatoryDecisionWins) {
  semaforr::domain::WorldModel world;
  semaforr::decision::DecisionCoordinator coordinator;
  coordinator.addMandatoryRule(std::make_unique<FixedRule>(std::nullopt));
  coordinator.addMandatoryRule(std::make_unique<FixedRule>(
      Decision{Action(ActionType::TurnLeft, 1U), "victory", "first decision"}));
  coordinator.addMandatoryRule(std::make_unique<FixedRule>(
      Decision{Action(ActionType::TurnRight, 1U), "enforcer", "must not run"}));

  const std::vector<Action> candidates{Action(ActionType::Forward, 1U),
                                       Action(ActionType::TurnLeft, 1U)};
  const auto result = coordinator.decide(DecisionContext{world}, candidates);
  EXPECT_EQ(result.action, Action(ActionType::TurnLeft, 1U));
  EXPECT_EQ(result.source, semaforr::decision::DecisionSource::MandatoryRule);
  EXPECT_EQ(result.tier, semaforr::decision::DecisionTier::TierOne);
}

TEST(TierThreeCoordinator, AggregatesRawScoresAndWeights) {
  semaforr::domain::WorldModel world;
  const Action forward(ActionType::Forward, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  semaforr::decision::DecisionCoordinator coordinator;
  coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
      "progress", std::vector<ActionScore>{{forward, 2.0}, {left, 1.0}}, 2.0));
  coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
      "clearance", std::vector<ActionScore>{{forward, -1.0}, {left, 1.0}},
      3.0));

  const std::vector<Action> candidates{forward, left};
  const auto result = coordinator.decide(DecisionContext{world}, candidates);
  EXPECT_EQ(result.action, left);
  ASSERT_EQ(result.contributions.size(), 4U);
  EXPECT_DOUBLE_EQ(result.contributions[0].raw_score, -1.0);
  EXPECT_DOUBLE_EQ(result.contributions[0].weighted_score, -3.0);
}

TEST(HardSafetyFilter, RejectsMissingLaserAndInvalidActionIndices) {
  using namespace semaforr;
  domain::WorldModel world;
  decision::HardSafetyFilter safety({0.2}, {0.5}, 0.25, 0.05);
  const std::vector<Action> candidates{
      Action::pause(), Action(ActionType::Forward, 1U),
      Action(ActionType::Forward, 2U), Action(ActionType::TurnLeft, 1U)};

  const auto without_laser =
      safety.filter(DecisionContext{world}, candidates);
  ASSERT_EQ(without_laser.safe_actions.size(), 1U);
  EXPECT_EQ(without_laser.safe_actions.front(), Action::pause());

  domain::LaserObservation laser;
  laser.angle_min = domain::Angle(-0.5);
  laser.angle_increment = domain::Angle(0.5);
  laser.minimum_range = domain::Distance(0.05);
  laser.maximum_range = domain::Distance(5.0);
  laser.ranges_m = {5.0, 0.35, 5.0};
  world.robot.laser = laser;
  world.robot.observed_at = std::chrono::steady_clock::now();
  const auto obstructed = safety.filter(DecisionContext{world}, candidates);
  EXPECT_EQ(std::find(obstructed.safe_actions.begin(),
                      obstructed.safe_actions.end(),
                      Action(ActionType::Forward, 1U)),
            obstructed.safe_actions.end());
  EXPECT_EQ(std::find(obstructed.safe_actions.begin(),
                      obstructed.safe_actions.end(),
                      Action(ActionType::Forward, 2U)),
            obstructed.safe_actions.end());
}

TEST(PlanningCoordinator, SelectsLowestCostAndUsesNameForStableTies) {
  semaforr::planning::PlanningCoordinator coordinator;
  coordinator.registerPlanner(std::make_unique<FixedPlanner>(
      "unavailable", semaforr::planning::PlanStatus::NoPath, 0.0));
  coordinator.registerPlanner(std::make_unique<FixedPlanner>(
      "zeta", semaforr::planning::PlanStatus::Success, 2.0));
  coordinator.registerPlanner(std::make_unique<FixedPlanner>(
      "alpha", semaforr::planning::PlanStatus::Success, 2.0));

  const auto selected = coordinator.selectPlan(
      {{{0.0, 0.0}, semaforr::domain::Angle::zero()}, {1.0, 1.0}, nullptr});
  ASSERT_TRUE(selected);
  EXPECT_EQ(selected->planner, "alpha");
  EXPECT_DOUBLE_EQ(selected->result.cost_m, 2.0);
}

TEST(PlanningCoordinator, RejectsInvalidRegistrationAndPlannerCosts) {
  semaforr::planning::PlanningCoordinator coordinator;
  EXPECT_THROW(coordinator.registerPlanner(nullptr), std::invalid_argument);
  EXPECT_THROW(coordinator.registerPlanner(std::make_unique<FixedPlanner>(
                   "", semaforr::planning::PlanStatus::Success, 1.0)),
               std::invalid_argument);

  coordinator.registerPlanner(std::make_unique<FixedPlanner>(
      "valid", semaforr::planning::PlanStatus::Success, 1.0));
  EXPECT_THROW(coordinator.registerPlanner(std::make_unique<FixedPlanner>(
                   "valid", semaforr::planning::PlanStatus::Success, 2.0)),
               std::invalid_argument);

  semaforr::planning::PlanningCoordinator invalid_cost;
  invalid_cost.registerPlanner(std::make_unique<FixedPlanner>(
      "nan", semaforr::planning::PlanStatus::Success,
      std::numeric_limits<double>::quiet_NaN()));
  EXPECT_THROW(
      invalid_cost.selectPlan(
          {{{0.0, 0.0}, semaforr::domain::Angle::zero()}, {1.0, 1.0}, nullptr}),
      std::domain_error);
}

}  // namespace

TEST(NavigationPhaseCoordinator, DelaysMissionUntilExplorationBudgetCompletes) {
  using namespace semaforr::navigation;
  NavigationPhaseCoordinator phases({true, 2U});
  EXPECT_EQ(phases.phase(), NavigationPhase::InitialExploration);
  EXPECT_FALSE(phases.missionActivationAllowed());
  phases.observe();
  EXPECT_EQ(phases.phase(), NavigationPhase::InitialExploration);
  phases.observe();
  EXPECT_TRUE(phases.explorationBudgetReached());
  EXPECT_EQ(phases.phase(), NavigationPhase::InitialExploration);
  phases.completeInitialExploration();
  EXPECT_EQ(phases.phase(), NavigationPhase::TargetNavigation);
  EXPECT_TRUE(phases.missionActivationAllowed());
  phases.completeMission();
  EXPECT_EQ(phases.phase(), NavigationPhase::MissionComplete);
}

TEST(NavigationPhaseCoordinator, EmitsExplicitLifecycleEvents) {
  using namespace semaforr::navigation;
  semaforr::domain::WorldModel world;
  semaforr::domain::RobotObservation observation;
  observation.laser.minimum_range = semaforr::domain::Distance(0.0);
  observation.laser.maximum_range = semaforr::domain::Distance(1.0);
  NavigationPhaseCoordinator phases({true, 1U});
  const auto update = phases.observe(observation, world);
  ASSERT_EQ(update.events.size(), 1U);
  EXPECT_EQ(update.events.front(), "initial_exploration_started");
  EXPECT_TRUE(phases.next(world).owns_decision);
  phases.completeInitialExploration();
  const auto events = phases.takeEvents();
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0], "initial_model_finalized");
  EXPECT_EQ(events[1], "target_navigation_started");
  EXPECT_TRUE(phases.next(world).mission_activation_allowed);
}

TEST(NavigationPhaseCoordinator, EmitsTimeLimitOnlyOnce) {
  using namespace semaforr::navigation;
  semaforr::domain::WorldModel world;
  semaforr::domain::RobotObservation observation;
  observation.laser.minimum_range = semaforr::domain::Distance(0.0);
  observation.laser.maximum_range = semaforr::domain::Distance(1.0);
  observation.observed_at =
      std::chrono::steady_clock::time_point{std::chrono::seconds(10)};
  NavigationPhaseCoordinator phases({true, 0U, 1.0});
  static_cast<void>(phases.observe(observation, world));
  observation.observed_at += std::chrono::seconds(1);
  const auto reached = phases.observe(observation, world);
  ASSERT_EQ(reached.events.size(), 1U);
  EXPECT_EQ(reached.events.front(), "exploration_time_limit_reached");
  EXPECT_TRUE(phases.explorationCompleteRequested());
  observation.observed_at += std::chrono::seconds(1);
  EXPECT_TRUE(phases.observe(observation, world).events.empty());
}

TEST(ExplorationCoordinator, EmitsCandidateLifecycleEvents) {
  using namespace semaforr;
  exploration::ExplorationCoordinator coordinator(0.1);
  domain::ActionSpace actions({0.1, 0.2}, {0.25, 0.5});
  domain::RobotObservation observation;
  observation.laser.angle_min =
      domain::Angle(-1.5707963267948966);
  observation.laser.angle_increment =
      domain::Angle(0.7853981633974483);
  observation.laser.minimum_range = domain::Distance(0.05);
  observation.laser.maximum_range = domain::Distance(5.0);
  observation.laser.ranges_m = {2.0, 2.0, 2.0, 2.0, 2.0};

  static_cast<void>(coordinator.decide(observation, actions));
  const auto selected = coordinator.decide(observation, actions);
  EXPECT_NE(std::find(selected.events.begin(), selected.events.end(),
                      "candidate_selected"),
            selected.events.end());
  static_cast<void>(coordinator.decide(observation, actions));
  observation.pose.position.x_m = 0.2;
  const auto completed = coordinator.decide(observation, actions);
  EXPECT_NE(std::find(completed.events.begin(), completed.events.end(),
                      "candidate_completed"),
            completed.events.end());
}
