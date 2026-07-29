#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>

#include <semaforr/decision/decision_coordinator.hpp>

namespace {

using semaforr::decision::ActionScore;
using semaforr::decision::Advisor;
using semaforr::decision::AdvisorEvaluation;
using semaforr::decision::DecisionContext;
using semaforr::decision::DecisionCoordinator;
using semaforr::decision::DecisionSource;
using semaforr::decision::Veto;
using semaforr::decision::VetoRule;
using semaforr::domain::Action;
using semaforr::domain::ActionType;

class FixedAdvisor final : public Advisor {
public:
  FixedAdvisor(std::string name, std::vector<ActionScore> scores, double weight = 1.0)
    : name_(std::move(name)), scores_(std::move(scores)), weight_(weight)
  {
  }

  std::string_view name() const noexcept override { return name_; }

  AdvisorEvaluation evaluate(
    const DecisionContext&,
    std::span<const Action>) const override
  {
    return {true, scores_, weight_, "fixed test scores"};
  }

private:
  std::string name_;
  std::vector<ActionScore> scores_;
  double weight_;
};

class FixedVeto final : public VetoRule {
public:
  explicit FixedVeto(Action action) : action_(action) {}

  std::vector<Veto> evaluate(const DecisionContext&) const override
  {
    return {{action_, "test-veto", "blocked for test"}};
  }

private:
  Action action_;
};

semaforr::domain::WorldModel world()
{
  return {};
}

TEST(DecisionCoordinator, EmptyCandidateSetReturnsSafeStop)
{
  auto model = world();
  DecisionCoordinator coordinator;
  const auto result = coordinator.decide(DecisionContext{model}, {});
  EXPECT_EQ(result.action, Action::pause());
  EXPECT_EQ(result.source, DecisionSource::SafeStop);
}

TEST(DecisionCoordinator, VetoedActionsCannotReenterAggregation)
{
  auto model = world();
  const Action forward(ActionType::Forward, 1U);
  const Action left(ActionType::TurnLeft, 1U);
  DecisionCoordinator coordinator;
  coordinator.addVetoRule(std::make_unique<FixedVeto>(forward));
  coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
    "scores-vetoed", std::vector<ActionScore>{{forward, 100.0}}));

  const std::vector<Action> candidates{forward, left};
  EXPECT_THROW(
    coordinator.decide(DecisionContext{model}, candidates),
    std::domain_error);
}

TEST(DecisionCoordinator, RejectsNonFiniteAdvice)
{
  auto model = world();
  const Action forward(ActionType::Forward, 1U);
  DecisionCoordinator coordinator;
  coordinator.addAdvisor(std::make_unique<FixedAdvisor>(
    "nan", std::vector<ActionScore>{
      {forward, std::numeric_limits<double>::quiet_NaN()}}));

  const std::vector<Action> candidates{forward};
  EXPECT_THROW(
    coordinator.decide(DecisionContext{model}, candidates),
    std::domain_error);
}

TEST(DecisionCoordinator, SameSeedProducesSameTieChoiceAndDiagnostics)
{
  auto model = world();
  const std::vector<Action> candidates{
    Action(ActionType::Forward, 1U),
    Action(ActionType::TurnLeft, 1U),
    Action(ActionType::TurnRight, 1U)};
  semaforr::decision::ArbitrationConfiguration configuration;
  configuration.random_seed = 42U;
  DecisionCoordinator first(configuration);
  DecisionCoordinator second(configuration);
  first.addAdvisor(std::make_unique<FixedAdvisor>(
    "tie", std::vector<ActionScore>{
      {candidates[0], 1.0}, {candidates[1], 1.0}, {candidates[2], 1.0}}));
  second.addAdvisor(std::make_unique<FixedAdvisor>(
    "tie", std::vector<ActionScore>{
      {candidates[0], 1.0}, {candidates[1], 1.0}, {candidates[2], 1.0}}));

  const auto first_result = first.decide(DecisionContext{model}, candidates);
  const auto second_result = second.decide(DecisionContext{model}, candidates);
  EXPECT_EQ(first_result.action, second_result.action);
  EXPECT_EQ(first_result.contributions, second_result.contributions);
}

TEST(DecisionCoordinator, NoAdvisorUsesConfiguredFallback)
{
  auto model = world();
  const Action left(ActionType::TurnLeft, 1U);
  semaforr::decision::ArbitrationConfiguration configuration;
  configuration.fallback = left;
  DecisionCoordinator coordinator(configuration);
  const std::vector<Action> candidates{left};
  const auto result = coordinator.decide(DecisionContext{model}, candidates);
  EXPECT_EQ(result.action, left);
  EXPECT_EQ(result.source, DecisionSource::Fallback);
}

}  // namespace
