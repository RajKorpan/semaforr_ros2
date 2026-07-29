#include <semaforr/decision/Arbitration.hpp>

#include <cassert>
#include <limits>
#include <map>
#include <vector>

int main() {
  using semaforr::decision::ActionArbitrationResult;
  using semaforr::decision::PlanArbitrationResult;
  using semaforr::decision::selectHighestScoringAction;
  using semaforr::decision::selectLowestCostPlan;

  const ActionArbitrationResult empty_action =
    selectHighestScoringAction({});
  assert(!empty_action.selected);
  assert(empty_action.action == FORRAction(PAUSE, 0));

  std::map<FORRAction, double> action_scores = {
    {FORRAction(LEFT_TURN, 2), 7.0},
    {FORRAction(RIGHT_TURN, 3), 7.0},
    {FORRAction(FORWARD, 4), std::numeric_limits<double>::quiet_NaN()},
    {FORRAction(PAUSE, 0), std::numeric_limits<double>::infinity()}
  };
  const ActionArbitrationResult action =
    selectHighestScoringAction(action_scores);
  assert(action.selected);
  assert(action.action == FORRAction(RIGHT_TURN, 3));
  assert(action.score == 7.0);

  const ActionArbitrationResult negative_action =
    selectHighestScoringAction({
      {FORRAction(LEFT_TURN, 2), -5000.0},
      {FORRAction(RIGHT_TURN, 3), -5000.0}
    });
  assert(negative_action.selected);
  assert(negative_action.action == FORRAction(RIGHT_TURN, 3));
  assert(negative_action.score == -5000.0);

  const PlanArbitrationResult empty_plan = selectLowestCostPlan({});
  assert(!empty_plan.selected);

  const PlanArbitrationResult invalid_plan = selectLowestCostPlan({
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()
  });
  assert(!invalid_plan.selected);

  const PlanArbitrationResult plan =
    selectLowestCostPlan({4.0, 2.0, 2.0, 3.0});
  assert(plan.selected);
  assert(plan.index == 1);
  assert(plan.cost == 2.0);

  const PlanArbitrationResult large_plan =
    selectLowestCostPlan({300000.0, 200000.0});
  assert(large_plan.selected);
  assert(large_plan.index == 1);
  assert(large_plan.cost == 200000.0);

  return 0;
}
