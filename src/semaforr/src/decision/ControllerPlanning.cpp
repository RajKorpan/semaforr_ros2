/*
 * Controller tier-two planning orchestration.
 */

#include <semaforr/decision/Controller.hpp>

void Controller::planForCurrentTask(
  Position current,
  bool selectNextTask) {
  const semaforr::decision::TierTwoResult result =
    tierTwoDecision->plan(current, selectNextTask);

  if (result.plan_selected) {
    decisionStats.chosenPlanner = result.chosen_planner;
  }
  decisionStats.planningComputationTime =
    result.computation_time_seconds;
}
