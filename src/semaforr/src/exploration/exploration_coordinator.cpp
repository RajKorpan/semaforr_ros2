/**
 * @file exploration_coordinator.cpp
 * @brief Exploration coordinator responsibilities.
 *
 * @details This file implements exploration coordinator behavior for initial or
 * reactive exploration. It records the declarations, settings, fixtures,
 * or guidance needed by that responsibility. Its package-relative location
 * is `src/exploration/exploration_coordinator.cpp`.
 */
#include <semaforr/exploration/exploration_coordinator.hpp>

namespace semaforr::exploration {

/**
 * @brief Performs the exploration coordinator operation for this subsystem.
 *
 * Arguments:
 * - @p candidate_completion_distance_m: Supplies candidate completion
 * distance m input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
ExplorationCoordinator::ExplorationCoordinator(
    double candidate_completion_distance_m)
    : ExplorationCoordinator([candidate_completion_distance_m] {
        HighLevelExplorationConfiguration configuration;
        configuration.candidate_completion_distance =
            domain::Distance(candidate_completion_distance_m);
        return configuration;
      }()) {}

/**
 * @brief Performs the exploration coordinator operation for this subsystem.
 *
 * Arguments:
 * - @p configuration: Supplies configuration input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
ExplorationCoordinator::ExplorationCoordinator(
    HighLevelExplorationConfiguration configuration)
    : explorer_(std::move(configuration)) {}

/**
 * @brief Selects package content for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 * - @p action_space: Supplies action space input to the operation.
 *
 * Returns:
 * - `ExplorationUpdate` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
ExplorationUpdate ExplorationCoordinator::decide(
    const domain::RobotObservation& observation,
    const domain::ActionSpace& action_space) {
  const auto elapsed = std::chrono::steady_clock::now() - started_at_;
  ExplorationUpdate update{
      explorer_.update(
          {observation, action_space,
           std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)}),
      {}};
  if (update.decision.event != CandidateLifecycleEvent::None)
    update.events.emplace_back(toString(update.decision.event));
  for (const auto& diagnostic : update.decision.diagnostics)
    update.events.push_back(
        "candidate_" + std::string(toString(diagnostic.kind)) + ":" +
        std::to_string(diagnostic.candidate_id) + ":" + diagnostic.reason);
  if (update.decision.state == HleState::FinalizeModel && !finalized_) {
    if (model_finalizer_) model_finalizer_();
    finalized_ = true;
    update.events.emplace_back("initial_model_finalized");
  }
  return update;
}

/**
 * @brief Performs the finish operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void ExplorationCoordinator::finish() noexcept {
  explorer_.finish();
  if (!finalized_) {
    if (model_finalizer_) model_finalizer_();
    finalized_ = true;
  }
}

}  // namespace semaforr::exploration
