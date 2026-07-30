#include <semaforr/exploration/exploration_coordinator.hpp>

namespace semaforr::exploration {

ExplorationCoordinator::ExplorationCoordinator(
    double candidate_completion_distance_m)
    : ExplorationCoordinator([candidate_completion_distance_m] {
        HighLevelExplorationConfiguration configuration;
        configuration.candidate_completion_distance =
            domain::Distance(candidate_completion_distance_m);
        return configuration;
      }()) {}

ExplorationCoordinator::ExplorationCoordinator(
    HighLevelExplorationConfiguration configuration)
    : explorer_(std::move(configuration)) {}

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
  if (update.decision.state == HleState::FinalizeModel && !finalized_) {
    if (model_finalizer_) model_finalizer_();
    finalized_ = true;
    update.events.emplace_back("initial_model_finalized");
  }
  return update;
}

void ExplorationCoordinator::finish() noexcept {
  explorer_.finish();
  if (!finalized_) {
    if (model_finalizer_) model_finalizer_();
    finalized_ = true;
  }
}

}  // namespace semaforr::exploration
