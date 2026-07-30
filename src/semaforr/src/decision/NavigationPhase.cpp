#include <semaforr/navigation/navigation_phase.hpp>
#include <stdexcept>

namespace semaforr::navigation {

std::string_view toString(NavigationPhase phase) noexcept {
  switch (phase) {
    case NavigationPhase::InitialExploration:
      return "initial_exploration";
    case NavigationPhase::TargetNavigation:
      return "target_navigation";
    case NavigationPhase::MissionComplete:
      return "mission_complete";
  }
  return "mission_complete";
}

NavigationPhaseCoordinator::NavigationPhaseCoordinator(
    PhaseConfiguration configuration)
    : configuration_(configuration),
      phase_(configuration.initial_exploration_enabled
                 ? NavigationPhase::InitialExploration
                 : NavigationPhase::TargetNavigation) {
  if (configuration_.initial_exploration_enabled &&
      configuration_.initial_exploration_observation_budget == 0U) {
    throw std::invalid_argument(
        "initial exploration requires a positive observation budget");
  }
  events_.push_back(configuration_.initial_exploration_enabled
                        ? "initial_exploration_started"
                        : "target_navigation_started");
}

void NavigationPhaseCoordinator::observe() {
  if (phase_ != NavigationPhase::InitialExploration) return;
  ++exploration_observations_;
}

void NavigationPhaseCoordinator::completeInitialExploration() {
  if (phase_ == NavigationPhase::InitialExploration) {
    events_.push_back("initial_model_finalized");
    phase_ = NavigationPhase::TargetNavigation;
    events_.push_back("target_navigation_started");
  }
}

void NavigationPhaseCoordinator::completeMission() {
  phase_ = NavigationPhase::MissionComplete;
}

PhaseUpdate NavigationPhaseCoordinator::observe(
    const domain::RobotObservation&, domain::WorldModel&) {
  observe();
  return {phase_, takeEvents()};
}

PhaseDecision NavigationPhaseCoordinator::next(
    const domain::WorldModel&) const noexcept {
  return {phase_, phase_ == NavigationPhase::InitialExploration,
          missionActivationAllowed()};
}

std::vector<std::string> NavigationPhaseCoordinator::takeEvents() {
  std::vector<std::string> result;
  result.swap(events_);
  return result;
}

}  // namespace semaforr::navigation
