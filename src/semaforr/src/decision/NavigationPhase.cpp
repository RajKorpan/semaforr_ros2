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
}

void NavigationPhaseCoordinator::observe() {
  if (phase_ != NavigationPhase::InitialExploration) return;
  ++exploration_observations_;
}

void NavigationPhaseCoordinator::completeInitialExploration() {
  if (phase_ == NavigationPhase::InitialExploration)
    phase_ = NavigationPhase::TargetNavigation;
}

void NavigationPhaseCoordinator::completeMission() {
  phase_ = NavigationPhase::MissionComplete;
}

}  // namespace semaforr::navigation
