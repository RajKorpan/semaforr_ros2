#include <semaforr/navigation/navigation_phase.hpp>
#include <chrono>
#include <cmath>
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
      (!std::isfinite(configuration_.initial_exploration_time_limit_s) ||
       configuration_.initial_exploration_time_limit_s <= 0.0)) {
    throw std::invalid_argument(
        "initial exploration requires a finite positive time limit");
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
    const domain::RobotObservation& observation, domain::WorldModel&) {
  observe();
  if (phase_ == NavigationPhase::InitialExploration) {
    const auto timestamp =
        observation.observed_at == std::chrono::steady_clock::time_point{}
            ? std::chrono::steady_clock::now()
            : observation.observed_at;
    if (!exploration_started_at_) exploration_started_at_ = timestamp;
    const auto elapsed = timestamp - *exploration_started_at_;
    if (!exploration_time_limit_reached_ &&
        elapsed >= std::chrono::duration<double>(
                       configuration_.initial_exploration_time_limit_s)) {
      exploration_time_limit_reached_ = true;
      events_.push_back("exploration_time_limit_reached");
    }
  }
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
