/**
 * @file navigation_phase.cpp
 * @brief Navigation phase responsibilities.
 *
 * @details This file implements navigation phase behavior for the SemaFORR
 * navigation package. It records the declarations, settings, fixtures, or
 * guidance needed by that responsibility. Its package-relative location is
 * `src/navigation/navigation_phase.cpp`.
 */
#include <chrono>
#include <cmath>
#include <semaforr/navigation/navigation_phase.hpp>
#include <stdexcept>

namespace semaforr::navigation {

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p phase: Supplies phase input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Performs the navigation phase coordinator operation for this
 * subsystem.
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

/**
 * @brief Processes package content for this subsystem.
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
void NavigationPhaseCoordinator::observe() {
  if (phase_ != NavigationPhase::InitialExploration) return;
  ++exploration_observations_;
}

/**
 * @brief Performs the complete initial exploration operation for this
 * subsystem.
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
void NavigationPhaseCoordinator::completeInitialExploration() {
  if (phase_ == NavigationPhase::InitialExploration) {
    events_.push_back("initial_model_finalized");
    phase_ = NavigationPhase::TargetNavigation;
    events_.push_back("target_navigation_started");
  }
}

/**
 * @brief Performs the complete mission operation for this subsystem.
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
void NavigationPhaseCoordinator::completeMission() {
  phase_ = NavigationPhase::MissionComplete;
}

/**
 * @brief Processes package content for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 * - @p WorldModel: Supplies world model input to the operation.
 *
 * Returns:
 * - `PhaseUpdate` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Performs the next operation for this subsystem.
 *
 * Arguments:
 * - @p WorldModel: Supplies world model input to the operation.
 *
 * Returns:
 * - `PhaseDecision` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
PhaseDecision NavigationPhaseCoordinator::next(
    const domain::WorldModel&) const noexcept {
  return {phase_, phase_ == NavigationPhase::InitialExploration,
          missionActivationAllowed()};
}

/**
 * @brief Performs the take events operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `std::vector<std::string>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<std::string> NavigationPhaseCoordinator::takeEvents() {
  std::vector<std::string> result;
  result.swap(events_);
  return result;
}

}  // namespace semaforr::navigation
