/**
 * @file navigation_phase.hpp
 * @brief Navigation phase responsibilities.
 *
 * @details This file defines navigation phase behavior for the SemaFORR
 * navigation package. It centers on `NavigationPhase`, `PhaseUpdate`,
 * `PhaseDecision`, `PhaseConfiguration`, `NavigationPhaseCoordinator`. Its
 * package-relative location is
 * `include/semaforr/navigation/navigation_phase.hpp`.
 */
#ifndef SEMAFORR_NAVIGATION_NAVIGATION_PHASE_HPP
#define SEMAFORR_NAVIGATION_NAVIGATION_PHASE_HPP

#include <chrono>
#include <cstddef>
#include <optional>
#include <semaforr/domain/observation.hpp>
#include <semaforr/domain/world_model.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::navigation {

/**
 * @brief Enumerates the supported navigation phase values used by this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class NavigationPhase {
  InitialExploration,
  TargetNavigation,
  MissionComplete
};

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
std::string_view toString(NavigationPhase phase) noexcept;

/**
 * @brief Encapsulates phase update state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct PhaseUpdate {
  NavigationPhase phase = NavigationPhase::TargetNavigation;
  std::vector<std::string> events;
};

/**
 * @brief Encapsulates phase decision state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct PhaseDecision {
  NavigationPhase phase = NavigationPhase::TargetNavigation;
  bool owns_decision = false;
  bool mission_activation_allowed = true;
};

/**
 * @brief Encapsulates phase configuration state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct PhaseConfiguration {
  bool initial_exploration_enabled = false;
  std::size_t initial_exploration_observation_budget = 0U;
  double initial_exploration_time_limit_s = 1200.0;
};

/**
 * @brief Encapsulates navigation phase coordinator state and behavior for
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class NavigationPhaseCoordinator {
 public:
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
  explicit NavigationPhaseCoordinator(PhaseConfiguration configuration = {});
  /**
   * @brief Performs the phase operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `NavigationPhase` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  NavigationPhase phase() const noexcept { return phase_; }
  /**
   * @brief Performs the exploration observations operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t explorationObservations() const noexcept {
    return exploration_observations_;
  }
  /**
   * @brief Performs the mission activation allowed operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool missionActivationAllowed() const noexcept {
    return phase_ == NavigationPhase::TargetNavigation;
  }
  /**
   * @brief Performs the exploration budget reached operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool explorationBudgetReached() const noexcept {
    return phase_ == NavigationPhase::InitialExploration &&
           configuration_.initial_exploration_observation_budget > 0U &&
           exploration_observations_ >=
               configuration_.initial_exploration_observation_budget;
  }
  /**
   * @brief Performs the exploration time limit reached operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool explorationTimeLimitReached() const noexcept {
    return phase_ == NavigationPhase::InitialExploration &&
           exploration_time_limit_reached_;
  }
  /**
   * @brief Performs the exploration complete requested operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool explorationCompleteRequested() const noexcept {
    return explorationBudgetReached() || explorationTimeLimitReached();
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
  void observe();
  /**
   * @brief Processes package content for this subsystem.
   *
   * Arguments:
   * - @p observation: Supplies observation input to the operation.
   * - @p world: Supplies world input to the operation.
   *
   * Returns:
   * - `PhaseUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PhaseUpdate observe(const domain::RobotObservation& observation,
                      domain::WorldModel& world);
  /**
   * @brief Performs the next operation for this subsystem.
   *
   * Arguments:
   * - @p world: Supplies world input to the operation.
   *
   * Returns:
   * - `PhaseDecision` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PhaseDecision next(const domain::WorldModel& world) const noexcept;
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
  std::vector<std::string> takeEvents();
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
  void completeInitialExploration();
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
  void completeMission();

 private:
  PhaseConfiguration configuration_;
  NavigationPhase phase_;
  std::size_t exploration_observations_ = 0U;
  std::optional<std::chrono::steady_clock::time_point> exploration_started_at_;
  bool exploration_time_limit_reached_ = false;
  std::vector<std::string> events_;
};

}  // namespace semaforr::navigation

#endif
