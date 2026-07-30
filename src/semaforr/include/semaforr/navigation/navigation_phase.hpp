#ifndef SEMAFORR_NAVIGATION_NAVIGATION_PHASE_HPP
#define SEMAFORR_NAVIGATION_NAVIGATION_PHASE_HPP

#include <cstddef>
#include <string_view>

namespace semaforr::navigation {

enum class NavigationPhase {
  InitialExploration,
  TargetNavigation,
  MissionComplete
};

std::string_view toString(NavigationPhase phase) noexcept;

struct PhaseConfiguration {
  bool initial_exploration_enabled = false;
  std::size_t initial_exploration_observation_budget = 0U;
};

class NavigationPhaseCoordinator {
 public:
  explicit NavigationPhaseCoordinator(PhaseConfiguration configuration = {});
  NavigationPhase phase() const noexcept { return phase_; }
  std::size_t explorationObservations() const noexcept {
    return exploration_observations_;
  }
  bool missionActivationAllowed() const noexcept {
    return phase_ == NavigationPhase::TargetNavigation;
  }
  bool explorationBudgetReached() const noexcept {
    return phase_ == NavigationPhase::InitialExploration &&
           exploration_observations_ >=
               configuration_.initial_exploration_observation_budget;
  }
  void observe();
  void completeInitialExploration();
  void completeMission();

 private:
  PhaseConfiguration configuration_;
  NavigationPhase phase_;
  std::size_t exploration_observations_ = 0U;
};

}  // namespace semaforr::navigation

#endif
