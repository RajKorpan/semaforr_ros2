#ifndef SEMAFORR_NAVIGATION_NAVIGATION_PHASE_HPP
#define SEMAFORR_NAVIGATION_NAVIGATION_PHASE_HPP

#include <chrono>
#include <cstddef>
#include <semaforr/domain/observation.hpp>
#include <semaforr/domain/world_model.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::navigation {

enum class NavigationPhase {
  InitialExploration,
  TargetNavigation,
  MissionComplete
};

std::string_view toString(NavigationPhase phase) noexcept;

struct PhaseUpdate {
  NavigationPhase phase = NavigationPhase::TargetNavigation;
  std::vector<std::string> events;
};

struct PhaseDecision {
  NavigationPhase phase = NavigationPhase::TargetNavigation;
  bool owns_decision = false;
  bool mission_activation_allowed = true;
};

struct PhaseConfiguration {
  bool initial_exploration_enabled = false;
  std::size_t initial_exploration_observation_budget = 0U;
  double initial_exploration_time_limit_s = 1200.0;
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
           configuration_.initial_exploration_observation_budget > 0U &&
           exploration_observations_ >=
               configuration_.initial_exploration_observation_budget;
  }
  bool explorationTimeLimitReached() const noexcept {
    return phase_ == NavigationPhase::InitialExploration &&
           exploration_time_limit_reached_;
  }
  bool explorationCompleteRequested() const noexcept {
    return explorationBudgetReached() || explorationTimeLimitReached();
  }
  void observe();
  PhaseUpdate observe(const domain::RobotObservation& observation,
                      domain::WorldModel& world);
  PhaseDecision next(const domain::WorldModel& world) const noexcept;
  std::vector<std::string> takeEvents();
  void completeInitialExploration();
  void completeMission();

 private:
  PhaseConfiguration configuration_;
  NavigationPhase phase_;
  std::size_t exploration_observations_ = 0U;
  std::optional<std::chrono::steady_clock::time_point>
      exploration_started_at_;
  bool exploration_time_limit_reached_ = false;
  std::vector<std::string> events_;
};

}  // namespace semaforr::navigation

#endif
