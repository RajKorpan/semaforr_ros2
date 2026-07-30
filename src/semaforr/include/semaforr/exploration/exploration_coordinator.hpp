#ifndef SEMAFORR_EXPLORATION_EXPLORATION_COORDINATOR_HPP
#define SEMAFORR_EXPLORATION_EXPLORATION_COORDINATOR_HPP

#include <chrono>
#include <functional>
#include <semaforr/exploration/high_level_explorer.hpp>
#include <string>
#include <vector>

namespace semaforr::exploration {

struct ExplorationUpdate {
  ExplorationResult decision;
  std::vector<std::string> events;
};

class ExplorationCoordinator {
 public:
  explicit ExplorationCoordinator(double candidate_completion_distance_m =
                                      0.1);
  explicit ExplorationCoordinator(HighLevelExplorationConfiguration);

  ExplorationUpdate decide(const domain::RobotObservation& observation,
                           const domain::ActionSpace& action_space);
  void setModelFinalizer(std::function<void()> finalizer) {
    model_finalizer_ = std::move(finalizer);
  }
  void finish() noexcept;
  PassageGridSnapshot passageGrid() const { return explorer_.passageGrid(); }

 private:
  HighLevelExplorer explorer_;
  std::chrono::steady_clock::time_point started_at_{
      std::chrono::steady_clock::now()};
  std::function<void()> model_finalizer_;
  bool finalized_ = false;
};

}  // namespace semaforr::exploration

#endif
