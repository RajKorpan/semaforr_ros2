#ifndef SEMAFORR_EXPLORATION_EXPLORATION_COORDINATOR_HPP
#define SEMAFORR_EXPLORATION_EXPLORATION_COORDINATOR_HPP

#include <optional>
#include <semaforr/exploration/highway_explorer.hpp>
#include <string>
#include <vector>

namespace semaforr::exploration {

struct ExplorationUpdate {
  HleDecision decision;
  std::vector<std::string> events;
};

class ExplorationCoordinator {
 public:
  explicit ExplorationCoordinator(double candidate_completion_distance_m =
                                      0.1);

  ExplorationUpdate decide(const domain::RobotObservation& observation,
                           const domain::ActionSpace& action_space);
  void finish() noexcept;

 private:
  struct CandidateKey {
    std::size_t first_beam;
    std::size_t last_beam;
    bool operator==(const CandidateKey&) const = default;
  };

  HighwayExplorer explorer_;
  double candidate_completion_distance_m_;
  std::optional<CandidateKey> selected_;
  std::optional<domain::Point2D> candidate_start_;
  bool traversal_started_{false};
};

}  // namespace semaforr::exploration

#endif
