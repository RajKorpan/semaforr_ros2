#include <algorithm>
#include <cmath>
#include <semaforr/exploration/exploration_coordinator.hpp>
#include <stdexcept>

namespace semaforr::exploration {

ExplorationCoordinator::ExplorationCoordinator(
    double candidate_completion_distance_m)
    : candidate_completion_distance_m_(candidate_completion_distance_m) {
  if (!std::isfinite(candidate_completion_distance_m_) ||
      candidate_completion_distance_m_ <= 0.0)
    throw std::invalid_argument(
        "candidate completion distance must be finite and positive");
}

ExplorationUpdate ExplorationCoordinator::decide(
    const domain::RobotObservation& observation,
    const domain::ActionSpace& action_space) {
  ExplorationUpdate update{explorer_.decide(observation, action_space), {}};
  if (selected_ && traversal_started_ && candidate_start_ &&
      domain::distance(*candidate_start_, observation.pose.position).meters() >=
          candidate_completion_distance_m_) {
    update.events.push_back("candidate_completed");
    selected_.reset();
    candidate_start_.reset();
    traversal_started_ = false;
  }
  if (!update.decision.candidates.empty()) {
    const auto selected = std::max_element(
        update.decision.candidates.begin(), update.decision.candidates.end(),
        [](const auto& first, const auto& second) {
          if (first.confidence != second.confidence)
            return first.confidence < second.confidence;
          return first.heading.radians() > second.heading.radians();
        });
    const CandidateKey key{selected->first_beam, selected->last_beam};
    if (!selected_ || *selected_ != key) {
      if (selected_)
        update.events.push_back("candidate_completed");
      selected_ = key;
      candidate_start_ = observation.pose.position;
      traversal_started_ = false;
      update.events.push_back("candidate_selected");
    }
  }
  if (update.decision.state == HleState::TraversePassage)
    traversal_started_ = true;
  return update;
}

void ExplorationCoordinator::finish() noexcept {
  explorer_.finish();
  selected_.reset();
  candidate_start_.reset();
  traversal_started_ = false;
}

}  // namespace semaforr::exploration
