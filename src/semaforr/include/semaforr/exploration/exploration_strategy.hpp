#ifndef SEMAFORR_EXPLORATION_EXPLORATION_STRATEGY_HPP
#define SEMAFORR_EXPLORATION_EXPLORATION_STRATEGY_HPP

#include <chrono>
#include <cstddef>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/exploration/exploration_result.hpp>
#include <vector>

namespace semaforr::exploration {

struct HighLevelExplorationConfiguration {
  domain::Distance minimum_clearance{0.8};
  domain::Angle heading_tolerance{0.2};
  domain::Distance candidate_completion_distance{0.1};
  domain::Distance cue_similarity_radius{0.5};
  domain::Distance passage_grid_resolution{0.5};
  std::size_t minimum_bundle_beams = 1U;
  std::chrono::duration<double> time_budget{1200.0};
  std::size_t decision_budget = 10000U;

  void validate() const;
};

struct ExplorationInput {
  const domain::RobotObservation& observation;
  const domain::ActionSpace& action_space;
  std::chrono::duration<double> elapsed{};
};

class ExplorationStrategy {
 public:
  virtual ~ExplorationStrategy() = default;
  virtual ExplorationResult update(const ExplorationInput& input) = 0;
  virtual void finish() noexcept = 0;
};

}  // namespace semaforr::exploration

#endif
