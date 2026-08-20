#ifndef SEMAFORR_EXPLORATION_EXPLORATION_STRATEGY_HPP
#define SEMAFORR_EXPLORATION_EXPLORATION_STRATEGY_HPP

#include <chrono>
#include <cstddef>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/grid_geometry.hpp>
#include <semaforr/domain/motion_model.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/exploration/exploration_result.hpp>
#include <vector>

namespace semaforr::exploration {

enum class HleBehaviorPolicy { Modernized, Compatibility };

struct HleAngularSector {
  domain::Angle minimum;
  domain::Angle maximum;
};

struct HighLevelExplorationConfiguration {
  HleBehaviorPolicy behavior_policy = HleBehaviorPolicy::Modernized;
  domain::Distance minimum_clearance{0.8};
  domain::Angle heading_tolerance{0.2};
  domain::Distance candidate_completion_distance{0.1};
  domain::Distance cue_similarity_radius{0.5};
  domain::Distance passage_grid_resolution{0.5};
  std::size_t minimum_bundle_beams = 1U;
  // Robot-relative angular sectors. Focus sectors are narrow cue generators;
  // Open sectors independently measure the wider side-space geometry.
  HleAngularSector left_focus{domain::Angle(0.6544984694978736),
                              domain::Angle(0.9162978572970231)};
  HleAngularSector right_focus{domain::Angle(-0.9162978572970231),
                               domain::Angle(-0.6544984694978736)};
  HleAngularSector left_open{domain::Angle(0.0),
                             domain::Angle(1.5707963267948966)};
  HleAngularSector right_open{domain::Angle(-1.5707963267948966),
                              domain::Angle(0.0)};
  double minimum_length_to_width_ratio = 1.5;
  domain::Distance minimum_passage_length{1.0};
  domain::Distance large_room_width{3.0};
  domain::Distance large_room_length{3.0};
  domain::Distance cue_clearance_margin{0.05};
  double maximum_width_change_ratio = 0.35;
  domain::Angle hard_turn_threshold{0.7853981633974483};
  domain::Distance end_of_passage_clearance{0.8};
  domain::Distance minimum_extension{0.25};
  std::chrono::duration<double> time_budget{1200.0};
  std::size_t decision_budget = 10000U;
  // When invalid, HLE creates an expandable representation-local geometry
  // around the first observation. A valid value supplies fixed/shared geometry.
  domain::GridGeometry passage_grid_geometry;

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
