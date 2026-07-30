#ifndef SEMAFORR_EXPLORATION_HIGHWAY_EXPLORER_HPP
#define SEMAFORR_EXPLORATION_HIGHWAY_EXPLORER_HPP

#include <cstddef>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/domain/world_model.hpp>
#include <string_view>
#include <vector>

namespace semaforr::exploration {

enum class PassageKind { Corridor, Doorway, IntersectionBranch };

struct PassageCandidate {
  domain::Angle heading;
  domain::Distance clearance;
  PassageKind kind = PassageKind::Corridor;
  double confidence = 0.0;
  std::size_t first_beam = 0U;
  std::size_t last_beam = 0U;
};

enum class HleState {
  Survey,
  AlignWithPassage,
  TraversePassage,
  ConfirmIntersection,
  Complete
};

std::string_view toString(HleState state) noexcept;

struct HleDecision {
  domain::Action action = domain::Action::pause();
  HleState state = HleState::Survey;
  std::vector<PassageCandidate> candidates;
  std::string_view rationale;
};

class HighwayExplorer {
 public:
  HighwayExplorer(double minimum_clearance_m = 0.8,
                  double heading_tolerance_rad = 0.2);

  HleDecision decide(const domain::RobotObservation& observation,
                     const domain::ActionSpace& action_space);
  void finish() noexcept { state_ = HleState::Complete; }
  HleState state() const noexcept { return state_; }

  static std::vector<PassageCandidate> detectPassages(
      const domain::LaserObservation& laser, double minimum_clearance_m);

 private:
  double minimum_clearance_m_;
  double heading_tolerance_rad_;
  HleState state_ = HleState::Survey;
};

}  // namespace semaforr::exploration

#endif
