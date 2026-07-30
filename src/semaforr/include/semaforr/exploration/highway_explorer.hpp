#ifndef SEMAFORR_EXPLORATION_HIGHWAY_EXPLORER_HPP
#define SEMAFORR_EXPLORATION_HIGHWAY_EXPLORER_HPP

#include <semaforr/exploration/high_level_explorer.hpp>

namespace semaforr::exploration {

// Compatibility view retained for existing callers. New code should use
// HighLevelExplorer and ExplorationResult.
struct PassageCandidate {
  domain::Angle heading;
  domain::Distance clearance;
  PassageKind kind = PassageKind::Corridor;
  double confidence = 0.0;
  std::size_t first_beam = 0U;
  std::size_t last_beam = 0U;
};

struct HleDecision {
  domain::Action action = domain::Action::pause();
  HleState state = HleState::Initialize;
  std::vector<PassageCandidate> candidates;
  std::string_view rationale;
};

class HighwayExplorer {
 public:
  HighwayExplorer(double minimum_clearance_m = 0.8,
                  double heading_tolerance_rad = 0.2);

  HleDecision decide(const domain::RobotObservation& observation,
                     const domain::ActionSpace& action_space);
  void finish() noexcept { explorer_.finish(); }
  HleState state() const noexcept { return explorer_.state(); }

  static std::vector<PassageCandidate> detectPassages(
      const domain::LaserObservation& laser, double minimum_clearance_m);

 private:
  HighLevelExplorationConfiguration configuration_;
  HighLevelExplorer explorer_;
};

}  // namespace semaforr::exploration

#endif
