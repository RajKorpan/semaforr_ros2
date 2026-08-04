#include <semaforr/exploration/highway_explorer.hpp>
#include <algorithm>
#include <cmath>

namespace semaforr::exploration {

HighwayExplorer::HighwayExplorer(double minimum_clearance_m,
                                 double heading_tolerance_rad)
    : configuration_{domain::Distance(minimum_clearance_m),
                     domain::Angle(heading_tolerance_rad)},
      explorer_(configuration_) {}

std::vector<PassageCandidate> HighwayExplorer::detectPassages(
    const domain::LaserObservation& laser, double minimum_clearance_m) {
  domain::RobotObservation observation;
  observation.laser = laser;
  HighLevelExplorationConfiguration configuration;
  configuration.minimum_clearance = domain::Distance(minimum_clearance_m);
  std::vector<PassageCandidate> result;
  for (const auto& candidate :
       HighLevelExplorer::discoverCandidates(observation, configuration)) {
    result.push_back({candidate.heading, candidate.clearance, candidate.kind,
                      candidate.priority, candidate.first_beam,
                      candidate.last_beam});
  }
  return result;
}

HleDecision HighwayExplorer::decide(
    const domain::RobotObservation& observation,
    const domain::ActionSpace& action_space) {
  auto candidates =
      detectPassages(observation.laser,
                     configuration_.minimum_clearance.meters());
  if (candidates.empty())
    return {domain::Action(domain::ActionType::TurnLeft, 1U),
            HleState::DiscoverCandidate, {}, "survey for passage"};
  const auto selected = std::max_element(
      candidates.begin(), candidates.end(), [](const auto& left,
                                                const auto& right) {
        if (left.confidence != right.confidence)
          return left.confidence < right.confidence;
        return left.heading.radians() > right.heading.radians();
      });
  const double heading = selected->heading.radians();
  if (std::abs(heading) > configuration_.heading_tolerance.radians()) {
    const auto& turns = action_space.rotation_angles_rad();
    const auto found =
        std::lower_bound(turns.begin(), turns.end(), std::abs(heading));
    const std::size_t index =
        found == turns.end()
            ? turns.size()
            : static_cast<std::size_t>(found - turns.begin()) + 1U;
    return {heading < 0.0
                ? domain::Action(domain::ActionType::TurnRight, index)
                : domain::Action(domain::ActionType::TurnLeft, index),
            HleState::PursueCandidate, std::move(candidates),
            "align with widest passage"};
  }
  const std::size_t magnitude =
      selected->clearance.meters() > 1.5
          ? action_space.move_distances_m().size()
          : 1U;
  return {domain::Action(domain::ActionType::Forward, magnitude),
          HleState::PursueCandidate, std::move(candidates),
          "traverse selected passage"};
}

}  // namespace semaforr::exploration
