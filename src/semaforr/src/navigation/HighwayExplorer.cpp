#include <algorithm>
#include <cmath>
#include <semaforr/exploration/highway_explorer.hpp>
#include <stdexcept>

namespace semaforr::exploration {
namespace {

domain::Action turnToward(double heading,
                          const domain::ActionSpace& action_space) {
  const auto& turns = action_space.rotation_angles_rad();
  const auto found = std::lower_bound(turns.begin(), turns.end(),
                                      std::abs(heading));
  const std::size_t index =
      found == turns.end() ? turns.size()
                           : static_cast<std::size_t>(found - turns.begin()) + 1U;
  return heading < 0.0
             ? domain::Action(domain::ActionType::TurnRight, index)
             : domain::Action(domain::ActionType::TurnLeft, index);
}

}  // namespace

std::string_view toString(HleState state) noexcept {
  switch (state) {
    case HleState::Survey: return "survey";
    case HleState::AlignWithPassage: return "align_with_passage";
    case HleState::TraversePassage: return "traverse_passage";
    case HleState::ConfirmIntersection: return "confirm_intersection";
    case HleState::Complete: return "complete";
  }
  return "complete";
}

HighwayExplorer::HighwayExplorer(double minimum_clearance_m,
                                 double heading_tolerance_rad)
    : minimum_clearance_m_(minimum_clearance_m),
      heading_tolerance_rad_(heading_tolerance_rad) {
  if (!(minimum_clearance_m_ > 0.0) ||
      !(heading_tolerance_rad_ > 0.0)) {
    throw std::invalid_argument("HLE thresholds must be positive");
  }
}

std::vector<PassageCandidate> HighwayExplorer::detectPassages(
    const domain::LaserObservation& laser, double minimum_clearance_m) {
  std::vector<PassageCandidate> result;
  std::size_t begin = 0U;
  while (begin < laser.ranges_m.size()) {
    while (begin < laser.ranges_m.size() &&
           laser.ranges_m[begin] < minimum_clearance_m) ++begin;
    if (begin == laser.ranges_m.size()) break;
    std::size_t end = begin;
    double clearance = laser.ranges_m[begin];
    while (end + 1U < laser.ranges_m.size() &&
           laser.ranges_m[end + 1U] >= minimum_clearance_m) {
      ++end;
      clearance = std::min(clearance, laser.ranges_m[end]);
    }
    const std::size_t middle = begin + (end - begin) / 2U;
    const double heading =
        laser.angle_min.radians() +
        static_cast<double>(middle) * laser.angle_increment.radians();
    const std::size_t width = end - begin + 1U;
    const PassageKind kind =
        width <= 2U ? PassageKind::Doorway : PassageKind::Corridor;
    result.push_back({domain::Angle(heading), domain::Distance(clearance), kind,
                      static_cast<double>(width) /
                          static_cast<double>(laser.ranges_m.size()),
                      begin, end});
    begin = end + 1U;
  }
  if (result.size() >= 3U)
    for (auto& candidate : result)
      candidate.kind = PassageKind::IntersectionBranch;
  return result;
}

HleDecision HighwayExplorer::decide(
    const domain::RobotObservation& observation,
    const domain::ActionSpace& action_space) {
  auto candidates = detectPassages(observation.laser, minimum_clearance_m_);
  if (state_ == HleState::Complete)
    return {domain::Action::pause(), state_, std::move(candidates),
            "exploration complete"};
  if (candidates.empty()) {
    state_ = HleState::Survey;
    return {domain::Action(domain::ActionType::TurnLeft, 1U), state_, {},
            "survey for passage"};
  }
  const auto selected = std::max_element(
      candidates.begin(), candidates.end(), [](const auto& first,
                                                const auto& second) {
        if (first.confidence != second.confidence)
          return first.confidence < second.confidence;
        return first.heading.radians() > second.heading.radians();
      });
  if (candidates.size() >= 3U) state_ = HleState::ConfirmIntersection;
  const double heading = selected->heading.radians();
  if (std::abs(heading) > heading_tolerance_rad_) {
    state_ = HleState::AlignWithPassage;
    return {turnToward(heading, action_space), state_, std::move(candidates),
            "align with widest passage"};
  }
  state_ = HleState::TraversePassage;
  const std::size_t magnitude =
      selected->clearance.meters() > 1.5
          ? action_space.move_distances_m().size()
          : 1U;
  return {domain::Action(domain::ActionType::Forward, magnitude), state_,
          std::move(candidates),
          "traverse selected passage"};
}

}  // namespace semaforr::exploration
