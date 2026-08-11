#ifndef SEMAFORR_EXPLORATION_EXPLORATION_CANDIDATE_HPP
#define SEMAFORR_EXPLORATION_EXPLORATION_CANDIDATE_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <semaforr/domain/geometry.hpp>
#include <string>

namespace semaforr::exploration {

using ExplorationCandidateId = std::uint64_t;

enum class PassageKind { Corridor, Doorway, IntersectionBranch, LargeRoom };
enum class PassageCueType { Generic, LeftOpen, RightOpen };
enum class ExplorationCandidateState {
  Queued,
  Selected,
  Pursuing,
  Suspended,
  Completed,
  Abandoned
};

struct ExplorationCandidate {
  ExplorationCandidateId id = 0U;
  domain::Point2D start;
  domain::Point2D endpoint;
  // Global direction. `heading` is retained as the discovery-relative angle
  // for legacy consumers and is never used for pursuit control.
  domain::Angle direction = domain::Angle::zero();
  domain::Angle heading = domain::Angle::zero();
  domain::Distance clearance = domain::Distance::zero();
  domain::Distance length = domain::Distance::zero();
  domain::Distance width = domain::Distance::zero();
  domain::Distance current_width = domain::Distance::zero();
  PassageKind kind = PassageKind::Corridor;
  PassageCueType cue_type = PassageCueType::Generic;
  double priority = 0.0;
  double confidence = 0.0;
  std::size_t first_beam = 0U;
  std::size_t last_beam = 0U;
  std::uint64_t discovery_observation_id = 0U;
  domain::Distance current_extension = domain::Distance::zero();
  std::optional<std::uint64_t> passage_id;
  ExplorationCandidateState state = ExplorationCandidateState::Queued;
  std::uint64_t revision = 1U;
};

struct CandidatePriority {
  bool operator()(const ExplorationCandidate& left,
                  const ExplorationCandidate& right) const noexcept {
    if (left.priority != right.priority)
      return left.priority < right.priority;
    return left.id > right.id;
  }
};

struct CueValidation {
  bool start_clear = false;
  bool midpoint_clear = false;
  bool endpoint_clear = false;
  std::size_t passage_identities = 0U;
  bool geometrically_reachable = false;
  bool accepted = false;
  std::string reason;
};

}  // namespace semaforr::exploration

#endif
