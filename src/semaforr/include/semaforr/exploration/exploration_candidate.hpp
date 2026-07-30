#ifndef SEMAFORR_EXPLORATION_EXPLORATION_CANDIDATE_HPP
#define SEMAFORR_EXPLORATION_EXPLORATION_CANDIDATE_HPP

#include <cstddef>
#include <cstdint>
#include <semaforr/domain/geometry.hpp>

namespace semaforr::exploration {

using ExplorationCandidateId = std::uint64_t;

enum class PassageKind { Corridor, Doorway, IntersectionBranch };

struct ExplorationCandidate {
  ExplorationCandidateId id = 0U;
  domain::Point2D start;
  domain::Angle heading;
  domain::Distance clearance;
  PassageKind kind = PassageKind::Corridor;
  double priority = 0.0;
  std::size_t first_beam = 0U;
  std::size_t last_beam = 0U;
};

struct CandidatePriority {
  bool operator()(const ExplorationCandidate& left,
                  const ExplorationCandidate& right) const noexcept {
    if (left.priority != right.priority)
      return left.priority < right.priority;
    return left.id > right.id;
  }
};

}  // namespace semaforr::exploration

#endif
