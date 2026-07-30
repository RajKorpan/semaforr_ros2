#ifndef SEMAFORR_EXPLORATION_EXPLORATION_RESULT_HPP
#define SEMAFORR_EXPLORATION_EXPLORATION_RESULT_HPP

#include <optional>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/exploration/exploration_candidate.hpp>
#include <string_view>
#include <vector>

namespace semaforr::exploration {

enum class HleState {
  Initialize,
  DiscoverCandidate,
  ReturnToCandidateStart,
  PursueCandidate,
  RecordPassage,
  SelectNextCandidate,
  FinalizeModel,
  Complete
};

enum class CandidateLifecycleEvent {
  None,
  Discovered,
  Selected,
  PursuitStarted,
  Completed,
  Exhausted
};

enum class ExplorationCompletionReason {
  None,
  CandidateQueueExhausted,
  TimeBudgetExceeded,
  DecisionBudgetExceeded,
  ExplicitlyFinished
};

struct ExplorationSubgoal {
  domain::Point2D position;
  ExplorationCandidateId candidate_id = 0U;
};

struct ExplorationResult {
  domain::Action action = domain::Action::pause();
  std::optional<ExplorationSubgoal> subgoal;
  HleState state = HleState::Initialize;
  CandidateLifecycleEvent event = CandidateLifecycleEvent::None;
  std::optional<ExplorationCandidateId> candidate_id;
  std::uint64_t passage_grid_revision = 0U;
  ExplorationCompletionReason completion_reason =
      ExplorationCompletionReason::None;
  std::vector<ExplorationCandidate> discovered;
  std::string_view rationale;
};

std::string_view toString(HleState state) noexcept;
std::string_view toString(CandidateLifecycleEvent event) noexcept;
std::string_view toString(ExplorationCompletionReason reason) noexcept;

}  // namespace semaforr::exploration

#endif
