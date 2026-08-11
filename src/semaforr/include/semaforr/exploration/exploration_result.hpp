#ifndef SEMAFORR_EXPLORATION_EXPLORATION_RESULT_HPP
#define SEMAFORR_EXPLORATION_EXPLORATION_RESULT_HPP

#include <optional>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/exploration/exploration_candidate.hpp>
#include <string>
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
  Merged,
  Rejected,
  Selected,
  PursuitStarted,
  Suspended,
  Completed,
  Abandoned,
  Exhausted
};

enum class PursuitTerminationReason {
  None,
  EndpointReached,
  EndOfPassageClearance,
  WidthChanged,
  HardTurn,
  LargeRoom,
  CandidateUnreachable,
  TimeBudgetExceeded,
  DecisionBudgetExceeded,
  ExplicitlyFinished
};

enum class CandidateDiagnosticKind {
  Created,
  Merged,
  Rejected,
  Selected,
  Suspended,
  Completed,
  Abandoned
};

struct CandidateDiagnostic {
  std::uint64_t sequence = 0U;
  ExplorationCandidateId candidate_id = 0U;
  CandidateDiagnosticKind kind = CandidateDiagnosticKind::Created;
  std::string reason;
  ExplorationCandidate candidate;
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
  PursuitTerminationReason pursuit_termination_reason =
      PursuitTerminationReason::None;
  std::vector<ExplorationCandidate> discovered;
  std::vector<CandidateDiagnostic> diagnostics;
  std::string_view rationale;
};

struct HleTraceEntry {
  std::uint64_t decision_id = 0U;
  domain::RobotObservation observation;
  ExplorationResult result;
};

std::string_view toString(HleState state) noexcept;
std::string_view toString(CandidateLifecycleEvent event) noexcept;
std::string_view toString(ExplorationCompletionReason reason) noexcept;
std::string_view toString(PursuitTerminationReason reason) noexcept;
std::string_view toString(CandidateDiagnosticKind kind) noexcept;

}  // namespace semaforr::exploration

#endif
