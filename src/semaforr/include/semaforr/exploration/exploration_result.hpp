/**
 * @file exploration_result.hpp
 * @brief Exploration result responsibilities.
 *
 * @details This file defines exploration result behavior for initial or
 * reactive exploration. It centers on `HleState`, `CandidateLifecycleEvent`,
 * `PursuitTerminationReason`, `CandidateDiagnosticKind`,
 * `CandidateDiagnostic`, `ExplorationCompletionReason`,
 * `ExplorationSubgoal`, `ExplorationResult`. Its package-relative location
 * is `include/semaforr/exploration/exploration_result.hpp`.
 */
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

/**
 * @brief Enumerates the supported hle state values used by this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Enumerates the supported candidate lifecycle event values used by
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Enumerates the supported pursuit termination reason values used by
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Enumerates the supported candidate diagnostic kind values used by
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class CandidateDiagnosticKind {
  Created,
  Merged,
  Rejected,
  Selected,
  Suspended,
  Completed,
  Abandoned
};

/**
 * @brief Encapsulates candidate diagnostic state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct CandidateDiagnostic {
  std::uint64_t sequence = 0U;
  ExplorationCandidateId candidate_id = 0U;
  CandidateDiagnosticKind kind = CandidateDiagnosticKind::Created;
  std::string reason;
  ExplorationCandidate candidate;
};

/**
 * @brief Enumerates the supported exploration completion reason values used
 * by this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class ExplorationCompletionReason {
  None,
  CandidateQueueExhausted,
  TimeBudgetExceeded,
  DecisionBudgetExceeded,
  ExplicitlyFinished
};

/**
 * @brief Encapsulates exploration subgoal state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct ExplorationSubgoal {
  domain::Point2D position;
  ExplorationCandidateId candidate_id = 0U;
};

/**
 * @brief Encapsulates exploration result state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Encapsulates hle trace entry state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct HleTraceEntry {
  std::uint64_t decision_id = 0U;
  domain::RobotObservation observation;
  ExplorationResult result;
};

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p state: Supplies state input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(HleState state) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p event: Supplies event input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(CandidateLifecycleEvent event) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p reason: Supplies reason input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(ExplorationCompletionReason reason) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p reason: Supplies reason input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(PursuitTerminationReason reason) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p kind: Supplies kind input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(CandidateDiagnosticKind kind) noexcept;

}  // namespace semaforr::exploration

#endif
