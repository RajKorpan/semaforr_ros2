/**
 * @file high_level_explorer.hpp
 * @brief High level explorer responsibilities.
 *
 * @details This file defines high level explorer behavior for initial or reactive
 * exploration. It centers on `HighLevelExplorer`. Its package-relative
 * location is `include/semaforr/exploration/high_level_explorer.hpp`.
 */
#ifndef SEMAFORR_EXPLORATION_HIGH_LEVEL_EXPLORER_HPP
#define SEMAFORR_EXPLORATION_HIGH_LEVEL_EXPLORER_HPP

#include <array>
#include <queue>
#include <semaforr/exploration/exploration_strategy.hpp>
#include <semaforr/exploration/passage_model.hpp>
#include <unordered_map>
#include <unordered_set>

namespace semaforr::exploration {

/**
 * @brief Encapsulates high level explorer state and behavior for this
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
class HighLevelExplorer final : public ExplorationStrategy {
 public:
  /**
   * @brief Performs the high level explorer operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit HighLevelExplorer(HighLevelExplorationConfiguration = {});

  /**
   * @brief Updates package content for this subsystem.
   *
   * Arguments:
   * - @p input: Supplies input input to the operation.
   *
   * Returns:
   * - `ExplorationResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ExplorationResult update(const ExplorationInput& input) override;
  /**
   * @brief Performs the finish operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void finish() noexcept override;
  /**
   * @brief Performs the state operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `HleState` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  HleState state() const noexcept { return state_; }
  /**
   * @brief Performs the passage grid operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `PassageGridSnapshot` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PassageGridSnapshot passageGrid() const;
  /**
   * @brief Performs the restore passage grid operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void restorePassageGrid(const PassageGridSnapshot&);
  /**
   * @brief Performs the unfinished candidates operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<ExplorationCandidate>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<ExplorationCandidate> unfinishedCandidates() const;
  /**
   * @brief Performs the exploration path operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<domain::Point2D>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<domain::Point2D>& explorationPath() const noexcept {
    return exploration_path_;
  }
  /**
   * @brief Performs the candidate diagnostics operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<CandidateDiagnostic>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<CandidateDiagnostic>& candidateDiagnostics() const noexcept {
    return candidate_diagnostics_;
  }
  /**
   * @brief Performs the trace operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<HleTraceEntry>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<HleTraceEntry>& trace() const noexcept { return trace_; }
  /**
   * @brief Performs the replay operation for this subsystem.
   *
   * Arguments:
   * - @p HleTraceEntry: Supplies hle trace entry input to the operation.
   *
   * Returns:
   * - `std::vector<ExplorationResult>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static std::vector<ExplorationResult> replay(
      const std::vector<HleTraceEntry>&);

  /**
   * @brief Performs the discover candidates operation for this subsystem.
   *
   * Arguments:
   * - @p RobotObservation: Supplies robot observation input to the
   * operation.
   * - @p argument_2: Supplies argument 2 input to the operation.
   *
   * Returns:
   * - `std::vector<ExplorationCandidate>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static std::vector<ExplorationCandidate> discoverCandidates(
      const domain::RobotObservation&, const HighLevelExplorationConfiguration&);
  /**
   * @brief Performs the measure bundles operation for this subsystem.
   *
   * Arguments:
   * - @p RobotObservation: Supplies robot observation input to the
   * operation.
   * - @p argument_2: Supplies argument 2 input to the operation.
   *
   * Returns:
   * - `std::array<HleBundleMeasurement, 4U>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static std::array<HleBundleMeasurement, 4U> measureBundles(
      const domain::RobotObservation&, const HighLevelExplorationConfiguration&);
  /**
   * @brief Evaluates cue for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   * - @p RobotObservation: Supplies robot observation input to the
   * operation.
   *
   * Returns:
   * - `CueValidation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  CueValidation evaluateCue(const ExplorationCandidate&,
                            const domain::RobotObservation&) const;
  /**
   * @brief Performs the cues similar operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   * - @p argument_2: Supplies argument 2 input to the operation.
   * - @p tolerance_m: Supplies tolerance m input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static bool cuesSimilar(const ExplorationCandidate&,
                          const ExplorationCandidate&, double tolerance_m);

 private:
  using CandidateQueue =
      std::priority_queue<ExplorationCandidate,
                          std::vector<ExplorationCandidate>, CandidatePriority>;
  /**
   * @brief Performs the cell key operation for this subsystem.
   *
   * Arguments:
   * - @p row: Supplies row input to the operation.
   * - @p column: Supplies column input to the operation.
   *
   * Returns:
   * - `std::int64_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  static std::int64_t cellKey(int row, int column) noexcept;
  /**
   * @brief Performs the cue key operation for this subsystem.
   *
   * Arguments:
   * - @p Point2D: Supplies point2 d input to the operation.
   *
   * Returns:
   * - `std::int64_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::int64_t cueKey(const domain::Point2D&) const noexcept;
  /**
   * @brief Performs the discover operation for this subsystem.
   *
   * Arguments:
   * - @p RobotObservation: Supplies robot observation input to the
   * operation.
   *
   * Returns:
   * - `std::vector<ExplorationCandidate>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<ExplorationCandidate> discover(
      const domain::RobotObservation&);
  /**
   * @brief Performs the merge target operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `std::optional<ExplorationCandidateId>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<ExplorationCandidateId> mergeTarget(
      const ExplorationCandidate&) const;
  /**
   * @brief Performs the merge candidate operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   * - @p argument_2: Supplies argument 2 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void mergeCandidate(ExplorationCandidateId,
                      const ExplorationCandidate&);
  /**
   * @brief Records diagnostic for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   * - @p argument_2: Supplies argument 2 input to the operation.
   * - @p string: Supplies string input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void recordDiagnostic(ExplorationCandidateId, CandidateDiagnosticKind,
                        std::string);
  /**
   * @brief Records trace for this subsystem.
   *
   * Arguments:
   * - @p RobotObservation: Supplies robot observation input to the
   * operation.
   * - @p argument_2: Supplies argument 2 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void recordTrace(const domain::RobotObservation&, const ExplorationResult&);
  /**
   * @brief Updates candidate extension for this subsystem.
   *
   * Arguments:
   * - @p RobotObservation: Supplies robot observation input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void updateCandidateExtension(const domain::RobotObservation&);
  /**
   * @brief Performs the pursuit termination operation for this subsystem.
   *
   * Arguments:
   * - @p RobotObservation: Supplies robot observation input to the
   * operation.
   *
   * Returns:
   * - `PursuitTerminationReason` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  PursuitTerminationReason pursuitTermination(
      const domain::RobotObservation&) const;
  /**
   * @brief Updates passage grid for this subsystem.
   *
   * Arguments:
   * - @p RobotObservation: Supplies robot observation input to the
   * operation.
   * - @p passage_id: Supplies passage id input to the operation.
   * - @p passage_number: Supplies passage number input to the operation.
   * - @p passage_start: Supplies passage start input to the operation.
   * - @p completion_state: Supplies completion state input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void updatePassageGrid(const domain::RobotObservation&,
                         ExplorationCandidateId passage_id,
                         std::uint64_t passage_number,
                         domain::Point2D passage_start,
                         PassageCompletionState completion_state);
  /**
   * @brief Performs the pursue operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `domain::Action` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Action pursue(const ExplorationInput&) const;

  HighLevelExplorationConfiguration configuration_;
  HleState state_ = HleState::Initialize;
  ExplorationCompletionReason completion_reason_ =
      ExplorationCompletionReason::None;
  CandidateQueue candidates_;
  std::unordered_set<std::int64_t> cue_cells_;
  std::unordered_map<ExplorationCandidateId, ExplorationCandidate>
      candidate_registry_;
  std::unordered_map<std::int64_t, PassageCell> passage_cells_;
  std::optional<domain::Point2D> passage_grid_reference_;
  std::vector<domain::Point2D> exploration_path_;
  std::optional<ExplorationCandidate> active_;
  ExplorationCandidateId next_candidate_id_ = 1U;
  std::uint64_t next_passage_id_ = 1U;
  std::uint64_t passage_revision_ = 0U;
  std::uint64_t observation_sequence_ = 0U;
  std::uint64_t diagnostic_sequence_ = 0U;
  std::size_t decisions_ = 0U;
  bool pursuit_started_ = false;
  PursuitTerminationReason active_termination_reason_ =
      PursuitTerminationReason::None;
  std::vector<CandidateDiagnostic> candidate_diagnostics_;
  std::vector<HleTraceEntry> trace_;
};

}  // namespace semaforr::exploration

#endif
