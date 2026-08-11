#ifndef SEMAFORR_EXPLORATION_HIGH_LEVEL_EXPLORER_HPP
#define SEMAFORR_EXPLORATION_HIGH_LEVEL_EXPLORER_HPP

#include <queue>
#include <semaforr/exploration/exploration_strategy.hpp>
#include <semaforr/exploration/passage_model.hpp>
#include <unordered_map>
#include <unordered_set>

namespace semaforr::exploration {

class HighLevelExplorer final : public ExplorationStrategy {
 public:
  explicit HighLevelExplorer(HighLevelExplorationConfiguration = {});

  ExplorationResult update(const ExplorationInput& input) override;
  void finish() noexcept override;
  HleState state() const noexcept { return state_; }
  PassageGridSnapshot passageGrid() const;
  std::vector<ExplorationCandidate> unfinishedCandidates() const;
  const std::vector<domain::Point2D>& explorationPath() const noexcept {
    return exploration_path_;
  }

  static std::vector<ExplorationCandidate> discoverCandidates(
      const domain::RobotObservation&, const HighLevelExplorationConfiguration&);

 private:
  using CandidateQueue =
      std::priority_queue<ExplorationCandidate,
                          std::vector<ExplorationCandidate>, CandidatePriority>;
  static std::int64_t cellKey(int row, int column) noexcept;
  std::int64_t cueKey(const domain::Point2D&) const noexcept;
  std::vector<ExplorationCandidate> discover(
      const domain::RobotObservation&);
  void updatePassageGrid(const domain::RobotObservation&,
                         ExplorationCandidateId passage_id,
                         domain::Point2D passage_start);
  domain::Action pursue(const ExplorationInput&) const;

  HighLevelExplorationConfiguration configuration_;
  HleState state_ = HleState::Initialize;
  ExplorationCompletionReason completion_reason_ =
      ExplorationCompletionReason::None;
  CandidateQueue candidates_;
  std::unordered_set<std::int64_t> cue_cells_;
  std::unordered_map<std::int64_t, PassageCell> passage_cells_;
  std::optional<domain::Point2D> passage_grid_reference_;
  std::vector<domain::Point2D> exploration_path_;
  std::optional<ExplorationCandidate> active_;
  ExplorationCandidateId next_candidate_id_ = 1U;
  std::uint64_t passage_revision_ = 0U;
  std::size_t decisions_ = 0U;
  bool pursuit_started_ = false;
};

}  // namespace semaforr::exploration

#endif
