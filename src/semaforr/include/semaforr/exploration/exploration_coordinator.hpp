/**
 * @file exploration_coordinator.hpp
 * @brief Exploration coordinator responsibilities.
 *
 * @details This file defines exploration coordinator behavior for initial or
 * reactive exploration. It centers on `ExplorationUpdate`,
 * `ExplorationCoordinator`. Its package-relative location is
 * `include/semaforr/exploration/exploration_coordinator.hpp`.
 */
#ifndef SEMAFORR_EXPLORATION_EXPLORATION_COORDINATOR_HPP
#define SEMAFORR_EXPLORATION_EXPLORATION_COORDINATOR_HPP

#include <chrono>
#include <functional>
#include <semaforr/exploration/high_level_explorer.hpp>
#include <string>
#include <vector>

namespace semaforr::exploration {

/**
 * @brief Encapsulates exploration update state and behavior for this
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
struct ExplorationUpdate {
  ExplorationResult decision;
  std::vector<std::string> events;
};

/**
 * @brief Encapsulates exploration coordinator state and behavior for this
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
class ExplorationCoordinator {
 public:
  /**
   * @brief Performs the exploration coordinator operation for this
   * subsystem.
   *
   * Arguments:
   * - @p candidate_completion_distance_m: Supplies candidate completion
   * distance m input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit ExplorationCoordinator(double candidate_completion_distance_m = 0.1);
  /**
   * @brief Performs the exploration coordinator operation for this
   * subsystem.
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
  explicit ExplorationCoordinator(HighLevelExplorationConfiguration);

  /**
   * @brief Selects package content for this subsystem.
   *
   * Arguments:
   * - @p observation: Supplies observation input to the operation.
   * - @p action_space: Supplies action space input to the operation.
   *
   * Returns:
   * - `ExplorationUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ExplorationUpdate decide(const domain::RobotObservation& observation,
                           const domain::ActionSpace& action_space);
  /**
   * @brief Sets model finalizer for this subsystem.
   *
   * Arguments:
   * - @p finalizer: Supplies finalizer input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void setModelFinalizer(std::function<void()> finalizer) {
    model_finalizer_ = std::move(finalizer);
  }
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
  void finish() noexcept;
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
  PassageGridSnapshot passageGrid() const { return explorer_.passageGrid(); }
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
  std::vector<ExplorationCandidate> unfinishedCandidates() const {
    return explorer_.unfinishedCandidates();
  }

 private:
  HighLevelExplorer explorer_;
  std::chrono::steady_clock::time_point started_at_{
      /**
       * @brief Performs the now operation for this subsystem.
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
      std::chrono::steady_clock::now()};
  /**
   * @brief Performs the void operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::function<` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::function<void()> model_finalizer_;
  bool finalized_ = false;
};

}  // namespace semaforr::exploration

#endif
