/**
 * @file passage_model.hpp
 * @brief Passage model responsibilities.
 *
 * @details This file defines passage model behavior for initial or reactive
 * exploration. It centers on `PassageCellState`, `PassageCompletionState`,
 * `PassageCell`, `PassageGridSnapshot`. Its package-relative location is
 * `include/semaforr/exploration/passage_model.hpp`.
 */
#ifndef SEMAFORR_EXPLORATION_PASSAGE_MODEL_HPP
#define SEMAFORR_EXPLORATION_PASSAGE_MODEL_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <semaforr/domain/grid_geometry.hpp>
#include <semaforr/exploration/exploration_candidate.hpp>
#include <vector>

namespace semaforr::exploration {

/**
 * @brief Enumerates the supported passage cell state values used by this
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
enum class PassageCellState { Free, Obstructed, Passage };
/**
 * @brief Enumerates the supported passage completion state values used by
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
enum class PassageCompletionState {
  Unassigned,
  InProgress,
  Suspended,
  Completed,
  Abandoned
};

/**
 * @brief Encapsulates passage cell state and behavior for this subsystem.
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
struct PassageCell {
  int row = 0;
  int column = 0;
  PassageCellState state = PassageCellState::Free;
  std::optional<std::uint64_t> passage_id;
  std::optional<ExplorationCandidateId> candidate_id;
  PassageCompletionState completion_state =
      PassageCompletionState::Unassigned;
  std::uint32_t evidence_count = 0U;
};

/**
 * @brief Encapsulates passage grid snapshot state and behavior for this
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
struct PassageGridSnapshot {
  domain::GridGeometry geometry;
  std::vector<PassageCell> cells;
  std::uint64_t revision = 0U;
};

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p state: Supplies state input to the operation.
 *
 * Returns:
 * - `const char*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const char* toString(PassageCellState state) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p state: Supplies state input to the operation.
 *
 * Returns:
 * - `const char*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const char* toString(PassageCompletionState state) noexcept;

}  // namespace semaforr::exploration

#endif
