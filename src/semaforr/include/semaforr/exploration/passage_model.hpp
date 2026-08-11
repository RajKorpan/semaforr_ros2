#ifndef SEMAFORR_EXPLORATION_PASSAGE_MODEL_HPP
#define SEMAFORR_EXPLORATION_PASSAGE_MODEL_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <semaforr/domain/grid_geometry.hpp>
#include <vector>

namespace semaforr::exploration {

enum class PassageCellState { Free, Obstructed, Passage };

struct PassageCell {
  int row = 0;
  int column = 0;
  PassageCellState state = PassageCellState::Free;
  std::optional<std::uint64_t> passage_id;
  std::uint32_t evidence_count = 0U;
};

struct PassageGridSnapshot {
  domain::GridGeometry geometry;
  std::vector<PassageCell> cells;
  std::uint64_t revision = 0U;
};

const char* toString(PassageCellState state) noexcept;

}  // namespace semaforr::exploration

#endif
