#ifndef SEMAFORR_EXPLORATION_PASSAGE_MODEL_HPP
#define SEMAFORR_EXPLORATION_PASSAGE_MODEL_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace semaforr::exploration {

struct PassageCell {
  int row = 0;
  int column = 0;
  std::uint32_t observations = 0U;
};

struct PassageGridSnapshot {
  double resolution_m = 0.5;
  std::vector<PassageCell> cells;
  std::uint64_t revision = 0U;
};

}  // namespace semaforr::exploration

#endif
