#include <semaforr/domain/grid_layers.hpp>

namespace semaforr::domain {
namespace {

template <typename SparseCell>
auto findSparse(const std::vector<SparseCell>& cells, std::size_t index) {
  return std::lower_bound(cells.begin(), cells.end(), index,
                          [](const auto& cell, std::size_t candidate) {
                            return cell.index < candidate;
                          });
}

template <typename SparseCell>
std::vector<SparseCell> roi(const std::vector<SparseCell>& cells,
                            const GridGeometry& geometry, Point2D minimum,
                            Point2D maximum) {
  std::vector<SparseCell> result;
  for (const auto& cell : cells) {
    const auto center = geometry.center(cell.index);
    if (center.x_m >= minimum.x_m && center.x_m <= maximum.x_m &&
        center.y_m >= minimum.y_m && center.y_m <= maximum.y_m)
      result.push_back(cell);
  }
  return result;
}

}  // namespace

std::uint32_t FamiliarityGrid::valueAt(std::size_t index) const noexcept {
  if (!cells.empty()) return index < cells.size() ? cells[index] : 0U;
  const auto& sparse = sparseCells();
  const auto found = findSparse(sparse, index);
  return found != sparse.end() && found->index == index ? found->value : 0U;
}

const std::vector<std::uint32_t>& FamiliarityGrid::denseCells() const {
  if (!cells.empty()) return cells;
  std::scoped_lock lock(dense_cache_->mutex);
  if (dense_cache_->revision != revision ||
      dense_cache_->cells.size() != columns * rows) {
    dense_cache_->cells.assign(columns * rows, 0U);
    for (const auto& cell : sparseCells())
      if (cell.index < dense_cache_->cells.size())
        dense_cache_->cells[cell.index] = cell.value;
    dense_cache_->revision = revision;
  }
  return dense_cache_->cells;
}

std::vector<SparseCountCell> FamiliarityGrid::regionOfInterest(
    Point2D minimum, Point2D maximum) const {
  if (!sparseCells().empty())
    return roi(sparseCells(), extent(), minimum, maximum);
  std::vector<SparseCountCell> sparse;
  for (std::size_t index = 0U; index < cells.size(); ++index)
    if (cells[index] != 0U) sparse.push_back({index, cells[index]});
  return roi(sparse, extent(), minimum, maximum);
}

SensedOccupancyCell SensedOccupancyGrid::valueAt(
    std::size_t index) const noexcept {
  if (!cells.empty()) return index < cells.size() ? cells[index]
                                                  : SensedOccupancyCell{};
  const auto& sparse = sparseCells();
  const auto found = findSparse(sparse, index);
  return found != sparse.end() && found->index == index
             ? found->value
             : SensedOccupancyCell{};
}

const std::vector<SensedOccupancyCell>& SensedOccupancyGrid::denseCells() const {
  if (!cells.empty()) return cells;
  std::scoped_lock lock(dense_cache_->mutex);
  if (dense_cache_->revision != revision ||
      dense_cache_->cells.size() != geometry.cellCount()) {
    dense_cache_->cells.assign(geometry.cellCount(), {});
    for (const auto& cell : sparseCells())
      if (cell.index < dense_cache_->cells.size())
        dense_cache_->cells[cell.index] = cell.value;
    dense_cache_->revision = revision;
  }
  return dense_cache_->cells;
}

std::vector<SparseSensedOccupancyCell> SensedOccupancyGrid::regionOfInterest(
    Point2D minimum, Point2D maximum) const {
  if (!sparseCells().empty())
    return roi(sparseCells(), geometry, minimum, maximum);
  std::vector<SparseSensedOccupancyCell> sparse;
  for (std::size_t index = 0U; index < cells.size(); ++index)
    if (cells[index].state != SensedOccupancyState::Unknown)
      sparse.push_back({index, cells[index]});
  return roi(sparse, geometry, minimum, maximum);
}

bool SparseCountGrid::valid() const noexcept {
  return extent().valid() &&
         (cells.size() == columns * rows ||
          (cells.empty() && std::all_of(sparseCells().begin(), sparseCells().end(),
                                        [&](const auto& cell) {
                                          return cell.index < columns * rows;
                                        })));
}

std::uint32_t SparseCountGrid::valueAt(std::size_t index) const noexcept {
  if (!cells.empty()) return index < cells.size() ? cells[index] : 0U;
  const auto& sparse = sparseCells();
  const auto found = findSparse(sparse, index);
  return found != sparse.end() && found->index == index ? found->value : 0U;
}

const std::vector<std::uint32_t>& SparseCountGrid::denseCells() const {
  if (!cells.empty()) return cells;
  std::scoped_lock lock(dense_cache_->mutex);
  if (dense_cache_->revision != revision ||
      dense_cache_->cells.size() != columns * rows) {
    dense_cache_->cells.assign(columns * rows, 0U);
    for (const auto& cell : sparseCells())
      if (cell.index < dense_cache_->cells.size())
        dense_cache_->cells[cell.index] = cell.value;
    dense_cache_->revision = revision;
  }
  return dense_cache_->cells;
}

std::vector<SparseCountCell> SparseCountGrid::regionOfInterest(
    Point2D minimum, Point2D maximum) const {
  if (!sparseCells().empty())
    return roi(sparseCells(), extent(), minimum, maximum);
  std::vector<SparseCountCell> sparse;
  for (std::size_t index = 0U; index < cells.size(); ++index)
    if (cells[index] != 0U) sparse.push_back({index, cells[index]});
  return roi(sparse, extent(), minimum, maximum);
}

std::size_t SparseCountGrid::observedCellCount() const noexcept {
  return cells.empty()
             ? sparseCells().size()
             : static_cast<std::size_t>(std::count_if(
                   cells.begin(), cells.end(),
                   [](std::uint32_t value) { return value != 0U; }));
}

}  // namespace semaforr::domain
