#include <algorithm>
#include <semaforr/domain/spatial_affordances.hpp>

namespace semaforr::domain {

const ConveyorCell* ConveyorGrid::at(Point2D point) const noexcept {
  const auto index = geometry.index(point);
  if (!index) return nullptr;
  const auto found = std::lower_bound(
      cells.begin(), cells.end(), *index,
      [](const ConveyorCell& cell, std::size_t value) {
        return cell.index < value;
      });
  return found != cells.end() && found->index == *index ? &*found : nullptr;
}

}  // namespace semaforr::domain
