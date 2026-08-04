#ifndef SEMAFORR_SPATIAL_COVERAGE_HPP
#define SEMAFORR_SPATIAL_COVERAGE_HPP

#include <cstddef>
#include <semaforr/domain/world_model.hpp>

namespace semaforr::spatial {

// Counts one-metre cells overlapped by the union of learned regions and trails.
std::size_t representedCoverageCells(const domain::SpatialModel& model);

}  // namespace semaforr::spatial

#endif
