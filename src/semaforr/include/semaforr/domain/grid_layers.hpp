#ifndef SEMAFORR_DOMAIN_GRID_LAYERS_HPP
#define SEMAFORR_DOMAIN_GRID_LAYERS_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <memory>
#include <mutex>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/grid_geometry.hpp>
#include <string>
#include <vector>
#include <utility>

namespace semaforr::domain {

using GridExtent = GridGeometry;

template <typename Cell>
struct LazyDenseGridCache {
  std::mutex mutex;
  std::vector<Cell> cells;
  std::size_t revision = 0U;
};

struct SparseCountCell {
  std::size_t index = 0U;
  std::uint32_t value = 0U;
  bool operator==(const SparseCountCell&) const = default;
};

struct SparseFamiliarityMetadata {
  std::size_t index = 0U;
  std::size_t last_observed_sequence = 0U;
  float confidence = 0.0F;
  bool operator==(const SparseFamiliarityMetadata&) const = default;
};

// Observation history only. A positive count means familiar, never free.
struct FamiliarityGrid {
  std::size_t columns = 0U;
  std::size_t rows = 0U;
  double resolution_m = 1.0;
  Point2D origin;
  std::vector<std::uint32_t> cells;
  std::size_t revision = 0U;
  std::vector<std::size_t> last_observed_sequence;
  std::vector<float> confidence;
  std::vector<SparseCountCell> sparse_cells;
  std::vector<SparseFamiliarityMetadata> sparse_metadata;
  std::shared_ptr<const std::vector<SparseCountCell>> sparse_snapshot;
  std::shared_ptr<const std::vector<SparseFamiliarityMetadata>>
      sparse_metadata_snapshot;
  std::string frame_id{"map"};
  std::size_t geometry_revision{0U};
  GridExtentMode extent_mode = GridExtentMode::Expandable;
  GridExtentSource extent_source =
      GridExtentSource::ConfiguredMaplessInitialBounds;

  FamiliarityGrid() = default;
  FamiliarityGrid(std::size_t grid_columns, std::size_t grid_rows,
                  double grid_resolution_m, Point2D grid_origin,
                  std::vector<std::uint32_t> observation_counts,
                  std::size_t model_revision,
                  std::vector<std::size_t> last_observed = {},
                  std::vector<float> observation_confidence = {})
      : columns(grid_columns),
        rows(grid_rows),
        resolution_m(grid_resolution_m),
        origin(grid_origin),
        cells(std::move(observation_counts)),
        revision(model_revision),
        last_observed_sequence(std::move(last_observed)),
        confidence(std::move(observation_confidence)) {}

  GridExtent extent() const noexcept {
    GridExtent result{columns, rows, resolution_m, origin, extent_mode,
                      extent_source};
    result.frame_id = frame_id;
    result.geometry_revision = geometry_revision;
    return result;
  }
  bool valid() const noexcept {
    return extent().valid() &&
           (cells.size() == columns * rows ||
            (cells.empty() && std::all_of(
                                  sparseCells().begin(), sparseCells().end(),
                                  [&](const auto& cell) {
                                    return cell.index < columns * rows;
                                  })));
  }
  std::uint32_t valueAt(std::size_t index) const noexcept;
  const std::vector<SparseCountCell>& sparseCells() const noexcept {
    return sparse_snapshot ? *sparse_snapshot : sparse_cells;
  }
  const std::vector<SparseFamiliarityMetadata>& sparseMetadata() const noexcept {
    return sparse_metadata_snapshot ? *sparse_metadata_snapshot
                                    : sparse_metadata;
  }
  const std::vector<std::uint32_t>& denseCells() const;
  std::vector<SparseCountCell> regionOfInterest(Point2D minimum,
                                                Point2D maximum) const;
  std::size_t observedCellCount() const noexcept {
    return cells.empty()
               ? sparseCells().size()
               : static_cast<std::size_t>(std::count_if(
                     cells.begin(), cells.end(),
                     [](std::uint32_t value) { return value != 0U; }));
  }

 private:
  mutable std::shared_ptr<LazyDenseGridCache<std::uint32_t>> dense_cache_ =
      std::make_shared<LazyDenseGridCache<std::uint32_t>>();
};

enum class SensedOccupancyState : std::uint8_t {
  Unknown,
  ObservedFree,
  ObservedOccupied
};

enum class OccupancyEvidenceSource : std::uint8_t {
  None = 0,
  StaticMap = 1U << 0U,
  CurrentSensor = 1U << 1U,
  AccumulatedSensorModel = 1U << 2U,
  DynamicObstacle = 1U << 3U,
  Inflation = 1U << 4U,
  UnknownSpacePolicy = 1U << 5U,
  OutsideGridExtent = 1U << 6U
};

constexpr OccupancyEvidenceSource operator|(OccupancyEvidenceSource left,
                                             OccupancyEvidenceSource right) {
  return static_cast<OccupancyEvidenceSource>(
      static_cast<std::uint8_t>(left) | static_cast<std::uint8_t>(right));
}
constexpr bool hasEvidence(OccupancyEvidenceSource value,
                           OccupancyEvidenceSource source) {
  return (static_cast<std::uint8_t>(value) &
          static_cast<std::uint8_t>(source)) != 0U;
}

struct SensedOccupancyCell {
  SensedOccupancyState state = SensedOccupancyState::Unknown;
  std::uint16_t free_evidence = 0U;
  std::uint16_t occupied_evidence = 0U;
  float confidence = 0.0F;
  std::size_t last_update_sequence = 0U;
  bool conflicting = false;
  bool dynamic = false;
  OccupancyEvidenceSource source = OccupancyEvidenceSource::None;
  bool operator==(const SensedOccupancyCell&) const = default;
};

struct SparseSensedOccupancyCell {
  std::size_t index = 0U;
  SensedOccupancyCell value;
  bool operator==(const SparseSensedOccupancyCell&) const = default;
};

struct SensedOccupancyGrid {
  GridExtent geometry;
  std::vector<SensedOccupancyCell> cells;
  std::vector<SparseSensedOccupancyCell> sparse_cells;
  std::shared_ptr<const std::vector<SparseSensedOccupancyCell>> sparse_snapshot;
  std::size_t revision = 0U;

  bool valid() const noexcept {
    return geometry.valid() &&
           (cells.size() == geometry.columns * geometry.rows ||
            (cells.empty() && std::all_of(
                                  sparseCells().begin(), sparseCells().end(),
                                  [&](const auto& cell) {
                                    return cell.index < geometry.cellCount();
                                  })));
  }
  std::size_t observedCellCount() const noexcept {
    if (cells.empty()) return sparseCells().size();
    return static_cast<std::size_t>(std::count_if(
        cells.begin(), cells.end(), [](const auto& cell) {
          return cell.state != SensedOccupancyState::Unknown;
        }));
  }
  SensedOccupancyCell valueAt(std::size_t index) const noexcept;
  const std::vector<SparseSensedOccupancyCell>& sparseCells() const noexcept {
    return sparse_snapshot ? *sparse_snapshot : sparse_cells;
  }
  const std::vector<SensedOccupancyCell>& denseCells() const;
  std::vector<SparseSensedOccupancyCell> regionOfInterest(
      Point2D minimum, Point2D maximum) const;

 private:
  mutable std::shared_ptr<LazyDenseGridCache<SensedOccupancyCell>> dense_cache_ =
      std::make_shared<LazyDenseGridCache<SensedOccupancyCell>>();
};

struct SparseCountGrid {
  std::size_t columns = 0U;
  std::size_t rows = 0U;
  double resolution_m = 1.0;
  Point2D origin;
  std::vector<std::uint32_t> cells;
  std::vector<SparseCountCell> sparse_cells;
  std::shared_ptr<const std::vector<SparseCountCell>> sparse_snapshot;
  std::size_t revision = 0U;

  SparseCountGrid() = default;
  SparseCountGrid(std::size_t grid_columns, std::size_t grid_rows,
                  double grid_resolution_m, Point2D grid_origin,
                  std::vector<std::uint32_t> dense_cells,
                  std::size_t model_revision)
      : columns(grid_columns),
        rows(grid_rows),
        resolution_m(grid_resolution_m),
        origin(grid_origin),
        cells(std::move(dense_cells)),
        revision(model_revision) {}

  GridExtent extent() const {
    return {columns, rows, resolution_m, origin};
  }
  bool valid() const noexcept;
  std::uint32_t valueAt(std::size_t index) const noexcept;
  const std::vector<SparseCountCell>& sparseCells() const noexcept {
    return sparse_snapshot ? *sparse_snapshot : sparse_cells;
  }
  const std::vector<std::uint32_t>& denseCells() const;
  std::vector<SparseCountCell> regionOfInterest(Point2D minimum,
                                                Point2D maximum) const;
  std::size_t observedCellCount() const noexcept;

 private:
  mutable std::shared_ptr<LazyDenseGridCache<std::uint32_t>> dense_cache_ =
      std::make_shared<LazyDenseGridCache<std::uint32_t>>();
};

enum class StaticOccupancyState : std::uint8_t {
  StaticFree,
  StaticOccupied,
  StaticUnknown
};

enum class TraversabilityState : std::uint8_t {
  Traversable,
  NonTraversable,
  UnknownPermitted,
  UnknownProhibited,
  InflatedObstacle,
  OutsidePlanningExtent
};

enum class UnknownSpacePolicy : std::uint8_t {
  Prohibited,
  HighCost,
  WithinSensorRange,
  ExplorationOnly
};

struct TraversabilityCell {
  TraversabilityState state = TraversabilityState::OutsidePlanningExtent;
  float cost_multiplier = 1.0F;
  OccupancyEvidenceSource provenance =
      OccupancyEvidenceSource::OutsideGridExtent;

  bool permitsTraversal() const noexcept {
    return state == TraversabilityState::Traversable ||
           state == TraversabilityState::UnknownPermitted;
  }
};

struct TraversabilityGrid {
  GridExtent geometry;
  std::vector<TraversabilityCell> cells;
  UnknownSpacePolicy unknown_policy = UnknownSpacePolicy::Prohibited;
  bool complete_prior_bounds = false;
  std::size_t source_static_revision = 0U;
  std::size_t source_sensed_revision = 0U;

  bool valid() const noexcept {
    return geometry.valid() && cells.size() == geometry.columns * geometry.rows;
  }
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_GRID_LAYERS_HPP
