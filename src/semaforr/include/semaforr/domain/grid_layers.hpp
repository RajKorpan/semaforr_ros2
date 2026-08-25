/**
 * @file grid_layers.hpp
 * @brief Grid layers responsibilities.
 *
 * @details This file defines grid layers behavior for ROS-independent domain state
 * and value types. It centers on `LazyDenseGridCache`, `SparseCountCell`,
 * `SparseFamiliarityMetadata`, `FamiliarityGrid`, `SensedOccupancyState`,
 * `OccupancyEvidenceSource`, `SensedOccupancyCell`,
 * `SparseSensedOccupancyCell`. Its package-relative location is
 * `include/semaforr/domain/grid_layers.hpp`.
 */
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

/**
 * @brief Encapsulates lazy dense grid cache state and behavior for this
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
template <typename Cell>
struct LazyDenseGridCache {
  std::mutex mutex;
  std::vector<Cell> cells;
  std::size_t revision = 0U;
};

/**
 * @brief Encapsulates sparse count cell state and behavior for this
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
struct SparseCountCell {
  std::size_t index = 0U;
  std::uint32_t value = 0U;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const SparseCountCell&) const = default;
};

/**
 * @brief Encapsulates sparse familiarity metadata state and behavior for
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
struct SparseFamiliarityMetadata {
  std::size_t index = 0U;
  std::size_t last_observed_sequence = 0U;
  float confidence = 0.0F;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const SparseFamiliarityMetadata&) const = default;
};

// Observation history only. A positive count means familiar, never free.
/**
 * @brief Encapsulates familiarity grid state and behavior for this
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

  /**
   * @brief Performs the familiarity grid operation for this subsystem.
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
  FamiliarityGrid() = default;
  /**
   * @brief Performs the familiarity grid operation for this subsystem.
   *
   * Arguments:
   * - @p grid_columns: Supplies grid columns input to the operation.
   * - @p grid_rows: Supplies grid rows input to the operation.
   * - @p grid_resolution_m: Supplies grid resolution m input to the
   * operation.
   * - @p grid_origin: Supplies grid origin input to the operation.
   * - @p observation_counts: Supplies observation counts input to the
   * operation.
   * - @p model_revision: Supplies model revision input to the operation.
   * - @p last_observed: Supplies last observed input to the operation.
   * - @p observation_confidence: Supplies observation confidence input to
   * the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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

  /**
   * @brief Performs the extent operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `GridExtent` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  GridExtent extent() const noexcept {
    GridExtent result{columns, rows, resolution_m, origin, extent_mode,
                      extent_source};
    result.frame_id = frame_id;
    result.geometry_revision = geometry_revision;
    return result;
  }
  /**
   * @brief Performs the valid operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool valid() const noexcept {
    return extent().valid() &&
           (cells.size() == columns * rows ||
            (cells.empty() && std::all_of(
                                  sparseCells().begin(), sparseCells().end(),
                                  [&](const auto& cell) {
                                    return cell.index < columns * rows;
                                  })));
  }
  /**
   * @brief Performs the value at operation for this subsystem.
   *
   * Arguments:
   * - @p index: Supplies index input to the operation.
   *
   * Returns:
   * - `std::uint32_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::uint32_t valueAt(std::size_t index) const noexcept;
  /**
   * @brief Performs the sparse cells operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<SparseCountCell>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<SparseCountCell>& sparseCells() const noexcept {
    return sparse_snapshot ? *sparse_snapshot : sparse_cells;
  }
  /**
   * @brief Performs the sparse metadata operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<SparseFamiliarityMetadata>&` containing the
   * operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<SparseFamiliarityMetadata>& sparseMetadata() const noexcept {
    return sparse_metadata_snapshot ? *sparse_metadata_snapshot
                                    : sparse_metadata;
  }
  /**
   * @brief Performs the dense cells operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<std::uint32_t>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<std::uint32_t>& denseCells() const;
  /**
   * @brief Performs the region of interest operation for this subsystem.
   *
   * Arguments:
   * - @p minimum: Supplies minimum input to the operation.
   * - @p maximum: Supplies maximum input to the operation.
   *
   * Returns:
   * - `std::vector<SparseCountCell>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<SparseCountCell> regionOfInterest(Point2D minimum,
                                                Point2D maximum) const;
  /**
   * @brief Processes d cell count for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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

/**
 * @brief Enumerates the supported sensed occupancy state values used by
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
enum class SensedOccupancyState : std::uint8_t {
  Unknown,
  ObservedFree,
  ObservedOccupied
};

/**
 * @brief Enumerates the supported occupancy evidence source values used by
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

/**
 * @brief Performs the operator operation for this subsystem.
 *
 * Arguments:
 * - @p left: Supplies left input to the operation.
 * - @p right: Supplies right input to the operation.
 *
 * Returns:
 * - `OccupancyEvidenceSource` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
constexpr OccupancyEvidenceSource operator|(OccupancyEvidenceSource left,
                                             OccupancyEvidenceSource right) {
  return static_cast<OccupancyEvidenceSource>(
      static_cast<std::uint8_t>(left) | static_cast<std::uint8_t>(right));
}
/**
 * @brief Reports whether evidence for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 * - @p source: Supplies source input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
constexpr bool hasEvidence(OccupancyEvidenceSource value,
                           OccupancyEvidenceSource source) {
  return (static_cast<std::uint8_t>(value) &
          static_cast<std::uint8_t>(source)) != 0U;
}

/**
 * @brief Encapsulates sensed occupancy cell state and behavior for this
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
struct SensedOccupancyCell {
  SensedOccupancyState state = SensedOccupancyState::Unknown;
  std::uint16_t free_evidence = 0U;
  std::uint16_t occupied_evidence = 0U;
  float confidence = 0.0F;
  std::size_t last_update_sequence = 0U;
  bool conflicting = false;
  bool dynamic = false;
  OccupancyEvidenceSource source = OccupancyEvidenceSource::None;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const SensedOccupancyCell&) const = default;
};

/**
 * @brief Encapsulates sparse sensed occupancy cell state and behavior for
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
struct SparseSensedOccupancyCell {
  std::size_t index = 0U;
  SensedOccupancyCell value;
  /**
   * @brief Performs the operator operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool operator==(const SparseSensedOccupancyCell&) const = default;
};

/**
 * @brief Encapsulates sensed occupancy grid state and behavior for this
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
struct SensedOccupancyGrid {
  GridExtent geometry;
  std::vector<SensedOccupancyCell> cells;
  std::vector<SparseSensedOccupancyCell> sparse_cells;
  std::shared_ptr<const std::vector<SparseSensedOccupancyCell>> sparse_snapshot;
  std::size_t revision = 0U;

  /**
   * @brief Performs the valid operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool valid() const noexcept {
    return geometry.valid() &&
           (cells.size() == geometry.columns * geometry.rows ||
            (cells.empty() && std::all_of(
                                  sparseCells().begin(), sparseCells().end(),
                                  [&](const auto& cell) {
                                    return cell.index < geometry.cellCount();
                                  })));
  }
  /**
   * @brief Processes d cell count for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t observedCellCount() const noexcept {
    if (cells.empty()) return sparseCells().size();
    return static_cast<std::size_t>(std::count_if(
        cells.begin(), cells.end(), [](const auto& cell) {
          return cell.state != SensedOccupancyState::Unknown;
        }));
  }
  /**
   * @brief Performs the value at operation for this subsystem.
   *
   * Arguments:
   * - @p index: Supplies index input to the operation.
   *
   * Returns:
   * - `SensedOccupancyCell` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  SensedOccupancyCell valueAt(std::size_t index) const noexcept;
  /**
   * @brief Performs the sparse cells operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<SparseSensedOccupancyCell>&` containing the
   * operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<SparseSensedOccupancyCell>& sparseCells() const noexcept {
    return sparse_snapshot ? *sparse_snapshot : sparse_cells;
  }
  /**
   * @brief Performs the dense cells operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<SensedOccupancyCell>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<SensedOccupancyCell>& denseCells() const;
  /**
   * @brief Performs the region of interest operation for this subsystem.
   *
   * Arguments:
   * - @p minimum: Supplies minimum input to the operation.
   * - @p maximum: Supplies maximum input to the operation.
   *
   * Returns:
   * - `std::vector<SparseSensedOccupancyCell>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<SparseSensedOccupancyCell> regionOfInterest(
      Point2D minimum, Point2D maximum) const;

 private:
  mutable std::shared_ptr<LazyDenseGridCache<SensedOccupancyCell>> dense_cache_ =
      std::make_shared<LazyDenseGridCache<SensedOccupancyCell>>();
};

/**
 * @brief Encapsulates sparse count grid state and behavior for this
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
struct SparseCountGrid {
  std::size_t columns = 0U;
  std::size_t rows = 0U;
  double resolution_m = 1.0;
  Point2D origin;
  std::vector<std::uint32_t> cells;
  std::vector<SparseCountCell> sparse_cells;
  std::shared_ptr<const std::vector<SparseCountCell>> sparse_snapshot;
  std::size_t revision = 0U;

  /**
   * @brief Performs the sparse count grid operation for this subsystem.
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
  SparseCountGrid() = default;
  /**
   * @brief Performs the sparse count grid operation for this subsystem.
   *
   * Arguments:
   * - @p grid_columns: Supplies grid columns input to the operation.
   * - @p grid_rows: Supplies grid rows input to the operation.
   * - @p grid_resolution_m: Supplies grid resolution m input to the
   * operation.
   * - @p grid_origin: Supplies grid origin input to the operation.
   * - @p dense_cells: Supplies dense cells input to the operation.
   * - @p model_revision: Supplies model revision input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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

  /**
   * @brief Performs the extent operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `GridExtent` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  GridExtent extent() const {
    return {columns, rows, resolution_m, origin};
  }
  /**
   * @brief Performs the valid operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool valid() const noexcept;
  /**
   * @brief Performs the value at operation for this subsystem.
   *
   * Arguments:
   * - @p index: Supplies index input to the operation.
   *
   * Returns:
   * - `std::uint32_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::uint32_t valueAt(std::size_t index) const noexcept;
  /**
   * @brief Performs the sparse cells operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<SparseCountCell>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<SparseCountCell>& sparseCells() const noexcept {
    return sparse_snapshot ? *sparse_snapshot : sparse_cells;
  }
  /**
   * @brief Performs the dense cells operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<std::uint32_t>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<std::uint32_t>& denseCells() const;
  /**
   * @brief Performs the region of interest operation for this subsystem.
   *
   * Arguments:
   * - @p minimum: Supplies minimum input to the operation.
   * - @p maximum: Supplies maximum input to the operation.
   *
   * Returns:
   * - `std::vector<SparseCountCell>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<SparseCountCell> regionOfInterest(Point2D minimum,
                                                Point2D maximum) const;
  /**
   * @brief Processes d cell count for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t observedCellCount() const noexcept;

 private:
  mutable std::shared_ptr<LazyDenseGridCache<std::uint32_t>> dense_cache_ =
      std::make_shared<LazyDenseGridCache<std::uint32_t>>();
};

/**
 * @brief Enumerates the supported static occupancy state values used by
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
enum class StaticOccupancyState : std::uint8_t {
  StaticFree,
  StaticOccupied,
  StaticUnknown
};

/**
 * @brief Enumerates the supported traversability state values used by this
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
enum class TraversabilityState : std::uint8_t {
  Traversable,
  NonTraversable,
  UnknownPermitted,
  UnknownProhibited,
  InflatedObstacle,
  OutsidePlanningExtent
};

/**
 * @brief Enumerates the supported unknown space policy values used by this
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
enum class UnknownSpacePolicy : std::uint8_t {
  Prohibited,
  HighCost,
  WithinSensorRange,
  ExplorationOnly
};

/**
 * @brief Encapsulates traversability cell state and behavior for this
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
struct TraversabilityCell {
  TraversabilityState state = TraversabilityState::OutsidePlanningExtent;
  float cost_multiplier = 1.0F;
  OccupancyEvidenceSource provenance =
      OccupancyEvidenceSource::OutsideGridExtent;

  /**
   * @brief Performs the permits traversal operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool permitsTraversal() const noexcept {
    return state == TraversabilityState::Traversable ||
           state == TraversabilityState::UnknownPermitted;
  }
};

/**
 * @brief Encapsulates traversability grid state and behavior for this
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
struct TraversabilityGrid {
  GridExtent geometry;
  std::vector<TraversabilityCell> cells;
  UnknownSpacePolicy unknown_policy = UnknownSpacePolicy::Prohibited;
  bool complete_prior_bounds = false;
  std::size_t source_static_revision = 0U;
  std::size_t source_sensed_revision = 0U;

  /**
   * @brief Performs the valid operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool valid() const noexcept {
    return geometry.valid() && cells.size() == geometry.columns * geometry.rows;
  }
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_GRID_LAYERS_HPP
