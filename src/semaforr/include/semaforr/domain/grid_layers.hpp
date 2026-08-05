#ifndef SEMAFORR_DOMAIN_GRID_LAYERS_HPP
#define SEMAFORR_DOMAIN_GRID_LAYERS_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <semaforr/domain/geometry.hpp>
#include <vector>
#include <utility>

namespace semaforr::domain {

struct GridExtent {
  std::size_t columns = 0U;
  std::size_t rows = 0U;
  double resolution_m = 1.0;
  Point2D origin;

  bool valid() const noexcept {
    return columns > 0U && rows > 0U && std::isfinite(resolution_m) &&
           resolution_m > 0.0 && origin.finite();
  }
  std::optional<std::size_t> index(Point2D point) const noexcept {
    if (!valid()) return std::nullopt;
    const auto column = static_cast<long long>(
        std::floor((point.x_m - origin.x_m) / resolution_m));
    const auto row = static_cast<long long>(
        std::floor((point.y_m - origin.y_m) / resolution_m));
    if (column < 0 || row < 0 ||
        column >= static_cast<long long>(columns) ||
        row >= static_cast<long long>(rows))
      return std::nullopt;
    return static_cast<std::size_t>(row) * columns +
           static_cast<std::size_t>(column);
  }
  Point2D center(std::size_t index) const noexcept {
    const auto row = index / columns;
    const auto column = index % columns;
    return {origin.x_m + (static_cast<double>(column) + 0.5) * resolution_m,
            origin.y_m + (static_cast<double>(row) + 0.5) * resolution_m};
  }
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
    return {columns, rows, resolution_m, origin};
  }
  bool valid() const noexcept {
    return extent().valid() && cells.size() == columns * rows;
  }
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
};

struct SensedOccupancyGrid {
  GridExtent geometry;
  std::vector<SensedOccupancyCell> cells;
  std::size_t revision = 0U;

  bool valid() const noexcept {
    return geometry.valid() && cells.size() == geometry.columns * geometry.rows;
  }
  std::size_t observedCellCount() const noexcept {
    return static_cast<std::size_t>(std::count_if(
        cells.begin(), cells.end(), [](const auto& cell) {
          return cell.state != SensedOccupancyState::Unknown;
        }));
  }
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
