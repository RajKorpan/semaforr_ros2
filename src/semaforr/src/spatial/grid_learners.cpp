#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/spatial/learners/grid_learners.hpp>
#include <stdexcept>
#include <unordered_set>

namespace semaforr::spatial {
namespace {

GridGeometry geometry(std::size_t columns, std::size_t rows,
                      double resolution_m, domain::Point2D origin) {
  if (columns == 0U || rows == 0U || !std::isfinite(resolution_m) ||
      resolution_m <= 0.0 || !origin.finite())
    throw std::invalid_argument("grid geometry must be finite and positive");
  return {columns, rows, resolution_m, origin};
}

std::optional<std::size_t> indexOf(const GridGeometry& grid,
                                   domain::Point2D point) {
  const auto column =
      static_cast<long long>(std::floor((point.x_m - grid.origin.x_m) /
                                        grid.resolution_m));
  const auto row =
      static_cast<long long>(std::floor((point.y_m - grid.origin.y_m) /
                                        grid.resolution_m));
  if (column < 0 || row < 0 ||
      column >= static_cast<long long>(grid.columns) ||
      row >= static_cast<long long>(grid.rows))
    return std::nullopt;
  return static_cast<std::size_t>(row) * grid.columns +
         static_cast<std::size_t>(column);
}

void increment(std::unordered_map<std::size_t, std::uint32_t>& cells,
               std::size_t index) {
  auto& value = cells[index];
  if (value != std::numeric_limits<std::uint32_t>::max()) ++value;
}

std::vector<SparseGridCell> sparseSnapshot(
    const std::unordered_map<std::size_t, std::uint32_t>& cells) {
  std::vector<SparseGridCell> result;
  result.reserve(cells.size());
  for (const auto& [index, value] : cells) result.push_back({index, value});
  std::sort(result.begin(), result.end(),
            [](const auto& first, const auto& second) {
              return first.index < second.index;
            });
  return result;
}

bool validRay(const domain::LaserObservation& laser, double measured) {
  return !std::isnan(measured) && measured >= laser.minimum_range.meters() &&
         (std::isfinite(measured)
              ? measured <= laser.maximum_range.meters()
              : measured > 0.0);
}

double rayExtent(const domain::LaserObservation& laser, double measured) {
  return std::isfinite(measured) ? measured : laser.maximum_range.meters();
}

bool obstacleHit(const domain::LaserObservation& laser, double measured) {
  return std::isfinite(measured) &&
         measured < laser.maximum_range.meters() -
                        domain::geometry_tolerance_m;
}

std::uint16_t saturatingIncrement(std::uint16_t value) {
  return value == std::numeric_limits<std::uint16_t>::max()
             ? value
             : static_cast<std::uint16_t>(value + 1U);
}

GridGeometry expandedGeometry(const GridGeometry& grid,
                              const std::vector<domain::Point2D>& points,
                              GridExtentPolicy policy) {
  if (policy == GridExtentPolicy::Fixed || points.empty()) return grid;
  long long min_column = 0, min_row = 0;
  long long max_column = static_cast<long long>(grid.columns) - 1;
  long long max_row = static_cast<long long>(grid.rows) - 1;
  for (const auto point : points) {
    const auto column = static_cast<long long>(std::floor(
        (point.x_m - grid.origin.x_m) / grid.resolution_m));
    const auto row = static_cast<long long>(std::floor(
        (point.y_m - grid.origin.y_m) / grid.resolution_m));
    min_column = std::min(min_column, column);
    min_row = std::min(min_row, row);
    max_column = std::max(max_column, column);
    max_row = std::max(max_row, row);
  }
  constexpr long long chunk = 32;
  const auto chunks = [=](long long cells) {
    return ((cells + chunk - 1) / chunk) * chunk;
  };
  const long long left = min_column < 0 ? chunks(-min_column) : 0;
  const long long bottom = min_row < 0 ? chunks(-min_row) : 0;
  const long long right =
      max_column >= static_cast<long long>(grid.columns)
          ? chunks(max_column - static_cast<long long>(grid.columns) + 1)
          : 0;
  const long long top = max_row >= static_cast<long long>(grid.rows)
                            ? chunks(max_row -
                                     static_cast<long long>(grid.rows) + 1)
                            : 0;
  return {grid.columns + static_cast<std::size_t>(left + right),
          grid.rows + static_cast<std::size_t>(bottom + top),
          grid.resolution_m,
          {grid.origin.x_m - static_cast<double>(left) * grid.resolution_m,
           grid.origin.y_m - static_cast<double>(bottom) * grid.resolution_m}};
}

template <typename Value>
void remap(std::unordered_map<std::size_t, Value>& cells,
           const GridGeometry& old_grid, const GridGeometry& new_grid) {
  if (old_grid.columns == new_grid.columns && old_grid.rows == new_grid.rows &&
      old_grid.origin == new_grid.origin)
    return;
  std::unordered_map<std::size_t, Value> result;
  result.reserve(cells.size());
  for (auto& [old_index, value] : cells) {
    const auto row = old_index / old_grid.columns;
    const auto column = old_index % old_grid.columns;
    const domain::Point2D center{
        old_grid.origin.x_m +
            (static_cast<double>(column) + 0.5) * old_grid.resolution_m,
        old_grid.origin.y_m +
            (static_cast<double>(row) + 0.5) * old_grid.resolution_m};
    if (const auto index = indexOf(new_grid, center))
      result.emplace(*index, std::move(value));
  }
  cells = std::move(result);
}

std::vector<domain::Point2D> observedExtentPoints(
    const NavigationEpisode& episode) {
  std::vector<domain::Point2D> points{episode.observation.pose.position};
  const auto& pose = episode.observation.pose;
  const auto& laser = episode.observation.laser;
  for (std::size_t ray = 0U; ray < laser.ranges_m.size(); ++ray) {
    const double measured = laser.ranges_m[ray];
    if (!validRay(laser, measured)) continue;
    const double angle = pose.heading.radians() + laser.angle_min.radians() +
                         static_cast<double>(ray) *
                             laser.angle_increment.radians();
    const double range = rayExtent(laser, measured);
    points.push_back({pose.position.x_m + std::cos(angle) * range,
                      pose.position.y_m + std::sin(angle) * range});
  }
  return points;
}

}  // namespace

KnownGridLearner::KnownGridLearner(std::size_t columns, std::size_t rows,
                                   double resolution_m,
                                   domain::Point2D origin,
                                   GridExtentPolicy extent_policy)
    : SpatialLearnerBase(
          SpatialRepresentation::KnownGrid, "known_grid",
          UpdateMode::Incremental,
          {true, true, false, false, "integrate every coherent laser view",
           {"Out", "low-level exploration"},
           UpdateSchedule::EveryObservation}),
      geometry_(geometry(columns, rows, resolution_m, origin)),
      extent_policy_(extent_policy) {}

void KnownGridLearner::onObserve(const NavigationEpisode& episode) {
  const auto expanded =
      expandedGeometry(geometry_, observedExtentPoints(episode), extent_policy_);
  remap(observations_, geometry_, expanded);
  remap(last_observed_sequence_, geometry_, expanded);
  geometry_ = expanded;
  const auto& pose = episode.observation.pose;
  const auto& laser = episode.observation.laser;
  const double step = geometry_.resolution_m * 0.5;
  for (std::size_t ray = 0U; ray < laser.ranges_m.size(); ++ray) {
    const double measured = laser.ranges_m[ray];
    if (!validRay(laser, measured)) continue;
    const double range = rayExtent(laser, measured);
    const double angle = pose.heading.radians() +
                         laser.angle_min.radians() +
                         static_cast<double>(ray) *
                             laser.angle_increment.radians();
    std::unordered_set<std::size_t> ray_cells;
    for (double distance = 0.0; distance < range; distance += step) {
      const domain::Point2D point{
          pose.position.x_m + std::cos(angle) * distance,
          pose.position.y_m + std::sin(angle) * distance};
      if (const auto index = indexOf(geometry_, point)) ray_cells.insert(*index);
    }
    const domain::Point2D endpoint{
        pose.position.x_m + std::cos(angle) * range,
        pose.position.y_m + std::sin(angle) * range};
    if (const auto index = indexOf(geometry_, endpoint)) ray_cells.insert(*index);
    for (const auto index : ray_cells) {
      increment(observations_, index);
      last_observed_sequence_[index] = episode.sequence;
    }
  }
  std::vector<FamiliarityCellMetadata> metadata;
  metadata.reserve(last_observed_sequence_.size());
  for (const auto& [index, sequence] : last_observed_sequence_) {
    const auto count = observations_.at(index);
    metadata.push_back({index, sequence,
                        static_cast<float>(1.0 - std::exp(-count / 3.0))});
  }
  std::sort(metadata.begin(), metadata.end(), [](const auto& a, const auto& b) {
    return a.index < b.index;
  });
  publish(KnownGridModel{geometry_, {}, sparseSnapshot(observations_),
                         std::move(metadata)},
          ModelStatus::Fresh,
          "familiarity integrated independently from occupancy");
}

void KnownGridLearner::onRebuild() {
  std::vector<FamiliarityCellMetadata> metadata;
  for (const auto& [index, sequence] : last_observed_sequence_) {
    const auto count = observations_.at(index);
    metadata.push_back({index, sequence,
                        static_cast<float>(1.0 - std::exp(-count / 3.0))});
  }
  std::sort(metadata.begin(), metadata.end(), [](const auto& a, const auto& b) {
    return a.index < b.index;
  });
  publish(KnownGridModel{geometry_, {}, sparseSnapshot(observations_),
                         std::move(metadata)},
          ModelStatus::Fresh,
          "known grid snapshot refreshed");
}

SensedOccupancyLearner::SensedOccupancyLearner(
    std::size_t columns, std::size_t rows, double resolution_m,
    domain::Point2D origin, SensedOccupancyLearningConfiguration configuration,
    GridExtentPolicy extent_policy)
    : SpatialLearnerBase(
          SpatialRepresentation::SensedOccupancy, "sensed_occupancy",
          UpdateMode::Incremental,
          {true, true, false, false,
           "integrate valid range rays as separate free and occupied evidence",
           {"sensor-grid planning", "occupancy fusion", "diagnostics"},
           UpdateSchedule::EveryObservation}),
      geometry_(geometry(columns, rows, resolution_m, origin)),
      configuration_(configuration),
      extent_policy_(extent_policy) {
  if (configuration_.free_observations_to_clear == 0U ||
      configuration_.dynamic_expiry_observations == 0U)
    throw std::invalid_argument("sensed occupancy thresholds must be positive");
}

void SensedOccupancyLearner::integrateFree(std::size_t index,
                                           std::size_t sequence) {
  auto& cell = cells_[index];
  cell.free_evidence = saturatingIncrement(cell.free_evidence);
  cell.conflicting = cell.occupied_evidence > 0U;
  const auto required = static_cast<std::uint32_t>(cell.occupied_evidence) +
                        configuration_.free_observations_to_clear;
  if (cell.state != domain::SensedOccupancyState::ObservedOccupied ||
      cell.free_evidence >= required) {
    cell.state = domain::SensedOccupancyState::ObservedFree;
    cell.dynamic = false;
  }
  cell.last_update_sequence = sequence;
  const auto total = static_cast<double>(cell.free_evidence) +
                     static_cast<double>(cell.occupied_evidence);
  cell.confidence = static_cast<float>(
      total == 0.0 ? 0.0 : std::max(cell.free_evidence, cell.occupied_evidence) /
                                  total);
  cell.source = domain::OccupancyEvidenceSource::CurrentSensor |
                domain::OccupancyEvidenceSource::AccumulatedSensorModel;
}

void SensedOccupancyLearner::integrateOccupied(std::size_t index,
                                               std::size_t sequence) {
  auto& cell = cells_[index];
  cell.occupied_evidence = saturatingIncrement(cell.occupied_evidence);
  cell.state = domain::SensedOccupancyState::ObservedOccupied;
  cell.conflicting = cell.free_evidence > 0U;
  cell.dynamic = configuration_.treat_obstacle_returns_as_dynamic;
  cell.last_update_sequence = sequence;
  const auto total = static_cast<double>(cell.free_evidence) +
                     static_cast<double>(cell.occupied_evidence);
  cell.confidence = static_cast<float>(cell.occupied_evidence / total);
  cell.source = domain::OccupancyEvidenceSource::CurrentSensor |
                domain::OccupancyEvidenceSource::AccumulatedSensorModel;
  if (cell.dynamic)
    cell.source = cell.source | domain::OccupancyEvidenceSource::DynamicObstacle;
}

void SensedOccupancyLearner::expireDynamic(std::size_t sequence) {
  for (auto& [index, cell] : cells_) {
    static_cast<void>(index);
    if (!cell.dynamic || sequence < cell.last_update_sequence ||
        sequence - cell.last_update_sequence <
            configuration_.dynamic_expiry_observations)
      continue;
    cell.occupied_evidence = 0U;
    cell.dynamic = false;
    cell.conflicting = false;
    cell.state = cell.free_evidence > 0U
                     ? domain::SensedOccupancyState::ObservedFree
                     : domain::SensedOccupancyState::Unknown;
    cell.confidence = cell.free_evidence > 0U ? 1.0F : 0.0F;
    cell.source = cell.free_evidence > 0U
                      ? domain::OccupancyEvidenceSource::AccumulatedSensorModel
                      : domain::OccupancyEvidenceSource::None;
  }
}

SensedOccupancyModel SensedOccupancyLearner::snapshotModel() const {
  SensedOccupancyModel model;
  model.geometry = {geometry_.columns, geometry_.rows, geometry_.resolution_m,
                    geometry_.origin};
  model.cells.resize(geometry_.columns * geometry_.rows);
  for (const auto& [index, cell] : cells_)
    if (index < model.cells.size()) model.cells[index] = cell;
  return model;
}

void SensedOccupancyLearner::onObserve(const NavigationEpisode& episode) {
  const auto expanded =
      expandedGeometry(geometry_, observedExtentPoints(episode), extent_policy_);
  remap(cells_, geometry_, expanded);
  geometry_ = expanded;
  expireDynamic(episode.sequence);
  const auto& pose = episode.observation.pose;
  const auto& laser = episode.observation.laser;
  const double step = geometry_.resolution_m * 0.5;
  for (std::size_t ray = 0U; ray < laser.ranges_m.size(); ++ray) {
    const double measured = laser.ranges_m[ray];
    if (!validRay(laser, measured)) continue;
    const double range = rayExtent(laser, measured);
    const bool hit = obstacleHit(laser, measured);
    const double angle = pose.heading.radians() + laser.angle_min.radians() +
                         static_cast<double>(ray) *
                             laser.angle_increment.radians();
    std::unordered_set<std::size_t> free_cells;
    for (double distance = 0.0; distance < range; distance += step) {
      const domain::Point2D point{
          pose.position.x_m + std::cos(angle) * distance,
          pose.position.y_m + std::sin(angle) * distance};
      if (const auto index = indexOf(geometry_, point)) free_cells.insert(*index);
    }
    const domain::Point2D endpoint{
        pose.position.x_m + std::cos(angle) * range,
        pose.position.y_m + std::sin(angle) * range};
    const auto endpoint_index = indexOf(geometry_, endpoint);
    if (hit && endpoint_index) free_cells.erase(*endpoint_index);
    if (!hit && endpoint_index) free_cells.insert(*endpoint_index);
    for (const auto index : free_cells) integrateFree(index, episode.sequence);
    if (hit && endpoint_index) integrateOccupied(*endpoint_index, episode.sequence);
  }
  publish(snapshotModel(), ModelStatus::Fresh,
          "valid rays integrated; hit endpoints remain occupied");
}

void SensedOccupancyLearner::onRebuild() {
  publish(snapshotModel(), ModelStatus::Fresh,
          "sensed occupancy snapshot refreshed");
}

InclusionGridLearner::InclusionGridLearner(
    std::size_t columns, std::size_t rows, double resolution_m,
    domain::Point2D origin, GridExtentPolicy extent_policy)
    : SpatialLearnerBase(
          SpatialRepresentation::InclusionGrid, "inclusion_grid",
          UpdateMode::Incremental,
          {true, false, false, true,
           "mark cells represented by accepted navigation episodes",
           {"low-level exploration", "coverage diagnostics"},
           UpdateSchedule::EveryObservation}),
      geometry_(geometry(columns, rows, resolution_m, origin)),
      extent_policy_(extent_policy) {}

void InclusionGridLearner::onObserve(const NavigationEpisode& episode) {
  const auto expanded = expandedGeometry(
      geometry_, {episode.observation.pose.position}, extent_policy_);
  remap(included_, geometry_, expanded);
  geometry_ = expanded;
  if (const auto index =
          indexOf(geometry_, episode.observation.pose.position))
    included_[*index] = 1U;
  publish(InclusionGridModel{geometry_, {}, sparseSnapshot(included_)},
          ModelStatus::Fresh,
          "visited decision cell included incrementally");
}

void InclusionGridLearner::onRebuild() {
  publish(InclusionGridModel{geometry_, {}, sparseSnapshot(included_)},
          ModelStatus::Fresh,
          "inclusion grid snapshot refreshed");
}

}  // namespace semaforr::spatial
