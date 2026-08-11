#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/spatial/learners/grid_learners.hpp>
#include <stdexcept>
#include <unordered_set>

namespace semaforr::spatial {
namespace {

GridGeometry geometry(std::size_t columns, std::size_t rows,
                      double resolution_m, domain::Point2D origin,
                      GridExtentPolicy policy, std::string frame_id) {
  if (columns == 0U || rows == 0U || !std::isfinite(resolution_m) ||
      resolution_m <= 0.0 || !origin.finite())
    throw std::invalid_argument("grid geometry must be finite and positive");
  GridGeometry result{
      columns, rows, resolution_m, origin,
      policy == GridExtentPolicy::Expand ? domain::GridExtentMode::Expandable
                                         : domain::GridExtentMode::Fixed,
      domain::GridExtentSource::ConfiguredMaplessInitialBounds};
  result.frame_id = std::move(frame_id);
  return result;
}

std::optional<std::size_t> indexOf(const GridGeometry& grid,
                                   domain::Point2D point) {
  return grid.index(point);
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
                              GridExtentPolicy policy,
                              const domain::GridExpansionPolicy& expansion) {
  if (policy == GridExtentPolicy::Fixed || points.empty()) return grid;
  auto result = grid;
  for (const auto point : points) {
    auto update = domain::expandToInclude(result, point, expansion);
    if (update.resource_limited) throw std::runtime_error(update.diagnostic);
    result = std::move(update.geometry);
  }
  return result;
}

void initializeAround(GridGeometry& grid, domain::Point2D pose) {
  const double width = grid.widthMeters();
  const double height = grid.heightMeters();
  grid.minimum = {pose.x_m - width * 0.5, pose.y_m - height * 0.5};
  grid.origin = grid.minimum;
  grid.maximum = {grid.minimum.x_m + width, grid.minimum.y_m + height};
  ++grid.geometry_revision;
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
    const domain::Point2D center = old_grid.center(old_index);
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
                                   GridExtentPolicy extent_policy,
                                   domain::GridExpansionPolicy expansion_policy,
                                   bool initialize_around_first_pose,
                                   std::string frame_id)
    : SpatialLearnerBase(
          SpatialRepresentation::KnownGrid, "known_grid",
          UpdateMode::Incremental,
          {true, true, false, false, "integrate every coherent laser view",
           {"Out", "low-level exploration"},
           UpdateSchedule::EveryObservation}),
      geometry_(geometry(columns, rows, resolution_m, origin, extent_policy,
                         std::move(frame_id))),
      extent_policy_(extent_policy),
      expansion_policy_(expansion_policy),
      initialize_around_first_pose_(initialize_around_first_pose) {}

void KnownGridLearner::onObserve(const NavigationEpisode& episode) {
  if (initialize_around_first_pose_) {
    initializeAround(geometry_, episode.observation.pose.position);
    initialize_around_first_pose_ = false;
  }
  const auto extent_points = observedExtentPoints(episode);
  if (extent_policy_ == GridExtentPolicy::Fixed)
    out_of_bounds_evidence_ += static_cast<std::size_t>(std::count_if(
        extent_points.begin(), extent_points.end(),
        [this](const auto point) { return !geometry_.index(point); }));
  const auto expanded = expandedGeometry(geometry_, extent_points,
                                         extent_policy_, expansion_policy_);
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
          "familiarity integrated independently from occupancy; " +
              std::to_string(out_of_bounds_evidence_) +
              " fixed-extent observations rejected");
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
    GridExtentPolicy extent_policy,
    domain::GridExpansionPolicy expansion_policy,
    bool initialize_around_first_pose, std::string frame_id)
    : SpatialLearnerBase(
          SpatialRepresentation::SensedOccupancy, "sensed_occupancy",
          UpdateMode::Incremental,
          {true, true, false, false,
           "integrate valid range rays as separate free and occupied evidence",
           {"sensor-grid planning", "occupancy fusion", "diagnostics"},
           UpdateSchedule::EveryObservation}),
      geometry_(geometry(columns, rows, resolution_m, origin, extent_policy,
                         std::move(frame_id))),
      configuration_(configuration),
      extent_policy_(extent_policy),
      expansion_policy_(expansion_policy),
      initialize_around_first_pose_(initialize_around_first_pose) {
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
  model.geometry = geometry_;
  model.sparse_cells.reserve(cells_.size());
  for (const auto& [index, cell] : cells_)
    if (index < geometry_.cellCount()) model.sparse_cells.push_back({index, cell});
  std::sort(model.sparse_cells.begin(), model.sparse_cells.end(),
            [](const auto& left, const auto& right) {
              return left.index < right.index;
            });
  return model;
}

void SensedOccupancyLearner::onObserve(const NavigationEpisode& episode) {
  if (initialize_around_first_pose_) {
    initializeAround(geometry_, episode.observation.pose.position);
    initialize_around_first_pose_ = false;
  }
  const auto extent_points = observedExtentPoints(episode);
  if (extent_policy_ == GridExtentPolicy::Fixed)
    out_of_bounds_evidence_ += static_cast<std::size_t>(std::count_if(
        extent_points.begin(), extent_points.end(),
        [this](const auto point) { return !geometry_.index(point); }));
  const auto expanded = expandedGeometry(geometry_, extent_points,
                                         extent_policy_, expansion_policy_);
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
          "valid rays integrated; hit endpoints remain occupied; " +
              std::to_string(out_of_bounds_evidence_) +
              " fixed-extent observations rejected");
}

void SensedOccupancyLearner::onRebuild() {
  publish(snapshotModel(), ModelStatus::Fresh,
          "sensed occupancy snapshot refreshed");
}

InclusionGridLearner::InclusionGridLearner(
    std::size_t columns, std::size_t rows, double resolution_m,
    domain::Point2D origin, GridExtentPolicy extent_policy,
    domain::GridExpansionPolicy expansion_policy,
    bool initialize_around_first_pose, std::string frame_id)
    : SpatialLearnerBase(
          SpatialRepresentation::InclusionGrid, "inclusion_grid",
          UpdateMode::Incremental,
          {true, false, true, true,
           "project learned regions, operational subtrails, and successful LLE traversal",
           {"low-level exploration", "coverage diagnostics"},
           UpdateSchedule::DuringLLEOnly}),
      geometry_(geometry(columns, rows, resolution_m, origin, extent_policy,
                         std::move(frame_id))),
      extent_policy_(extent_policy),
      expansion_policy_(expansion_policy),
      initialize_around_first_pose_(initialize_around_first_pose) {}

void InclusionGridLearner::onObserve(const NavigationEpisode& episode) {
  if (initialize_around_first_pose_) {
    initializeAround(geometry_, episode.observation.pose.position);
    initialize_around_first_pose_ = false;
  }
  if (!episode.execution_result || !episode.execution_result->successful())
    return;
  const auto start = episode.execution_result->start_pose.position;
  const auto finish = episode.execution_result->final_pose.position;
  const double length = domain::distance(start, finish).meters();
  if (length <= domain::geometry_tolerance_m) return;
  const auto expanded = expandedGeometry(geometry_, {start, finish},
                                         extent_policy_, expansion_policy_);
  remap(included_, geometry_, expanded);
  remap(lle_included_, geometry_, expanded);
  geometry_ = expanded;
  const double step = geometry_.resolution_m * 0.5;
  const std::size_t samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / step)));
  bool changed = false;
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double fraction = static_cast<double>(sample) /
                            static_cast<double>(samples);
    const domain::Point2D point{
        start.x_m + (finish.x_m - start.x_m) * fraction,
        start.y_m + (finish.y_m - start.y_m) * fraction};
    if (const auto index = indexOf(geometry_, point)) {
      lle_included_[*index] = 1U;
      changed = included_.emplace(*index, 1U).second || changed;
    }
  }
  if (changed)
    publish(InclusionGridModel{geometry_, {}, sparseSnapshot(included_)},
            ModelStatus::Fresh,
            "successful LLE traversal added as learned subtrail inclusion");
}

void InclusionGridLearner::replaceRepresented(
    const RegionModel& regions, const PassageSkeletonModel& skeleton) {
  std::vector<domain::Point2D> extent;
  for (const auto& region : regions.learned_regions) {
    const double radius = region.boundary.radius.meters();
    extent.push_back({region.boundary.center.x_m - radius,
                      region.boundary.center.y_m - radius});
    extent.push_back({region.boundary.center.x_m + radius,
                      region.boundary.center.y_m + radius});
  }
  for (const auto& edge : skeleton.region_edges)
    extent.insert(extent.end(), edge.supporting_subtrail.begin(),
                  edge.supporting_subtrail.end());
  const auto previous_geometry = geometry_;
  const auto expanded = expandedGeometry(geometry_, extent, extent_policy_,
                                         expansion_policy_);
  remap(lle_included_, geometry_, expanded);
  geometry_ = expanded;
  std::unordered_map<std::size_t, std::uint32_t> represented = lle_included_;
  for (const auto& region : regions.learned_regions) {
    const double radius = region.boundary.radius.meters();
    const auto minimum = geometry_.cell(
        {region.boundary.center.x_m - radius + domain::geometry_tolerance_m,
         region.boundary.center.y_m - radius + domain::geometry_tolerance_m});
    const auto maximum = geometry_.cell(
        {region.boundary.center.x_m + radius - domain::geometry_tolerance_m,
         region.boundary.center.y_m + radius - domain::geometry_tolerance_m});
    if (!minimum || !maximum) continue;
    for (std::size_t row = minimum->second; row <= maximum->second; ++row) {
      for (std::size_t column = minimum->first; column <= maximum->first;
           ++column) {
        const auto point = geometry_.center(column, row);
        if (domain::distance(point, region.boundary.center).meters() <= radius)
          represented[row * geometry_.columns + column] = 1U;
      }
    }
  }
  const double sample_step = geometry_.resolution_m * 0.5;
  for (const auto& edge : skeleton.region_edges) {
    for (std::size_t point = 1U; point < edge.supporting_subtrail.size();
         ++point) {
      const auto start = edge.supporting_subtrail[point - 1U];
      const auto finish = edge.supporting_subtrail[point];
      const double length = domain::distance(start, finish).meters();
      const std::size_t samples = std::max<std::size_t>(
          1U, static_cast<std::size_t>(std::ceil(length / sample_step)));
      for (std::size_t sample = 0U; sample <= samples; ++sample) {
        const double fraction = static_cast<double>(sample) /
                                static_cast<double>(samples);
        const domain::Point2D location{
            start.x_m + (finish.x_m - start.x_m) * fraction,
            start.y_m + (finish.y_m - start.y_m) * fraction};
        if (const auto index = indexOf(geometry_, location))
          represented[*index] = 1U;
      }
    }
  }
  if (represented == included_ && geometry_ == previous_geometry) return;
  included_ = std::move(represented);
  publish(InclusionGridModel{geometry_, {}, sparseSnapshot(included_)},
          ModelStatus::Fresh,
          "inclusion rebuilt from learned region area and supporting subtrails");
}

void InclusionGridLearner::onRebuild() {
  publish(InclusionGridModel{geometry_, {}, sparseSnapshot(included_)},
          ModelStatus::Fresh,
          "region/subtrail inclusion snapshot refreshed");
}

}  // namespace semaforr::spatial
