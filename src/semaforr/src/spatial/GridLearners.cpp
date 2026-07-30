#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/spatial/grid_learners.hpp>
#include <stdexcept>

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

void increment(std::vector<std::uint32_t>& cells, std::size_t index) {
  if (cells[index] != std::numeric_limits<std::uint32_t>::max())
    ++cells[index];
}

}  // namespace

KnownGridLearner::KnownGridLearner(std::size_t columns, std::size_t rows,
                                   double resolution_m,
                                   domain::Point2D origin)
    : SpatialLearnerBase(
          SpatialRepresentation::KnownGrid, "known_grid",
          UpdateMode::Incremental,
          {true, true, false, false, "integrate every coherent laser view",
           {"Out", "low-level exploration"},
           UpdateSchedule::EveryObservation}),
      geometry_(geometry(columns, rows, resolution_m, origin)),
      observations_(columns * rows, 0U) {}

void KnownGridLearner::onObserve(const NavigationEpisode& episode) {
  const auto& pose = episode.observation.pose;
  const auto& laser = episode.observation.laser;
  const double step = geometry_.resolution_m * 0.5;
  for (std::size_t ray = 0U; ray < laser.ranges_m.size(); ++ray) {
    const double measured = laser.ranges_m[ray];
    const double range =
        std::isfinite(measured)
            ? std::clamp(measured, laser.minimum_range.meters(),
                         laser.maximum_range.meters())
            : laser.maximum_range.meters();
    const double angle = pose.heading.radians() +
                         laser.angle_min.radians() +
                         static_cast<double>(ray) *
                             laser.angle_increment.radians();
    for (double distance = 0.0; distance <= range; distance += step) {
      const domain::Point2D point{
          pose.position.x_m + std::cos(angle) * distance,
          pose.position.y_m + std::sin(angle) * distance};
      if (const auto index = indexOf(geometry_, point))
        increment(observations_, *index);
    }
  }
  publish(KnownGridModel{geometry_, observations_}, ModelStatus::Fresh,
          "coherent laser view integrated incrementally");
}

void KnownGridLearner::onRebuild() {
  publish(KnownGridModel{geometry_, observations_}, ModelStatus::Fresh,
          "known grid snapshot refreshed");
}

InclusionGridLearner::InclusionGridLearner(
    std::size_t columns, std::size_t rows, double resolution_m,
    domain::Point2D origin)
    : SpatialLearnerBase(
          SpatialRepresentation::InclusionGrid, "inclusion_grid",
          UpdateMode::Incremental,
          {true, false, false, true,
           "mark cells represented by accepted navigation episodes",
           {"low-level exploration", "coverage diagnostics"},
           UpdateSchedule::EveryObservation}),
      geometry_(geometry(columns, rows, resolution_m, origin)),
      included_(columns * rows, 0U) {}

void InclusionGridLearner::onObserve(const NavigationEpisode& episode) {
  if (const auto index =
          indexOf(geometry_, episode.observation.pose.position))
    included_[*index] = 1U;
  publish(InclusionGridModel{geometry_, included_}, ModelStatus::Fresh,
          "visited decision cell included incrementally");
}

void InclusionGridLearner::onRebuild() {
  publish(InclusionGridModel{geometry_, included_}, ModelStatus::Fresh,
          "inclusion grid snapshot refreshed");
}

}  // namespace semaforr::spatial
