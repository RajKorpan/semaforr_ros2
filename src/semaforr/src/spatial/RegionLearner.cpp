#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/spatial/region_learner.hpp>
#include <stdexcept>

namespace semaforr::spatial {
namespace {
std::uint64_t regionKey(long long x, long long y) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
         static_cast<std::uint32_t>(y);
}
}  // namespace

RegionLearner::RegionLearner(double cluster_radius_m,
                             std::size_t minimum_observations)
    : SpatialLearnerBase(
          SpatialRepresentation::Regions, "region", UpdateMode::Incremental,
          {true,
           true,
           false,
           false,
           "incrementally merge local freespace observations",
           {"RegionLeaverLinear", "RegionLeaverRotation", "skeleton planner"},
           UpdateSchedule::EveryObservation}),
      cluster_radius_m_(cluster_radius_m),
      minimum_observations_(minimum_observations) {
  if (!std::isfinite(cluster_radius_m_) || cluster_radius_m_ <= 0.0) {
    throw std::invalid_argument(
        "region cluster radius must be finite and positive");
  }
  if (minimum_observations_ == 0U) {
    throw std::invalid_argument("region minimum observations must be positive");
  }
}

void RegionLearner::onObserve(const NavigationEpisode& episode) {
  const auto point = episode.observation.pose.position;
  const auto finite_range = std::min_element(
      episode.observation.laser.ranges_m.begin(),
      episode.observation.laser.ranges_m.end(),
      [](double left, double right) {
        return (std::isfinite(left) ? left
                                    : std::numeric_limits<double>::max()) <
               (std::isfinite(right) ? right
                                     : std::numeric_limits<double>::max());
      });
  double sensed_radius = 0.25;
  if (finite_range != episode.observation.laser.ranges_m.end() &&
      std::isfinite(*finite_range))
    sensed_radius = std::clamp(*finite_range, 0.25, cluster_radius_m_);

  std::size_t nearest = model_.regions.size();
  double nearest_distance = cluster_radius_m_;
  const auto bucket_x =
      static_cast<long long>(std::floor(point.x_m / cluster_radius_m_));
  const auto bucket_y =
      static_cast<long long>(std::floor(point.y_m / cluster_radius_m_));
  for (long long y = bucket_y - 1; y <= bucket_y + 1; ++y) {
    for (long long x = bucket_x - 1; x <= bucket_x + 1; ++x) {
      const auto found = spatial_index_.find(regionKey(x, y));
      if (found == spatial_index_.end()) continue;
      for (const auto index : found->second) {
        const double distance =
            domain::distance(model_.regions[index].center, point).meters();
        if (distance <= nearest_distance) {
          nearest = index;
          nearest_distance = distance;
        }
      }
    }
  }
  if (nearest == model_.regions.size()) {
    model_.regions.push_back({point, domain::Distance(sensed_radius)});
    observation_counts_.push_back(1U);
    const auto key = regionKey(bucket_x, bucket_y);
    spatial_index_[key].push_back(
        model_.regions.size() - 1U);
    region_buckets_.push_back(key);
  } else {
    auto& region = model_.regions[nearest];
    const auto count = ++observation_counts_[nearest];
    region.center.x_m += (point.x_m - region.center.x_m) /
                         static_cast<double>(count);
    region.center.y_m += (point.y_m - region.center.y_m) /
                         static_cast<double>(count);
    region.radius = domain::Distance(std::clamp(
        std::max(region.radius.meters(), sensed_radius), 0.25,
        cluster_radius_m_));
    const auto new_key = regionKey(
        static_cast<long long>(
            std::floor(region.center.x_m / cluster_radius_m_)),
        static_cast<long long>(
            std::floor(region.center.y_m / cluster_radius_m_)));
    if (new_key != region_buckets_[nearest]) {
      auto& old_bucket = spatial_index_[region_buckets_[nearest]];
      old_bucket.erase(
          std::remove(old_bucket.begin(), old_bucket.end(), nearest),
          old_bucket.end());
      spatial_index_[new_key].push_back(nearest);
      region_buckets_[nearest] = new_key;
    }
  }
  const bool enough = std::any_of(
      observation_counts_.begin(), observation_counts_.end(),
      [this](std::size_t count) { return count >= minimum_observations_; });
  publish(model_, enough ? ModelStatus::Fresh : ModelStatus::Incomplete,
          enough ? "regions updated incrementally"
                 : "region observations below publication threshold");
}

void RegionLearner::onRebuild() {
  publish(model_, model_.regions.empty() ? ModelStatus::Incomplete
                                         : ModelStatus::Fresh,
          "incremental region snapshot refreshed");
}

}  // namespace semaforr::spatial
