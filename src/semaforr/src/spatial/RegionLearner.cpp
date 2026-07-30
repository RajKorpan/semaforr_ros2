#include <algorithm>
#include <cmath>
#include <semaforr/spatial/region_learner.hpp>
#include <stdexcept>

namespace semaforr::spatial {

RegionLearner::RegionLearner(double cluster_radius_m,
                             std::size_t minimum_observations)
    : SpatialLearnerBase(
          SpatialRepresentation::Regions, "region", UpdateMode::RebuildOnDemand,
          {true,
           true,
           false,
           false,
           "cluster accumulated poses when rebuild is requested",
           {"RegionLeaverLinear", "RegionLeaverRotation", "skeleton planner"}}),
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

void RegionLearner::onObserve(const NavigationEpisode&) {}

void RegionLearner::onRebuild() {
  struct Cluster {
    std::vector<domain::Point2D> points;
  };
  std::vector<Cluster> clusters;
  for (const NavigationEpisode& episode : episodes()) {
    const domain::Point2D point = episode.observation.pose.position;
    auto found = std::find_if(
        clusters.begin(), clusters.end(), [&](const Cluster& cluster) {
          return domain::distance(cluster.points.front(), point).meters() <=
                 cluster_radius_m_;
        });
    if (found == clusters.end()) {
      clusters.push_back({{point}});
    } else {
      found->points.push_back(point);
    }
  }

  RegionModel model;
  for (const Cluster& cluster : clusters) {
    if (cluster.points.size() < minimum_observations_) {
      continue;
    }
    domain::Point2D center;
    for (const domain::Point2D& point : cluster.points) {
      center.x_m += point.x_m;
      center.y_m += point.y_m;
    }
    center.x_m /= static_cast<double>(cluster.points.size());
    center.y_m /= static_cast<double>(cluster.points.size());
    double radius = 0.25;
    for (const domain::Point2D& point : cluster.points) {
      radius = std::max(radius, domain::distance(center, point).meters());
    }
    model.regions.push_back(
        {center, domain::Distance(std::min(cluster_radius_m_, radius + 0.25))});
  }
  publish(model,
          model.regions.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
          model.regions.empty()
              ? "no pose cluster reached the observation threshold"
              : "regions rebuilt from pose clusters");
}

}  // namespace semaforr::spatial
