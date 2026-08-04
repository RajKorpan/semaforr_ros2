#include <cmath>
#include <cstdint>
#include <semaforr/spatial/learners/hallway_learner.hpp>
#include <stdexcept>
#include <unordered_set>

#include "learning_geometry.hpp"

namespace semaforr::spatial {

HallwayLearner::HallwayLearner(double minimum_centerline_length_m)
    : SpatialLearnerBase(
          SpatialRepresentation::Hallways, "hallway",
          UpdateMode::RebuildOnDemand,
          {true,
           true,
           false,
           false,
           "extract traversed centerlines when rebuild is requested",
           {"hallway advisors", "hallwayskel and skeletonhall planners"},
           UpdateSchedule::EndOfTarget}),
      minimum_centerline_length_m_(minimum_centerline_length_m) {
  if (!std::isfinite(minimum_centerline_length_m_) ||
      minimum_centerline_length_m_ <= 0.0) {
    throw std::invalid_argument(
        "hallway minimum centerline length must be finite and positive");
  }
}

void HallwayLearner::onObserve(const NavigationEpisode&) {}

void HallwayLearner::onRebuild() {
  HallwayModel model;
  std::unordered_set<std::uint64_t> occupied_bins;
  for (std::size_t index = 1U; index < episodes().size(); ++index) {
    const auto& previous = episodes()[index - 1U];
    const auto& current = episodes()[index];
    if (current.active_task != previous.active_task ||
        current.observation.laser.ranges_m.empty()) {
      continue;
    }
    domain::Segment2D centerline{previous.observation.pose.position,
                                 current.observation.pose.position};
    if (centerline.length().meters() >= minimum_centerline_length_m_) {
      const double angle = std::atan2(
          centerline.end.y_m - centerline.start.y_m,
          centerline.end.x_m - centerline.start.x_m);
      constexpr double pi = 3.14159265358979323846;
      const auto orientation = static_cast<std::uint64_t>(
          std::floor((angle + pi) / (pi / 8.0))) & 15U;
      const auto cell_x = static_cast<std::uint64_t>(
          static_cast<std::uint32_t>(std::floor(
              (centerline.start.x_m + centerline.end.x_m) * 2.5)));
      const auto cell_y = static_cast<std::uint64_t>(
          static_cast<std::uint32_t>(std::floor(
              (centerline.start.y_m + centerline.end.y_m) * 2.5)));
      const std::uint64_t key =
          (orientation << 56U) ^ (cell_x << 28U) ^ cell_y;
      if (occupied_bins.insert(key).second)
        model.centerlines.push_back(centerline);
    }
  }
  publish(
      model,
      model.centerlines.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
      model.centerlines.empty() ? "no sufficiently long traversed centerline"
                                : "hallway centerlines rebuilt from traversed "
                                  "scan-supported segments");
}

}  // namespace semaforr::spatial
