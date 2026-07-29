#include <semaforr/spatial/hallway_learner.hpp>

#include <cmath>
#include <stdexcept>

#include "LearningGeometry.hpp"

namespace semaforr::spatial {

HallwayLearner::HallwayLearner(double minimum_centerline_length_m)
  : SpatialLearnerBase(
      SpatialRepresentation::Hallways,
      "hallway",
      UpdateMode::RebuildOnDemand,
      {true, true, false, false,
       "extract traversed centerlines when rebuild is requested",
       {"hallway advisors", "hallwayskel and skeletonhall planners"}}),
    minimum_centerline_length_m_(minimum_centerline_length_m)
{
  if (!std::isfinite(minimum_centerline_length_m_) ||
      minimum_centerline_length_m_ <= 0.0) {
    throw std::invalid_argument(
      "hallway minimum centerline length must be finite and positive");
  }
}

void HallwayLearner::onObserve(const NavigationEpisode&)
{
}

void HallwayLearner::onRebuild()
{
  HallwayModel model;
  for (std::size_t index = 1U; index < episodes().size(); ++index) {
    const auto& previous = episodes()[index - 1U];
    const auto& current = episodes()[index];
    if (current.active_task != previous.active_task ||
        current.observation.laser.ranges_m.empty()) {
      continue;
    }
    domain::Segment2D centerline{
      previous.observation.pose.position,
      current.observation.pose.position};
    if (centerline.length().meters() >= minimum_centerline_length_m_) {
      detail::appendUnique(model.centerlines, centerline, 0.2);
    }
  }
  publish(
    model,
    model.centerlines.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
    model.centerlines.empty() ? "no sufficiently long traversed centerline" :
      "hallway centerlines rebuilt from traversed scan-supported segments");
}

}  // namespace semaforr::spatial
