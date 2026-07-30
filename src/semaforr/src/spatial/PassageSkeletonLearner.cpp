#include <cmath>
#include <semaforr/spatial/passage_skeleton_learner.hpp>
#include <stdexcept>

namespace semaforr::spatial {

PassageSkeletonLearner::PassageSkeletonLearner(double minimum_node_spacing_m)
    : SpatialLearnerBase(
          SpatialRepresentation::PassagesAndSkeleton, "passage_skeleton",
          UpdateMode::Incremental,
          {true,
           true,
           false,
           true,
           "append spaced path nodes and connectivity incrementally",
           {"skeleton", "hallwayskel", "skeletonhall", "passage planners"}}),
      minimum_node_spacing_m_(minimum_node_spacing_m) {
  if (!std::isfinite(minimum_node_spacing_m_) ||
      minimum_node_spacing_m_ <= 0.0) {
    throw std::invalid_argument(
        "skeleton node spacing must be finite and positive");
  }
}

void PassageSkeletonLearner::onObserve(const NavigationEpisode& episode) {
  const domain::Point2D point = episode.observation.pose.position;
  const bool task_changed = last_task_ && episode.active_task != last_task_;
  if (model_.nodes.empty() || task_changed ||
      domain::distance(model_.nodes.back(), point).meters() >=
          minimum_node_spacing_m_) {
    const auto previous = model_.nodes.size();
    model_.nodes.push_back(point);
    if (previous > 0U && !task_changed)
      model_.edges.push_back({previous - 1U, previous});
  }
  last_task_ = episode.active_task;
  publish(model_, model_.edges.empty() ? ModelStatus::Incomplete
                                      : ModelStatus::Fresh,
          model_.edges.empty() ? "at least two spaced observations are required"
                               : "skeleton connectivity updated incrementally");
}

void PassageSkeletonLearner::onRebuild() {
  publish(model_, model_.edges.empty() ? ModelStatus::Incomplete
                                      : ModelStatus::Fresh,
          "incremental skeleton snapshot refreshed");
}

}  // namespace semaforr::spatial
