#include <algorithm>
#include <cmath>
#include <semaforr/spatial/learners/passage_skeleton_learner.hpp>
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
           {"skeleton", "hallwayskel", "skeletonhall", "passage planners"},
           UpdateSchedule::AfterCompletedAction}),
      minimum_node_spacing_m_(minimum_node_spacing_m) {
  if (!std::isfinite(minimum_node_spacing_m_) ||
      minimum_node_spacing_m_ <= 0.0) {
    throw std::invalid_argument(
        "skeleton node spacing must be finite and positive");
  }
}

void PassageSkeletonLearner::onObserve(const NavigationEpisode& episode) {
  if (!episode.action_completed) return;
  const domain::Point2D point = episode.observation.pose.position;
  const bool task_changed = last_task_ && episode.active_task != last_task_;
  if (model_.nodes.empty() || task_changed ||
      domain::distance(model_.nodes.back(), point).meters() >=
          minimum_node_spacing_m_) {
    const auto previous = model_.nodes.size();
    model_.nodes.push_back(point);
    if (previous > 0U && !task_changed) {
      model_.edges.push_back({previous - 1U, previous});
      model_.component_by_node.push_back(
          model_.component_by_node[previous - 1U]);
    } else {
      const std::size_t component =
          model_.component_by_node.empty()
              ? 0U
              : *std::max_element(model_.component_by_node.begin(),
                                  model_.component_by_node.end()) +
                    1U;
      model_.component_by_node.push_back(component);
    }
    ++model_.connectivity_revision;
    publish(model_, model_.edges.empty() ? ModelStatus::Incomplete
                                        : ModelStatus::Fresh,
            model_.edges.empty()
                ? "at least two spaced observations are required"
                : "skeleton connectivity and component cache updated");
  }
  last_task_ = episode.active_task;
}

void PassageSkeletonLearner::onRebuild() {
  publish(model_, model_.edges.empty() ? ModelStatus::Incomplete
                                      : ModelStatus::Fresh,
          "incremental skeleton snapshot refreshed");
}

}  // namespace semaforr::spatial
