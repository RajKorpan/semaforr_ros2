#include <cmath>
#include <semaforr/spatial/passage_skeleton_learner.hpp>
#include <stdexcept>

namespace semaforr::spatial {

PassageSkeletonLearner::PassageSkeletonLearner(double minimum_node_spacing_m)
    : SpatialLearnerBase(
          SpatialRepresentation::PassagesAndSkeleton, "passage_skeleton",
          UpdateMode::RebuildOnDemand,
          {true,
           true,
           false,
           true,
           "simplify traversed paths into a graph when rebuild is requested",
           {"skeleton", "hallwayskel", "skeletonhall", "passage planners"}}),
      minimum_node_spacing_m_(minimum_node_spacing_m) {
  if (!std::isfinite(minimum_node_spacing_m_) ||
      minimum_node_spacing_m_ <= 0.0) {
    throw std::invalid_argument(
        "skeleton node spacing must be finite and positive");
  }
}

void PassageSkeletonLearner::onObserve(const NavigationEpisode&) {}

void PassageSkeletonLearner::onRebuild() {
  PassageSkeletonModel model;
  std::optional<domain::TaskId> task;
  for (const NavigationEpisode& episode : episodes()) {
    const domain::Point2D point = episode.observation.pose.position;
    const bool task_changed = task && episode.active_task != task;
    if (model.nodes.empty() || task_changed ||
        domain::distance(model.nodes.back(), point).meters() >=
            minimum_node_spacing_m_) {
      model.nodes.push_back(point);
      if (model.nodes.size() >= 2U && !task_changed) {
        model.edges.push_back(
            {model.nodes.size() - 2U, model.nodes.size() - 1U});
      }
    }
    task = episode.active_task;
  }
  publish(model,
          model.edges.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
          model.edges.empty()
              ? "at least two spaced observations are required"
              : "passage and skeleton graph rebuilt from traversed paths");
}

}  // namespace semaforr::spatial
