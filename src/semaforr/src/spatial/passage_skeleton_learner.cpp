#include <algorithm>
#include <cmath>
#include <semaforr/spatial/learners/passage_skeleton_learner.hpp>
#include <stdexcept>

namespace semaforr::spatial {

PassageSkeletonLearner::PassageSkeletonLearner(
    double minimum_node_spacing_m, SpatialLearningMode mode,
    RegionLearningConfiguration region_configuration,
    TrailLearningConfiguration trail_configuration)
    : SpatialLearnerBase(
          SpatialRepresentation::PassagesAndSkeleton, "passage_skeleton",
          mode == SpatialLearningMode::Compatibility
              ? UpdateMode::RebuildOnDemand
              : UpdateMode::Incremental,
          {true,
           true,
           false,
           true,
           mode == SpatialLearningMode::Compatibility
               ? "build region nodes, direct-transition edges, shortest subtrails, and visibility"
               : "append a distinctly named sampled-pose path graph",
           {"skeleton", "hallwayskel", "skeletonhall", "passage planners"},
           mode == SpatialLearningMode::Compatibility
               ? UpdateSchedule::EndOfTarget
               : UpdateSchedule::AfterSuccessfulActionCompletion}),
      minimum_node_spacing_m_(minimum_node_spacing_m),
      mode_(mode),
      region_configuration_(region_configuration),
      trail_configuration_(trail_configuration) {
  if (!std::isfinite(minimum_node_spacing_m_) ||
      minimum_node_spacing_m_ <= 0.0) {
    throw std::invalid_argument(
        "skeleton node spacing must be finite and positive");
  }
}

void PassageSkeletonLearner::onObserve(const NavigationEpisode& episode) {
  if (mode_ == SpatialLearningMode::Compatibility) return;
  if (!episode.actionSucceeded()) return;
  if (model_.nodes.empty() && episode.execution_result) {
    model_.nodes.push_back(episode.execution_result->start_pose.position);
    model_.component_by_node.push_back(0U);
  }
  const domain::Point2D point = episode.execution_result->final_pose.position;
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
    model_.sampled_path_nodes = model_.nodes;
    model_.sampled_path_edges = model_.edges;
    publish(model_, model_.edges.empty() ? ModelStatus::Incomplete
                                        : ModelStatus::Fresh,
            model_.edges.empty()
                ? "at least two spaced observations are required"
                : "skeleton connectivity and component cache updated");
  }
  last_task_ = episode.active_task;
}

void PassageSkeletonLearner::onRebuild() {
  if (mode_ == SpatialLearningMode::Compatibility) {
    const auto paths = completedPathsFromEpisodes(episodes());
    const auto regions = learnDecisionRegions(episodes(), region_configuration_);
    std::vector<domain::LearnedTrail> trails;
    for (const auto& path : paths) {
      auto trail = learnVisibilityTrail(path, path.id, trail_configuration_);
      if (trail.markers.size() >= 2U) trails.push_back(std::move(trail));
    }
    model_ = learnRegionSkeleton(regions, trails, paths);
    publish(model_, model_.region_nodes.empty() ? ModelStatus::Incomplete
                                                : ModelStatus::Fresh,
            model_.region_nodes.empty()
                ? "no reconciled regions are available for the skeleton"
                : "region skeleton rebuilt with visibility and shortest subtrails");
    return;
  }
  publish(model_, model_.edges.empty() ? ModelStatus::Incomplete
                                      : ModelStatus::Fresh,
          "incremental skeleton snapshot refreshed");
}

}  // namespace semaforr::spatial
