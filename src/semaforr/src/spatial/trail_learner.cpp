#include <cmath>
#include <semaforr/spatial/learners/trail_learner.hpp>
#include <stdexcept>

namespace semaforr::spatial {

TrailLearner::TrailLearner(
    double minimum_sample_distance_m, SpatialLearningMode mode,
    TrailLearningConfiguration compatibility)
    : SpatialLearnerBase(
          SpatialRepresentation::Trails, "trail",
          mode == SpatialLearningMode::Compatibility
              ? UpdateMode::RebuildOnDemand
              : UpdateMode::Incremental,
          {true,
           mode == SpatialLearningMode::Compatibility,
           false,
           true,
           mode == SpatialLearningMode::Compatibility
               ? "derive backward historical-visibility markers at target completion"
               : "append distance-sampled execution-confirmed poses",
           {"TrailerLinear", "TrailerRotation", "trail path planner"},
           mode == SpatialLearningMode::Compatibility
               ? UpdateSchedule::EndOfTarget
               : UpdateSchedule::AfterSuccessfulActionCompletion}),
      minimum_sample_distance_m_(minimum_sample_distance_m),
      mode_(mode),
      compatibility_(compatibility) {
  if (!std::isfinite(minimum_sample_distance_m_) ||
      minimum_sample_distance_m_ <= 0.0) {
    throw std::invalid_argument(
        "trail minimum sample distance must be finite and positive");
  }
}

void TrailLearner::onObserve(const NavigationEpisode& episode) {
  if (mode_ == SpatialLearningMode::Compatibility) return;
  if (!episode.actionSucceeded()) return;
  if (model_.trails.empty() || episode.task_started) {
    model_.trails.emplace_back();
  }
  auto& trail = model_.trails.back();
  if (trail.empty() && episode.execution_result)
    trail.push_back(episode.execution_result->start_pose.position);
  const domain::Point2D position =
      episode.execution_result->final_pose.position;
  if (trail.empty() || domain::distance(trail.back(), position).meters() >=
                           minimum_sample_distance_m_) {
    trail.push_back(position);
  }
  const bool complete = trail.size() >= 2U;
  publish(model_, complete ? ModelStatus::Fresh : ModelStatus::Incomplete,
          complete ? "trail updated incrementally"
                   : "a trail requires at least two distinct poses");
}

void TrailLearner::onRebuild() {
  if (mode_ == SpatialLearningMode::Compatibility) {
    TrailModel rebuilt;
    for (const auto& path : completedPathsFromEpisodes(episodes())) {
      auto trail = learnVisibilityTrail(path, path.id, compatibility_);
      if (trail.markers.size() < 2U) continue;
      std::vector<domain::Point2D> markers;
      for (const auto& marker : trail.markers)
        markers.push_back(marker.pose.position);
      rebuilt.trails.push_back(std::move(markers));
      rebuilt.learned_trails.push_back(std::move(trail));
    }
    model_ = std::move(rebuilt);
    publish(model_, model_.learned_trails.empty() ? ModelStatus::Incomplete
                                                  : ModelStatus::Fresh,
            model_.learned_trails.empty()
                ? "no execution-confirmed completed path can form a trail"
                : "trails rebuilt by backward historical visibility");
    return;
  }
  const bool complete =
      !model_.trails.empty() && model_.trails.back().size() >= 2U;
  publish(
      model_, complete ? ModelStatus::Fresh : ModelStatus::Incomplete,
      complete ? "trail snapshot rebuilt" : "insufficient poses for a trail");
}

}  // namespace semaforr::spatial
