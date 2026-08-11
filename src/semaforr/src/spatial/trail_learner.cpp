#include <cmath>
#include <semaforr/spatial/learners/trail_learner.hpp>
#include <stdexcept>

namespace semaforr::spatial {

TrailLearner::TrailLearner(double minimum_sample_distance_m)
    : SpatialLearnerBase(
          SpatialRepresentation::Trails, "trail", UpdateMode::Incremental,
          {true,
           false,
           false,
           true,
           "append after each accepted pose; start a trace at task boundaries",
           {"TrailerLinear", "TrailerRotation", "trail path planner"},
           UpdateSchedule::AfterSuccessfulActionCompletion}),
      minimum_sample_distance_m_(minimum_sample_distance_m) {
  if (!std::isfinite(minimum_sample_distance_m_) ||
      minimum_sample_distance_m_ <= 0.0) {
    throw std::invalid_argument(
        "trail minimum sample distance must be finite and positive");
  }
}

void TrailLearner::onObserve(const NavigationEpisode& episode) {
  if (!episode.actionSucceeded()) return;
  if (model_.trails.empty() || episode.task_started) {
    model_.trails.emplace_back();
  }
  auto& trail = model_.trails.back();
  if (trail.empty() && episode.execution_result)
    trail.push_back(episode.execution_result->start_pose.position);
  const domain::Point2D position = episode.observation.pose.position;
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
  const bool complete =
      !model_.trails.empty() && model_.trails.back().size() >= 2U;
  publish(
      model_, complete ? ModelStatus::Fresh : ModelStatus::Incomplete,
      complete ? "trail snapshot rebuilt" : "insufficient poses for a trail");
}

}  // namespace semaforr::spatial
