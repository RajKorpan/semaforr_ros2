#include <cmath>
#include <semaforr/spatial/learners/conveyor_learner.hpp>
#include <stdexcept>

#include "learning_geometry.hpp"

namespace semaforr::spatial {

ConveyorLearner::ConveyorLearner(
    double minimum_traversal_distance_m, SpatialLearningMode mode,
    ConveyorLearningConfiguration compatibility)
    : SpatialLearnerBase(
          SpatialRepresentation::Conveyors, "conveyor",
          mode == SpatialLearningMode::Compatibility
              ? UpdateMode::RebuildOnDemand
              : UpdateMode::Incremental,
          {true,
           mode == SpatialLearningMode::Compatibility,
           true,
           false,
           mode == SpatialLearningMode::Compatibility
               ? "rasterize successful learned trails into a frequency grid"
               : "merge completed traversal segments",
           {"ConveyLinear", "ConveyRotation", "conveyor-cost planners"},
           mode == SpatialLearningMode::Compatibility
               ? UpdateSchedule::EndOfTarget
               : UpdateSchedule::AfterSuccessfulActionCompletion}),
      minimum_traversal_distance_m_(minimum_traversal_distance_m),
      mode_(mode),
      compatibility_(compatibility) {
  if (!std::isfinite(minimum_traversal_distance_m_) ||
      minimum_traversal_distance_m_ <= 0.0) {
    throw std::invalid_argument(
        "conveyor traversal distance must be finite and positive");
  }
}

void ConveyorLearner::onObserve(const NavigationEpisode& episode) {
  if (mode_ == SpatialLearningMode::Compatibility) return;
  if (!episode.actionSucceeded()) return;
  if (episode.execution_result && episode.selected_action &&
      episode.selected_action->type() == domain::ActionType::Forward) {
    domain::Segment2D traversal{
        episode.execution_result->start_pose.position,
        episode.execution_result->final_pose.position};
    if (traversal.length().meters() >= minimum_traversal_distance_m_) {
      bool merged = false;
      for (ConveyorFlow& flow : model_.flows) {
        if (detail::equivalent(flow.axis, traversal, 0.15)) {
          ++flow.traversals;
          merged = true;
          break;
        }
      }
      if (!merged) {
        model_.flows.push_back({traversal, 1U});
      }
    }
  }
  publish(model_,
          model_.flows.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
          model_.flows.empty() ? "no completed forward traversal observed"
                               : "conveyor flows updated incrementally");
}

void ConveyorLearner::onRebuild() {
  if (mode_ == SpatialLearningMode::Compatibility) {
    std::vector<domain::LearnedTrail> successful_trails;
    for (const auto& path : completedPathsFromEpisodes(episodes())) {
      if (!path.target_reached) continue;
      auto trail = learnVisibilityTrail(path, path.id);
      if (trail.markers.size() >= 2U)
        successful_trails.push_back(std::move(trail));
    }
    model_ = learnConveyorGrid(successful_trails, compatibility_);
    publish(model_, model_.grid.cells.empty() ? ModelStatus::Incomplete
                                              : ModelStatus::Fresh,
            model_.grid.cells.empty()
                ? "no successful completed trail has conveyor evidence"
                : "conveyor frequency grid rebuilt from successful trails");
    return;
  }
  publish(model_,
          model_.flows.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
          model_.flows.empty() ? "no conveyor evidence"
                               : "conveyor snapshot rebuilt");
}

}  // namespace semaforr::spatial
