#include <cmath>
#include <semaforr/spatial/learners/conveyor_learner.hpp>
#include <stdexcept>

#include "learning_geometry.hpp"

namespace semaforr::spatial {

ConveyorLearner::ConveyorLearner(double minimum_traversal_distance_m)
    : SpatialLearnerBase(
          SpatialRepresentation::Conveyors, "conveyor", UpdateMode::Incremental,
          {true,
           false,
           true,
           false,
           "accumulate a traversal when the next observation completes it",
           {"ConveyLinear", "ConveyRotation", "conveyor-cost planners"},
           UpdateSchedule::AfterSuccessfulActionCompletion}),
      minimum_traversal_distance_m_(minimum_traversal_distance_m) {
  if (!std::isfinite(minimum_traversal_distance_m_) ||
      minimum_traversal_distance_m_ <= 0.0) {
    throw std::invalid_argument(
        "conveyor traversal distance must be finite and positive");
  }
}

void ConveyorLearner::onObserve(const NavigationEpisode& episode) {
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
  publish(model_,
          model_.flows.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
          model_.flows.empty() ? "no conveyor evidence"
                               : "conveyor snapshot rebuilt");
}

}  // namespace semaforr::spatial
