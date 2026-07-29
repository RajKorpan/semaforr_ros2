#include <semaforr/spatial/conveyor_learner.hpp>

#include <cmath>
#include <stdexcept>

#include "LearningGeometry.hpp"

namespace semaforr::spatial {

ConveyorLearner::ConveyorLearner(double minimum_traversal_distance_m)
  : SpatialLearnerBase(
      SpatialRepresentation::Conveyors,
      "conveyor",
      UpdateMode::Incremental,
      {true, false, true, false,
       "accumulate a traversal when the next observation completes it",
       {"ConveyLinear", "ConveyRotation", "conveyor-cost planners"}}),
    minimum_traversal_distance_m_(minimum_traversal_distance_m)
{
  if (!std::isfinite(minimum_traversal_distance_m_) ||
      minimum_traversal_distance_m_ <= 0.0) {
    throw std::invalid_argument(
      "conveyor traversal distance must be finite and positive");
  }
}

void ConveyorLearner::onObserve(const NavigationEpisode& episode)
{
  const domain::Point2D current = episode.observation.pose.position;
  // The displacement ending at this observation was caused by the action
  // selected in the preceding episode, not by the action selected now.
  if (previous_position_ && previous_action_ &&
      previous_action_->type() == domain::ActionType::Forward) {
    domain::Segment2D traversal{*previous_position_, current};
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
  previous_position_ = current;
  previous_action_ = episode.selected_action;
  publish(
    model_,
    model_.flows.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
    model_.flows.empty() ? "no completed forward traversal observed" :
      "conveyor flows updated incrementally");
}

void ConveyorLearner::onRebuild()
{
  publish(
    model_,
    model_.flows.empty() ? ModelStatus::Incomplete : ModelStatus::Fresh,
    model_.flows.empty() ? "no conveyor evidence" :
      "conveyor snapshot rebuilt");
}

}  // namespace semaforr::spatial
