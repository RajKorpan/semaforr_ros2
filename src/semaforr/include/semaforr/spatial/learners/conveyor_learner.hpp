#ifndef SEMAFORR_SPATIAL_CONVEYOR_LEARNER_HPP
#define SEMAFORR_SPATIAL_CONVEYOR_LEARNER_HPP

#include <optional>
#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

class ConveyorLearner final : public SpatialLearnerBase {
 public:
  explicit ConveyorLearner(
      double minimum_traversal_distance_m = 0.05,
      SpatialLearningMode mode = SpatialLearningMode::Modernized,
      ConveyorLearningConfiguration compatibility = {});

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;

  double minimum_traversal_distance_m_;
  SpatialLearningMode mode_;
  ConveyorLearningConfiguration compatibility_;
  std::optional<domain::Point2D> previous_position_;
  std::optional<domain::Action> previous_action_;
  ConveyorModel model_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_CONVEYOR_LEARNER_HPP
