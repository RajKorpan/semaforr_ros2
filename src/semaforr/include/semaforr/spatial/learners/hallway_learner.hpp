#ifndef SEMAFORR_SPATIAL_HALLWAY_LEARNER_HPP
#define SEMAFORR_SPATIAL_HALLWAY_LEARNER_HPP

#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

class HallwayLearner final : public SpatialLearnerBase {
 public:
  explicit HallwayLearner(
      double minimum_centerline_length_m = 0.5,
      SpatialLearningMode mode = SpatialLearningMode::Modernized,
      HallwayLearningConfiguration compatibility = {});

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;

  double minimum_centerline_length_m_;
  SpatialLearningMode mode_;
  HallwayLearningConfiguration compatibility_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_HALLWAY_LEARNER_HPP
