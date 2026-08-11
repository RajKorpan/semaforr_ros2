#ifndef SEMAFORR_SPATIAL_TRAIL_LEARNER_HPP
#define SEMAFORR_SPATIAL_TRAIL_LEARNER_HPP

#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

class TrailLearner final : public SpatialLearnerBase {
 public:
  explicit TrailLearner(
      double minimum_sample_distance_m = 0.05,
      SpatialLearningMode mode = SpatialLearningMode::Modernized,
      TrailLearningConfiguration compatibility = {});

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;

  double minimum_sample_distance_m_;
  SpatialLearningMode mode_;
  TrailLearningConfiguration compatibility_;
  TrailModel model_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_TRAIL_LEARNER_HPP
