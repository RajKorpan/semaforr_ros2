#ifndef SEMAFORR_SPATIAL_TRAIL_LEARNER_HPP
#define SEMAFORR_SPATIAL_TRAIL_LEARNER_HPP

#include <semaforr/spatial/spatial_learner_base.hpp>

namespace semaforr::spatial {

class TrailLearner final : public SpatialLearnerBase {
 public:
  explicit TrailLearner(double minimum_sample_distance_m = 0.05);

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;

  double minimum_sample_distance_m_;
  TrailModel model_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_TRAIL_LEARNER_HPP
