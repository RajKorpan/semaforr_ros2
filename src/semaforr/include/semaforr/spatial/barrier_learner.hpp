#ifndef SEMAFORR_SPATIAL_BARRIER_LEARNER_HPP
#define SEMAFORR_SPATIAL_BARRIER_LEARNER_HPP

#include <semaforr/spatial/spatial_learner_base.hpp>

namespace semaforr::spatial {

class BarrierLearner final : public SpatialLearnerBase {
 public:
  explicit BarrierLearner(double maximum_segment_length_m = 0.5);

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;

  double maximum_segment_length_m_;
  BarrierModel model_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_BARRIER_LEARNER_HPP
