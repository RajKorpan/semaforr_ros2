#ifndef SEMAFORR_SPATIAL_HALLWAY_LEARNER_HPP
#define SEMAFORR_SPATIAL_HALLWAY_LEARNER_HPP

#include <semaforr/spatial/spatial_learner_base.hpp>

namespace semaforr::spatial {

class HallwayLearner final : public SpatialLearnerBase {
public:
  explicit HallwayLearner(double minimum_centerline_length_m = 0.5);

private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;

  double minimum_centerline_length_m_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_HALLWAY_LEARNER_HPP
