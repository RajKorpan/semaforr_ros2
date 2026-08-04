#ifndef SEMAFORR_SPATIAL_DOOR_EXIT_LEARNER_HPP
#define SEMAFORR_SPATIAL_DOOR_EXIT_LEARNER_HPP

#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

class DoorExitLearner final : public SpatialLearnerBase {
 public:
  DoorExitLearner(double minimum_range_jump_m = 0.75,
                  double maximum_opening_width_m = 2.5);

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;

  double minimum_range_jump_m_;
  double maximum_opening_width_m_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_DOOR_EXIT_LEARNER_HPP
