#ifndef SEMAFORR_SPATIAL_CIRCUMSTANCE_LEARNER_HPP
#define SEMAFORR_SPATIAL_CIRCUMSTANCE_LEARNER_HPP

#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

class CircumstanceLearner final : public SpatialLearnerBase {
 public:
  CircumstanceLearner();

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;
  CircumstanceModel model_;
};

}  // namespace semaforr::spatial

#endif
