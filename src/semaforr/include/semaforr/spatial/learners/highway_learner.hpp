#ifndef SEMAFORR_SPATIAL_HIGHWAY_LEARNER_HPP
#define SEMAFORR_SPATIAL_HIGHWAY_LEARNER_HPP

#include <semaforr/spatial/learner_base.hpp>

namespace semaforr::spatial {

class HighwayLearner final : public SpatialLearnerBase {
 public:
  explicit HighwayLearner(double minimum_node_spacing_m = 0.75,
                          double passage_clearance_m = 0.8);

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;
  void rebuildIntersections();
  void smoothTouchedGrid();
  void extractHighways();

  double minimum_node_spacing_m_;
  double passage_clearance_m_;
  std::size_t minimum_extent_cells_ = 3U;
  HighwayModel model_;
};

}  // namespace semaforr::spatial

#endif
