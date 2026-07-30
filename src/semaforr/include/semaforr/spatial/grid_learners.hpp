#ifndef SEMAFORR_SPATIAL_GRID_LEARNERS_HPP
#define SEMAFORR_SPATIAL_GRID_LEARNERS_HPP

#include <semaforr/spatial/spatial_learner_base.hpp>
#include <unordered_map>

namespace semaforr::spatial {

class KnownGridLearner final : public SpatialLearnerBase {
 public:
  KnownGridLearner(std::size_t columns = 200U, std::size_t rows = 200U,
                   double resolution_m = 1.0,
                   domain::Point2D origin = {});

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;
  GridGeometry geometry_;
  std::unordered_map<std::size_t, std::uint32_t> observations_;
};

class InclusionGridLearner final : public SpatialLearnerBase {
 public:
  InclusionGridLearner(std::size_t columns = 200U, std::size_t rows = 200U,
                       double resolution_m = 1.0,
                       domain::Point2D origin = {});

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;
  GridGeometry geometry_;
  std::unordered_map<std::size_t, std::uint32_t> included_;
};

}  // namespace semaforr::spatial

#endif
