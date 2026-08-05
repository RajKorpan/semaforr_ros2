#ifndef SEMAFORR_SPATIAL_GRID_LEARNERS_HPP
#define SEMAFORR_SPATIAL_GRID_LEARNERS_HPP

#include <semaforr/spatial/learner_base.hpp>
#include <unordered_map>

namespace semaforr::spatial {

enum class GridExtentPolicy { Fixed, Expand };

class KnownGridLearner final : public SpatialLearnerBase {
 public:
  KnownGridLearner(std::size_t columns = 200U, std::size_t rows = 200U,
                   double resolution_m = 1.0,
                   domain::Point2D origin = {},
                   GridExtentPolicy extent_policy = GridExtentPolicy::Expand);

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;
  GridGeometry geometry_;
  std::unordered_map<std::size_t, std::uint32_t> observations_;
  std::unordered_map<std::size_t, std::size_t> last_observed_sequence_;
  GridExtentPolicy extent_policy_;
};

struct SensedOccupancyLearningConfiguration {
  std::uint16_t free_observations_to_clear = 3U;
  std::size_t dynamic_expiry_observations = 30U;
  bool treat_obstacle_returns_as_dynamic = true;
};

class SensedOccupancyLearner final : public SpatialLearnerBase {
 public:
  SensedOccupancyLearner(
      std::size_t columns = 200U, std::size_t rows = 200U,
      double resolution_m = 1.0, domain::Point2D origin = {},
      SensedOccupancyLearningConfiguration configuration = {},
      GridExtentPolicy extent_policy = GridExtentPolicy::Expand);

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;
  void integrateFree(std::size_t index, std::size_t sequence);
  void integrateOccupied(std::size_t index, std::size_t sequence);
  void expireDynamic(std::size_t sequence);
  SensedOccupancyModel snapshotModel() const;

  GridGeometry geometry_;
  SensedOccupancyLearningConfiguration configuration_;
  std::unordered_map<std::size_t, domain::SensedOccupancyCell> cells_;
  GridExtentPolicy extent_policy_;
};

class InclusionGridLearner final : public SpatialLearnerBase {
 public:
  InclusionGridLearner(std::size_t columns = 200U, std::size_t rows = 200U,
                       double resolution_m = 1.0,
                       domain::Point2D origin = {},
                       GridExtentPolicy extent_policy = GridExtentPolicy::Expand);

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;
  GridGeometry geometry_;
  std::unordered_map<std::size_t, std::uint32_t> included_;
  GridExtentPolicy extent_policy_;
};

}  // namespace semaforr::spatial

#endif
