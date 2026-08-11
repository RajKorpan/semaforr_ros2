#ifndef SEMAFORR_SPATIAL_HIGHWAY_LEARNER_HPP
#define SEMAFORR_SPATIAL_HIGHWAY_LEARNER_HPP

#include <map>
#include <semaforr/spatial/learner_base.hpp>
#include <set>
#include <string>

namespace semaforr::spatial {

enum class HighwaySmoothingPolicy {
  VonNeumannThreeOfFour,
  DirectionalGapFill
};

enum class HighwayComponentSelectionPolicy {
  MostIntersections,
  LargestVertexCount
};

struct HighwayLearningConfiguration {
  double minimum_node_spacing_m{0.75};
  double passage_clearance_m{0.8};
  double grid_resolution_m{0.5};
  domain::Point2D grid_origin;
  std::string frame_id{"map"};
  std::size_t minimum_extent_cells{3U};
  HighwaySmoothingPolicy smoothing_policy{
      HighwaySmoothingPolicy::VonNeumannThreeOfFour};
  HighwayComponentSelectionPolicy component_selection_policy{
      HighwayComponentSelectionPolicy::MostIntersections};
  domain::GridGeometry fixed_geometry;
};

using HighwayCellSet = std::set<std::pair<long long, long long>>;

HighwayCellSet smoothHighwayCells(
    const HighwayCellSet& free_cells,
    const HighwayCellSet& obstructed_cells,
    const HighwayCellSet& labeled_cells,
    HighwaySmoothingPolicy policy);

struct HighwayComponentSelection {
  std::vector<std::size_t> component_by_vertex;
  std::size_t selected_component{0U};
};

HighwayComponentSelection selectHighwayComponent(
    const domain::Graph<domain::Intersection, domain::HighwayEdge>& graph,
    HighwayComponentSelectionPolicy policy);

class HighwayLearner final : public SpatialLearnerBase {
 public:
  explicit HighwayLearner(double minimum_node_spacing_m = 0.75,
                          double passage_clearance_m = 0.8);
  explicit HighwayLearner(HighwayLearningConfiguration configuration);

 private:
  void onObserve(const NavigationEpisode& episode) override;
  void onRebuild() override;
  void rebuildIntersections();
  void smoothTouchedGrid();
  void extractHighways();
  void materializeGrid();
  std::pair<long long, long long> worldCell(domain::Point2D) const;
  domain::Point2D worldCenter(long long row, long long column) const;
  std::vector<domain::Point2D> historicalSubtrail(
      domain::Point2D from, domain::Point2D to) const;

  HighwayLearningConfiguration configuration_;
  HighwayCellSet free_cells_;
  HighwayCellSet obstructed_cells_;
  HighwayCellSet highway_cells_;
  std::set<long long> touched_world_rows_;
  std::set<long long> touched_world_columns_;
  HighwayModel model_;
};

}  // namespace semaforr::spatial

#endif
