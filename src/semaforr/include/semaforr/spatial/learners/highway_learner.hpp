/**
 * @file highway_learner.hpp
 * @brief Highway learner responsibilities.
 *
 * @details This file defines highway learner behavior for learned spatial
 * representations and their lifecycle. It centers on
 * `HighwaySmoothingPolicy`, `HighwayComponentSelectionPolicy`,
 * `HighwayLearningConfiguration`, `HighwayComponentSelection`,
 * `HighwayLearner`. Its package-relative location is
 * `include/semaforr/spatial/learners/highway_learner.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_HIGHWAY_LEARNER_HPP
#define SEMAFORR_SPATIAL_HIGHWAY_LEARNER_HPP

#include <map>
#include <semaforr/spatial/learner_base.hpp>
#include <set>
#include <string>

namespace semaforr::spatial {

/**
 * @brief Enumerates the supported highway smoothing policy values used by
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class HighwaySmoothingPolicy { VonNeumannThreeOfFour, DirectionalGapFill };

/**
 * @brief Enumerates the supported highway component selection policy values
 * used by this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class HighwayComponentSelectionPolicy {
  MostIntersections,
  LargestVertexCount
};

/**
 * @brief Encapsulates highway learning configuration state and behavior for
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
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

/**
 * @brief Performs the smooth highway cells operation for this subsystem.
 *
 * Arguments:
 * - @p free_cells: Supplies free cells input to the operation.
 * - @p obstructed_cells: Supplies obstructed cells input to the operation.
 * - @p labeled_cells: Supplies labeled cells input to the operation.
 * - @p policy: Supplies policy input to the operation.
 *
 * Returns:
 * - `HighwayCellSet` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
HighwayCellSet smoothHighwayCells(const HighwayCellSet& free_cells,
                                  const HighwayCellSet& obstructed_cells,
                                  const HighwayCellSet& labeled_cells,
                                  HighwaySmoothingPolicy policy);

/**
 * @brief Encapsulates highway component selection state and behavior for
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct HighwayComponentSelection {
  std::vector<std::size_t> component_by_vertex;
  std::size_t selected_component{0U};
};

/**
 * @brief Performs the select highway component operation for this
 * subsystem.
 *
 * Arguments:
 * - @p graph: Supplies graph input to the operation.
 * - @p policy: Supplies policy input to the operation.
 *
 * Returns:
 * - `HighwayComponentSelection` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
HighwayComponentSelection selectHighwayComponent(
    const domain::Graph<domain::Intersection, domain::HighwayEdge>& graph,
    HighwayComponentSelectionPolicy policy);

/**
 * @brief Encapsulates highway learner state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class HighwayLearner final : public SpatialLearnerBase {
 public:
  /**
   * @brief Performs the highway learner operation for this subsystem.
   *
   * Arguments:
   * - @p minimum_node_spacing_m: Supplies minimum node spacing m input to
   * the operation.
   * - @p passage_clearance_m: Supplies passage clearance m input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit HighwayLearner(double minimum_node_spacing_m = 0.75,
                          double passage_clearance_m = 0.8);
  /**
   * @brief Performs the highway learner operation for this subsystem.
   *
   * Arguments:
   * - @p configuration: Supplies configuration input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit HighwayLearner(HighwayLearningConfiguration configuration);

 private:
  /**
   * @brief Performs the on observe operation for this subsystem.
   *
   * Arguments:
   * - @p episode: Supplies episode input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void onObserve(const NavigationEpisode& episode) override;
  /**
   * @brief Performs the on rebuild operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void onRebuild() override;
  /**
   * @brief Performs the rebuild intersections operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void rebuildIntersections();
  /**
   * @brief Performs the smooth touched grid operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void smoothTouchedGrid();
  /**
   * @brief Performs the extract highways operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void extractHighways();
  /**
   * @brief Performs the materialize grid operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void materializeGrid();
  /**
   * @brief Performs the world cell operation for this subsystem.
   *
   * Arguments:
   * - @p Point2D: Supplies point2 d input to the operation.
   *
   * Returns:
   * - `std::pair<long long, long long>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::pair<long long, long long> worldCell(domain::Point2D) const;
  /**
   * @brief Performs the world center operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   * - @p argument_2: Supplies argument 2 input to the operation.
   *
   * Returns:
   * - `domain::Point2D` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Point2D worldCenter(long long row, long long column) const;
  /**
   * @brief Performs the historical subtrail operation for this subsystem.
   *
   * Arguments:
   * - @p from: Supplies from input to the operation.
   * - @p to: Supplies to input to the operation.
   *
   * Returns:
   * - `std::vector<domain::Point2D>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<domain::Point2D> historicalSubtrail(domain::Point2D from,
                                                  domain::Point2D to) const;

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
