/**
 * @file highway_learner.cpp
 * @brief Highway learner responsibilities.
 *
 * @details This file implements highway learner behavior for learned spatial
 * representations and their lifecycle. It centers on `Score`. Its
 * package-relative location is `src/spatial/highway_learner.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <semaforr/spatial/learners/highway_learner.hpp>
#include <set>
#include <stdexcept>

namespace semaforr::spatial {
namespace {

/**
 * @brief Performs the smoothing name operation for this subsystem.
 *
 * Arguments:
 * - @p policy: Supplies policy input to the operation.
 *
 * Returns:
 * - `const char*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const char* smoothingName(HighwaySmoothingPolicy policy) noexcept {
  return policy == HighwaySmoothingPolicy::VonNeumannThreeOfFour
             ? "von_neumann_three_of_four"
             : "directional_gap_fill";
}

/**
 * @brief Performs the component name operation for this subsystem.
 *
 * Arguments:
 * - @p policy: Supplies policy input to the operation.
 *
 * Returns:
 * - `const char*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
const char* componentName(HighwayComponentSelectionPolicy policy) noexcept {
  return policy == HighwayComponentSelectionPolicy::MostIntersections
             ? "most_intersections"
             : "largest_vertex_count";
}

/**
 * @brief Performs the polyline length operation for this subsystem.
 *
 * Arguments:
 * - @p points: Supplies points input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double polylineLength(const std::vector<domain::Point2D>& points) {
  double result = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index)
    result += domain::distance(points[index - 1U], points[index]).meters();
  return result;
}

/**
 * @brief Performs the scan passage count operation for this subsystem.
 *
 * Arguments:
 * - @p laser: Supplies laser input to the operation.
 * - @p minimum_clearance_m: Supplies minimum clearance m input to the
 * operation.
 *
 * Returns:
 * - `std::size_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::size_t scanPassageCount(const domain::LaserObservation& laser,
                             double minimum_clearance_m) {
  std::size_t run = 0U;
  std::size_t passages = 0U;
  for (const double range : laser.ranges_m) {
    const double clear = std::isfinite(range)
                             ? range
                             : laser.maximum_range.meters();
    run = clear >= minimum_clearance_m ? run + 1U : 0U;
    if (run == 3U) ++passages;
  }
  return passages;
}

/**
 * @brief Performs the configuration with thresholds operation for this
 * subsystem.
 *
 * Arguments:
 * - @p minimum_node_spacing_m: Supplies minimum node spacing m input to the
 * operation.
 * - @p passage_clearance_m: Supplies passage clearance m input to the
 * operation.
 *
 * Returns:
 * - `HighwayLearningConfiguration` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
HighwayLearningConfiguration configurationWithThresholds(
    double minimum_node_spacing_m, double passage_clearance_m) {
  HighwayLearningConfiguration result;
  result.minimum_node_spacing_m = minimum_node_spacing_m;
  result.passage_clearance_m = passage_clearance_m;
  return result;
}

}  // namespace

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
HighwayCellSet smoothHighwayCells(
    const HighwayCellSet& free_cells,
    const HighwayCellSet& obstructed_cells,
    const HighwayCellSet& labeled_cells,
    HighwaySmoothingPolicy policy) {
  HighwayCellSet result = labeled_cells;
  HighwayCellSet candidates = free_cells;
  if (policy == HighwaySmoothingPolicy::DirectionalGapFill)
    for (const auto& [row, column] : labeled_cells)
      for (long long delta = -1; delta <= 1; ++delta) {
        candidates.insert({row + delta, column});
        candidates.insert({row, column + delta});
      }
  for (const auto& cell : candidates) {
    if (result.contains(cell) || obstructed_cells.contains(cell)) continue;
    const auto [row, column] = cell;
    if (policy == HighwaySmoothingPolicy::VonNeumannThreeOfFour) {
      const std::size_t free_neighbors =
          static_cast<std::size_t>(free_cells.contains({row - 1, column})) +
          static_cast<std::size_t>(free_cells.contains({row + 1, column})) +
          static_cast<std::size_t>(free_cells.contains({row, column - 1})) +
          static_cast<std::size_t>(free_cells.contains({row, column + 1}));
      if (free_neighbors >= 3U) result.insert(cell);
    } else if ((labeled_cells.contains({row, column - 1}) &&
                labeled_cells.contains({row, column + 1})) ||
               (labeled_cells.contains({row - 1, column}) &&
                labeled_cells.contains({row + 1, column}))) {
      result.insert(cell);
    }
  }
  return result;
}

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
    HighwayComponentSelectionPolicy policy) {
  HighwayComponentSelection result;
  result.component_by_vertex.assign(
      graph.vertices.size(), std::numeric_limits<std::size_t>::max());
  std::vector<std::vector<std::size_t>> adjacency(graph.vertices.size());
  for (const auto& edge : graph.edges) {
    if (edge.from >= adjacency.size() || edge.to >= adjacency.size()) continue;
    adjacency[edge.from].push_back(edge.to);
    adjacency[edge.to].push_back(edge.from);
  }
  struct Score { std::size_t vertices{0U}, intersections{0U}; };
  std::vector<Score> scores;
  for (std::size_t root = 0U; root < adjacency.size(); ++root) {
    if (result.component_by_vertex[root] !=
        std::numeric_limits<std::size_t>::max())
      continue;
    const auto id = scores.size();
    std::vector<std::size_t> pending{root};
    result.component_by_vertex[root] = id;
    Score score;
    while (!pending.empty()) {
      const auto vertex = pending.back();
      pending.pop_back();
      ++score.vertices;
      if (!graph.vertices[vertex].terminal_access) ++score.intersections;
      for (const auto neighbor : adjacency[vertex])
        if (result.component_by_vertex[neighbor] ==
            std::numeric_limits<std::size_t>::max()) {
          result.component_by_vertex[neighbor] = id;
          pending.push_back(neighbor);
        }
    }
    scores.push_back(score);
  }
  for (std::size_t candidate = 1U; candidate < scores.size(); ++candidate) {
    const bool better =
        policy == HighwayComponentSelectionPolicy::MostIntersections
            ? std::pair{scores[candidate].intersections,
                        scores[candidate].vertices} >
                  std::pair{scores[result.selected_component].intersections,
                            scores[result.selected_component].vertices}
            : std::pair{scores[candidate].vertices,
                        scores[candidate].intersections} >
                  std::pair{scores[result.selected_component].vertices,
                            scores[result.selected_component].intersections};
    if (better) result.selected_component = candidate;
  }
  return result;
}

/**
 * @brief Performs the highway learner operation for this subsystem.
 *
 * Arguments:
 * - @p minimum_node_spacing_m: Supplies minimum node spacing m input to the
 * operation.
 * - @p passage_clearance_m: Supplies passage clearance m input to the
 * operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
HighwayLearner::HighwayLearner(double minimum_node_spacing_m,
                               double passage_clearance_m)
    : HighwayLearner(configurationWithThresholds(minimum_node_spacing_m,
                                                 passage_clearance_m)) {}

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
HighwayLearner::HighwayLearner(HighwayLearningConfiguration configuration)
    : SpatialLearnerBase(
          SpatialRepresentation::Highways, "highways",
          UpdateMode::Incremental,
          {true, true, true, false, "every exploration observation",
           {"HighwayPlan", "Enforcer"},
           UpdateSchedule::EndOfInitialExploration}),
      configuration_(std::move(configuration)) {
  if (!(configuration_.minimum_node_spacing_m > 0.0) ||
      !(configuration_.passage_clearance_m > 0.0) ||
      !(configuration_.grid_resolution_m > 0.0) ||
      configuration_.frame_id.empty() ||
      configuration_.minimum_extent_cells == 0U ||
      !configuration_.grid_origin.finite())
    throw std::invalid_argument("highway learner geometry and thresholds are invalid");
  if (configuration_.fixed_geometry.valid()) {
    configuration_.fixed_geometry.validate();
    configuration_.grid_origin = configuration_.fixed_geometry.origin;
    configuration_.grid_resolution_m =
        configuration_.fixed_geometry.resolution_m;
    configuration_.frame_id = configuration_.fixed_geometry.frame_id;
  }
  model_.smoothing_policy = smoothingName(configuration_.smoothing_policy);
  model_.component_selection_policy =
      componentName(configuration_.component_selection_policy);
}

/**
 * @brief Performs the world cell operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `std::pair<long long, long long>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::pair<long long, long long> HighwayLearner::worldCell(
    domain::Point2D point) const {
  return {static_cast<long long>(std::floor(
              (point.y_m - configuration_.grid_origin.y_m) /
              configuration_.grid_resolution_m)),
          static_cast<long long>(std::floor(
              (point.x_m - configuration_.grid_origin.x_m) /
              configuration_.grid_resolution_m))};
}

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
domain::Point2D HighwayLearner::worldCenter(long long row,
                                            long long column) const {
  return {configuration_.grid_origin.x_m +
              (static_cast<double>(column) + 0.5) *
                  configuration_.grid_resolution_m,
          configuration_.grid_origin.y_m +
              (static_cast<double>(row) + 0.5) *
                  configuration_.grid_resolution_m};
}

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
std::vector<domain::Point2D> HighwayLearner::historicalSubtrail(
    domain::Point2D from, domain::Point2D to) const {
  if (model_.nodes.empty()) return {from, to};
  const auto nearest = [&](domain::Point2D point) {
    std::size_t selected = 0U;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < model_.nodes.size(); ++index) {
      const double candidate = domain::distance(model_.nodes[index], point).meters();
      if (candidate < best) {
        best = candidate;
        selected = index;
      }
    }
    return selected;
  };
  const auto from_index = nearest(from);
  const auto to_index = nearest(to);
  std::vector<domain::Point2D> result;
  if (from_index <= to_index) {
    result.insert(result.end(), model_.nodes.begin() +
                                    static_cast<std::ptrdiff_t>(from_index),
                  model_.nodes.begin() +
                      static_cast<std::ptrdiff_t>(to_index + 1U));
  } else {
    for (std::size_t index = from_index;; --index) {
      result.push_back(model_.nodes[index]);
      if (index == to_index) break;
    }
  }
  if (result.empty() || domain::distance(result.front(), from).meters() >
                            domain::geometry_tolerance_m)
    result.insert(result.begin(), from);
  if (domain::distance(result.back(), to).meters() >
      domain::geometry_tolerance_m)
    result.push_back(to);
  return result;
}

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
void HighwayLearner::rebuildIntersections() {
  std::vector<std::size_t> degree(model_.nodes.size(), 0U);
  for (const auto& edge : model_.edges) {
    if (edge.from < degree.size()) ++degree[edge.from];
    if (edge.to < degree.size()) ++degree[edge.to];
  }
  model_.intersections.clear();
  for (std::size_t node = 0U; node < degree.size(); ++node)
    if (degree[node] >= 3U) model_.intersections.push_back({node, degree[node]});
}

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
void HighwayLearner::smoothTouchedGrid() {
  const auto smoothed = smoothHighwayCells(
      free_cells_, obstructed_cells_, highway_cells_,
      configuration_.smoothing_policy);
  for (const auto& cell : smoothed)
    if (!highway_cells_.contains(cell)) {
      touched_world_rows_.insert(cell.first);
      touched_world_columns_.insert(cell.second);
    }
  highway_cells_ = smoothed;
}

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
void HighwayLearner::materializeGrid() {
  model_.grid_labels.clear();
  model_.touched_rows.clear();
  model_.touched_columns.clear();
  if (highway_cells_.empty()) {
    model_.geometry = {};
    return;
  }
  long long minimum_row = highway_cells_.begin()->first;
  long long maximum_row = minimum_row;
  long long minimum_column = highway_cells_.begin()->second;
  long long maximum_column = minimum_column;
  for (const auto& [row, column] : highway_cells_) {
    minimum_row = std::min(minimum_row, row);
    maximum_row = std::max(maximum_row, row);
    minimum_column = std::min(minimum_column, column);
    maximum_column = std::max(maximum_column, column);
  }
  if (configuration_.fixed_geometry.valid()) {
    model_.geometry = configuration_.fixed_geometry;
    minimum_row = 0;
    minimum_column = 0;
  } else {
    model_.geometry = domain::GridGeometry::fromBounds(
        configuration_.frame_id,
        {configuration_.grid_origin.x_m +
             static_cast<double>(minimum_column) *
                 configuration_.grid_resolution_m,
         configuration_.grid_origin.y_m +
             static_cast<double>(minimum_row) *
                 configuration_.grid_resolution_m},
        {configuration_.grid_origin.x_m +
             static_cast<double>(maximum_column + 1) *
                 configuration_.grid_resolution_m,
         configuration_.grid_origin.y_m +
             static_cast<double>(maximum_row + 1) *
                 configuration_.grid_resolution_m},
        configuration_.grid_resolution_m, domain::GridExtentMode::Expandable,
        domain::GridExtentSource::RepresentationLocalBounds,
        domain::GridOutOfBoundsBehavior::ExpandBeforeInsert, 1U);
  }
  for (const auto& [row, column] : highway_cells_)
    model_.grid_labels.push_back(
        {static_cast<int>(row - minimum_row),
         static_cast<int>(column - minimum_column), 1U});
  for (const auto row : touched_world_rows_)
    model_.touched_rows.push_back(static_cast<int>(row - minimum_row));
  for (const auto column : touched_world_columns_)
    model_.touched_columns.push_back(static_cast<int>(column - minimum_column));
}

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
void HighwayLearner::extractHighways() {
  std::map<int, std::vector<int>> rows;
  std::map<int, std::vector<int>> columns;
  for (const auto& label : model_.grid_labels) {
    if (label.label == 0U) continue;
    rows[label.row].push_back(label.column);
    columns[label.column].push_back(label.row);
  }
  model_.highways.clear();
  const auto extract = [this](auto& bins, domain::Axis axis) {
    for (auto& [fixed, values] : bins) {
      std::sort(values.begin(), values.end());
      values.erase(std::unique(values.begin(), values.end()), values.end());
      std::size_t begin = 0U;
      while (begin < values.size()) {
        std::size_t end = begin;
        while (end + 1U < values.size() && values[end + 1U] == values[end] + 1)
          ++end;
        if (end - begin + 1U >= configuration_.minimum_extent_cells) {
          domain::Highway highway;
          highway.id = model_.highways.size();
          highway.axis = axis;
          for (std::size_t index = begin; index <= end; ++index)
            highway.cells.push_back(axis == domain::Axis::Horizontal
                                        ? domain::GridCell{fixed, values[index]}
                                        : domain::GridCell{values[index], fixed});
          model_.highways.push_back(std::move(highway));
        }
        begin = end + 1U;
      }
    }
  };
  extract(rows, domain::Axis::Horizontal);
  extract(columns, domain::Axis::Vertical);

  std::map<std::pair<int, int>, std::vector<domain::HighwayId>> memberships;
  for (const auto& highway : model_.highways)
    for (const auto& cell : highway.cells)
      memberships[{cell.row, cell.column}].push_back(highway.id);

  model_.graph = {};
  std::map<std::pair<int, int>, domain::IntersectionId> intersection_ids;
  const auto ensureIntersection =
      [this, &intersection_ids](const domain::GridCell& cell, bool terminal) {
        const auto key = std::make_pair(cell.row, cell.column);
        const auto found = intersection_ids.find(key);
        if (found != intersection_ids.end()) {
          if (!terminal) model_.graph.vertices[found->second].terminal_access = false;
          return found->second;
        }
        const auto id = model_.graph.vertices.size();
        intersection_ids.emplace(key, id);
        model_.graph.vertices.push_back(
            {id, cell,
             model_.geometry.center(static_cast<std::size_t>(cell.column),
                                    static_cast<std::size_t>(cell.row)),
             terminal});
        return id;
      };
  for (const auto& [cell, highways] : memberships)
    if (highways.size() >= 2U) ensureIntersection({cell.first, cell.second}, false);

  for (auto& highway : model_.highways) {
    highway.endpoints.clear();
    for (const auto& cell : highway.cells) {
      const auto found = intersection_ids.find({cell.row, cell.column});
      if (found != intersection_ids.end()) highway.endpoints.push_back(found->second);
    }
    const auto addTerminal = [&](const domain::GridCell& cell) {
      const auto id = ensureIntersection(cell, true);
      if (std::find(highway.endpoints.begin(), highway.endpoints.end(), id) ==
          highway.endpoints.end())
        highway.endpoints.push_back(id);
    };
    if (highway.endpoints.empty()) {
      addTerminal(highway.cells.front());
      addTerminal(highway.cells.back());
    } else if (highway.endpoints.size() == 1U) {
      const auto& known = model_.graph.vertices[highway.endpoints.front()].cell;
      addTerminal(highway.cells.front() == known ? highway.cells.back()
                                                 : highway.cells.front());
    }
    std::sort(highway.endpoints.begin(), highway.endpoints.end());
    if (highway.endpoints.size() < 2U) continue;
    const auto from = highway.endpoints.front();
    const auto to = highway.endpoints.back();
    auto subtrail = historicalSubtrail(model_.graph.vertices[from].position,
                                       model_.graph.vertices[to].position);
    model_.graph.edges.push_back(
        {from, to, highway.id, polylineLength(subtrail), {highway.id},
         std::move(subtrail)});
  }

  const auto selection = selectHighwayComponent(
      model_.graph, configuration_.component_selection_policy);
  if (selection.component_by_vertex.empty()) return;
  const auto& component = selection.component_by_vertex;
  const auto selected = selection.selected_component;
  model_.graph.edges.erase(
      std::remove_if(model_.graph.edges.begin(), model_.graph.edges.end(),
                     [&](const auto& edge) {
                       return component[edge.from] != selected;
                     }),
      model_.graph.edges.end());
  std::set<domain::HighwayId> retained;
  for (const auto& edge : model_.graph.edges) retained.insert(edge.highway);
  model_.highways.erase(
      std::remove_if(model_.highways.begin(), model_.highways.end(),
                     [&](const auto& highway) {
                       return !retained.contains(highway.id);
                     }),
      model_.highways.end());
  const auto missing = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> remap(model_.graph.vertices.size(), missing);
  std::vector<domain::Intersection> vertices;
  for (std::size_t old = 0U; old < model_.graph.vertices.size(); ++old) {
    if (component[old] != selected) continue;
    remap[old] = vertices.size();
    auto vertex = model_.graph.vertices[old];
    vertex.id = vertices.size();
    vertices.push_back(std::move(vertex));
  }
  for (auto& edge : model_.graph.edges) {
    edge.from = remap[edge.from];
    edge.to = remap[edge.to];
  }
  for (auto& highway : model_.highways) {
    std::vector<domain::IntersectionId> endpoints;
    for (const auto endpoint : highway.endpoints)
      if (endpoint < remap.size() && remap[endpoint] != missing)
        endpoints.push_back(remap[endpoint]);
    highway.endpoints = std::move(endpoints);
  }
  model_.graph.vertices = std::move(vertices);
}

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
void HighwayLearner::onObserve(const NavigationEpisode& episode) {
  if (!episode.initial_exploration) return;
  const auto& observation = episode.observation;
  const double resolution = configuration_.grid_resolution_m;
  for (std::size_t beam = 0U; beam < observation.laser.ranges_m.size(); ++beam) {
    const double range = observation.laser.ranges_m[beam];
    if (!std::isfinite(range) ||
        range < observation.laser.minimum_range.meters())
      continue;
    const double extent = std::min(range, observation.laser.maximum_range.meters());
    const bool hit = range + domain::geometry_tolerance_m <
                     observation.laser.maximum_range.meters();
    const std::size_t samples = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(extent / resolution)));
    for (std::size_t sample = 0U; sample <= samples; ++sample) {
      const double angle = observation.pose.heading.radians() +
                           observation.laser.angle_min.radians() +
                           static_cast<double>(beam) *
                               observation.laser.angle_increment.radians();
      const double distance = extent * static_cast<double>(sample) /
                              static_cast<double>(samples);
      const domain::Point2D point{
          observation.pose.position.x_m + distance * std::cos(angle),
          observation.pose.position.y_m + distance * std::sin(angle)};
      if (configuration_.fixed_geometry.valid() &&
          !configuration_.fixed_geometry.contains(point))
        continue;
      const auto cell = worldCell(point);
      if (hit && sample == samples) {
        obstructed_cells_.insert(cell);
        free_cells_.erase(cell);
      } else if (!obstructed_cells_.contains(cell)) {
        free_cells_.insert(cell);
      }
    }
  }
  const std::size_t passages = scanPassageCount(
      observation.laser, configuration_.passage_clearance_m);
  if (passages == 0U) return;
  const auto point = observation.pose.position;
  if (configuration_.fixed_geometry.valid() &&
      !configuration_.fixed_geometry.contains(point))
    return;
  if (!model_.nodes.empty() &&
      domain::distance(model_.nodes.back(), point).meters() <
          configuration_.minimum_node_spacing_m)
    return;
  const std::size_t node = model_.nodes.size();
  model_.nodes.push_back(point);
  if (node > 0U) model_.edges.push_back({node - 1U, node});
  const auto labelPoint = [this](domain::Point2D sample) {
    if (configuration_.fixed_geometry.valid() &&
        !configuration_.fixed_geometry.contains(sample))
      return;
    const auto cell = worldCell(sample);
    if (obstructed_cells_.contains(cell)) return;
    highway_cells_.insert(cell);
    free_cells_.insert(cell);
    touched_world_rows_.insert(cell.first);
    touched_world_columns_.insert(cell.second);
  };
  if (node == 0U) {
    labelPoint(point);
  } else {
    const auto& start = model_.nodes[node - 1U];
    const double length = domain::distance(start, point).meters();
    const auto samples = std::max<std::size_t>(
        1U, static_cast<std::size_t>(
                std::ceil(length / (configuration_.grid_resolution_m / 2.0))));
    for (std::size_t sample = 0U; sample <= samples; ++sample) {
      const double fraction = static_cast<double>(sample) /
                              static_cast<double>(samples);
      labelPoint({start.x_m + fraction * (point.x_m - start.x_m),
                  start.y_m + fraction * (point.y_m - start.y_m)});
    }
  }
  if (passages >= 3U && node > 1U) {
    std::size_t branch = 0U;
    double best = domain::distance(model_.nodes[0], point).meters();
    for (std::size_t candidate = 1U; candidate + 1U < node; ++candidate) {
      const double candidate_distance =
          domain::distance(model_.nodes[candidate], point).meters();
      if (candidate_distance < best) {
        branch = candidate;
        best = candidate_distance;
      }
    }
    if (branch != node - 1U) model_.edges.push_back({branch, node});
  }
}

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
void HighwayLearner::onRebuild() {
  smoothTouchedGrid();
  materializeGrid();
  extractHighways();
  rebuildIntersections();
  publish(model_, model_.nodes.size() >= 2U ? ModelStatus::Fresh
                                            : ModelStatus::Incomplete,
          std::string{"highway graph published; smoothing="} +
              model_.smoothing_policy + "; component_selection=" +
              model_.component_selection_policy);
}

}  // namespace semaforr::spatial
