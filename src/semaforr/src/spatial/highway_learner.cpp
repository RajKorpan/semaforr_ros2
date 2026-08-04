#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/exploration/highway_explorer.hpp>
#include <semaforr/spatial/learners/highway_learner.hpp>
#include <map>
#include <set>
#include <stdexcept>

namespace semaforr::spatial {

HighwayLearner::HighwayLearner(double minimum_node_spacing_m,
                               double passage_clearance_m)
    : SpatialLearnerBase(
          SpatialRepresentation::Highways, "highways",
          UpdateMode::Incremental,
          {true, true, true, false, "every exploration observation",
           {"HighwayPlan", "Enforcer"},
           UpdateSchedule::EndOfInitialExploration}),
      minimum_node_spacing_m_(minimum_node_spacing_m),
      passage_clearance_m_(passage_clearance_m) {
  if (!(minimum_node_spacing_m_ > 0.0) ||
      !(passage_clearance_m_ > 0.0))
    throw std::invalid_argument("highway learner thresholds must be positive");
}

void HighwayLearner::rebuildIntersections() {
  std::vector<std::size_t> degree(model_.nodes.size(), 0U);
  for (const auto& edge : model_.edges) {
    if (edge.from < degree.size()) ++degree[edge.from];
    if (edge.to < degree.size()) ++degree[edge.to];
  }
  model_.intersections.clear();
  for (std::size_t node = 0U; node < degree.size(); ++node)
    if (degree[node] >= 3U)
      model_.intersections.push_back({node, degree[node]});
}

void HighwayLearner::smoothTouchedGrid() {
  std::set<std::pair<std::size_t, std::size_t>> cells;
  for (const auto& label : model_.grid_labels)
    if (label.label != 0U) cells.emplace(label.row, label.column);
  std::vector<HighwayGridLabel> additions;
  for (const auto row : model_.touched_rows) {
    for (const auto column : model_.touched_columns) {
      if (!cells.contains({row, column}) &&
          cells.contains({row, column > 0U ? column - 1U : column}) &&
          cells.contains({row, column + 1U}))
        additions.push_back({row, column, 1U});
    }
  }
  for (const auto column : model_.touched_columns) {
    for (const auto row : model_.touched_rows) {
      if (!cells.contains({row, column}) &&
          cells.contains({row > 0U ? row - 1U : row, column}) &&
          cells.contains({row + 1U, column}))
        additions.push_back({row, column, 1U});
    }
  }
  for (const auto& addition : additions)
    if (cells.insert({addition.row, addition.column}).second)
      model_.grid_labels.push_back(addition);
}

void HighwayLearner::extractHighways() {
  std::map<std::size_t, std::vector<std::size_t>> rows;
  std::map<std::size_t, std::vector<std::size_t>> columns;
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
        while (end + 1U < values.size() &&
               values[end + 1U] == values[end] + 1U)
          ++end;
        if (end - begin + 1U >= minimum_extent_cells_) {
          domain::Highway highway;
          highway.id = model_.highways.size();
          highway.axis = axis;
          for (std::size_t index = begin; index <= end; ++index)
            highway.cells.push_back(
                axis == domain::Axis::Horizontal
                    ? domain::GridCell{static_cast<int>(fixed),
                                       static_cast<int>(values[index])}
                    : domain::GridCell{static_cast<int>(values[index]),
                                       static_cast<int>(fixed)});
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
          if (!terminal)
            model_.graph.vertices[found->second].terminal_access = false;
          return found->second;
        }
        const auto id = model_.graph.vertices.size();
        intersection_ids.emplace(key, id);
        model_.graph.vertices.push_back(
            {id, cell,
             {static_cast<double>(cell.column) + 0.5,
              static_cast<double>(cell.row) + 0.5},
             terminal});
        return id;
      };
  for (const auto& [cell, highways] : memberships)
    if (highways.size() >= 2U)
      ensureIntersection({cell.first, cell.second}, false);

  for (auto& highway : model_.highways) {
    highway.endpoints.clear();
    for (const auto& cell : highway.cells) {
      const auto found = intersection_ids.find({cell.row, cell.column});
      if (found != intersection_ids.end())
        highway.endpoints.push_back(found->second);
    }
    // A one-ended extent is a spur; a zero-ended extent receives two terminal
    // access intersections so every retained highway is graph-operational.
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
    if (highway.endpoints.size() >= 2U) {
      const auto from = highway.endpoints.front();
      const auto to = highway.endpoints.back();
      model_.graph.edges.push_back(
          {from, to, highway.id,
           domain::distance(model_.graph.vertices[from].position,
                            model_.graph.vertices[to].position)
               .meters(),
           {highway.id}});
    }
  }

  // Retain only the largest connected component of the highway graph.
  std::vector<std::vector<std::size_t>> adjacency(model_.graph.vertices.size());
  for (const auto& edge : model_.graph.edges) {
    adjacency[edge.from].push_back(edge.to);
    adjacency[edge.to].push_back(edge.from);
  }
  std::vector<int> component(adjacency.size(), -1);
  std::vector<std::size_t> sizes;
  for (std::size_t root = 0U; root < adjacency.size(); ++root) {
    if (component[root] >= 0) continue;
    const int id = static_cast<int>(sizes.size());
    std::vector<std::size_t> pending{root};
    component[root] = id;
    std::size_t size = 0U;
    while (!pending.empty()) {
      const auto node = pending.back();
      pending.pop_back();
      ++size;
      for (const auto neighbor : adjacency[node])
        if (component[neighbor] < 0) {
          component[neighbor] = id;
          pending.push_back(neighbor);
        }
    }
    sizes.push_back(size);
  }
  if (!sizes.empty()) {
    const auto selected = static_cast<int>(
        std::distance(sizes.begin(),
                      std::max_element(sizes.begin(), sizes.end())));
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
}

void HighwayLearner::onObserve(const NavigationEpisode& episode) {
  if (!episode.initial_exploration) return;
  const auto passages = exploration::HighwayExplorer::detectPassages(
      episode.observation.laser, passage_clearance_m_);
  if (passages.empty()) {
    return;
  }
  const auto point = episode.observation.pose.position;
  if (model_.nodes.empty() ||
      domain::distance(model_.nodes.back(), point).meters() >=
          minimum_node_spacing_m_) {
    const std::size_t node = model_.nodes.size();
    model_.nodes.push_back(point);
    if (node > 0U) model_.edges.push_back({node - 1U, node});
    const auto labelPoint = [this](const domain::Point2D& sample) {
      if (sample.x_m < 0.0 || sample.y_m < 0.0) return;
      const auto column = static_cast<std::size_t>(std::floor(sample.x_m));
      const auto row = static_cast<std::size_t>(std::floor(sample.y_m));
      const auto duplicate = std::find_if(
          model_.grid_labels.begin(), model_.grid_labels.end(),
          [row, column](const HighwayGridLabel& label) {
            return label.row == row && label.column == column;
          });
      if (duplicate == model_.grid_labels.end())
        model_.grid_labels.push_back({row, column, 1U});
      if (std::find(model_.touched_rows.begin(), model_.touched_rows.end(),
                    row) == model_.touched_rows.end())
        model_.touched_rows.push_back(row);
      if (std::find(model_.touched_columns.begin(),
                    model_.touched_columns.end(),
                    column) == model_.touched_columns.end())
        model_.touched_columns.push_back(column);
    };
    if (node == 0U) {
      labelPoint(point);
    } else {
      const auto& start = model_.nodes[node - 1U];
      const double length = domain::distance(start, point).meters();
      const auto samples =
          std::max<std::size_t>(1U, static_cast<std::size_t>(
                                       std::ceil(length / 0.5)));
      for (std::size_t sample = 0U; sample <= samples; ++sample) {
        const double fraction =
            static_cast<double>(sample) / static_cast<double>(samples);
        labelPoint({start.x_m + fraction * (point.x_m - start.x_m),
                    start.y_m + fraction * (point.y_m - start.y_m)});
      }
    }
    if (passages.size() >= 3U && node > 0U) {
      // Preserve a deterministic branch observation without inventing metric
      // geometry: the current node is connected to the nearest earlier node
      // not already used by the trail edge.
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
      if (node > 1U && branch != node - 1U)
        model_.edges.push_back({branch, node});
    }
  }
}

void HighwayLearner::onRebuild() {
  smoothTouchedGrid();
  extractHighways();
  rebuildIntersections();
  publish(model_, model_.nodes.size() >= 2U ? ModelStatus::Fresh
                                            : ModelStatus::Incomplete,
          "highway graph and touched grid rows/columns published");
}

}  // namespace semaforr::spatial
