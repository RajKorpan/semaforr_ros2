#include <algorithm>
#include <semaforr/exploration/highway_explorer.hpp>
#include <semaforr/spatial/highway_learner.hpp>
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

void HighwayLearner::onObserve(const NavigationEpisode& episode) {
  if (!episode.initial_exploration) return;
  const auto passages = exploration::HighwayExplorer::detectPassages(
      episode.observation.laser, passage_clearance_m_);
  if (passages.empty()) {
    markIncomplete("no traversable passage observed");
    return;
  }
  const auto point = episode.observation.pose.position;
  if (model_.nodes.empty() ||
      domain::distance(model_.nodes.back(), point).meters() >=
          minimum_node_spacing_m_) {
    const std::size_t node = model_.nodes.size();
    model_.nodes.push_back(point);
    if (node > 0U) model_.edges.push_back({node - 1U, node});
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
  rebuildIntersections();
  publish(model_, model_.nodes.size() >= 2U ? ModelStatus::Fresh
                                            : ModelStatus::Incomplete,
          "incremental highway graph");
}

void HighwayLearner::onRebuild() {
  rebuildIntersections();
  publish(model_, model_.nodes.size() >= 2U ? ModelStatus::Fresh
                                            : ModelStatus::Incomplete,
          "highway graph rebuilt");
}

}  // namespace semaforr::spatial
