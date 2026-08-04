#include <algorithm>
#include <cmath>
#include <limits>
#include <semaforr/domain/motion_model.hpp>
#include <semaforr/spatial/learners/circumstance_learner.hpp>
#include <stdexcept>

namespace semaforr::spatial {
namespace {

double assignmentConfidence(double distance, std::size_t cells) {
  if (cells == 0U) return 0.0;
  return std::clamp(1.0 - distance / static_cast<double>(cells), 0.0, 1.0);
}

domain::SettingNormalizationConfiguration settingConfiguration(
    const CircumstanceLearningConfiguration& configuration) {
  return {configuration.setting_resolution_m,
          configuration.setting_radius_m,
          configuration.assignment_confidence_threshold,
          configuration.similarity_l1_threshold,
          configuration.distance_bin_base_m,
          configuration.angle_bin_count};
}

std::optional<std::size_t> matchingCluster(
    const CircumstanceModel& model, const domain::NormalizedSetting& setting) {
  const auto match = domain::matchCircumstance(model, setting);
  if (!match) return std::nullopt;
  const auto cluster = std::find_if(
      model.clusters.begin(), model.clusters.end(),
      [&](const auto& item) { return item.id == match->id; });
  return cluster == model.clusters.end()
             ? std::nullopt
             : std::optional<std::size_t>(
                   static_cast<std::size_t>(cluster - model.clusters.begin()));
}

bool sensed(const domain::RobotObservation& observation,
            domain::Point2D point) {
  const auto& laser = observation.laser;
  if (laser.ranges_m.empty() || laser.angle_increment.radians() == 0.0)
    return false;
  const double distance =
      domain::distance(observation.pose.position, point).meters();
  if (distance > laser.maximum_range.meters()) return false;
  const double relative = domain::Angle::normalize(
      std::atan2(point.y_m - observation.pose.position.y_m,
                 point.x_m - observation.pose.position.x_m) -
      observation.pose.heading.radians());
  const long center = std::lround(
      (relative - laser.angle_min.radians()) /
      laser.angle_increment.radians());
  std::size_t clear = 0U;
  for (long offset = -2; offset <= 2; ++offset) {
    const long beam = center + offset;
    if (beam < 0 || beam >= static_cast<long>(laser.ranges_m.size())) continue;
    const double range = laser.ranges_m[static_cast<std::size_t>(beam)];
    if (!std::isfinite(range) || range + domain::geometry_tolerance_m >= distance)
      ++clear;
  }
  return clear >= 3U;
}

std::optional<domain::Action> hypotheticalAction(
    const NavigationEpisode& episode, domain::Point2D marker) {
  if (episode.viable_actions.empty()) return std::nullopt;
  const domain::ActionSpace action_space(episode.move_distances_m,
                                          episode.rotation_angles_rad);
  std::optional<domain::Action> best;
  double best_score = std::numeric_limits<double>::infinity();
  const double desired = std::atan2(
      marker.y_m - episode.observation.pose.position.y_m,
      marker.x_m - episode.observation.pose.position.x_m);
  for (const auto& action : episode.viable_actions) {
    if (!action_space.contains(action)) continue;
    const auto next = domain::expectedPoseAfterAction(
        episode.observation.pose, action, action_space);
    const double score = domain::distance(next.position, marker).meters() +
                         0.25 * std::abs(domain::Angle::normalize(
                                    desired - next.heading.radians()));
    if (score < best_score || (score == best_score && (!best || action < *best))) {
      best = action;
      best_score = score;
    }
  }
  return best;
}

void updateCaseStatistics(domain::CircumstanceCaseEvidence& evidence) {
  std::map<domain::Action, std::size_t> hypothetical_counts;
  double accuracy_sum = 0.0;
  evidence.evidence = 0U;
  for (const auto& pair : evidence.action_pairs) {
    evidence.evidence += pair.occurrences;
    hypothetical_counts[pair.hypothetical] += pair.occurrences;
    const double credit = pair.actual == pair.hypothetical
                              ? 1.0
                              : pair.actual.type() == pair.hypothetical.type()
                                    ? 0.5
                                    : 0.0;
    accuracy_sum += credit * static_cast<double>(pair.occurrences);
  }
  evidence.accuracy = evidence.evidence == 0U
                          ? 0.0
                          : accuracy_sum /
                                static_cast<double>(evidence.evidence);
  evidence.confidence.clear();
  std::size_t maximum = 0U;
  for (const auto& [action, count] : hypothetical_counts)
    maximum = std::max(maximum, count);
  for (const auto& [action, count] : hypothetical_counts)
    evidence.confidence[action] =
        (1.0 + static_cast<double>(count)) /
        (1.0 + static_cast<double>(maximum));
}

}  // namespace

void CircumstanceLearningConfiguration::validate() const {
  const bool finite = std::isfinite(setting_resolution_m) &&
                      std::isfinite(setting_radius_m) &&
                      std::isfinite(assignment_confidence_threshold) &&
                      std::isfinite(similarity_l1_threshold) &&
                      std::isfinite(accuracy_threshold) &&
                      std::isfinite(action_confidence_threshold) &&
                      std::isfinite(distance_bin_base_m);
  if (!finite || setting_resolution_m <= 0.0 || setting_radius_m <= 0.0 ||
      minimum_cluster_size == 0U || reclustering_threshold == 0U ||
      minimum_case_evidence == 0U || similarity_l1_threshold <= 0.0 ||
      distance_bin_base_m <= 0.0 || angle_bin_count == 0U ||
      assignment_confidence_threshold < 0.0 ||
      assignment_confidence_threshold > 1.0 || accuracy_threshold < 0.0 ||
      accuracy_threshold > 1.0 || action_confidence_threshold < 0.0 ||
      action_confidence_threshold > 1.0)
    throw std::invalid_argument(
        "circumstance learning thresholds are outside their valid ranges");
}

CircumstanceLearner::CircumstanceLearner(
    CircumstanceLearningConfiguration configuration)
    : SpatialLearnerBase(
          SpatialRepresentation::Circumstances, "circumstance",
          UpdateMode::Incremental,
          {true, true, true, true,
           "normalize every view; validate case evidence at target boundaries",
           {"Precedent"}, UpdateSchedule::EndOfTarget}),
      configuration_(std::move(configuration)) {
  configuration_.validate();
  model_.minimum_cluster_size = configuration_.minimum_cluster_size;
  model_.minimum_case_evidence = configuration_.minimum_case_evidence;
  model_.assignment_confidence_threshold =
      configuration_.assignment_confidence_threshold;
  model_.similarity_l1_threshold = configuration_.similarity_l1_threshold;
  model_.accuracy_threshold = configuration_.accuracy_threshold;
  model_.action_confidence_threshold =
      configuration_.action_confidence_threshold;
  model_.distance_bin_base_m = configuration_.distance_bin_base_m;
  model_.angle_bin_count = configuration_.angle_bin_count;
}

void CircumstanceLearner::updateClusters(
    const domain::NormalizedSetting& setting) {
  if (const auto match = matchingCluster(model_, setting)) {
    auto& cluster = model_.clusters[*match];
    const double previous = static_cast<double>(cluster.evidence);
    ++cluster.evidence;
    for (std::size_t index = 0U; index < setting.freespace.size(); ++index)
      cluster.centroid.freespace[index] =
          (cluster.centroid.freespace[index] * previous +
           setting.freespace[index]) /
          static_cast<double>(cluster.evidence);
    cluster.assignment_confidence = assignmentConfidence(
        domain::settingL1Distance(setting, cluster.centroid),
        setting.freespace.size());
  } else {
    unclustered_.push_back(setting);
    if (unclustered_.size() >= configuration_.reclustering_threshold)
      recluster();
  }
  model_.unclustered_settings = unclustered_.size();
}

void CircumstanceLearner::recluster() {
  std::vector<bool> assigned(unclustered_.size(), false);
  std::vector<domain::NormalizedSetting> remainder;
  for (std::size_t seed = 0U; seed < unclustered_.size(); ++seed) {
    if (assigned[seed]) continue;
    std::vector<std::size_t> group;
    for (std::size_t candidate = seed; candidate < unclustered_.size();
         ++candidate) {
      if (!assigned[candidate] &&
          domain::settingL1Distance(unclustered_[seed],
                                    unclustered_[candidate]) <
              configuration_.similarity_l1_threshold)
        group.push_back(candidate);
    }
    if (group.size() < configuration_.minimum_cluster_size) {
      remainder.push_back(unclustered_[seed]);
      assigned[seed] = true;
      continue;
    }
    domain::NormalizedSetting centroid = unclustered_[seed];
    std::fill(centroid.freespace.begin(), centroid.freespace.end(), 0.0);
    for (const auto member : group)
      for (std::size_t cell = 0U; cell < centroid.freespace.size(); ++cell)
        centroid.freespace[cell] += unclustered_[member].freespace[cell];
    for (double& cell : centroid.freespace)
      cell /= static_cast<double>(group.size());
    std::vector<std::size_t> accepted;
    for (const auto member : group) {
      const double confidence = assignmentConfidence(
          domain::settingL1Distance(unclustered_[member], centroid),
          centroid.freespace.size());
      if (confidence >= configuration_.assignment_confidence_threshold)
        accepted.push_back(member);
    }
    if (accepted.size() < configuration_.minimum_cluster_size) {
      remainder.push_back(unclustered_[seed]);
      assigned[seed] = true;
      continue;
    }
    const std::size_t id = model_.clusters.empty()
                               ? 0U
                               : model_.clusters.back().id + 1U;
    model_.clusters.push_back(
        {id, std::move(centroid), accepted.size(), 1.0});
    for (const auto member : accepted) assigned[member] = true;
  }
  for (std::size_t index = 0U; index < unclustered_.size(); ++index)
    if (!assigned[index]) remainder.push_back(std::move(unclustered_[index]));
  unclustered_ = std::move(remainder);
}

void CircumstanceLearner::onObserve(const NavigationEpisode& episode) {
  const auto setting = domain::normalizeSetting(
      episode.observation.laser, settingConfiguration(configuration_));
  updateClusters(setting);
  if (episode.selected_action && episode.active_target && episode.active_task)
    pending_experiences_.push_back({setting, episode, false});
}

void CircumstanceLearner::processExperiences() {
  for (std::size_t index = 0U; index < pending_experiences_.size(); ++index) {
    auto& current = pending_experiences_[index];
    if (current.processed) continue;
    const auto cluster = matchingCluster(model_, current.setting);
    if (!cluster) continue;
    std::optional<domain::Point2D> marker;
    double marker_goal_distance = std::numeric_limits<double>::infinity();
    for (std::size_t later = index + 1U;
         later < pending_experiences_.size(); ++later) {
      const auto& candidate = pending_experiences_[later];
      if (candidate.episode.active_task != current.episode.active_task) break;
      const auto point = candidate.episode.observation.pose.position;
      if (!sensed(current.episode.observation, point)) continue;
      const double goal_distance =
          domain::distance(point, *current.episode.active_target).meters();
      if (goal_distance < marker_goal_distance) {
        marker = point;
        marker_goal_distance = goal_distance;
      }
    }
    if (!marker) continue;
    const auto hypothetical = hypotheticalAction(current.episode, *marker);
    if (!hypothetical) continue;
    const auto key = domain::circumstanceCaseKey(
        model_.clusters[*cluster].id, current.episode.observation.pose,
        *current.episode.active_target, model_);
    auto evidence = std::find_if(
        model_.cases.begin(), model_.cases.end(),
        [&](const auto& item) { return item.key == key; });
    if (evidence == model_.cases.end()) {
      model_.cases.push_back({key, {}, 0U, 0.0, {}});
      evidence = std::prev(model_.cases.end());
    }
    auto pair = std::find_if(
        evidence->action_pairs.begin(), evidence->action_pairs.end(),
        [&](const auto& item) {
          return item.actual == *current.episode.selected_action &&
                 item.hypothetical == *hypothetical;
        });
    if (pair == evidence->action_pairs.end())
      evidence->action_pairs.push_back(
          {*current.episode.selected_action, *hypothetical, 1U});
    else
      ++pair->occurrences;
    updateCaseStatistics(*evidence);
    current.processed = true;
  }
}

void CircumstanceLearner::onRebuild() {
  if (unclustered_.size() >= configuration_.minimum_cluster_size) recluster();
  processExperiences();
  model_.unclustered_settings = unclustered_.size();
  const bool complete = !model_.clusters.empty();
  publish(model_, complete ? ModelStatus::Fresh : ModelStatus::Incomplete,
          complete ? "circumstances and case evidence updated"
                   : "minimum circumstance evidence has not been met");
}

}  // namespace semaforr::spatial
