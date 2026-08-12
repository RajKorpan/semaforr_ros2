#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
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

domain::CaseOutcome caseOutcome(
    domain::ExecutionCompletionStatus status) noexcept {
  using S = domain::ExecutionCompletionStatus;
  switch (status) {
    case S::Succeeded: return domain::CaseOutcome::Successful;
    case S::PartialMovement: return domain::CaseOutcome::Partial;
    case S::TimedOut: return domain::CaseOutcome::TimedOut;
    case S::SafetyInterrupted: return domain::CaseOutcome::SafetyInterrupted;
    case S::Cancelled:
    case S::NavigationModeTransition:
    case S::SensorLost:
    case S::Shutdown:
      return domain::CaseOutcome::Cancelled;
    case S::GoalPreempted: return domain::CaseOutcome::Preempted;
    case S::NoMovement:
    case S::ControllerRejected:
    case S::ControllerFailure:
      return domain::CaseOutcome::Failed;
    case S::ClockReset:
    case S::OdometryReset:
      return domain::CaseOutcome::Unknown;
  }
  return domain::CaseOutcome::Unknown;
}

void recomputeCase(domain::CircumstanceCaseEvidence& evidence) {
  evidence.evidence = 0U;
  evidence.confidence.clear();
  for (auto& action : evidence.actions) {
    evidence.evidence += action.executed;
    action.accuracy = action.effective_evidence <= 0.0
                          ? 0.0
                          : action.success_credit / action.effective_evidence;
    // Laplace-smoothed historical success probability. Evidence sufficiency
    // remains a separate gate and is never hidden inside this value.
    action.confidence = (1.0 + action.success_credit) /
                        (2.0 + action.effective_evidence);
    evidence.confidence[action.action] = action.confidence;
  }
  // Case accuracy expresses whether this case contains a reliably successful
  // precedent to contrast against weak actions; it is therefore the strongest
  // action reliability, not the average over good and poor choices.
  evidence.accuracy = 0.0;
  for (const auto& action : evidence.actions)
    evidence.accuracy = std::max(evidence.accuracy, action.accuracy);
}

}  // namespace

void CircumstanceLearningConfiguration::validate() const {
  const bool finite = std::isfinite(setting_resolution_m) &&
                      std::isfinite(setting_radius_m) &&
                      std::isfinite(assignment_confidence_threshold) &&
                      std::isfinite(similarity_l1_threshold) &&
                      std::isfinite(accuracy_threshold) &&
                      std::isfinite(action_confidence_threshold) &&
                      std::isfinite(distance_bin_base_m) &&
                      std::isfinite(partial_success_credit);
  if (!finite || setting_resolution_m <= 0.0 || setting_radius_m <= 0.0 ||
      minimum_cluster_size == 0U || reclustering_threshold == 0U ||
      minimum_case_evidence == 0U || minimum_action_evidence == 0U ||
      similarity_l1_threshold <= 0.0 ||
      distance_bin_base_m <= 0.0 || angle_bin_count == 0U ||
      assignment_confidence_threshold < 0.0 ||
      assignment_confidence_threshold > 1.0 || accuracy_threshold < 0.0 ||
      accuracy_threshold > 1.0 || action_confidence_threshold < 0.0 ||
      action_confidence_threshold > 1.0 || partial_success_credit < 0.0 ||
      partial_success_credit > 1.0 || model_version.empty() ||
      feature_version.empty() ||
      (persistence_policy != "session_only" &&
       persistence_policy != "load_save" &&
       persistence_policy != "load_only" &&
       persistence_policy != "save_only") ||
      (persistence_policy != "session_only" && model_path.empty()))
    throw std::invalid_argument(
        "circumstance learning thresholds are outside their valid ranges");
}

CircumstanceLearner::CircumstanceLearner(
    CircumstanceLearningConfiguration configuration)
    : SpatialLearnerBase(
          SpatialRepresentation::Circumstances, "circumstance",
          UpdateMode::Incremental,
          {true, true, true, true,
           "normalize every view; retain selection context; learn only from "
           "terminal execution outcomes",
           {"Precedent", "TierThreeCircumstanceWeighting"},
           UpdateSchedule::AfterAnyTerminalActionResult}),
      configuration_(std::move(configuration)) {
  configuration_.validate();
  model_.minimum_cluster_size = configuration_.minimum_cluster_size;
  model_.minimum_case_evidence = configuration_.minimum_case_evidence;
  model_.learning_mode = configuration_.mode;
  model_.model_version = configuration_.model_version;
  model_.classifier_version =
      configuration_.mode == domain::CircumstanceLearningMode::DissertationCompatible
          ? configuration_.classifier_version
          : "not_applicable";
  model_.feature_version = configuration_.feature_version;
  model_.similarity_metric = "normalized_l1";
  model_.reclustering_policy =
      configuration_.mode == domain::CircumstanceLearningMode::DissertationCompatible
          ? "similarity_graph_components_then_softmax_assignment"
          : "threshold_batch_then_normalized_l1_assignment";
  model_.assignment_confidence_threshold =
      configuration_.assignment_confidence_threshold;
  model_.similarity_l1_threshold = configuration_.similarity_l1_threshold;
  model_.accuracy_threshold = configuration_.accuracy_threshold;
  model_.action_confidence_threshold =
      configuration_.action_confidence_threshold;
  model_.distance_bin_base_m = configuration_.distance_bin_base_m;
  model_.angle_bin_count = configuration_.angle_bin_count;
  const bool should_load = configuration_.persistence_policy == "load_only" ||
                           configuration_.persistence_policy == "load_save";
  if (should_load && std::filesystem::exists(configuration_.model_path)) {
    std::ifstream input(configuration_.model_path);
    if (!input)
      throw std::runtime_error("cannot open configured circumstance model '" +
                               configuration_.model_path + "'");
    model_ = domain::loadCircumstanceModel(
        input, configuration_.model_version, configuration_.feature_version,
        configuration_.classifier_version);
    for (auto& cluster : model_.clusters)
      cluster.creation_method = domain::CircumstanceCreationMethod::LoadedModel;
  } else if (configuration_.persistence_policy == "load_only") {
    throw std::runtime_error("configured circumstance model does not exist: " +
                             configuration_.model_path);
  }
}

void CircumstanceLearner::updateClusters(
    const domain::NormalizedSetting& setting) {
  ++model_.metrics.observations;
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
    cluster.last_update_sequence = model_.metrics.observations;
    ++cluster.revision;
    const auto classified = classify(setting);
    if (classified) {
      ++model_.metrics.assignments;
      model_.metrics.assignment_confidence_sum += classified->confidence;
    }
  } else {
    ++model_.metrics.unmatched;
    unclustered_.push_back(setting);
    if (unclustered_.size() >= configuration_.reclustering_threshold)
      recluster();
  }
  model_.unclustered_settings = unclustered_.size();
}

void CircumstanceLearner::recluster() {
  ++model_.metrics.reclusterings;
  std::vector<bool> assigned(unclustered_.size(), false);
  std::vector<domain::NormalizedSetting> remainder;
  for (std::size_t seed = 0U; seed < unclustered_.size(); ++seed) {
    if (assigned[seed]) continue;
    // Connected components of the thresholded similarity graph are the
    // zero-eigenvalue spectral components. This gives compatibility mode its
    // offline graph-clustering lifecycle without introducing an opaque ML
    // runtime; adapted mode deliberately uses the same deterministic batch.
    std::vector<std::size_t> group{seed};
    assigned[seed] = true;
    for (std::size_t cursor = 0U; cursor < group.size(); ++cursor) {
      const auto member = group[cursor];
      for (std::size_t candidate = 0U; candidate < unclustered_.size();
           ++candidate) {
        if (!assigned[candidate] &&
            domain::settingL1Distance(unclustered_[member],
                                      unclustered_[candidate]) <
                configuration_.similarity_l1_threshold) {
          assigned[candidate] = true;
          group.push_back(candidate);
        }
      }
    }
    if (group.size() < configuration_.minimum_cluster_size) {
      for (const auto member : group)
        remainder.push_back(std::move(unclustered_[member]));
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
      for (const auto member : group)
        remainder.push_back(std::move(unclustered_[member]));
      continue;
    }
    const domain::CircumstanceId id = model_.next_circumstance_id++;
    const auto method =
        configuration_.mode ==
                domain::CircumstanceLearningMode::DissertationCompatible &&
            model_.clusters.empty()
            ? domain::CircumstanceCreationMethod::OfflineSimilarityGraph
            : domain::CircumstanceCreationMethod::OnlineReclustering;
    model_.clusters.push_back({id, std::move(centroid), accepted.size(), 1.0,
                               method, 1U, model_.metrics.observations, 1U,
                               false});
    for (const auto member : group)
      if (std::find(accepted.begin(), accepted.end(), member) ==
          accepted.end())
        remainder.push_back(std::move(unclustered_[member]));
  }
  unclustered_ = std::move(remainder);
}

void CircumstanceLearner::onObserve(const NavigationEpisode& episode) {
  if (episode.event == LearningEvent::ActionTerminal) {
    recordTerminal(episode);
    return;
  }
  const auto setting = domain::normalizeSetting(
      episode.observation.laser, settingConfiguration(configuration_));
  if (episode.event == LearningEvent::SensorObservation) {
    updateClusters(setting);
    return;
  }
  if (episode.event == LearningEvent::DecisionSelected)
    recordDecision(episode, setting);
}

std::optional<domain::CircumstanceMatch> CircumstanceLearner::classify(
    const domain::NormalizedSetting& setting) const {
  return domain::matchCircumstance(model_, setting);
}

void CircumstanceLearner::recordDecision(
    const NavigationEpisode& episode,
    const domain::NormalizedSetting& setting) {
  if (!episode.selection || !episode.active_target) return;
  if (std::any_of(pending_experiences_.begin(), pending_experiences_.end(),
                  [&](const auto& pending) {
                    return pending.action_id == episode.selection->action_id;
                  }))
    return;
  const auto match = classify(setting);
  PendingExperience pending;
  pending.decision_id = episode.selection->decision_id;
  pending.action_id = episode.selection->action_id;
  pending.task_id = episode.selection->task_id;
  pending.circumstance_id = match ? std::optional(match->id) : std::nullopt;
  pending.circumstance_revision = model_.revision;
  pending.assignment_confidence = match ? match->confidence : 0.0;
  pending.setting = setting;
  pending.starting_pose = episode.selection->expected_start;
  pending.target = episode.active_target;
  pending.action = episode.selection->action;
  pending_experiences_.push_back(std::move(pending));
}

void CircumstanceLearner::recordTerminal(const NavigationEpisode& episode) {
  if (!episode.execution_result) return;
  const auto& result = *episode.execution_result;
  if (std::any_of(resolved_outcomes_.begin(), resolved_outcomes_.end(),
                  [&](const auto& resolved) {
                    return resolved.first == result.action_id;
                  }))
    return;
  auto pending = std::find_if(
      pending_experiences_.begin(), pending_experiences_.end(),
      [&](const auto& item) {
        return item.action_id == result.action_id &&
               item.decision_id == result.decision_id;
      });
  if (pending == pending_experiences_.end()) {
    // A terminal episode still contains the immutable selection-time
    // observation. Recover context without fabricating a successful result.
    if (!episode.selection || !episode.active_target) return;
    const auto setting = domain::normalizeSetting(
        episode.observation.laser, settingConfiguration(configuration_));
    recordDecision(episode, setting);
    pending = std::find_if(pending_experiences_.begin(),
                           pending_experiences_.end(), [&](const auto& item) {
                             return item.action_id == result.action_id;
                           });
    if (pending == pending_experiences_.end()) return;
  }
  if (pending->task_id != result.task_id) return;
  pending->result = result;
  if (updateCase(*pending, result)) {
    pending->terminal = true;
    resolved_outcomes_.emplace_back(result.action_id,
                                    caseOutcome(result.status));
    if (resolved_outcomes_.size() > 4096U)
      resolved_outcomes_.erase(resolved_outcomes_.begin(),
                               resolved_outcomes_.begin() + 1024);
  }
}

bool CircumstanceLearner::updateCase(
    PendingExperience& pending,
    const domain::ActionExecutionResult& result) {
  std::optional<domain::CircumstanceMatch> match;
  if (pending.circumstance_id) {
    domain::CircumstanceId resolved = *pending.circumstance_id;
    if (const auto migration = std::find_if(
            model_.migrations.begin(), model_.migrations.end(),
            [&](const auto& item) { return item.previous_id == resolved; });
        migration != model_.migrations.end())
      resolved = migration->new_id;
    const auto cluster = std::find_if(
        model_.clusters.begin(), model_.clusters.end(), [&](const auto& item) {
          return item.id == resolved && !item.retired;
        });
    if (cluster != model_.clusters.end())
      match = domain::CircumstanceMatch{
          resolved, domain::settingL1Distance(pending.setting,
                                              cluster->centroid),
          pending.assignment_confidence,
          model_.learning_mode ==
                  domain::CircumstanceLearningMode::DissertationCompatible
              ? "centroid_softmax_probability"
              : "normalized_centroid_similarity"};
  }
  if (!match) match = classify(pending.setting);
  if (!match || !pending.target) return false;
  pending.circumstance_id = match->id;
  pending.assignment_confidence = match->confidence;
  const auto key = domain::circumstanceCaseKey(
      match->id, pending.starting_pose, *pending.target, model_);
  auto evidence = std::find_if(model_.cases.begin(), model_.cases.end(),
                               [&](const auto& item) {
                                 return item.key == key;
                               });
  if (evidence == model_.cases.end()) {
    model_.cases.push_back({});
    evidence = std::prev(model_.cases.end());
    evidence->key = key;
  }
  auto action = std::find_if(evidence->actions.begin(),
                             evidence->actions.end(), [&](const auto& item) {
                               return item.action == pending.action;
                             });
  if (action == evidence->actions.end()) {
    evidence->actions.push_back({});
    action = std::prev(evidence->actions.end());
    action->action = pending.action;
  }
  ++action->selected;
  const domain::CaseOutcome outcome = caseOutcome(result.status);
  action->last_outcome = outcome;
  action->last_update_sequence = result.action_id;
  switch (outcome) {
    case domain::CaseOutcome::Successful:
      ++action->executed;
      ++action->successful;
      action->effective_evidence += 1.0;
      action->success_credit += 1.0;
      break;
    case domain::CaseOutcome::Partial:
      ++action->executed;
      ++action->partial;
      action->effective_evidence += 1.0;
      action->success_credit += configuration_.partial_success_credit;
      break;
    case domain::CaseOutcome::Failed:
      ++action->executed;
      ++action->failed;
      action->effective_evidence += 1.0;
      break;
    case domain::CaseOutcome::TimedOut:
      ++action->executed;
      ++action->timeouts;
      action->effective_evidence += 1.0;
      break;
    case domain::CaseOutcome::SafetyInterrupted:
      ++action->executed;
      ++action->safety_interruptions;
      if (configuration_.safety_interruption_is_negative_evidence)
        action->effective_evidence += 1.0;
      break;
    case domain::CaseOutcome::Cancelled:
      ++action->cancellations;
      break;
    case domain::CaseOutcome::Preempted:
      ++action->preemptions;
      break;
    case domain::CaseOutcome::Unknown:
      ++action->unknown;
      break;
  }
  ++evidence->revision;
  recomputeCase(*evidence);
  return true;
}

void CircumstanceLearner::onRebuild() {
  if (unclustered_.size() >= configuration_.minimum_cluster_size) recluster();
  for (auto& pending : pending_experiences_)
    if (!pending.terminal && pending.result &&
        updateCase(pending, *pending.result)) {
      pending.terminal = true;
      resolved_outcomes_.emplace_back(
          pending.action_id, caseOutcome(pending.result->status));
    }
  pending_experiences_.erase(
      std::remove_if(pending_experiences_.begin(), pending_experiences_.end(),
                     [](const auto& pending) { return pending.terminal; }),
      pending_experiences_.end());
  model_.unclustered_settings = unclustered_.size();
  const bool complete = !model_.clusters.empty();
  publish(model_, complete ? ModelStatus::Fresh : ModelStatus::Incomplete,
          complete ? "circumstances and case evidence updated"
                   : "minimum circumstance evidence has not been met");
  if (configuration_.persistence_policy == "save_only" ||
      configuration_.persistence_policy == "load_save") {
    const std::filesystem::path destination(configuration_.model_path);
    if (destination.has_parent_path())
      std::filesystem::create_directories(destination.parent_path());
    const auto temporary = destination.string() + ".tmp";
    {
      std::ofstream output(temporary, std::ios::trunc);
      if (!output)
        throw std::runtime_error(
            "cannot create configured circumstance model '" + temporary +
            "'");
      domain::saveCircumstanceModel(model_, output);
    }
    std::error_code error;
    std::filesystem::rename(temporary, destination, error);
    if (error) {
      std::filesystem::remove(destination, error);
      error.clear();
      std::filesystem::rename(temporary, destination, error);
    }
    if (error)
      throw std::runtime_error("cannot install circumstance model '" +
                               destination.string() + "': " +
                               error.message());
  }
}

}  // namespace semaforr::spatial
