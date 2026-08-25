/**
 * @file replay.cpp
 * @brief Replay responsibilities.
 *
 * @details This file implements replay behavior for replay, experimental
 * validation, and performance measurement. It records the declarations,
 * settings, fixtures, or guidance needed by that responsibility. Its
 * package-relative location is `src/validation/replay.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <semaforr/validation/replay.hpp>
#include <sstream>
#include <stdexcept>

namespace semaforr::validation {
namespace {

/**
 * @brief Performs the timestamp operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `std::uint64_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::uint64_t timestamp(const domain::ExecutionTimestamp value) {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          value.time_since_epoch()).count());
}

/**
 * @brief Performs the timepoint operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `domain::ExecutionTimestamp` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
domain::ExecutionTimestamp timepoint(std::uint64_t value) {
  return domain::ExecutionTimestamp(std::chrono::nanoseconds(value));
}

/**
 * @brief Performs the digest operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string digest(std::string_view value) {
  std::uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char byte : value) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  std::ostringstream output;
  output << std::hex << std::setw(16) << std::setfill('0') << hash;
  return output.str();
}

/**
 * @brief Performs the encode range operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string encodeRange(double value) {
  if (std::isnan(value)) return "nan";
  if (value == std::numeric_limits<double>::infinity()) return "+inf";
  if (value == -std::numeric_limits<double>::infinity()) return "-inf";
  std::ostringstream output;
  output << std::setprecision(17) << value;
  return output.str();
}

/**
 * @brief Performs the decode range operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double decodeRange(const std::string& value) {
  if (value == "nan") return std::numeric_limits<double>::quiet_NaN();
  if (value == "+inf") return std::numeric_limits<double>::infinity();
  if (value == "-inf") return -std::numeric_limits<double>::infinity();
  std::size_t consumed{};
  const double decoded = std::stod(value, &consumed);
  if (consumed != value.size())
    throw std::runtime_error("replay trace: malformed laser range");
  return decoded;
}

/**
 * @brief Performs the advisor digest operation for this subsystem.
 *
 * Arguments:
 * - @p result: Supplies result input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string advisorDigest(const decision::DecisionResult& result) {
  std::ostringstream value;
  value << std::setprecision(17);
  for (const auto& contribution : result.contributions)
    value << contribution.advisor << ':' << static_cast<int>(contribution.action.type())
          << ':' << contribution.action.magnitude_index() << ':'
          << contribution.raw_score << ':' << contribution.normalized_score
          << ':' << contribution.advisor_mean << ':'
          << contribution.advisor_standard_deviation << ':'
          << contribution.relative_support << ':' << contribution.weight << ':'
          << contribution.weighted_score << ':' << contribution.final_total
          << ';';
  for (const auto& total : result.tier_three_totals)
    value << "total:" << static_cast<int>(total.action.type()) << ':'
          << total.action.magnitude_index() << ':' << total.total << ':'
          << total.chapter_five_comment_total << ';';
  const auto& confidence = result.decision_confidence;
  value << "confidence:" << confidence.selected_comment_sum << ':'
        << confidence.advisor_count << ':'
        << confidence.normalized_support_proportion << ':'
        << confidence.action_total_mean << ':'
        << confidence.action_total_standard_deviation << ':'
        << confidence.gamma << ':' << confidence.zeta << ':'
        << confidence.lambda << ':' << confidence.agreement_category << ':'
        << confidence.support_category << ':' << confidence.category;
  return digest(value.str());
}

/**
 * @brief Constructs digest for this subsystem.
 *
 * Arguments:
 * - @p result: Supplies result input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string planDigest(const decision::DecisionResult& result) {
  std::ostringstream value;
  value << std::setprecision(17);
  for (const auto& candidate : result.planning_candidates) {
    value << candidate.plan_id << ':' << candidate.planner << ':'
          << candidate.summed_score << ':' << candidate.tied_for_best << ':';
    for (const auto& point : candidate.geometry)
      value << point.x_m << ',' << point.y_m << ';';
  }
  for (const auto& candidate : result.planning_tie_candidates)
    value << "tie:" << candidate << ';';
  value << result.planning_tie_break_reason;
  return digest(value.str());
}

/**
 * @brief Performs the explanation digest operation for this subsystem.
 *
 * Arguments:
 * - @p result: Supplies result input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string explanationDigest(const decision::DecisionResult& result) {
  std::ostringstream value;
  for (const auto& event : result.decision_cycle)
    value << event.order << ':' << event.tier << ':' << event.component << ':'
          << event.outcome << ':' << event.reason_code << ';';
  for (const auto& source : result.source_provenance) value << source << ';';
  for (const auto& event : result.plan_execution_events) value << event << ';';
  value << result.circumstance_reason << ':' << result.enforcer_reason;
  return digest(value.str());
}

/**
 * @brief Writes strings for this subsystem.
 *
 * Arguments:
 * - @p output: Supplies output input to the operation.
 * - @p tag: Supplies tag input to the operation.
 * - @p values: Supplies values input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void writeStrings(std::ostream& output, std::string_view tag,
                  const std::vector<std::string>& values) {
  for (const auto& value : values)
    output << tag << ' ' << std::quoted(value) << '\n';
}

/**
 * @brief Performs the require operation for this subsystem.
 *
 * Arguments:
 * - @p condition: Supplies condition input to the operation.
 * - @p message: Supplies message input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void require(bool condition, std::string_view message) {
  if (!condition) throw std::runtime_error("replay trace: " + std::string(message));
}

/**
 * @brief Performs the compare operation for this subsystem.
 *
 * Arguments:
 * - @p differences: Supplies differences input to the operation.
 * - @p cycle: Supplies cycle input to the operation.
 * - @p field: Supplies field input to the operation.
 * - @p expected: Supplies expected input to the operation.
 * - @p actual: Supplies actual input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void compare(std::vector<ReplayDifference>& differences, std::size_t cycle,
             std::string field, const std::string& expected,
             const std::string& actual) {
  if (expected != actual)
    differences.push_back({cycle, std::move(field), expected, actual});
}

}  // namespace

/**
 * @brief Performs the replay decision operation for this subsystem.
 *
 * Arguments:
 * - @p result: Supplies result input to the operation.
 * - @p revisions: Supplies revisions input to the operation.
 *
 * Returns:
 * - `ReplayDecision` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
ReplayDecision replayDecision(const decision::DecisionResult& result,
                              const domain::DependencyRevisions& revisions) {
  return {result.decision_id,
          result.action_id,
          result.action,
          std::string(decision::toString(result.tier)),
          std::string(decision::toString(result.source)),
          result.selected_policy,
          result.planner,
          result.plan_revision,
          revisions,
          advisorDigest(result),
          planDigest(result),
          explanationDigest(result),
          result.social_input_source,
          result.social_prediction_source,
          result.social_input_status,
          result.formation_evidence_available,
          result.formation_evidence_participated};
}

/**
 * @brief Performs the run recorder operation for this subsystem.
 *
 * Arguments:
 * - @p metadata: Supplies metadata input to the operation.
 * - @p trace_path: Supplies trace path input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
RunRecorder::RunRecorder(RunMetadata metadata,
                         std::filesystem::path trace_path)
    : trace_{std::move(metadata), {}}, trace_path_(std::move(trace_path)) {
  require(trace_.metadata.schema_version == 1U ||
              trace_.metadata.schema_version == 2U,
          "unsupported schema version");
  require(!trace_.metadata.configuration_snapshot.empty(),
          "configuration snapshot is required");
  require(!trace_.metadata.configuration_fingerprint.empty(),
          "configuration fingerprint is required");
}

/**
 * @brief Records observation for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 * - @p sensor_timestamp_ns: Supplies sensor timestamp ns input to the
 * operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void RunRecorder::recordObservation(
    const domain::RobotObservation& observation,
    std::optional<std::uint64_t> sensor_timestamp_ns) {
  observation.laser.validate();
  pending_observation_ = observation;
  pending_sensor_timestamp_ns_ =
      sensor_timestamp_ns.value_or(timestamp(observation.observed_at));
}

/**
 * @brief Records decision for this subsystem.
 *
 * Arguments:
 * - @p decision: Supplies decision input to the operation.
 * - @p revisions: Supplies revisions input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void RunRecorder::recordDecision(
    const decision::DecisionResult& decision,
    const domain::DependencyRevisions& revisions) {
  require(pending_observation_.has_value(),
          "decision has no preceding sensor observation");
  ReplayCycle cycle;
  cycle.sensor_timestamp_ns = *pending_sensor_timestamp_ns_;
  cycle.observation = std::move(*pending_observation_);
  cycle.expected = replayDecision(decision, revisions);
  trace_.cycles.push_back(std::move(cycle));
  pending_observation_.reset();
  pending_sensor_timestamp_ns_.reset();
  if (!trace_path_.empty()) flush();
}

/**
 * @brief Records controller outcome for this subsystem.
 *
 * Arguments:
 * - @p outcome: Supplies outcome input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void RunRecorder::recordControllerOutcome(
    const domain::ActionExecutionResult& outcome) {
  const auto found = std::find_if(
      trace_.cycles.rbegin(), trace_.cycles.rend(), [&](const auto& cycle) {
        return cycle.expected.action_id == outcome.action_id &&
               cycle.expected.decision_id == outcome.decision_id;
      });
  require(found != trace_.cycles.rend(),
          "controller outcome references an unknown decision/action");
  require(!found->controller_outcome.has_value(),
          "duplicate controller outcome");
  found->controller_outcome = outcome;
  if (!trace_path_.empty()) flush();
}

/**
 * @brief Performs the flush operation for this subsystem.
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
void RunRecorder::flush() const {
  require(!trace_path_.empty(), "trace path is empty");
  save(trace_, trace_path_);
}

/**
 * @brief Serializes package content for this subsystem.
 *
 * Arguments:
 * - @p trace: Supplies trace input to the operation.
 * - @p path: Supplies path input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void RunRecorder::save(const RunTrace& trace,
                       const std::filesystem::path& path) {
  const auto parent = path.parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);
  std::ofstream output(path, std::ios::trunc);
  if (!output) throw std::runtime_error("cannot create replay trace '" + path.string() + "'");
  output << std::setprecision(17);
  const auto& metadata = trace.metadata;
  output << "SEMAFORR_REPLAY " << metadata.schema_version << '\n'
         << "META " << std::quoted(metadata.configuration_snapshot) << ' '
         << std::quoted(metadata.configuration_fingerprint) << ' '
         << std::quoted(metadata.behavior_mode) << ' '
         << std::quoted(metadata.profile) << ' '
         << std::quoted(metadata.map_checksum) << ' '
         << std::quoted(metadata.source_revision) << ' '
         << std::quoted(metadata.test_suite_revision) << '\n'
         << "SEEDS " << metadata.seeds.tier_three_ties << ' '
         << metadata.seeds.lle_fallback << ' ' << metadata.seeds.planner_ties
         << ' ' << metadata.seeds.clustering << ' '
         << metadata.seeds.simulation_noise << '\n';
  writeStrings(output, "MODEL", metadata.model_versions);
  writeStrings(output, "COMPONENT", metadata.component_manifest);
  writeStrings(output, "DEVIATION", metadata.compatibility_deviations);
  for (const auto& task : metadata.task_sequence)
    output << "TASK " << task.x_m << ' ' << task.y_m << '\n';
  for (const auto& cycle : trace.cycles) {
    const auto& observation = cycle.observation;
    output << "CYCLE " << cycle.sensor_timestamp_ns << ' '
           << observation.pose.position.x_m << ' '
           << observation.pose.position.y_m << ' '
           << observation.pose.heading.radians() << ' '
           << observation.laser.angle_min.radians() << ' '
           << observation.laser.angle_increment.radians() << ' '
           << observation.laser.minimum_range.meters() << ' '
           << observation.laser.maximum_range.meters() << ' '
           << observation.laser.ranges_m.size();
    for (const double range : observation.laser.ranges_m)
      output << ' ' << std::quoted(encodeRange(range));
    output << '\n';
    if (observation.crowd) {
      output << "CROWD " << std::quoted(observation.crowd->frame_id) << ' '
             << observation.crowd->observed_at.count() << ' '
             << observation.crowd->data_age.count();
      if (metadata.schema_version >= 2U)
        output << ' ' << std::quoted(observation.crowd->provenance) << ' '
               << observation.crowd->pedestrians.size() << ' '
               << observation.crowd->formations.size() << '\n';
      else
        output << ' ' << observation.crowd->pedestrians.size() << '\n';
      for (const auto& pedestrian : observation.crowd->pedestrians) {
        output << "PEDESTRIAN " << std::quoted(pedestrian.id) << ' '
               << pedestrian.position.x_m << ' ' << pedestrian.position.y_m
               << ' ' << pedestrian.velocity_mps.x_m << ' '
               << pedestrian.velocity_mps.y_m << ' ' << pedestrian.confidence;
        for (const double covariance : pedestrian.position_covariance)
          output << ' ' << covariance;
        output << ' ' << pedestrian.predicted_trajectory.size();
        for (const auto& prediction : pedestrian.predicted_trajectory)
          output << ' ' << prediction.position.x_m << ' '
                 << prediction.position.y_m << ' '
                 << prediction.predicted_at.count();
        if (metadata.schema_version >= 2U)
          output << ' ' << std::quoted(pedestrian.prediction_source) << ' '
                 << pedestrian.formation_index.has_value() << ' '
                 << pedestrian.formation_index.value_or(0U);
        output << '\n';
      }
      if (metadata.schema_version >= 2U) {
        for (const auto& formation : observation.crowd->formations) {
          output << "FORMATION " << std::quoted(formation.formation_type)
                 << ' ' << formation.center.x_m << ' ' << formation.center.y_m
                 << ' ' << formation.confidence << ' '
                 << formation.member_ids.size();
          for (const auto& member : formation.member_ids)
            output << ' ' << std::quoted(member);
          output << '\n';
        }
      }
    } else {
      output << "NO_CROWD\n";
    }
    const auto& expected = cycle.expected;
    output << "DECISION " << expected.decision_id << ' ' << expected.action_id
           << ' ' << static_cast<int>(expected.action.type()) << ' '
           << expected.action.magnitude_index() << ' '
           << std::quoted(expected.tier) << ' ' << std::quoted(expected.source)
           << ' ' << std::quoted(expected.selected_policy) << ' '
           << expected.planner.has_value() << ' '
           << std::quoted(expected.planner.value_or("")) << ' '
           << expected.plan_revision << ' '
           << std::quoted(expected.advisor_scores_digest) << ' '
           << std::quoted(expected.plan_digest) << ' '
           << std::quoted(expected.explanation_digest) << ' '
           << expected.spatial_revisions.size();
    for (const auto& [dependency, revision] : expected.spatial_revisions)
      output << ' ' << static_cast<int>(dependency) << ' ' << revision;
    output << '\n';
    if (metadata.schema_version >= 2U)
      output << "SOCIAL_DECISION "
             << std::quoted(expected.social_input_source) << ' '
             << std::quoted(expected.social_prediction_source) << ' '
             << std::quoted(expected.social_input_status) << ' '
             << expected.formation_evidence_available << ' '
             << expected.formation_evidence_participated << '\n';
    if (cycle.controller_outcome) {
      const auto& outcome = *cycle.controller_outcome;
      output << "OUTCOME " << outcome.decision_id << ' ' << outcome.action_id
             << ' ' << static_cast<int>(outcome.status) << ' '
             << timestamp(outcome.started_at) << ' ' << timestamp(outcome.finished_at)
             << ' ' << outcome.start_pose.position.x_m << ' '
             << outcome.start_pose.position.y_m << ' '
             << outcome.start_pose.heading.radians() << ' '
             << outcome.final_pose.position.x_m << ' '
             << outcome.final_pose.position.y_m << ' '
             << outcome.final_pose.heading.radians() << ' '
             << outcome.distance_achieved_m << ' '
             << outcome.rotation_achieved_rad << ' ' << outcome.timed_out << ' '
             << outcome.safety_interruption << ' ' << outcome.controller_failure
             << ' ' << outcome.collision << ' ' << outcome.near_collision << ' '
             << std::quoted(outcome.cancellation_reason) << '\n';
    } else {
      output << "NO_OUTCOME\n";
    }
    output << "END_CYCLE\n";
  }
}

/**
 * @brief Loads package content for this subsystem.
 *
 * Arguments:
 * - @p path: Supplies path input to the operation.
 *
 * Returns:
 * - `RunTrace` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
RunTrace RunRecorder::load(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open replay trace '" + path.string() + "'");
  RunTrace trace;
  std::string tag;
  input >> tag >> trace.metadata.schema_version;
  require(tag == "SEMAFORR_REPLAY" &&
              (trace.metadata.schema_version == 1U ||
               trace.metadata.schema_version == 2U),
          "unsupported or malformed header");
  input >> tag;
  require(tag == "META", "missing metadata");
  input >> std::quoted(trace.metadata.configuration_snapshot)
        >> std::quoted(trace.metadata.configuration_fingerprint)
        >> std::quoted(trace.metadata.behavior_mode)
        >> std::quoted(trace.metadata.profile)
        >> std::quoted(trace.metadata.map_checksum)
        >> std::quoted(trace.metadata.source_revision)
        >> std::quoted(trace.metadata.test_suite_revision);
  input >> tag;
  require(tag == "SEEDS", "missing random seeds");
  input >> trace.metadata.seeds.tier_three_ties
        >> trace.metadata.seeds.lle_fallback
        >> trace.metadata.seeds.planner_ties
        >> trace.metadata.seeds.clustering
        >> trace.metadata.seeds.simulation_noise;
  while (input >> tag) {
    if (tag == "MODEL" || tag == "COMPONENT" || tag == "DEVIATION") {
      std::string value;
      input >> std::quoted(value);
      auto& destination = tag == "MODEL" ? trace.metadata.model_versions
                          : tag == "COMPONENT" ? trace.metadata.component_manifest
                                               : trace.metadata.compatibility_deviations;
      destination.push_back(std::move(value));
    } else if (tag == "TASK") {
      domain::Point2D task;
      input >> task.x_m >> task.y_m;
      trace.metadata.task_sequence.push_back(task);
    } else if (tag == "CYCLE") {
      ReplayCycle cycle;
      double heading{}, angle_min{}, angle_increment{}, minimum{}, maximum{};
      std::size_t ranges{};
      input >> cycle.sensor_timestamp_ns >> cycle.observation.pose.position.x_m
            >> cycle.observation.pose.position.y_m >> heading >> angle_min
            >> angle_increment >> minimum >> maximum >> ranges;
      cycle.observation.pose.heading = domain::Angle(heading);
      cycle.observation.laser.angle_min = domain::Angle(angle_min);
      cycle.observation.laser.angle_increment = domain::Angle(angle_increment);
      cycle.observation.laser.minimum_range = domain::Distance(minimum);
      cycle.observation.laser.maximum_range = domain::Distance(maximum);
      cycle.observation.laser.ranges_m.resize(ranges);
      for (double& range : cycle.observation.laser.ranges_m) {
        std::string encoded;
        input >> std::quoted(encoded);
        range = decodeRange(encoded);
      }
      cycle.observation.observed_at = timepoint(cycle.sensor_timestamp_ns);
      input >> tag;
      if (tag == "CROWD") {
        domain::CrowdObservation crowd;
        std::int64_t observed{}, age{};
        std::size_t pedestrians{}, formations{};
        input >> std::quoted(crowd.frame_id) >> observed >> age;
        if (trace.metadata.schema_version >= 2U)
          input >> std::quoted(crowd.provenance) >> pedestrians >> formations;
        else
          input >> pedestrians;
        crowd.observed_at = std::chrono::nanoseconds(observed);
        crowd.data_age = std::chrono::nanoseconds(age);
        for (std::size_t i = 0; i < pedestrians; ++i) {
          input >> tag;
          require(tag == "PEDESTRIAN", "missing pedestrian record");
          domain::PedestrianObservation pedestrian;
          std::size_t predictions{};
          input >> std::quoted(pedestrian.id) >> pedestrian.position.x_m
                >> pedestrian.position.y_m >> pedestrian.velocity_mps.x_m
                >> pedestrian.velocity_mps.y_m >> pedestrian.confidence;
          for (double& covariance : pedestrian.position_covariance) input >> covariance;
          input >> predictions;
          for (std::size_t p = 0; p < predictions; ++p) {
            domain::PredictedPosition prediction;
            std::int64_t when{};
            input >> prediction.position.x_m >> prediction.position.y_m >> when;
            prediction.predicted_at = std::chrono::nanoseconds(when);
            pedestrian.predicted_trajectory.push_back(prediction);
          }
          if (trace.metadata.schema_version >= 2U) {
            bool has_formation{};
            std::size_t formation_index{};
            input >> std::quoted(pedestrian.prediction_source) >> has_formation
                  >> formation_index;
            if (has_formation)
              pedestrian.formation_index = formation_index;
          }
          crowd.pedestrians.push_back(std::move(pedestrian));
        }
        for (std::size_t i = 0U; i < formations; ++i) {
          input >> tag;
          require(tag == "FORMATION", "missing formation record");
          domain::FormationObservation formation;
          std::size_t members{};
          input >> std::quoted(formation.formation_type)
                >> formation.center.x_m >> formation.center.y_m
                >> formation.confidence >> members;
          for (std::size_t member = 0U; member < members; ++member) {
            std::string id;
            input >> std::quoted(id);
            formation.member_ids.push_back(std::move(id));
          }
          crowd.formations.push_back(std::move(formation));
        }
        crowd.validate();
        cycle.observation.crowd = std::move(crowd);
      } else {
        require(tag == "NO_CROWD", "malformed crowd record");
      }
      input >> tag;
      require(tag == "DECISION", "missing decision record");
      int action_type{};
      std::size_t magnitude{}, revision_count{};
      bool has_planner{};
      std::string planner;
      input >> cycle.expected.decision_id >> cycle.expected.action_id
            >> action_type >> magnitude >> std::quoted(cycle.expected.tier)
            >> std::quoted(cycle.expected.source)
            >> std::quoted(cycle.expected.selected_policy) >> has_planner
            >> std::quoted(planner) >> cycle.expected.plan_revision
            >> std::quoted(cycle.expected.advisor_scores_digest)
            >> std::quoted(cycle.expected.plan_digest)
            >> std::quoted(cycle.expected.explanation_digest) >> revision_count;
      cycle.expected.action = domain::Action(
          static_cast<domain::ActionType>(action_type), magnitude);
      if (has_planner) cycle.expected.planner = std::move(planner);
      for (std::size_t i = 0; i < revision_count; ++i) {
        int dependency{};
        domain::Revision revision{};
        input >> dependency >> revision;
        cycle.expected.spatial_revisions[
            static_cast<domain::ModelDependency>(dependency)] = revision;
      }
      input >> tag;
      if (trace.metadata.schema_version >= 2U) {
        require(tag == "SOCIAL_DECISION", "missing social decision record");
        input >> std::quoted(cycle.expected.social_input_source)
              >> std::quoted(cycle.expected.social_prediction_source)
              >> std::quoted(cycle.expected.social_input_status)
              >> cycle.expected.formation_evidence_available
              >> cycle.expected.formation_evidence_participated;
        input >> tag;
      }
      if (tag == "OUTCOME") {
        domain::ActionExecutionResult outcome;
        int status{};
        std::uint64_t started{}, finished{};
        double start_heading{}, final_heading{};
        input >> outcome.decision_id >> outcome.action_id >> status >> started
              >> finished >> outcome.start_pose.position.x_m
              >> outcome.start_pose.position.y_m >> start_heading
              >> outcome.final_pose.position.x_m >> outcome.final_pose.position.y_m
              >> final_heading >> outcome.distance_achieved_m
              >> outcome.rotation_achieved_rad >> outcome.timed_out
              >> outcome.safety_interruption >> outcome.controller_failure
              >> outcome.collision >> outcome.near_collision
              >> std::quoted(outcome.cancellation_reason);
        outcome.status = static_cast<domain::ExecutionCompletionStatus>(status);
        outcome.started_at = timepoint(started);
        outcome.finished_at = timepoint(finished);
        outcome.start_pose.heading = domain::Angle(start_heading);
        outcome.final_pose.heading = domain::Angle(final_heading);
        cycle.controller_outcome = std::move(outcome);
      } else {
        require(tag == "NO_OUTCOME", "malformed controller outcome");
      }
      input >> tag;
      require(tag == "END_CYCLE", "missing cycle terminator");
      trace.cycles.push_back(std::move(cycle));
    } else {
      throw std::runtime_error("replay trace: unknown record '" + tag + "'");
    }
  }
  return trace;
}

/**
 * @brief Performs the run operation for this subsystem.
 *
 * Arguments:
 * - @p trace: Supplies trace input to the operation.
 * - @p active_metadata: Supplies active metadata input to the operation.
 * - @p decide: Supplies decide input to the operation.
 *
 * Returns:
 * - `ReplayReport` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
ReplayReport OfflineReplay::run(const RunTrace& trace,
                                const RunMetadata& active_metadata,
                                const DecisionFunction& decide) {
  ReplayReport report;
  compare(report.differences, 0U, "configuration_fingerprint",
          trace.metadata.configuration_fingerprint,
          active_metadata.configuration_fingerprint);
  compare(report.differences, 0U, "configuration_snapshot",
          trace.metadata.configuration_snapshot,
          active_metadata.configuration_snapshot);
  compare(report.differences, 0U, "map_checksum", trace.metadata.map_checksum,
          active_metadata.map_checksum);
  compare(report.differences, 0U, "source_revision",
          trace.metadata.source_revision, active_metadata.source_revision);
  compare(report.differences, 0U, "test_suite_revision",
          trace.metadata.test_suite_revision,
          active_metadata.test_suite_revision);
  compare(report.differences, 0U, "behavior_mode",
          trace.metadata.behavior_mode, active_metadata.behavior_mode);
  compare(report.differences, 0U, "profile", trace.metadata.profile,
          active_metadata.profile);
  if (trace.metadata.model_versions != active_metadata.model_versions)
    report.differences.push_back(
        {0U, "model_versions", "recorded versions", "active versions"});
  if (trace.metadata.component_manifest != active_metadata.component_manifest)
    report.differences.push_back(
        {0U, "component_manifest", "recorded components",
         "active components"});
  if (trace.metadata.task_sequence != active_metadata.task_sequence)
    report.differences.push_back(
        {0U, "task_sequence", "recorded tasks", "active tasks"});
  if (trace.metadata.compatibility_deviations !=
      active_metadata.compatibility_deviations)
    report.differences.push_back(
        {0U, "compatibility_deviations", "recorded deviations",
         "active deviations"});
  compare(report.differences, 0U, "seed.tier_three_ties",
          std::to_string(trace.metadata.seeds.tier_three_ties),
          std::to_string(active_metadata.seeds.tier_three_ties));
  compare(report.differences, 0U, "seed.lle_fallback",
          std::to_string(trace.metadata.seeds.lle_fallback),
          std::to_string(active_metadata.seeds.lle_fallback));
  compare(report.differences, 0U, "seed.planner_ties",
          std::to_string(trace.metadata.seeds.planner_ties),
          std::to_string(active_metadata.seeds.planner_ties));
  compare(report.differences, 0U, "seed.clustering",
          std::to_string(trace.metadata.seeds.clustering),
          std::to_string(active_metadata.seeds.clustering));
  compare(report.differences, 0U, "seed.simulation_noise",
          std::to_string(trace.metadata.seeds.simulation_noise),
          std::to_string(active_metadata.seeds.simulation_noise));
  std::optional<domain::ActionExecutionResult> preceding_outcome;
  for (std::size_t index = 0; index < trace.cycles.size(); ++index) {
    const auto actual =
        decide(trace.cycles[index].observation, preceding_outcome);
    const auto& expected = trace.cycles[index].expected;
    compare(report.differences, index, "decision_id",
            std::to_string(expected.decision_id),
            std::to_string(actual.decision_id));
    compare(report.differences, index, "action_id",
            std::to_string(expected.action_id), std::to_string(actual.action_id));
    compare(report.differences, index, "action",
            std::to_string(static_cast<int>(expected.action.type())) + ":" +
                std::to_string(expected.action.magnitude_index()),
            std::to_string(static_cast<int>(actual.action.type())) + ":" +
                std::to_string(actual.action.magnitude_index()));
    compare(report.differences, index, "tier", expected.tier, actual.tier);
    compare(report.differences, index, "source", expected.source, actual.source);
    compare(report.differences, index, "selected_policy",
            expected.selected_policy, actual.selected_policy);
    compare(report.differences, index, "planner",
            expected.planner.value_or(""), actual.planner.value_or(""));
    compare(report.differences, index, "plan_revision",
            std::to_string(expected.plan_revision),
            std::to_string(actual.plan_revision));
    for (const auto& [dependency, revision] : expected.spatial_revisions) {
      const auto found = actual.spatial_revisions.find(dependency);
      compare(report.differences, index,
              "spatial_revision." + std::string(domain::toString(dependency)),
              std::to_string(revision),
              found == actual.spatial_revisions.end()
                  ? "missing"
                  : std::to_string(found->second));
    }
    for (const auto& [dependency, revision] : actual.spatial_revisions) {
      if (!expected.spatial_revisions.contains(dependency))
        report.differences.push_back(
            {index,
             "spatial_revision." +
                 std::string(domain::toString(dependency)),
             "missing", std::to_string(revision)});
    }
    compare(report.differences, index, "advisor_scores",
            expected.advisor_scores_digest, actual.advisor_scores_digest);
    compare(report.differences, index, "plans", expected.plan_digest,
            actual.plan_digest);
    compare(report.differences, index, "explanations",
            expected.explanation_digest, actual.explanation_digest);
    compare(report.differences, index, "social_input_source",
            expected.social_input_source, actual.social_input_source);
    compare(report.differences, index, "social_prediction_source",
            expected.social_prediction_source,
            actual.social_prediction_source);
    compare(report.differences, index, "social_input_status",
            expected.social_input_status, actual.social_input_status);
    compare(report.differences, index, "formation_evidence_available",
            std::to_string(expected.formation_evidence_available),
            std::to_string(actual.formation_evidence_available));
    compare(report.differences, index, "formation_evidence_participated",
            std::to_string(expected.formation_evidence_participated),
            std::to_string(actual.formation_evidence_participated));
    preceding_outcome = trace.cycles[index].controller_outcome;
  }
  report.reproduced = report.differences.empty();
  return report;
}

}  // namespace semaforr::validation
