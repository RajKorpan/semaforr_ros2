#ifndef SEMAFORR_VALIDATION_REPLAY_HPP
#define SEMAFORR_VALIDATION_REPLAY_HPP

#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/domain/model_revision.hpp>
#include <semaforr/domain/observation.hpp>
#include <string>
#include <vector>

namespace semaforr::validation {

struct RandomSeeds {
  std::uint32_t tier_three_ties{0U};
  std::uint32_t lle_fallback{0U};
  std::uint32_t planner_ties{0U};
  std::uint32_t clustering{0U};
  std::uint32_t simulation_noise{0U};

  bool operator==(const RandomSeeds&) const = default;
};

struct RunMetadata {
  std::uint32_t schema_version{1U};
  std::string configuration_snapshot;
  std::string configuration_fingerprint;
  std::string behavior_mode;
  std::string profile;
  std::string map_checksum;
  std::string source_revision;
  std::string test_suite_revision;
  std::vector<std::string> model_versions;
  std::vector<std::string> component_manifest;
  std::vector<domain::Point2D> task_sequence;
  std::vector<std::string> compatibility_deviations;
  RandomSeeds seeds;
};

struct ReplayDecision {
  domain::DecisionId decision_id{0U};
  domain::ActionId action_id{0U};
  domain::Action action{domain::Action::pause()};
  std::string tier;
  std::string source;
  std::string selected_policy;
  std::optional<std::string> planner;
  domain::Revision plan_revision{0U};
  domain::DependencyRevisions spatial_revisions;
  std::string advisor_scores_digest;
  std::string plan_digest;
  std::string explanation_digest;

  bool operator==(const ReplayDecision&) const = default;
};

struct ReplayCycle {
  std::uint64_t sensor_timestamp_ns{0U};
  domain::RobotObservation observation;
  ReplayDecision expected;
  std::optional<domain::ActionExecutionResult> controller_outcome;
};

struct RunTrace {
  RunMetadata metadata;
  std::vector<ReplayCycle> cycles;
};

struct ReplayDifference {
  std::size_t cycle{0U};
  std::string field;
  std::string expected;
  std::string actual;
};

struct ReplayReport {
  bool reproduced{false};
  std::vector<ReplayDifference> differences;
};

ReplayDecision replayDecision(const decision::DecisionResult& result,
                              const domain::DependencyRevisions& revisions);

class RunRecorder {
 public:
  RunRecorder(RunMetadata metadata, std::filesystem::path trace_path = {});
  void recordObservation(
      const domain::RobotObservation& observation,
      std::optional<std::uint64_t> sensor_timestamp_ns = std::nullopt);
  void recordDecision(const decision::DecisionResult& decision,
                      const domain::DependencyRevisions& revisions);
  void recordControllerOutcome(const domain::ActionExecutionResult& outcome);
  const RunTrace& trace() const noexcept { return trace_; }
  void flush() const;

  static void save(const RunTrace& trace, const std::filesystem::path& path);
  static RunTrace load(const std::filesystem::path& path);

 private:
  RunTrace trace_;
  std::filesystem::path trace_path_;
  std::optional<domain::RobotObservation> pending_observation_;
  std::optional<std::uint64_t> pending_sensor_timestamp_ns_;
};

class OfflineReplay {
 public:
  using DecisionFunction = std::function<ReplayDecision(
      const domain::RobotObservation&,
      const std::optional<domain::ActionExecutionResult>&)>;

  static ReplayReport run(const RunTrace& trace,
                          const RunMetadata& active_metadata,
                          const DecisionFunction& decide);
};

}  // namespace semaforr::validation

#endif
