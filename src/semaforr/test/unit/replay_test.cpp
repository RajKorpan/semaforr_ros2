/**
 * @file replay_test.cpp
 * @brief Replay test responsibilities.
 *
 * @details This file exercises replay test behavior for automated verification and
 * regression testing. It centers on
 * `RoundTripCapturesInputsSeedsRevisionsAndControllerOutcome`,
 * `ReproducesAndAttributesConfigurationOrDecisionDifferences`,
 * `RejectsLifecycleRecordsWithoutMatchingInputs`,
 * `FeedsEachControllerOutcomeToTheFollowingDecisionCycle`. Its
 * package-relative location is `test/unit/replay_test.cpp`.
 */
#include <cmath>
#include <filesystem>
#include <gtest/gtest.h>
#include <limits>
#include <semaforr/social/crowd_field_learner.hpp>
#include <semaforr/validation/replay.hpp>

namespace {

/**
 * @brief Performs the metadata operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `semaforr::validation::RunMetadata` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::validation::RunMetadata metadata() {
  semaforr::validation::RunMetadata value;
  value.configuration_snapshot = "mode=modernized;profile=custom";
  value.configuration_fingerprint = "0123456789abcdef";
  value.behavior_mode = "modernized";
  value.profile = "custom";
  value.map_checksum = "mapless";
  value.source_revision = "source-1";
  value.test_suite_revision = "tests-1";
  value.model_versions = {"circumstance:v2", "classifier:v1"};
  value.component_manifest = {"tier:tier_one", "tier:tier_three"};
  value.task_sequence = {{2.0, 3.0}, {-1.0, 4.0}};
  value.compatibility_deviations = {"behavior_mode:modernized"};
  value.seeds = {11U, 12U, 13U, 14U, 15U};
  return value;
}

/**
 * @brief Performs the observation operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `semaforr::domain::RobotObservation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::RobotObservation observation() {
  semaforr::domain::RobotObservation value;
  value.pose = {{-2.5, 4.0}, semaforr::domain::Angle(0.25)};
  value.laser.angle_min = semaforr::domain::Angle(-1.0);
  value.laser.angle_increment = semaforr::domain::Angle(0.5);
  value.laser.minimum_range = semaforr::domain::Distance(0.1);
  value.laser.maximum_range = semaforr::domain::Distance(8.0);
  value.laser.ranges_m = {
      1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), 8.0};
  value.observed_at = semaforr::domain::ExecutionTimestamp(
      std::chrono::nanoseconds(1234567));
  semaforr::domain::CrowdObservation crowd;
  crowd.frame_id = "map";
  crowd.observed_at = std::chrono::nanoseconds(1200000);
  crowd.data_age = std::chrono::nanoseconds(34567);
  crowd.provenance = "social_context_tracked";
  semaforr::domain::PedestrianObservation pedestrian;
  pedestrian.id = "person-1";
  pedestrian.position = {1.0, 2.0};
  pedestrian.velocity_mps = {0.2, -0.1};
  pedestrian.confidence = 0.9;
  pedestrian.position_covariance = {1.0, 0.0, 0.0, 1.0};
  pedestrian.predicted_trajectory.push_back(
      {{1.2, 1.9}, std::chrono::nanoseconds(1300000)});
  pedestrian.prediction_source = "gst";
  pedestrian.formation_index = 0U;
  crowd.pedestrians.push_back(pedestrian);
  crowd.formations.push_back(
      {{"person-1"}, "side_by_side", {1.0, 2.0}, 0.8});
  value.crowd = crowd;
  return value;
}

/**
 * @brief Performs the decision operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `semaforr::decision::DecisionResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::decision::DecisionResult decision() {
  semaforr::decision::DecisionResult value;
  value.decision_id = 41U;
  value.action_id = 42U;
  value.action = semaforr::domain::Action(
      semaforr::domain::ActionType::Forward, 1U);
  value.tier = semaforr::decision::DecisionTier::TierThree;
  value.source = semaforr::decision::DecisionSource::TierThreeAdvisor;
  value.selected_policy = "advisor_arbitration";
  value.contributions.push_back(
      {"greedy", value.action, 7.0, 7.0, 5.0, 2.0, 1.0, 1.0, 7.0,
       true, 7.0, "target progress", 3U});
  value.decision_cycle.push_back(
      {1U, "tier3", "greedy", {value.action}, std::nullopt, {},
       "advisor_scored_continue", false, std::nullopt, "greedy:scored"});
  value.source_provenance = {"sensor:sensed_occupancy"};
  value.social_input_source = "social_context_tracked";
  value.social_prediction_source = "gst";
  value.social_input_status = "ready";
  value.formation_evidence_available = true;
  return value;
}

}  // namespace

TEST(Replay, RoundTripCapturesInputsSeedsRevisionsAndControllerOutcome) {
  const auto path = std::filesystem::temp_directory_path() /
                    "semaforr-replay-roundtrip.trace";
  semaforr::validation::RunRecorder recorder(metadata());
  recorder.recordObservation(observation(), 7654321U);
  const semaforr::domain::DependencyRevisions revisions{
      {semaforr::domain::ModelDependency::Familiarity, 2U},
      {semaforr::domain::ModelDependency::SensedOccupancy, 4U}};
  recorder.recordDecision(decision(), revisions);
  semaforr::domain::ActionExecutionResult outcome;
  outcome.decision_id = 41U;
  outcome.action_id = 42U;
  outcome.started_at = semaforr::domain::ExecutionTimestamp(
      std::chrono::nanoseconds(2000000));
  outcome.finished_at = semaforr::domain::ExecutionTimestamp(
      std::chrono::nanoseconds(3000000));
  outcome.status = semaforr::domain::ExecutionCompletionStatus::Succeeded;
  outcome.start_pose = observation().pose;
  outcome.final_pose = {{-2.0, 4.0}, semaforr::domain::Angle(0.25)};
  outcome.distance_achieved_m = 0.5;
  outcome.rotation_achieved_rad = 0.25;
  outcome.near_collision = true;
  outcome.cancellation_reason = "actual controller outcome";
  recorder.recordControllerOutcome(outcome);
  semaforr::validation::RunRecorder::save(recorder.trace(), path);

  const auto restored = semaforr::validation::RunRecorder::load(path);
  ASSERT_EQ(restored.cycles.size(), 1U);
  EXPECT_EQ(restored.cycles.front().sensor_timestamp_ns, 7654321U);
  EXPECT_EQ(restored.metadata.seeds, metadata().seeds);
  EXPECT_EQ(restored.metadata.task_sequence, metadata().task_sequence);
  ASSERT_TRUE(restored.cycles.front().observation.crowd.has_value());
  ASSERT_EQ(restored.cycles.front().observation.laser.ranges_m.size(), 4U);
  EXPECT_TRUE(
      std::isnan(restored.cycles.front().observation.laser.ranges_m[1]));
  EXPECT_TRUE(
      std::isinf(restored.cycles.front().observation.laser.ranges_m[2]));
  EXPECT_EQ(restored.cycles.front().observation.crowd->frame_id, "map");
  EXPECT_EQ(restored.cycles.front().observation.crowd->provenance,
            "social_context_tracked");
  ASSERT_EQ(restored.cycles.front().observation.crowd->pedestrians.size(), 1U);
  EXPECT_EQ(restored.cycles.front().observation.crowd->pedestrians.front().id,
            "person-1");
  EXPECT_EQ(restored.cycles.front().observation.crowd->pedestrians.front()
                .predicted_trajectory.size(),
            1U);
  EXPECT_EQ(restored.cycles.front().observation.crowd->pedestrians.front()
                .prediction_source,
            "gst");
  ASSERT_EQ(restored.cycles.front().observation.crowd->formations.size(), 1U);
  EXPECT_EQ(restored.cycles.front().expected.social_input_source,
            "social_context_tracked");
  EXPECT_EQ(restored.cycles.front().expected.social_prediction_source, "gst");
  EXPECT_EQ(restored.cycles.front().expected.spatial_revisions, revisions);
  ASSERT_TRUE(restored.cycles.front().controller_outcome.has_value());
  EXPECT_TRUE(restored.cycles.front().controller_outcome->successful());
  EXPECT_EQ(restored.cycles.front().expected.action, decision().action);
  EXPECT_EQ(restored.cycles.front().controller_outcome->final_pose,
            outcome.final_pose);
  EXPECT_DOUBLE_EQ(
      restored.cycles.front().controller_outcome->distance_achieved_m, 0.5);
  EXPECT_DOUBLE_EQ(
      restored.cycles.front().controller_outcome->rotation_achieved_rad, 0.25);
  EXPECT_TRUE(restored.cycles.front().controller_outcome->near_collision);
  EXPECT_EQ(restored.cycles.front().controller_outcome->cancellation_reason,
            "actual controller outcome");

  semaforr::social::CrowdFieldLearnerConfiguration crowd_configuration;
  crowd_configuration.geometry = {"map", 12.0, 10.0, 1.0, -5.0, -2.0};
  crowd_configuration.minimum_update_period_s = 0.0;
  semaforr::social::CrowdFieldLearner recorded_learner(crowd_configuration);
  semaforr::social::CrowdFieldLearner replayed_learner(crowd_configuration);
  const auto recorded_observation = observation();
  ASSERT_TRUE(recorded_observation.crowd);
  ASSERT_TRUE(recorded_learner.observe(recorded_observation.pose,
                                       recorded_observation.laser,
                                       *recorded_observation.crowd));
  const auto& replayed_observation = restored.cycles.front().observation;
  ASSERT_TRUE(replayed_observation.crowd);
  ASSERT_TRUE(replayed_learner.observe(replayed_observation.pose,
                                       replayed_observation.laser,
                                       *replayed_observation.crowd));
  EXPECT_EQ(replayed_learner.snapshot(), recorded_learner.snapshot());
  std::filesystem::remove(path);
}

TEST(Replay, ReproducesAndAttributesConfigurationOrDecisionDifferences) {
  semaforr::validation::RunRecorder recorder(metadata());
  recorder.recordObservation(observation());
  const semaforr::domain::DependencyRevisions revisions{
      {semaforr::domain::ModelDependency::Regions, 9U}};
  recorder.recordDecision(decision(), revisions);
  const auto expected = recorder.trace().cycles.front().expected;

  auto report = semaforr::validation::OfflineReplay::run(
      recorder.trace(), metadata(),
      [&](const auto&, const auto&) { return expected; });
  EXPECT_TRUE(report.reproduced);

  auto changed_metadata = metadata();
  changed_metadata.source_revision = "source-2";
  changed_metadata.seeds.lle_fallback = 99U;
  report = semaforr::validation::OfflineReplay::run(
      recorder.trace(), changed_metadata, [&](const auto&, const auto&) {
        auto changed = expected;
        changed.advisor_scores_digest = "different";
        return changed;
      });
  EXPECT_FALSE(report.reproduced);
  EXPECT_TRUE(std::any_of(report.differences.begin(), report.differences.end(),
                          [](const auto& difference) {
                            return difference.field == "source_revision";
                          }));
  EXPECT_TRUE(std::any_of(report.differences.begin(), report.differences.end(),
                          [](const auto& difference) {
                            return difference.field == "advisor_scores";
                          }));
  EXPECT_TRUE(std::any_of(report.differences.begin(), report.differences.end(),
                          [](const auto& difference) {
                            return difference.field == "seed.lle_fallback";
                          }));
}

TEST(Replay, RejectsLifecycleRecordsWithoutMatchingInputs) {
  semaforr::validation::RunRecorder recorder(metadata());
  EXPECT_THROW(recorder.recordDecision(decision(), {}), std::runtime_error);
  semaforr::domain::ActionExecutionResult outcome;
  outcome.decision_id = 1U;
  outcome.action_id = 2U;
  EXPECT_THROW(recorder.recordControllerOutcome(outcome), std::runtime_error);
}

TEST(Replay, FeedsEachControllerOutcomeToTheFollowingDecisionCycle) {
  semaforr::validation::RunRecorder recorder(metadata());
  auto first = decision();
  recorder.recordObservation(observation());
  recorder.recordDecision(first, {});
  semaforr::domain::ActionExecutionResult outcome;
  outcome.decision_id = first.decision_id;
  outcome.action_id = first.action_id;
  outcome.status = semaforr::domain::ExecutionCompletionStatus::Succeeded;
  recorder.recordControllerOutcome(outcome);

  auto second = decision();
  second.decision_id = 43U;
  second.action_id = 44U;
  recorder.recordObservation(observation());
  recorder.recordDecision(second, {});

  std::size_t calls{};
  const auto report = semaforr::validation::OfflineReplay::run(
      recorder.trace(), metadata(),
      [&](const auto&, const auto& preceding) {
        if (calls == 0U)
          EXPECT_FALSE(preceding.has_value());
        else {
          EXPECT_TRUE(preceding.has_value());
          if (preceding) {
            EXPECT_EQ(preceding->action_id, first.action_id);
          }
        }
        return recorder.trace().cycles[calls++].expected;
      });
  EXPECT_TRUE(report.reproduced);
  EXPECT_EQ(calls, 2U);
}
