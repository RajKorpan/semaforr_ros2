#include <gtest/gtest.h>

#include <semaforr/validation/allocation_probe.hpp>
#include <semaforr/validation/experiment_metrics.hpp>

namespace {

semaforr::decision::DecisionResult makeDecision(
    double x_m, semaforr::navigation::NavigationPhase phase,
    std::string policy, double latency_s) {
  semaforr::decision::DecisionResult result;
  result.robot_pose.position = {x_m, 0.0};
  result.navigation_phase = phase;
  result.configuration_fingerprint = "0123456789abcdef";
  result.selected_policy = std::move(policy);
  result.decision_latency_s = latency_s;
  return result;
}

}  // namespace

TEST(ExperimentMetrics, ReportsPerformanceEfficiencyAndInterventions) {
  using namespace semaforr;
  validation::ExperimentMetricsCollector metrics("complex_office", "highway",
                                                  100U);
  metrics.record({makeDecision(
                      0.0, navigation::NavigationPhase::InitialExploration,
                      "hle:candidate", 0.1),
                  1.0, 0.0, 0.25, {3U, 64U}, 20U});
  metrics.record({makeDecision(
                      2.0, navigation::NavigationPhase::InitialExploration,
                      "hle:candidate", 0.2),
                  2.0, 0.0, 0.5, {5U, 96U}, 30U});
  metrics.record({makeDecision(5.0,
                               navigation::NavigationPhase::TargetNavigation,
                               "reactive:LLE", 0.3),
                  3.0, 0.75, 0.1, {7U, 128U}, 40U});
  metrics.recordTargetOutcome(true);
  metrics.recordTargetOutcome(false);

  const auto summary = metrics.summary();
  EXPECT_EQ(summary.targets_attempted, 2U);
  EXPECT_EQ(summary.targets_succeeded, 1U);
  EXPECT_DOUBLE_EQ(summary.success_rate, 0.5);
  EXPECT_DOUBLE_EQ(summary.exploration_distance_m, 2.0);
  EXPECT_DOUBLE_EQ(summary.target_distance_m, 3.0);
  EXPECT_DOUBLE_EQ(summary.total_runtime_s, 6.0);
  EXPECT_DOUBLE_EQ(summary.planning_latency_s, 0.75);
  EXPECT_DOUBLE_EQ(summary.model_update_cost_s, 0.85);
  EXPECT_EQ(summary.allocation_count, 15U);
  EXPECT_EQ(summary.allocation_bytes, 288U);
  EXPECT_DOUBLE_EQ(summary.coverage, 0.4);
  EXPECT_DOUBLE_EQ(summary.intervention_frequency.at("hle:candidate"),
                   2.0 / 3.0);
  EXPECT_DOUBLE_EQ(summary.intervention_frequency.at("reactive:LLE"),
                   1.0 / 3.0);
  EXPECT_NE(metrics.serialize().find("\"allocations\""),
            std::string::npos);
}

TEST(ExperimentMetrics, CoverageUsesUnionOfRegionsAndTrailsAtOneMetre) {
  semaforr::domain::SpatialModel model;
  model.inclusion_grid.columns = 10U;
  model.inclusion_grid.rows = 10U;
  model.inclusion_grid.resolution_m = 1.0;
  model.learned_regions.push_back(
      {{2.5, 2.5}, semaforr::domain::Distance(0.4)});
  model.trails.push_back({{0.1, 0.1}, {1.9, 0.1}});
  EXPECT_EQ(
      semaforr::validation::ExperimentMetricsCollector::coveredCells(model),
      3U);
}

TEST(ExperimentMetrics, RejectsInvalidMeasurements) {
  semaforr::validation::ExperimentMetricsCollector metrics("room", "full",
                                                            10U);
  auto observation = semaforr::validation::ExperimentObservation{};
  observation.runtime_s = -1.0;
  EXPECT_THROW(metrics.record(observation), std::invalid_argument);
}

TEST(ExperimentMetrics, AllocationProbeCountsProcessAllocations) {
  const auto before = semaforr::validation::allocationSnapshot();
  auto* values = new int[128];
  values[0] = 7;
  const auto difference = semaforr::validation::allocationDifference(
      before, semaforr::validation::allocationSnapshot());
  EXPECT_EQ(values[0], 7);
  delete[] values;
  EXPECT_GE(difference.count, 1U);
  EXPECT_GE(difference.bytes, 128U * sizeof(int));
}
