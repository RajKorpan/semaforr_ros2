/**
 * @file performance_regression_test.cpp
 * @brief Performance regression test responsibilities.
 *
 * @details This file exercises performance regression test behavior for automated
 * verification and regression testing. It centers on `ForwardAdvisor`,
 * `MeasuresDecisionLatencyAndAllocations`,
 * `MeasuresLargeSparseGridBehavior`, `MeasuresHighLevelCueProcessing`,
 * `MeasuresHallwayPairProcessing`, `MeasuresSerializationTimeAndSize`,
 * `MeasuresPlanCacheHitRate`. Its package-relative location is
 * `test/performance/performance_regression_test.cpp`.
 */
#include <gtest/gtest.h>

#include <chrono>
#include <iomanip>
#include <memory>
#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/exploration/high_level_explorer.hpp>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/planning_coordinator.hpp>
#include <semaforr/spatial/chapter3_learning.hpp>
#include <semaforr/spatial/spatial_learning_coordinator.hpp>
#include <semaforr/validation/allocation_probe.hpp>
#include <sstream>
#include <span>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

/**
 * @brief Performs the seconds operation for this subsystem.
 *
 * Arguments:
 * - @p duration: Supplies duration input to the operation.
 *
 * Returns:
 * - `std::string` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string seconds(Clock::duration duration) {
  std::ostringstream stream;
  stream << std::setprecision(17)
         << std::chrono::duration<double>(duration).count();
  return stream.str();
}

/**
 * @brief Encapsulates forward advisor state and behavior for this
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
class ForwardAdvisor final : public semaforr::decision::Advisor {
 public:
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string_view name() const noexcept override { return "forward"; }

  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - `semaforr::decision::AdvisorEvaluation` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  semaforr::decision::AdvisorEvaluation evaluate(
      const semaforr::decision::DecisionContext&,
      std::span<const semaforr::domain::Action> candidates) const override {
    semaforr::decision::AdvisorEvaluation result;
    result.participated = true;
    result.explanation = "performance fixture";
    for (const auto& action : candidates) {
      result.scores.push_back(
          {action, action.type() == semaforr::domain::ActionType::Forward
                       ? 8.0
                       : 2.0});
    }
    return result;
  }
};

/**
 * @brief Performs the observation operation for this subsystem.
 *
 * Arguments:
 * - @p x: Supplies x input to the operation.
 * - @p y: Supplies y input to the operation.
 *
 * Returns:
 * - `semaforr::domain::RobotObservation` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::RobotObservation observation(double x = 0.0,
                                                double y = 0.0) {
  semaforr::domain::RobotObservation result;
  result.pose = {{x, y}, semaforr::domain::Angle::zero()};
  result.laser.angle_min = semaforr::domain::Angle(-0.61);
  result.laser.angle_increment = semaforr::domain::Angle(0.01);
  result.laser.minimum_range = semaforr::domain::Distance(0.1);
  result.laser.maximum_range = semaforr::domain::Distance(10.0);
  result.laser.ranges_m.assign(123U, 4.0);
  return result;
}

/**
 * @brief Performs the episode operation for this subsystem.
 *
 * Arguments:
 * - @p sequence: Supplies sequence input to the operation.
 * - @p x: Supplies x input to the operation.
 *
 * Returns:
 * - `semaforr::spatial::NavigationEpisode` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::spatial::NavigationEpisode episode(std::size_t sequence,
                                              double x = 0.0) {
  semaforr::spatial::NavigationEpisode result;
  result.sequence = sequence;
  result.observation = observation(x);
  return result;
}

/**
 * @brief Performs the path point operation for this subsystem.
 *
 * Arguments:
 * - @p id: Supplies id input to the operation.
 * - @p y: Supplies y input to the operation.
 *
 * Returns:
 * - `semaforr::domain::PathDecisionPoint` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::PathDecisionPoint pathPoint(std::uint64_t id, double y) {
  semaforr::domain::PathDecisionPoint point;
  point.selection.decision_id = id;
  point.selection.action_id = id;
  point.selection.action = semaforr::domain::Action(
      semaforr::domain::ActionType::Forward, 1U);
  point.selection.expected_start =
      {{0.0, y}, semaforr::domain::Angle::zero()};
  point.execution.decision_id = id;
  point.execution.action_id = id;
  point.execution.status =
      semaforr::domain::ExecutionCompletionStatus::Succeeded;
  point.execution.start_pose = point.selection.expected_start;
  point.execution.final_pose =
      {{8.0, y}, semaforr::domain::Angle::zero()};
  point.execution.distance_achieved_m = 8.0;
  point.executed_action = point.selection.action;
  point.decision_observation = observation(0.0, y);
  point.decision_observation.laser.angle_min =
      semaforr::domain::Angle(-3.14159265358979323846);
  point.decision_observation.laser.angle_increment =
      semaforr::domain::Angle(3.14159265358979323846 / 4.0);
  point.decision_observation.laser.ranges_m.assign(9U, 10.0);
  return point;
}

/**
 * @brief Performs the hallway evidence operation for this subsystem.
 *
 * Arguments:
 * - @p count: Supplies count input to the operation.
 *
 * Returns:
 * - `std::vector<semaforr::domain::CompletedPath>` containing the operation
 * result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::vector<semaforr::domain::CompletedPath> hallwayEvidence(
    std::size_t count) {
  std::vector<semaforr::domain::CompletedPath> paths;
  paths.reserve(count);
  for (std::size_t index = 0U; index < count; ++index) {
    semaforr::domain::CompletedPath path;
    path.id = index + 1U;
    path.target_reached = true;
    path.decision_points.push_back(
        pathPoint(index + 1U, static_cast<double>(index % 8U) * 0.4));
    paths.push_back(std::move(path));
  }
  return paths;
}

/**
 * @brief Constructs ning map for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `semaforr::domain::StaticMap` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::StaticMap planningMap() {
  semaforr::domain::StaticMap map;
  map.source = "performance-fixture";
  map.bounds = {{0.0, 0.0}, {20.0, 20.0}};
  map.occupancy =
      {20U, 20U, 1.0, {0.0, 0.0},
       std::vector<semaforr::domain::StaticOccupancyState>(
           400U, semaforr::domain::StaticOccupancyState::StaticFree)};
  return map;
}

TEST(PerformanceRegression, MeasuresDecisionLatencyAndAllocations) {
  semaforr::decision::ArbitrationConfiguration configuration;
  configuration.random_seed = 7U;
  semaforr::decision::DecisionCoordinator coordinator(configuration);
  coordinator.addAdvisor(std::make_unique<ForwardAdvisor>());
  semaforr::domain::WorldModel world;
  const std::vector<semaforr::domain::Action> actions{
      semaforr::domain::Action::pause(),
      {semaforr::domain::ActionType::Forward, 1U},
      {semaforr::domain::ActionType::TurnLeft, 1U},
      {semaforr::domain::ActionType::TurnRight, 1U}};

  constexpr std::size_t iterations = 500U;
  const auto allocations_before =
      semaforr::validation::allocationSnapshot();
  const auto started = Clock::now();
  std::size_t forward_decisions = 0U;
  for (std::size_t index = 0U; index < iterations; ++index) {
    if (coordinator.decide({world}, actions).action.type() ==
        semaforr::domain::ActionType::Forward)
      ++forward_decisions;
  }
  const auto elapsed = Clock::now() - started;
  const auto allocations = semaforr::validation::allocationDifference(
      allocations_before, semaforr::validation::allocationSnapshot());
  EXPECT_EQ(forward_decisions, iterations);
  RecordProperty("iterations", static_cast<int>(iterations));
  RecordProperty("total_seconds", seconds(elapsed));
  RecordProperty("mean_decision_seconds",
                 seconds(elapsed / iterations));
  RecordProperty("allocation_count",
                 static_cast<int>(allocations.count));
  RecordProperty("allocation_bytes",
                 std::to_string(allocations.bytes));
}

TEST(PerformanceRegression, MeasuresLargeSparseGridBehavior) {
  semaforr::spatial::LearnedGridConfiguration grid;
  grid.initial_width_m = 2000.0;
  grid.initial_height_m = 2000.0;
  grid.resolution_m = 0.5;
  grid.extent_policy = semaforr::spatial::GridExtentPolicy::Fixed;
  grid.initialize_around_first_pose = false;
  auto learning = semaforr::spatial::SpatialLearningCoordinator::defaults(
      100U, {}, {}, grid);
  const auto started = Clock::now();
  learning.observeSensor(episode(1U));
  semaforr::domain::SpatialModel world;
  learning.applyTo(world);
  const auto elapsed = Clock::now() - started;
  EXPECT_TRUE(world.known_grid.cells.empty());
  EXPECT_LT(world.known_grid.sparseCells().size(), 1000U);
  RecordProperty("represented_dense_cells",
                 std::to_string(world.known_grid.columns *
                                world.known_grid.rows));
  RecordProperty("stored_sparse_cells",
                 static_cast<int>(world.known_grid.sparseCells().size()));
  RecordProperty("observe_and_project_seconds", seconds(elapsed));
  RecordProperty("estimated_projection_bytes",
                 std::to_string(
                     learning.lastProjectionMetrics().estimated_bytes_copied));
}

TEST(PerformanceRegression, MeasuresHighLevelCueProcessing) {
  semaforr::exploration::HighLevelExplorationConfiguration configuration;
  configuration.behavior_policy =
      semaforr::exploration::HleBehaviorPolicy::Compatibility;
  auto view = observation();
  view.laser.angle_min =
      semaforr::domain::Angle(-1.5707963267948966);
  view.laser.angle_increment = semaforr::domain::Angle(
      3.1415926535897932 / 659.0);
  view.laser.ranges_m.assign(660U, 4.0);
  constexpr std::size_t iterations = 250U;
  std::size_t candidates = 0U;
  const auto started = Clock::now();
  for (std::size_t index = 0U; index < iterations; ++index)
    candidates += semaforr::exploration::HighLevelExplorer::discoverCandidates(
                      view, configuration)
                      .size();
  const auto elapsed = Clock::now() - started;
  EXPECT_GT(candidates, 0U);
  RecordProperty("iterations", static_cast<int>(iterations));
  RecordProperty("candidates", static_cast<int>(candidates));
  RecordProperty("total_seconds", seconds(elapsed));
}

TEST(PerformanceRegression, MeasuresHallwayPairProcessing) {
  const auto paths = hallwayEvidence(4U);
  semaforr::spatial::HallwayLearningConfiguration configuration;
  configuration.initial_sigma = 0.0;
  constexpr std::size_t iterations = 100U;
  std::size_t hallway_count = 0U;
  const auto started = Clock::now();
  for (std::size_t index = 0U; index < iterations; ++index)
    hallway_count +=
        semaforr::spatial::learnCompatibilityHallways(paths, configuration)
            .hallways.size();
  const auto elapsed = Clock::now() - started;
  EXPECT_GT(hallway_count, 0U);
  RecordProperty("iterations", static_cast<int>(iterations));
  RecordProperty("input_segments", static_cast<int>(paths.size()));
  RecordProperty("hallways", static_cast<int>(hallway_count));
  RecordProperty("total_seconds", seconds(elapsed));
}

TEST(PerformanceRegression, MeasuresSerializationTimeAndSize) {
  auto learning = semaforr::spatial::SpatialLearningCoordinator::defaults(100U);
  for (std::size_t index = 0U; index < 12U; ++index)
    learning.observe(episode(index + 1U, static_cast<double>(index) * 0.25));
  learning.rebuildAll();
  constexpr std::size_t iterations = 20U;
  std::size_t bytes = 0U;
  const auto started = Clock::now();
  for (std::size_t index = 0U; index < iterations; ++index)
    bytes += learning.serializeAll().size();
  const auto elapsed = Clock::now() - started;
  EXPECT_GT(bytes, 0U);
  RecordProperty("iterations", static_cast<int>(iterations));
  RecordProperty("mean_serialized_bytes",
                 std::to_string(bytes / iterations));
  RecordProperty("total_seconds", seconds(elapsed));
}

TEST(PerformanceRegression, MeasuresPlanCacheHitRate) {
  semaforr::planning::PlanningCoordinator coordinator;
  coordinator.registerPlanner(
      std::make_unique<semaforr::planning::DomainPlanner>(
          "distance", semaforr::planning::PlanObjective::Distance));
  const auto map = planningMap();
  semaforr::domain::SpatialModel spatial;
  const semaforr::planning::PlanningRequest request{
      {{0.5, 0.5}, semaforr::domain::Angle::zero()},
      {18.5, 18.5}, &spatial, nullptr, &map};
  constexpr std::size_t requests = 50U;
  const auto started = Clock::now();
  for (std::size_t index = 0U; index < requests; ++index)
    ASSERT_TRUE(coordinator.selectPlan(request));
  const auto elapsed = Clock::now() - started;
  EXPECT_EQ(coordinator.cacheHits(), requests - 1U);
  RecordProperty("requests", static_cast<int>(requests));
  RecordProperty("cache_hits", static_cast<int>(coordinator.cacheHits()));
  RecordProperty("cache_hit_rate",
                 std::to_string(static_cast<double>(coordinator.cacheHits()) /
                                static_cast<double>(requests)));
  RecordProperty("total_seconds", seconds(elapsed));
}

}  // namespace
