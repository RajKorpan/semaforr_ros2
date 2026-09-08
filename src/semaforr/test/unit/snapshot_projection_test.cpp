/**
 * @file snapshot_projection_test.cpp
 * @brief Snapshot projection test responsibilities.
 *
 * @details This file exercises snapshot projection test behavior for automated
 * verification and regression testing. It centers on `BenchmarkResult`,
 * `SharedPublicationHasStableLifetimeAndIdentity`,
 * `LazyDenseAndRegionOfInterestAreExplicitAndCached`,
 * `GridPublicationNamesOnlyChangedCellRanges`,
 * `SmallAndLargeMapsAvoidUnchangedProjectionCopies`. Its package-relative
 * location is `test/unit/snapshot_projection_test.cpp`.
 */
#include <gtest/gtest.h>

#include <chrono>
#include <iomanip>
#include <semaforr/spatial/learners/grid_learners.hpp>
#include <semaforr/spatial/spatial_learning_coordinator.hpp>
#include <semaforr/validation/allocation_probe.hpp>
#include <sstream>

namespace {

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
  result.observation.pose = {{x, 0.0}, semaforr::domain::Angle::zero()};
  result.observation.laser.angle_min = semaforr::domain::Angle(-0.2);
  result.observation.laser.angle_increment = semaforr::domain::Angle(0.2);
  result.observation.laser.minimum_range = semaforr::domain::Distance(0.05);
  result.observation.laser.maximum_range = semaforr::domain::Distance(8.0);
  result.observation.laser.ranges_m = {4.0, 5.0, 4.0};
  return result;
}

/**
 * @brief Performs the coordinator operation for this subsystem.
 *
 * Arguments:
 * - @p extent_m: Supplies extent m input to the operation.
 *
 * Returns:
 * - `semaforr::spatial::SpatialLearningCoordinator` containing the
 * operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::spatial::SpatialLearningCoordinator coordinator(double extent_m) {
  semaforr::spatial::LearnedGridConfiguration grid;
  grid.initial_width_m = extent_m;
  grid.initial_height_m = extent_m;
  grid.resolution_m = 0.5;
  grid.extent_policy = semaforr::spatial::GridExtentPolicy::Fixed;
  grid.initialize_around_first_pose = false;
  return semaforr::spatial::SpatialLearningCoordinator::defaults(100U, {}, {},
                                                                 grid);
}

/**
 * @brief Encapsulates benchmark result state and behavior for this
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
struct BenchmarkResult {
  semaforr::spatial::SnapshotProjectionMetrics first;
  semaforr::spatial::SnapshotProjectionMetrics unchanged;
  semaforr::validation::AllocationSnapshot first_allocations;
  semaforr::validation::AllocationSnapshot unchanged_allocations;
};

/**
 * @brief Performs the benchmark operation for this subsystem.
 *
 * Arguments:
 * - @p extent_m: Supplies extent m input to the operation.
 *
 * Returns:
 * - `BenchmarkResult` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
BenchmarkResult benchmark(double extent_m) {
  auto learning = coordinator(extent_m);
  learning.observeSensor(episode(1U));
  semaforr::domain::SpatialModel world;
  const auto first_before = semaforr::validation::allocationSnapshot();
  learning.applyTo(world);
  const auto first_after = semaforr::validation::allocationSnapshot();
  const auto first_metrics = learning.lastProjectionMetrics();
  const auto unchanged_before = semaforr::validation::allocationSnapshot();
  learning.applyTo(world);
  const auto unchanged_after = semaforr::validation::allocationSnapshot();
  EXPECT_TRUE(world.known_grid.cells.empty());
  EXPECT_TRUE(world.sensed_occupancy.cells.empty());
  EXPECT_TRUE(world.inclusion_grid.cells.empty());
  EXPECT_FALSE(world.known_grid.sparseCells().empty());
  EXPECT_FALSE(world.sensed_occupancy.sparseCells().empty());
  // Sensor observations populate familiarity and sensed occupancy, not
  // learned inclusion. Inclusion is published from regions/subtrails or
  // successful LLE traversal.
  EXPECT_TRUE(world.inclusion_grid.sparseCells().empty());
  return {first_metrics, learning.lastProjectionMetrics(),
          semaforr::validation::allocationDifference(first_before, first_after),
          semaforr::validation::allocationDifference(unchanged_before,
                                                     unchanged_after)};
}

TEST(ImmutableSnapshots, SharedPublicationHasStableLifetimeAndIdentity) {
  semaforr::spatial::SharedSpatialSnapshot retained;
  {
    semaforr::spatial::KnownGridLearner learner(
        40U, 40U, 0.5, {-10.0, -10.0},
        semaforr::spatial::GridExtentPolicy::Fixed);
    learner.observe(episode(1U));
    const auto first = learner.sharedSnapshot();
    const auto second = learner.sharedSnapshot();
    ASSERT_TRUE(first);
    EXPECT_EQ(first.get(), second.get());
    retained = first;
  }
  ASSERT_TRUE(retained);
  EXPECT_TRUE(retained->usable());
  EXPECT_FALSE(std::get<semaforr::spatial::KnownGridModel>(retained->payload)
                   .sparse_observations.empty());
}

TEST(SparseSnapshots, LazyDenseAndRegionOfInterestAreExplicitAndCached) {
  auto learning = coordinator(100.0);
  learning.observeSensor(episode(1U));
  semaforr::domain::SpatialModel world;
  learning.applyTo(world);
  const auto sparse_size = world.known_grid.sparseCells().size();
  ASSERT_GT(sparse_size, 0U);
  const auto roi = world.known_grid.regionOfInterest({-1.0, -1.0}, {2.0, 1.0});
  EXPECT_LE(roi.size(), sparse_size);
  const auto& first = world.known_grid.denseCells();
  const auto& second = world.known_grid.denseCells();
  EXPECT_EQ(first.data(), second.data());
  EXPECT_EQ(first.size(), world.known_grid.columns * world.known_grid.rows);
  EXPECT_EQ(world.known_grid.sparseCells().size(), sparse_size);
}

TEST(ChangeSets, GridPublicationNamesOnlyChangedCellRanges) {
  semaforr::spatial::KnownGridLearner learner(
      80U, 80U, 0.5, {-20.0, -20.0},
      semaforr::spatial::GridExtentPolicy::Fixed);
  learner.observe(episode(1U));
  const auto first = learner.sharedSnapshot();
  ASSERT_TRUE(first);
  EXPECT_FALSE(first->changes.changed_cell_ranges.empty());
  learner.observe(episode(2U, 0.25));
  const auto second = learner.sharedSnapshot();
  ASSERT_TRUE(second);
  EXPECT_GT(second->revision, first->revision);
  EXPECT_FALSE(second->changes.changed_cell_ranges.empty());
  EXPECT_LT(second->changes.changed_cell_ranges.size(),
            std::get<semaforr::spatial::KnownGridModel>(second->payload)
                .sparse_observations.size());
}

TEST(ProjectionBenchmark, SmallAndLargeMapsAvoidUnchangedProjectionCopies) {
  const auto small = benchmark(20.0);
  const auto large = benchmark(2000.0);
  EXPECT_EQ(small.first.dense_cells_copied, 0U);
  EXPECT_EQ(large.first.dense_cells_copied, 0U);
  EXPECT_GT(small.first.sparse_cells_shared, 0U);
  EXPECT_GT(large.first.sparse_cells_shared, 0U);
  EXPECT_EQ(small.unchanged.representations_projected, 0U);
  EXPECT_EQ(large.unchanged.representations_projected, 0U);
  EXPECT_EQ(small.unchanged.estimated_bytes_copied, 0U);
  EXPECT_EQ(large.unchanged.estimated_bytes_copied, 0U);
  EXPECT_LE(large.first.estimated_bytes_copied,
            small.first.estimated_bytes_copied + 1024U);
  const auto seconds = [](double value) {
    std::ostringstream stream;
    stream << std::setprecision(17) << value;
    return stream.str();
  };
  RecordProperty("small_projection_seconds",
                 seconds(small.first.projection_time_s));
  RecordProperty("large_projection_seconds",
                 seconds(large.first.projection_time_s));
  RecordProperty("small_allocations", small.first_allocations.count);
  RecordProperty("large_allocations", large.first_allocations.count);
  RecordProperty("small_allocation_bytes", small.first_allocations.bytes);
  RecordProperty("large_allocation_bytes", large.first_allocations.bytes);
  RecordProperty("small_peak_projection_bytes",
                 small.first.peak_projection_bytes);
  RecordProperty("large_peak_projection_bytes",
                 large.first.peak_projection_bytes);
  RecordProperty("unchanged_large_allocations",
                 large.unchanged_allocations.count);
  EXPECT_DOUBLE_EQ(small.first.lock_duration_s, 0.0);
  EXPECT_DOUBLE_EQ(large.first.lock_duration_s, 0.0);
}

}  // namespace
