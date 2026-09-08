/**
 * @file grid_layers_test.cpp
 * @brief Grid layers test responsibilities.
 *
 * @details This file exercises grid layers test behavior for automated
 * verification and regression testing. It centers on
 * `HitEndpointIsOccupiedWhileFamiliarityOnlyRecordsObservation`,
 * `MaximumRangeIsFreeAndInvalidBeamsAreIgnored`,
 * `FamiliarityCountsEachCoveredCellOncePerDecisionObservation`,
 * `FamiliarityCountsMaximumRangeButIgnoresInvalidRays`,
 * `FamiliarityIsInvariantToDifferentRayCountsAcrossAdjacentCells`,
 * `ExtentPolicyExpandsOrClipsExplicitly`,
 * `MaplessGeometryInitializesAroundFirstPoseWithoutFabricatingFreeSpace`,
 * `RepeatedFreeEvidenceClearsDynamicOccupancyAndStaleHitsExpire`. Its
 * package-relative location is `test/unit/grid_layers_test.cpp`.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/traversability.hpp>
#include <semaforr/spatial/learners/grid_learners.hpp>

namespace {

/**
 * @brief Performs the scan operation for this subsystem.
 *
 * Arguments:
 * - @p sequence: Supplies sequence input to the operation.
 * - @p range: Supplies range input to the operation.
 * - @p maximum: Supplies maximum input to the operation.
 *
 * Returns:
 * - `semaforr::spatial::NavigationEpisode` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::spatial::NavigationEpisode scan(std::size_t sequence, double range,
                                          double maximum = 4.0) {
  semaforr::domain::LaserObservation laser;
  laser.angle_min = semaforr::domain::Angle::zero();
  laser.angle_increment = semaforr::domain::Angle(0.1);
  laser.minimum_range = semaforr::domain::Distance(0.1);
  laser.maximum_range = semaforr::domain::Distance(maximum);
  laser.ranges_m = {range};
  semaforr::domain::RobotObservation observation;
  observation.pose = {{0.5, 0.5}, semaforr::domain::Angle::zero()};
  observation.laser = std::move(laser);
  semaforr::spatial::NavigationEpisode episode;
  episode.sequence = sequence;
  episode.observation = std::move(observation);
  return episode;
}

/**
 * @brief Performs the static map operation for this subsystem.
 *
 * Arguments:
 * - @p columns: Supplies columns input to the operation.
 * - @p rows: Supplies rows input to the operation.
 *
 * Returns:
 * - `semaforr::domain::StaticMap` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::domain::StaticMap staticMap(std::size_t columns,
                                      std::size_t rows = 1U) {
  semaforr::domain::StaticMap map;
  map.source = "grid-layer-test";
  map.format = "test";
  map.bounds = {{0.0, 0.0},
                {static_cast<double>(columns), static_cast<double>(rows)}};
  map.walls = {{{0.0, 0.0}, {static_cast<double>(columns), 0.0}}};
  map.occupancy = {
      columns,
      rows,
      1.0,
      {0.0, 0.0},
      std::vector<semaforr::domain::StaticOccupancyState>(
          columns * rows, semaforr::domain::StaticOccupancyState::StaticFree)};
  return map;
}

/**
 * @brief Performs the familiarity at operation for this subsystem.
 *
 * Arguments:
 * - @p model: Supplies model input to the operation.
 * - @p index: Supplies index input to the operation.
 *
 * Returns:
 * - `std::uint32_t` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::uint32_t familiarityAt(const semaforr::spatial::KnownGridModel& model,
                            std::size_t index) {
  const auto found = std::find_if(
      model.sparse_observations.begin(), model.sparse_observations.end(),
      [index](const auto& cell) { return cell.index == index; });
  return found == model.sparse_observations.end() ? 0U : found->value;
}

}  // namespace

TEST(GridLayers, HitEndpointIsOccupiedWhileFamiliarityOnlyRecordsObservation) {
  using namespace semaforr;
  spatial::SensedOccupancyLearner occupancy(6U, 1U, 1.0);
  spatial::KnownGridLearner familiarity(6U, 1U, 1.0);
  const auto episode = scan(1U, 2.0);
  occupancy.observe(episode);
  familiarity.observe(episode);

  const auto sensed =
      std::get<spatial::SensedOccupancyModel>(occupancy.snapshot().payload);
  EXPECT_TRUE(sensed.cells.empty());
  EXPECT_EQ(sensed.valueAt(0U).state,
            domain::SensedOccupancyState::ObservedFree);
  EXPECT_EQ(sensed.valueAt(1U).state,
            domain::SensedOccupancyState::ObservedFree);
  EXPECT_EQ(sensed.valueAt(2U).state,
            domain::SensedOccupancyState::ObservedOccupied);
  const auto known =
      std::get<spatial::KnownGridModel>(familiarity.snapshot().payload);
  EXPECT_TRUE(std::any_of(known.sparse_observations.begin(),
                          known.sparse_observations.end(),
                          [](const auto& cell) { return cell.index == 2U; }));
}

TEST(GridLayers, MaximumRangeIsFreeAndInvalidBeamsAreIgnored) {
  using namespace semaforr;
  spatial::SensedOccupancyLearner maximum(6U, 1U, 1.0);
  maximum.observe(scan(1U, 4.0));
  const auto max_model =
      std::get<spatial::SensedOccupancyModel>(maximum.snapshot().payload);
  EXPECT_EQ(max_model.valueAt(4U).state,
            domain::SensedOccupancyState::ObservedFree);
  EXPECT_EQ(std::count_if(
                max_model.sparseCells().begin(), max_model.sparseCells().end(),
                [](const auto& sparse) {
                  return sparse.value.state ==
                         domain::SensedOccupancyState::ObservedOccupied;
                }),
            0);

  spatial::SensedOccupancyLearner invalid(6U, 1U, 1.0);
  auto invalid_scan = scan(1U, std::numeric_limits<double>::quiet_NaN());
  invalid_scan.observation.laser.ranges_m = {
      std::numeric_limits<double>::quiet_NaN(), 0.05, 9.0,
      -std::numeric_limits<double>::infinity()};
  invalid.observe(invalid_scan);
  EXPECT_EQ(std::get<spatial::SensedOccupancyModel>(invalid.snapshot().payload)
                .observedCellCount(),
            0U);
}

TEST(GridLayers, FamiliarityCountsEachCoveredCellOncePerDecisionObservation) {
  using namespace semaforr;
  spatial::KnownGridLearner familiarity(6U, 2U, 1.0);
  auto dense = scan(1U, 2.0);
  dense.observation.laser.angle_increment = domain::Angle::zero();
  dense.observation.laser.ranges_m.assign(10U, 2.0);
  familiarity.observe(dense);

  auto model =
      std::get<spatial::KnownGridModel>(familiarity.snapshot().payload);
  EXPECT_EQ(familiarityAt(model, 0U), 1U);
  EXPECT_EQ(familiarityAt(model, 1U), 1U);
  EXPECT_EQ(familiarityAt(model, 2U), 1U);

  for (std::size_t sequence = 2U; sequence <= 5U; ++sequence) {
    auto repeated = dense;
    repeated.sequence = sequence;
    familiarity.observe(repeated);
  }
  model = std::get<spatial::KnownGridModel>(familiarity.snapshot().payload);
  EXPECT_EQ(familiarityAt(model, 0U), 5U);
  EXPECT_EQ(familiarityAt(model, 1U), 5U);
  EXPECT_EQ(familiarityAt(model, 2U), 5U);
}

TEST(GridLayers, FamiliarityCountsMaximumRangeButIgnoresInvalidRays) {
  using namespace semaforr;
  spatial::KnownGridLearner familiarity(6U, 1U, 1.0);
  auto episode = scan(1U, 4.0);
  episode.observation.laser.angle_increment = domain::Angle::zero();
  episode.observation.laser.ranges_m = {
      4.0, std::numeric_limits<double>::quiet_NaN(), 0.05,
      std::numeric_limits<double>::infinity()};
  familiarity.observe(episode);
  const auto model =
      std::get<spatial::KnownGridModel>(familiarity.snapshot().payload);
  EXPECT_EQ(familiarityAt(model, 0U), 1U);
  EXPECT_EQ(familiarityAt(model, 4U), 1U);
  EXPECT_EQ(model.sparse_observations.size(), 5U);
}

TEST(GridLayers,
     FamiliarityIsInvariantToDifferentRayCountsAcrossAdjacentCells) {
  using namespace semaforr;
  spatial::KnownGridLearner familiarity(5U, 5U, 1.0);
  auto episode = scan(1U, 2.0);
  episode.observation.laser.angle_min = domain::Angle::zero();
  episode.observation.laser.angle_increment = domain::Angle(0.7853981633974483);
  episode.observation.laser.ranges_m = {2.0, 2.0, 2.0};
  familiarity.observe(episode);
  const auto model =
      std::get<spatial::KnownGridModel>(familiarity.snapshot().payload);

  // The robot's cell is crossed by all three rays while the two neighboring
  // branches are crossed by different subsets. Every covered cell still gets
  // exactly one increment from this decision observation.
  ASSERT_FALSE(model.sparse_observations.empty());
  EXPECT_TRUE(std::all_of(model.sparse_observations.begin(),
                          model.sparse_observations.end(),
                          [](const auto& cell) { return cell.value == 1U; }));
}

TEST(GridLayers, ExtentPolicyExpandsOrClipsExplicitly) {
  using namespace semaforr;
  spatial::SensedOccupancyLearner expanding(2U, 2U, 1.0, {}, {},
                                            spatial::GridExtentPolicy::Expand);
  expanding.observe(scan(1U, 4.0));
  const auto expanded =
      std::get<spatial::SensedOccupancyModel>(expanding.snapshot().payload);
  EXPECT_GT(expanded.geometry.columns, 2U);
  const auto endpoint = expanded.geometry.index({4.5, 0.5});
  ASSERT_TRUE(endpoint);
  EXPECT_EQ(expanded.valueAt(*endpoint).state,
            domain::SensedOccupancyState::ObservedFree);

  spatial::SensedOccupancyLearner fixed(2U, 2U, 1.0, {}, {},
                                        spatial::GridExtentPolicy::Fixed);
  fixed.observe(scan(1U, 4.0));
  const auto clipped =
      std::get<spatial::SensedOccupancyModel>(fixed.snapshot().payload);
  EXPECT_EQ(clipped.geometry.columns, 2U);
  EXPECT_FALSE(clipped.geometry.index({4.5, 0.5}));
}

TEST(GridLayers,
     MaplessGeometryInitializesAroundFirstPoseWithoutFabricatingFreeSpace) {
  using namespace semaforr;
  domain::GridExpansionPolicy expansion;
  expansion.margin_m = 0.0;
  expansion.increment_cells = 4U;
  spatial::SensedOccupancyLearner learner(4U, 4U, 1.0, {}, {},
                                          spatial::GridExtentPolicy::Expand,
                                          expansion, true, "odom");
  auto episode = scan(1U, std::numeric_limits<double>::quiet_NaN());
  episode.observation.pose.position = {10.0, -5.0};
  learner.observe(episode);
  const auto model =
      std::get<spatial::SensedOccupancyModel>(learner.snapshot().payload);
  EXPECT_EQ(model.geometry.frame_id, "odom");
  EXPECT_DOUBLE_EQ(model.geometry.minimum.x_m, 8.0);
  EXPECT_DOUBLE_EQ(model.geometry.minimum.y_m, -7.0);
  EXPECT_EQ(model.geometry.geometry_revision, 2U);
  EXPECT_EQ(model.observedCellCount(), 0U);
}

TEST(GridLayers, RepeatedFreeEvidenceClearsDynamicOccupancyAndStaleHitsExpire) {
  using namespace semaforr;
  spatial::SensedOccupancyLearningConfiguration configuration;
  configuration.free_observations_to_clear = 2U;
  configuration.dynamic_expiry_observations = 3U;
  spatial::SensedOccupancyLearner learner(6U, 1U, 1.0, {}, configuration);
  learner.observe(scan(1U, 2.0));
  learner.observe(scan(2U, 4.0));
  learner.observe(scan(3U, 4.0));
  learner.observe(scan(4U, 4.0));
  const auto cleared =
      std::get<spatial::SensedOccupancyModel>(learner.snapshot().payload);
  EXPECT_EQ(cleared.valueAt(2U).state,
            domain::SensedOccupancyState::ObservedFree);
  EXPECT_TRUE(cleared.valueAt(2U).conflicting);

  spatial::SensedOccupancyLearner expiry(6U, 1U, 1.0, {}, configuration);
  expiry.observe(scan(1U, 2.0));
  for (std::size_t sequence = 2U; sequence <= 4U; ++sequence)
    expiry.observe(scan(sequence, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(std::get<spatial::SensedOccupancyModel>(expiry.snapshot().payload)
                .valueAt(2U)
                .state,
            domain::SensedOccupancyState::Unknown);
}

TEST(GridLayers, StaticAndSensedOccupancyFuseWithoutMutatingThePrior) {
  using namespace semaforr;
  auto map = staticMap(3U);
  map.occupancy.cells[0] = domain::StaticOccupancyState::StaticOccupied;
  domain::SensedOccupancyGrid sensed;
  sensed.geometry = {3U, 1U, 1.0, {0.0, 0.0}};
  sensed.cells.resize(3U);
  sensed.cells[0].state = domain::SensedOccupancyState::ObservedFree;
  sensed.cells[0].source = domain::OccupancyEvidenceSource::CurrentSensor;
  sensed.cells[2].state = domain::SensedOccupancyState::ObservedOccupied;
  sensed.cells[2].dynamic = true;
  sensed.cells[2].source = domain::OccupancyEvidenceSource::DynamicObstacle;
  planning::TraversabilityConfiguration configuration;
  configuration.robot_radius_m = 0.0;
  configuration.safety_clearance_m = 0.0;
  configuration.localization_uncertainty_m = 0.0;
  configuration.dynamic_obstacle_margin_m = 0.0;
  const auto result = planning::deriveTraversability(
      planning::OccupancySourceMode::StaticMapWithSensors, &map, &sensed,
      configuration);
  ASSERT_TRUE(result.grid.valid());
  EXPECT_EQ(result.static_conflicts, 1U);
  EXPECT_EQ(result.grid.cells[0].state,
            domain::TraversabilityState::NonTraversable);
  EXPECT_EQ(result.grid.cells[2].state,
            domain::TraversabilityState::NonTraversable);
  EXPECT_EQ(map.occupancy.cells[0],
            domain::StaticOccupancyState::StaticOccupied);
}

TEST(GridLayers, InflationAndPartialPlannerExcludeOccupiedEndpoints) {
  using namespace semaforr;
  domain::SensedOccupancyGrid sensed;
  sensed.geometry = {5U, 3U, 1.0, {0.0, 0.0}};
  sensed.cells.resize(15U);
  for (auto& cell : sensed.cells)
    cell.state = domain::SensedOccupancyState::ObservedFree;
  sensed.cells[7].state = domain::SensedOccupancyState::ObservedOccupied;
  planning::TraversabilityConfiguration inflation;
  inflation.robot_radius_m = 1.01;
  inflation.safety_clearance_m = 0.0;
  inflation.localization_uncertainty_m = 0.0;
  inflation.dynamic_obstacle_margin_m = 0.0;
  const auto inflated = planning::deriveTraversability(
      planning::OccupancySourceMode::SensorDerivedPartial, nullptr, &sensed,
      inflation);
  EXPECT_GE(inflated.inflated_cells, 4U);
  EXPECT_TRUE(domain::hasEvidence(inflated.grid.cells[6].provenance,
                                  domain::OccupancyEvidenceSource::Inflation));

  domain::SpatialModel spatial;
  spatial.sensed_occupancy.geometry = {3U, 1U, 1.0, {0.0, 0.0}};
  spatial.sensed_occupancy.cells.resize(3U);
  spatial.sensed_occupancy.cells[0].state =
      domain::SensedOccupancyState::ObservedFree;
  spatial.sensed_occupancy.cells[1].state =
      domain::SensedOccupancyState::ObservedFree;
  spatial.sensed_occupancy.cells[2].state =
      domain::SensedOccupancyState::ObservedOccupied;
  planning::DomainPlanner planner(
      "sensor_distance", planning::PlanObjective::Distance,
      planning::OccupancySourceMode::SensorDerivedPartial);
  planning::TraversabilityConfiguration no_inflation;
  no_inflation.robot_radius_m = no_inflation.safety_clearance_m =
      no_inflation.localization_uncertainty_m =
          no_inflation.dynamic_obstacle_margin_m = 0.0;
  const auto plan = planner.plan({{{0.5, 0.5}, domain::Angle::zero()},
                                  {2.5, 0.5},
                                  &spatial,
                                  nullptr,
                                  nullptr,
                                  no_inflation});
  EXPECT_EQ(plan.status, planning::PlanStatus::NoPath);
}
