/**
 * @file map_parser_test.cpp
 * @brief Map parser test responsibilities.
 *
 * @details This file exercises map parser test behavior for automated
 * verification and regression testing. It centers on
 * `ParsesValidatedMeterBasedSegments`,
 * `RejectsMalformedVerticesPrecisely`, `RejectsEmptyAndMalformedXmlMaps`,
 * `RejectsMissingRequiredBounds`,
 * `RetainsConcavePolygonAndDisconnectedObstacles`,
 * `ResolvesAbsolutePackageRelativeAndNamedExampleMaps`,
 * `MissingMapDiagnosticIdentifiesRequest`,
 * `BuildsGeometryOccupancyAndSupportsNegativeOrigin`. Its package-relative
 * location is `test/unit/map_parser_test.cpp`.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <semaforr/planning/domain_planner.hpp>
#include <semaforr/planning/map_parser.hpp>
#include <semaforr/planning/planner_registry.hpp>
#include <semaforr/planning/static_map_loader.hpp>
#include <sstream>

#ifndef SEMAFORR_TEST_WORKSPACE_SOURCE_DIR
#define SEMAFORR_TEST_WORKSPACE_SOURCE_DIR "."
#endif

TEST(MapParser, ParsesValidatedMeterBasedSegments) {
  std::istringstream input(R"xml(
    <Experiment>
      <ObstacleSet>
        <Obstacle closed="1">
          <Vertex p_x="1.0" p_y="2.0"/>
          <Vertex p_x="3.0" p_y="4.0"/>
        </Obstacle>
      </ObstacleSet>
    </Experiment>)xml");
  const auto map = semaforr::planning::parseMapXml(input, "inline map");
  ASSERT_EQ(map.walls.size(), 1U);
  EXPECT_DOUBLE_EQ(map.walls.front().start.x_m, 1.0);
  EXPECT_DOUBLE_EQ(map.walls.front().end.y_m, 4.0);
}

TEST(MapParser, RejectsMalformedVerticesPrecisely) {
  std::istringstream input(R"xml(
    <ObstacleSet><Obstacle>
      <Vertex p_x="one" p_y="2"/>
      <Vertex p_x="3" p_y="4"/>
    </Obstacle></ObstacleSet>)xml");
  EXPECT_THROW(semaforr::planning::parseMapXml(input, "bad map"),
               std::runtime_error);
}

TEST(MapParser, RejectsEmptyAndMalformedXmlMaps) {
  std::istringstream empty("<ObstacleSet></ObstacleSet>");
  EXPECT_THROW(semaforr::planning::parseMapXml(empty, "empty map"),
               std::runtime_error);
  std::istringstream malformed(
      "<ObstacleSet><Obstacle><Vertex p_x=\"1\" p_y=\"1\"/>"
      "<Vertex p_x=\"2\" p_y=\"2\"/></Obstacle>");
  EXPECT_THROW(semaforr::planning::parseMapXml(malformed, "malformed map"),
               std::runtime_error);
}

TEST(StaticMapLoader, RejectsMissingRequiredBounds) {
  namespace fs = std::filesystem;
  const fs::path workspace(SEMAFORR_TEST_WORKSPACE_SOURCE_DIR);
  semaforr::config::StaticMapConfiguration configuration;
  EXPECT_THROW(semaforr::planning::loadStaticMap(
                   workspace / "src/semaforr/test/fixtures/maps/negative.xml",
                   {0, 10, 0.5}, configuration),
               std::runtime_error);
}

TEST(MapParser, RetainsConcavePolygonAndDisconnectedObstacles) {
  std::istringstream input(R"xml(
    <ObstacleSet>
      <Obstacle closed="1">
        <Vertex p_x="0" p_y="0"/><Vertex p_x="3" p_y="0"/>
        <Vertex p_x="1" p_y="1"/><Vertex p_x="0" p_y="3"/>
      </Obstacle>
      <Obstacle><Vertex p_x="5" p_y="0"/><Vertex p_x="5" p_y="4"/></Obstacle>
    </ObstacleSet>)xml");
  const auto map = semaforr::planning::parseMapXml(input, "concave");
  EXPECT_EQ(map.obstacle_polygons.size(), 1U);
  EXPECT_EQ(map.walls.size(), 5U);
}

TEST(MapResolution, ResolvesAbsolutePackageRelativeAndNamedExampleMaps) {
  namespace fs = std::filesystem;
  const fs::path workspace(SEMAFORR_TEST_WORKSPACE_SOURCE_DIR);
  const fs::path examples = workspace / "src/examples";
  semaforr::planning::MapSearchPaths paths;
  paths.working_directory = workspace;
  paths.package_shares["semaforr_examples"] = examples;
  paths.example_core = examples / "core";
  const auto expected =
      fs::weakly_canonical(examples / "core/map-a/map-aS.xml");
  EXPECT_EQ(semaforr::planning::resolveMapPath(expected.string(), paths),
            expected);
  EXPECT_EQ(semaforr::planning::resolveMapPath(
                "package://semaforr_examples/core/map-a/map-aS.xml", paths),
            expected);
  EXPECT_EQ(semaforr::planning::resolveMapPath("map-a", paths), expected);
}

TEST(MapResolution, MissingMapDiagnosticIdentifiesRequest) {
  semaforr::planning::MapSearchPaths paths;
  try {
    static_cast<void>(semaforr::planning::resolveMapPath("absent-map", paths));
    FAIL() << "expected resolution failure";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("absent-map"), std::string::npos);
  }
}

TEST(StaticMapLoader, BuildsGeometryOccupancyAndSupportsNegativeOrigin) {
  namespace fs = std::filesystem;
  const fs::path workspace(SEMAFORR_TEST_WORKSPACE_SOURCE_DIR);
  const auto path = workspace / "src/semaforr/test/fixtures/maps/negative.xml";
  semaforr::config::StaticMapConfiguration configuration;
  configuration.mode = semaforr::config::MapOperatingMode::MapEnabled;
  configuration.origin_x_m = -5.0;
  configuration.origin_y_m = -5.0;
  configuration.occupancy_resolution_m = 0.5;
  const auto map =
      semaforr::planning::loadStaticMap(path, {10, 10, 0.5}, configuration);
  EXPECT_TRUE(map.geometryAvailable());
  EXPECT_TRUE(map.occupancyAvailable());
  EXPECT_EQ(map.provenance, semaforr::domain::GeometryProvenance::StaticMap);
  EXPECT_FALSE(map.lineOfSight({{-4.0, 0.0}, {4.0, 0.0}}));
}

TEST(StaticMapLoader, InfersPaddedBoundsWhenTheFormatHasNoBounds) {
  namespace fs = std::filesystem;
  const fs::path workspace(SEMAFORR_TEST_WORKSPACE_SOURCE_DIR);
  semaforr::config::StaticMapConfiguration configuration;
  configuration.mode = semaforr::config::MapOperatingMode::MapEnabled;
  configuration.bounds_policy = "infer";
  configuration.inferred_bounds_padding_m = 2.0;
  configuration.occupancy_resolution_m = 0.5;
  const auto map = semaforr::planning::loadStaticMap(
      workspace / "src/semaforr/test/fixtures/maps/negative.xml", {0, 0, 0.5},
      configuration);
  EXPECT_TRUE(map.occupancyAvailable());
  EXPECT_EQ(map.occupancy.geometry.extent_source,
            semaforr::domain::GridExtentSource::InferredMapBounds);
  EXPECT_EQ(map.occupancy.geometry.extent_mode,
            semaforr::domain::GridExtentMode::Fixed);
  EXPECT_LT(map.bounds.minimum.x_m, -1.0);
  EXPECT_GT(map.bounds.maximum.x_m, 1.0);
}

TEST(StaticMapLoader, RejectsUnsupportedFormatAndOutOfBoundsGeometry) {
  namespace fs = std::filesystem;
  const fs::path workspace(SEMAFORR_TEST_WORKSPACE_SOURCE_DIR);
  semaforr::config::StaticMapConfiguration configuration;
  EXPECT_THROW(semaforr::planning::loadStaticMap(
                   workspace / "src/examples/core/maze/mazeRoadmap.txt",
                   {10, 10, 0.5}, configuration),
               std::runtime_error);
  EXPECT_THROW(
      semaforr::planning::loadStaticMap(
          workspace / "src/semaforr/test/fixtures/maps/out_of_bounds.xml",
          {10, 10, 0.5}, configuration),
      std::runtime_error);
}

TEST(ComponentGating, RegistryDeclaresKnownMapRequirementsExplicitly) {
  const auto registry = semaforr::planning::defaultPlannerRegistry();
  EXPECT_EQ(registry.mapRequirement("distance"),
            semaforr::planning::StaticMapRequirement::Required);
  EXPECT_EQ(registry.mapRequirement("density"),
            semaforr::planning::StaticMapRequirement::Required);
  EXPECT_EQ(registry.mapRequirement("region"),
            semaforr::planning::StaticMapRequirement::Optional);
  EXPECT_EQ(registry.occupancyRequirement("sensor_distance"),
            semaforr::planning::OccupancyRequirement::SensedPartial);
  EXPECT_EQ(registry.mapRequirement("skeleton"),
            semaforr::planning::StaticMapRequirement::Independent);
  EXPECT_EQ(registry.mapRequirement("highway"),
            semaforr::planning::StaticMapRequirement::Independent);
}

TEST(MapPlanning, RoutesAroundMappedWallAndStaysInsideBounds) {
  namespace fs = std::filesystem;
  const fs::path workspace(SEMAFORR_TEST_WORKSPACE_SOURCE_DIR);
  semaforr::config::StaticMapConfiguration configuration;
  configuration.origin_x_m = -5.0;
  configuration.origin_y_m = -5.0;
  configuration.occupancy_resolution_m = 0.5;
  const auto map = semaforr::planning::loadStaticMap(
      workspace / "src/semaforr/test/fixtures/maps/negative.xml", {10, 10, 0.5},
      configuration);
  semaforr::domain::SpatialModel learned;
  semaforr::planning::DomainPlanner planner(
      "distance", semaforr::planning::PlanObjective::Distance);
  const auto result =
      planner.plan({{{-4.0, 0.0}, semaforr::domain::Angle::zero()},
                    {4.0, 0.0},
                    &learned,
                    nullptr,
                    &map});
  ASSERT_TRUE(result.succeeded()) << result.explanation;
  EXPECT_TRUE(
      std::any_of(result.path.begin(), result.path.end(),
                  [](const auto& point) { return std::abs(point.y_m) > 4.0; }));
  EXPECT_TRUE(std::all_of(
      result.path.begin(), result.path.end(),
      [&](const auto& point) { return map.bounds.contains(point); }));
  EXPECT_NE(result.explanation.find("static map"), std::string::npos);
}

TEST(MapPlanning, RefusesMaplessGridPlanningAndDoesNotUseLearnedGrid) {
  semaforr::domain::SpatialModel learned;
  learned.known_grid = {2U, 1U, 1.0, {0.0, 0.0}, {1U, 1U}, 1U};
  semaforr::planning::DomainPlanner planner(
      "distance", semaforr::planning::PlanObjective::Distance);
  const auto result = planner.plan(
      {{{0.0, 0.0}, semaforr::domain::Angle::zero()}, {1.0, 0.0}, &learned});
  EXPECT_EQ(result.status, semaforr::planning::PlanStatus::PlannerUnavailable);
}

TEST(StaticMapOwnership, LearnedRepresentationsCannotMutateStaticGeometry) {
  namespace fs = std::filesystem;
  const fs::path workspace(SEMAFORR_TEST_WORKSPACE_SOURCE_DIR);
  semaforr::config::StaticMapConfiguration configuration;
  configuration.origin_x_m = -5.0;
  configuration.origin_y_m = -5.0;
  const auto immutable = std::make_unique<const semaforr::domain::StaticMap>(
      semaforr::planning::loadStaticMap(
          workspace / "src/semaforr/test/fixtures/maps/negative.xml",
          {10, 10, 0.5}, configuration));
  semaforr::domain::WorldModel world;
  world.static_map = immutable.get();
  const auto walls = world.static_map->walls;
  world.spatial.obstacle_polygons.emplace_back(
      std::vector<semaforr::domain::Point2D>{
          {1.0, 1.0}, {2.0, 1.0}, {1.0, 2.0}});
  world.spatial.known_grid.cells.push_back(1U);
  EXPECT_EQ(world.static_map->walls, walls);
  EXPECT_EQ(world.static_map->provenance,
            semaforr::domain::GeometryProvenance::StaticMap);
}
