/**
 * @file map_runtime_test.cpp
 * @brief Map runtime test responsibilities.
 *
 * @details This file exercises map runtime test behavior for automated
 * verification and regression testing. It centers on
 * `MaplessStartupSkipsLoadingAndGridPlannerRegistration`,
 * `MapEnabledStartupInstallsImmutableMapAndEnablesGridPlanner`,
 * `ExplicitFailurePolicyControlsStartup`. Its package-relative location is
 * `test/integration/map_runtime_test.cpp`.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <semaforr/ros/navigation_engine_adapter.hpp>
#include <string>

#ifndef SEMAFORR_TEST_SOURCE_DIR
#define SEMAFORR_TEST_SOURCE_DIR "."
#endif

namespace {

/**
 * @brief Performs the configuration operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `semaforr::config::Configuration` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
semaforr::config::Configuration configuration() {
  semaforr::config::Configuration result;
  auto& navigation = result.navigation;
  navigation.task_decision_limit = 20;
  navigation.can_see_point_epsilon = 0.005;
  navigation.laser_scan_radian_increment = 0.005817;
  navigation.robot_footprint = 0.2794;
  navigation.robot_footprint_buffer = 0.05;
  navigation.max_laser_range = 5.0;
  navigation.max_forward_action_buffer = 0.1;
  navigation.max_forward_action_sweep_angle = 0.5236;
  navigation.move_actions = {0.1, 0.2, 0.4};
  navigation.rotate_actions = {0.1, 0.5, 1.0};
  navigation.planners.distance = true;
  navigation.planners.skeleton = false;
  navigation.crowd_learning.enabled = false;
  result.experiment.social.enabled = false;
  result.experiment.social_enabled = false;
  result.experiment.tiers.tier_three = false;
  result.map_dimensions = {10, 10, 0.5};
  result.tasks = {{4.0, 0.0}};
  result.static_map.origin_x_m = -5.0;
  result.static_map.origin_y_m = -5.0;
  result.static_map.occupancy_resolution_m = 0.5;
  return result;
}

/**
 * @brief Reports whether diagnostic for this subsystem.
 *
 * Arguments:
 * - @p adapter: Supplies adapter input to the operation.
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool hasDiagnostic(const semaforr::ros::NavigationEngineAdapter& adapter,
                   const std::string& value) {
  const auto& diagnostics = adapter.startupDiagnostics();
  return std::find(diagnostics.begin(), diagnostics.end(), value) !=
         diagnostics.end();
}

}  // namespace

TEST(MapRuntime, MaplessStartupSkipsLoadingAndGridPlannerRegistration) {
  auto configured = configuration();
  configured.static_map.mode = semaforr::config::MapOperatingMode::Mapless;
  configured.static_map.path = "this-file-must-not-be-opened.xml";
  semaforr::ros::NavigationEngineAdapter adapter(std::move(configured));
  EXPECT_FALSE(adapter.worldModel().static_map);
  EXPECT_FALSE(adapter.worldModel().map_capabilities.map_available);
  EXPECT_TRUE(hasDiagnostic(adapter, "map_status:mapless_parser_not_invoked"));
  EXPECT_TRUE(hasDiagnostic(adapter, "planner_disabled_no_map:distance"));
}

TEST(MapRuntime, MapEnabledStartupInstallsImmutableMapAndEnablesGridPlanner) {
  auto configured = configuration();
  configured.static_map.mode = semaforr::config::MapOperatingMode::MapEnabled;
  configured.static_map.path = std::string(SEMAFORR_TEST_SOURCE_DIR) +
                               "/test/fixtures/maps/negative.xml";
  semaforr::ros::NavigationEngineAdapter adapter(std::move(configured));
  ASSERT_TRUE(adapter.worldModel().static_map);
  EXPECT_TRUE(adapter.worldModel().map_capabilities.map_geometry_available);
  EXPECT_TRUE(adapter.worldModel().map_capabilities.map_occupancy_available);
  EXPECT_TRUE(
      adapter.worldModel().map_capabilities.map_based_planning_available);
  EXPECT_TRUE(hasDiagnostic(adapter, "planner_enabled:distance"));
  EXPECT_TRUE(adapter.worldModel().spatial.obstacle_polygons.empty());
}

TEST(MapRuntime, ExplicitFailurePolicyControlsStartup) {
  auto lenient = configuration();
  lenient.static_map.mode = semaforr::config::MapOperatingMode::MapEnabled;
  lenient.static_map.path = "missing-static-map.xml";
  lenient.static_map.failure_policy =
      semaforr::config::MapLoadFailurePolicy::DisableMap;
  semaforr::ros::NavigationEngineAdapter adapter(std::move(lenient));
  EXPECT_FALSE(adapter.worldModel().map_capabilities.map_available);
  EXPECT_TRUE(hasDiagnostic(adapter, "map_status:load_failed_map_disabled"));
  EXPECT_TRUE(hasDiagnostic(adapter, "planner_disabled_no_map:distance"));

  auto strict = configuration();
  strict.static_map.mode = semaforr::config::MapOperatingMode::MapEnabled;
  strict.static_map.path = "missing-static-map.xml";
  strict.static_map.failure_policy =
      semaforr::config::MapLoadFailurePolicy::FailStartup;
  EXPECT_THROW(semaforr::ros::NavigationEngineAdapter(std::move(strict)),
               std::runtime_error);
}
