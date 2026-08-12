#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <semaforr/config/navigation_configuration.hpp>
#include <stdexcept>
#include <string>
#include <utility>

#ifndef SEMAFORR_TEST_SOURCE_DIR
#define SEMAFORR_TEST_SOURCE_DIR "."
#endif

namespace {

semaforr::config::Configuration validConfiguration() {
  semaforr::config::Configuration configuration;
  configuration.navigation.task_decision_limit = 20;
  configuration.navigation.can_see_point_epsilon = 0.005;
  configuration.navigation.laser_scan_radian_increment = 0.005817;
  configuration.navigation.robot_footprint = 0.2794;
  configuration.navigation.robot_footprint_buffer = 0.05;
  configuration.navigation.max_laser_range = 5.0;
  configuration.navigation.max_forward_action_buffer = 0.1;
  configuration.navigation.max_forward_action_sweep_angle = 0.5236;
  configuration.navigation.move_actions = {0.1, 0.2, 0.4};
  configuration.navigation.rotate_actions = {0.1, 0.5, 1.0};
  configuration.navigation.planners.skeleton = true;
  configuration.map_dimensions = {200, 200, 0.3};
  configuration.advisors = {{"goal_progress", "goal progress", true, 1.0, {}},
                            {"clearance", "clearance", true, 1.0, {}}};
  configuration.tasks = {{1.0, 2.0}, {3.0, 4.0}};
  configuration.map_file = std::string(SEMAFORR_TEST_SOURCE_DIR) +
                           "/config/stage_tutorial/stage_tutorialS.xml";
  return configuration;
}

template <typename Operation>
void assertThrowsContaining(Operation operation, const std::string& expected) {
  try {
    operation();
    assert(false && "expected configuration validation to fail");
  } catch (const std::runtime_error& error) {
    if (std::string(error.what()).find(expected) == std::string::npos)
      std::cerr << "expected error containing '" << expected << "', got '"
                << error.what() << "'\n";
    assert(std::string(error.what()).find(expected) != std::string::npos);
  }
}

}  // namespace

int main() {
  const auto valid = validConfiguration();
  assert(semaforr::config::behaviorModeFromString("modernized") ==
         semaforr::config::BehaviorMode::Modernized);
  assert(semaforr::config::behaviorModeFromString("compatibility") ==
         semaforr::config::BehaviorMode::Compatibility);
  assert(semaforr::config::spatialLearningProfileFromString(
             "chapter3_compatibility") ==
         semaforr::config::SpatialLearningProfile::Chapter3Compatibility);
  assert(semaforr::config::mapOperatingModeFromString("mapless") ==
         semaforr::config::MapOperatingMode::Mapless);
  assert(semaforr::config::mapOperatingModeFromString("map_enabled") ==
         semaforr::config::MapOperatingMode::MapEnabled);
  semaforr::config::validateConfiguration(valid);
  assert(semaforr::config::configurationFingerprint(valid).size() == 16U);
  const auto manifest = semaforr::config::componentManifest(valid);
  assert(std::find(manifest.begin(), manifest.end(),
                   "behavior_mode:modernized") != manifest.end());
  assert(std::find(manifest.begin(), manifest.end(),
                   "spatial_learning_profile:modernized") != manifest.end());
  assert(std::find(manifest.begin(), manifest.end(),
                   "tier2_maximum_planning_attempts_per_task:3") !=
         manifest.end());
  for (const std::string profile :
       {"full", "tier1_only", "tier1_tier3", "tier3_only",
        "tier1_tier2_tier3", "no_initial_exploration",
        "no_opportunistic_exploration", "no_spatial_model", "no_social",
        "purely_reactive", "original", "doors", "least_angle", "access",
        "tentative", "hallways", "shortest_path", "cost_graph", "wander",
        "deliberator", "forward_only", "global_exploration",
        "local_exploration", "highway", "circumstances", "naive",
        "custom"}) {
    assert(semaforr::config::toString(
               semaforr::config::ablationProfileFromString(profile)) ==
           profile);
  }
  {
    auto profiled = valid;
    profiled.experiment.profile =
        semaforr::config::AblationProfile::PurelyReactive;
    semaforr::config::applyAblationProfile(profiled);
    semaforr::config::validateConfiguration(profiled);
    assert(!profiled.experiment.tiers.tier_two);
    assert(profiled.advisors.back().name == "random");
  }
  {
    auto profiled = valid;
    profiled.experiment.profile = semaforr::config::AblationProfile::Highway;
    semaforr::config::applyAblationProfile(profiled);
    semaforr::config::validateConfiguration(profiled);
    assert(profiled.experiment.initial_exploration.enabled);
    assert(profiled.experiment.reactive_exploration_enabled);
    assert(profiled.navigation.planners.skeleton);
    assert(profiled.navigation.planners.highway);
  }
  {
    auto profiled = valid;
    profiled.experiment.profile =
        semaforr::config::AblationProfile::NoSocial;
    semaforr::config::applyAblationProfile(profiled);
    semaforr::config::validateConfiguration(profiled);
    assert(!profiled.experiment.social_enabled);
    assert(!profiled.navigation.crowd_learning.enabled);
  }
  {
    auto profiled = valid;
    profiled.experiment.profile =
        semaforr::config::AblationProfile::TierOneOnly;
    semaforr::config::applyAblationProfile(profiled);
    for (auto& advisor : profiled.advisors) advisor.active = false;
    semaforr::config::validateConfiguration(profiled);
    assert(profiled.experiment.tiers.tier_one);
    assert(!profiled.experiment.tiers.tier_three);
  }
  const std::string source_dir = SEMAFORR_TEST_SOURCE_DIR;
  const auto loaded = semaforr::config::loadStructuredConfiguration(
      valid.navigation, valid.map_dimensions, valid.advisors,
      source_dir + "/config/example/mission.conf", valid.map_file);
  assert(loaded.tasks.size() == 3U);

  {
    auto unavailable = valid;
    unavailable.experiment.behavior_mode =
        semaforr::config::BehaviorMode::Compatibility;
    assertThrowsContaining(
        [&unavailable]() {
          semaforr::config::validateConfiguration(unavailable);
        },
        "compatibility' is reserved but not operational");
  }
  {
    auto invalid = valid;
    invalid.experiment.tiers.maximum_planning_attempts_per_task = 0U;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "maximum_planning_attempts_per_task");
  }
  {
    auto changed = valid;
    changed.experiment.tiers.maximum_planning_attempts_per_task = 7U;
    assert(semaforr::config::configurationFingerprint(changed) !=
           semaforr::config::configurationFingerprint(valid));
    const auto changed_manifest = semaforr::config::componentManifest(changed);
    assert(std::find(changed_manifest.begin(), changed_manifest.end(),
                     "tier2_maximum_planning_attempts_per_task:7") !=
           changed_manifest.end());
  }
  {
    auto invalid = valid;
    invalid.experiment.tiers.tier_one = false;
    invalid.experiment.safety_envelope.enabled = false;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "invariant platform boundary");
  }
  {
    auto invalid = valid;
    invalid.experiment.tiers.reactive_planners.push_back("mystery");
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "unknown reactive planner");
  }
  {
    auto invalid = valid;
    invalid.experiment.reactive_exploration_enabled = true;
    invalid.navigation.planners = {};
    invalid.navigation.planners.distance = false;
    invalid.navigation.planners.skeleton = false;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "global replanning strategy");
  }
  {
    auto invalid = valid;
    invalid.navigation.planners.highway = true;
    invalid.navigation.highways_on = false;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "HighwayPlan requires");
  }
  {
    auto invalid = valid;
    invalid.navigation.highways_on = true;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "requires HLE output");
  }
  {
    auto invalid = valid;
    invalid.experiment.social.enabled = false;
    invalid.navigation.crowd_learning.enabled = false;
    invalid.advisors.push_back(
        {"social_navigation", "social", true, 1.0, {}});
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "requires social.enabled");
  }
  {
    auto changed = valid;
    changed.experiment.tiers.tier_one_rules.pop_back();
    assert(semaforr::config::configurationFingerprint(changed) !=
           semaforr::config::configurationFingerprint(valid));
  }
  {
    auto changed = valid;
    changed.navigation.spatial_learning_profile =
        semaforr::config::SpatialLearningProfile::Chapter3Compatibility;
    assert(semaforr::config::configurationFingerprint(changed) !=
           semaforr::config::configurationFingerprint(valid));
    const auto changed_manifest = semaforr::config::componentManifest(changed);
    assert(std::find(changed_manifest.begin(), changed_manifest.end(),
                     "spatial_learning_profile:chapter3_compatibility") !=
           changed_manifest.end());
  }
  {
    auto changed = valid;
    changed.experiment.initial_exploration.behavior_policy = "compatibility";
    changed.experiment.initial_exploration.hard_turn_threshold_rad = 0.6;
    assert(semaforr::config::configurationFingerprint(changed) !=
           semaforr::config::configurationFingerprint(valid));
    const auto manifest = semaforr::config::componentManifest(changed);
    assert(std::find(manifest.begin(), manifest.end(),
                     "hle_behavior_policy:compatibility") != manifest.end());
  }
  {
    auto invalid = valid;
    invalid.experiment.initial_exploration.enabled = true;
    invalid.experiment.initial_exploration.behavior_policy = "approximate";
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "behavior_policy");
  }
  {
    auto changed = valid;
    changed.experiment.reactive_exploration_behavior_policy =
        "compatibility";
    changed.experiment.reactive_exploration_stalled_history_extension = false;
    assert(semaforr::config::configurationFingerprint(changed) !=
           semaforr::config::configurationFingerprint(valid));
    const auto manifest = semaforr::config::componentManifest(changed);
    assert(std::find(manifest.begin(), manifest.end(),
                     "lle_behavior_policy:compatibility") != manifest.end());
    assert(std::find(manifest.begin(), manifest.end(),
                     "lle_stalled_history_extension:false") !=
           manifest.end());
    semaforr::config::validateConfiguration(changed);
  }
  {
    auto invalid = valid;
    invalid.experiment.reactive_exploration_behavior_policy = "approximate";
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "exploration.reactive.behavior_policy");
  }
  {
    auto invalid = valid;
    invalid.experiment.reactive_exploration_closest_target_bin_m = 0.0;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "closest_target_bin_m");
  }
  {
    auto changed = valid;
    changed.navigation.grids.highway_smoothing_policy =
        "von_neumann_three_of_four";
    changed.navigation.grids.highway_component_selection_policy =
        "most_intersections";
    changed.navigation.grids.highway_origin_x_m = -12.5;
    assert(semaforr::config::configurationFingerprint(changed) !=
           semaforr::config::configurationFingerprint(valid));
    const auto manifest = semaforr::config::componentManifest(changed);
    assert(std::find(manifest.begin(), manifest.end(),
                     "highway_smoothing_policy:von_neumann_three_of_four") !=
           manifest.end());
    assert(std::find(manifest.begin(), manifest.end(),
                     "highway_component_selection_policy:most_intersections") !=
           manifest.end());
  }
  {
    auto invalid = valid;
    invalid.navigation.grids.highway_smoothing_policy = "unnamed_magic";
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "highway policies");
  }
  {
    auto invalid = valid;
    invalid.experiment.initial_exploration.enabled = true;
    invalid.experiment.initial_exploration.observation_budget = 0U;
    invalid.experiment.initial_exploration.time_limit_s = 0.0;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "HLE requires");
  }
  {
    auto invalid = valid;
    invalid.navigation.move_actions = {0.2, 0.1};
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "strictly increasing");
  }
  {
    auto invalid = valid;
    invalid.navigation.move_actions.clear();
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "must not be empty");
  }
  {
    auto invalid = valid;
    invalid.navigation.rotate_actions = {0.1, 0.1};
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "strictly increasing");
  }
  {
    auto invalid = valid;
    invalid.navigation.task_decision_limit = 0;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "decision_limit");
  }
  {
    auto invalid = valid;
    invalid.navigation.robot_footprint =
        std::numeric_limits<double>::quiet_NaN();
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "safety thresholds");
  }
  {
    auto invalid = valid;
    invalid.navigation.a_star_on = true;
    invalid.navigation.planners.skeleton = false;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "features.astar");
  }
  {
    auto invalid = valid;
    invalid.advisors.front().name = "unknown";
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "unknown advisor");
  }
  {
    auto invalid = valid;
    invalid.navigation.planners.risk = true;
    invalid.navigation.planners.skeleton = false;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "require skeleton");
  }
  {
    auto invalid = valid;
    invalid.navigation.planners.skeleton = true;
    invalid.navigation.planners.flow = true;
    invalid.navigation.crowd_learning.enabled = false;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "crowd-cost planners");
  }
  {
    auto invalid = valid;
    invalid.navigation.crowd_learning.estimator = "mystery";
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "crowd learning");
  }
  {
    auto invalid = valid;
    invalid.map_dimensions.length = 0;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "map dimensions");
  }
  {
    auto invalid = valid;
    invalid.advisors.clear();
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "at least one advisor");
  }
  {
    auto invalid = valid;
    for (auto& advisor : invalid.advisors) {
      advisor.active = false;
    }
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "decision-producing advisor");
  }
  {
    auto invalid = valid;
    invalid.advisors[1].name = invalid.advisors[0].name;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "duplicate advisor");
  }
  {
    auto invalid = valid;
    invalid.advisors.front().weight = std::numeric_limits<double>::infinity();
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "invalid weight");
  }
  {
    auto invalid = valid;
    invalid.tasks.front().x = 500.0;
    invalid.static_map.mode = semaforr::config::MapOperatingMode::MapEnabled;
    invalid.static_map.path = invalid.map_file;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "outside configured dimensions");
  }
  {
    auto invalid = valid;
    invalid.tasks.clear();
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "at least one task");
  }
  {
    auto mapless = valid;
    mapless.map_file.clear();
    mapless.static_map.path.clear();
    mapless.static_map.mode = semaforr::config::MapOperatingMode::Mapless;
    semaforr::config::validateConfiguration(mapless);
    const auto mapless_manifest = semaforr::config::componentManifest(mapless);
    assert(std::find(mapless_manifest.begin(), mapless_manifest.end(),
                     "map_mode:mapless") != mapless_manifest.end());
  }
  {
    auto invalid = valid;
    invalid.static_map.mode =
        semaforr::config::MapOperatingMode::MapEnabled;
    invalid.static_map.path.clear();
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "map-enabled operation requires map.path");
  }
  {
    auto invalid = valid;
    invalid.navigation.planners.distance = true;
    invalid.static_map.mode = semaforr::config::MapOperatingMode::Mapless;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "static-map grid and crowd planners require");
  }
  {
    auto invalid = valid;
    invalid.navigation.planners.sensor_distance = true;
    invalid.navigation.sensed_occupancy_on = false;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "sensor_distance requires features.sensed_occupancy");
  }
  {
    auto invalid = valid;
    invalid.navigation.grids.extent_policy = "elastic";
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "grid geometry");
  }
  {
    auto changed = valid;
    changed.experiment.tier_three_scoring_policy =
        "compatibility_comments";
    changed.experiment.tier_three_tie_policy = "exact";
    semaforr::config::validateConfiguration(changed);
    assert(semaforr::config::configurationFingerprint(changed) !=
           semaforr::config::configurationFingerprint(valid));
  }
  {
    auto invalid = valid;
    invalid.experiment.tier_three_scoring_policy = "legacy_vote";
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "tiers.tier3.scoring_policy");
  }
  {
    auto invalid = valid;
    invalid.experiment.tier_three_tie_tolerance = -1.0;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "tiers.tier3.tie_tolerance");
  }
  for (const auto& fixture :
       {"empty_tasks.conf", "invalid_tasks.conf", "extra_task_token.conf"}) {
    assertThrowsContaining(
        [&valid, &source_dir, fixture]() {
          static_cast<void>(semaforr::config::loadStructuredConfiguration(
              valid.navigation, valid.map_dimensions, valid.advisors,
              source_dir + "/test/fixtures/config/" + fixture, valid.map_file));
        },
        fixture == std::string("empty_tasks.conf")
            ? "at least one task"
            : (fixture == std::string("invalid_tasks.conf")
                   ? "invalid number"
                   : "exactly x and y"));
  }

  return 0;
}
