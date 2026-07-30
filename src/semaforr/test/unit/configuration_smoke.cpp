#include <cassert>
#include <limits>
#include <semaforr/config/Configuration.hpp>
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
    assert(std::string(error.what()).find(expected) != std::string::npos);
  }
}

}  // namespace

int main() {
  const auto valid = validConfiguration();
  semaforr::config::validateConfiguration(valid);
  assert(semaforr::config::configurationFingerprint(valid).size() == 16U);
  assert(!semaforr::config::componentManifest(valid).empty());
  for (const std::string profile :
       {"full", "tier1_only", "tier1_tier3", "tier3_only",
        "tier1_tier2_tier3", "no_initial_exploration",
        "no_opportunistic_exploration", "no_spatial_model", "no_social",
        "custom"}) {
    assert(semaforr::config::toString(
               semaforr::config::ablationProfileFromString(profile)) ==
           profile);
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
    auto invalid = valid;
    invalid.experiment.tiers.tier_one = false;
    invalid.experiment.safety_envelope.enabled = false;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "Tier 1 may be disabled");
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
    auto invalid = valid;
    invalid.experiment.initial_exploration.enabled = true;
    invalid.experiment.initial_exploration.observation_budget = 0U;
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "observation budget");
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
    auto invalid = valid;
    invalid.map_file = "does-not-exist.xml";
    assertThrowsContaining(
        [&invalid]() { semaforr::config::validateConfiguration(invalid); },
        "cannot open map file");
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
