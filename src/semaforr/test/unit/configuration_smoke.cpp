#include <semaforr/config/Configuration.hpp>

#include <cassert>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#ifndef SEMAFORR_TEST_SOURCE_DIR
#define SEMAFORR_TEST_SOURCE_DIR "."
#endif

namespace {

std::string readFile(const std::string& filename) {
  std::ifstream input(filename);
  assert(input.is_open());
  std::ostringstream contents;
  contents << input.rdbuf();
  return contents.str();
}

template <typename Operation>
void assertThrowsContaining(
    Operation operation,
    const std::string& expected_message) {
  try {
    operation();
    assert(false && "expected std::runtime_error");
  } catch (const std::runtime_error& error) {
    assert(
        std::string(error.what()).find(expected_message) != std::string::npos);
  }
}

}  // namespace

int main() {
  const std::string source_dir = SEMAFORR_TEST_SOURCE_DIR;
  const std::string tutorial_dir = source_dir + "/config/stage_tutorial";
  const semaforr::config::ConfigurationFiles files{
      source_dir + "/config/advisors.conf",
      source_dir + "/config/params.conf",
      tutorial_dir + "/stage_tutorialS.xml",
      tutorial_dir + "/target.conf",
      tutorial_dir + "/dimensions.conf"};

  const semaforr::config::Configuration configuration =
      semaforr::config::loadConfiguration(files);
  const auto& controller = configuration.controller;

  assert(controller.task_decision_limit == 20);
  assert(controller.plan_limit == 500);
  assert(controller.move_actions.size() == 6);
  assert(controller.move_actions.front() == 0.1);
  assert(controller.move_actions.back() == 3.2);
  assert(controller.rotate_actions.size() == 6);
  assert(controller.trails_on);
  assert(controller.conveyors_on);
  assert(controller.regions_on);
  assert(controller.doors_on);
  assert(!controller.a_star_on);
  assert(!controller.planners.skeleton);
  assert(!controller.planners.hallway_skeleton);
  assert(configuration.map_dimensions.length == 200);
  assert(configuration.map_dimensions.height == 200);
  assert(configuration.map_dimensions.granularity == 0.3);
  assert(configuration.advisors.size() == 64);
  assert(configuration.advisors.front().name == "Greedy");
  assert(configuration.advisors.front().active);
  assert(configuration.tasks.size() == 8);
  assert(configuration.tasks.front().x == 1.0);
  assert(configuration.tasks.back().y == 95.0);
  assert(configuration.map_file == files.map);

  const std::string valid_parameters = readFile(files.parameters);

  assertThrowsContaining(
      [&]() {
        std::istringstream input(valid_parameters + "\nunknownSetting 1\n");
        semaforr::config::parseControllerConfiguration(input, "params-test");
      },
      "unknown setting 'unknownSetting'");

  assertThrowsContaining(
      [&]() {
        std::istringstream input(valid_parameters + "\ntrailsOn 1\n");
        semaforr::config::parseControllerConfiguration(input, "params-test");
      },
      "duplicate setting 'trailsOn'");

  std::string invalid_flag = valid_parameters;
  invalid_flag.replace(
      invalid_flag.find("trailsOn 1"),
      std::string("trailsOn 1").size(),
      "trailsOn 2");
  assertThrowsContaining(
      [&]() {
        std::istringstream input(invalid_flag);
        semaforr::config::parseControllerConfiguration(input, "params-test");
      },
      "expected boolean flag 0 or 1");

  std::string missing_setting = valid_parameters;
  missing_setting.replace(
      missing_setting.find("planLimit 500"),
      std::string("planLimit 500").size(),
      "# removed plan limit");
  assertThrowsContaining(
      [&]() {
        std::istringstream input(missing_setting);
        semaforr::config::parseControllerConfiguration(input, "params-test");
      },
      "missing required setting 'planLimit'");

  std::string unsorted_actions = valid_parameters;
  unsorted_actions.replace(
      unsorted_actions.find("move 0 0.1 0.2 0.4 0.8 1.6 3.2"),
      std::string("move 0 0.1 0.2 0.4 0.8 1.6 3.2").size(),
      "move 0 0.2 0.1");
  assertThrowsContaining(
    [&unsorted_actions]() {
      std::istringstream input(unsorted_actions);
      semaforr::config::parseControllerConfiguration(input, "actions-test");
    },
    "move actions must be strictly increasing");

  assertThrowsContaining(
      []() {
        std::istringstream input("200 200 0.3\n100 100 0.2\n");
        semaforr::config::parseMapDimensions(input, "dimensions-test");
      },
      "dimensions-test:2");

  assertThrowsContaining(
      []() {
        std::istringstream input("Greedy description maybe 1 0 0 0 0\n");
        semaforr::config::parseAdvisorConfigurations(input, "advisors-test");
      },
      "advisor state must be 't' or 'f'");

  {
    std::istringstream input("1 2 # inline comment\n3.5 4.5\n");
    const auto tasks =
        semaforr::config::parseTaskConfigurations(input, "tasks-test");
    assert(tasks.size() == 2);
    assert(tasks[1].x == 3.5);
  }

  semaforr::config::ConfigurationFiles missing_map = files;
  missing_map.map = tutorial_dir + "/does-not-exist.xml";
  assertThrowsContaining(
      [&]() {
        semaforr::config::loadConfiguration(missing_map);
      },
      "cannot open map file");

  {
    auto invalid = configuration;
    invalid.controller.planners.cusum = true;
    assertThrowsContaining(
      [&invalid]() {
        semaforr::config::validateConfiguration(invalid);
      },
      "crowd-learning estimators, not planners");
  }

  {
    auto invalid = configuration;
    invalid.controller.planners.risk = true;
    invalid.controller.planners.skeleton = false;
    assertThrowsContaining(
      [&invalid]() {
        semaforr::config::validateConfiguration(invalid);
      },
      "require the skeleton planner");
  }

  return 0;
}
