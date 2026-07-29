#include <semaforr/config/Configuration.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace semaforr {
namespace config {
namespace {

using DoubleMember = double ControllerConfiguration::*;
using BoolMember = bool ControllerConfiguration::*;
using PlannerBoolMember = bool PlannerConfiguration::*;

std::runtime_error errorAt(
    const std::string& source_name,
    std::size_t line_number,
    const std::string& message) {
  std::ostringstream output;
  output << source_name;
  if (line_number != 0) {
    output << ':' << line_number;
  }
  output << ": " << message;
  return std::runtime_error(output.str());
}

std::vector<std::string> tokensFrom(std::string line) {
  const std::size_t comment = line.find('#');
  if (comment != std::string::npos) {
    line.erase(comment);
  }

  std::istringstream input(line);
  std::vector<std::string> tokens;
  std::string token;
  while (input >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

double parseDouble(
    const std::string& token,
    const std::string& source_name,
    std::size_t line_number) {
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(token, &consumed);
  } catch (const std::exception&) {
    throw errorAt(source_name, line_number, "invalid number '" + token + "'");
  }
  if (consumed != token.size() || !std::isfinite(value)) {
    throw errorAt(source_name, line_number, "invalid number '" + token + "'");
  }
  return value;
}

int parseInt(
    const std::string& token,
    const std::string& source_name,
    std::size_t line_number) {
  std::size_t consumed = 0;
  long long value = 0;
  try {
    value = std::stoll(token, &consumed);
  } catch (const std::exception&) {
    throw errorAt(source_name, line_number, "invalid integer '" + token + "'");
  }
  if (consumed != token.size() ||
      value < std::numeric_limits<int>::min() ||
      value > std::numeric_limits<int>::max()) {
    throw errorAt(source_name, line_number, "invalid integer '" + token + "'");
  }
  return static_cast<int>(value);
}

bool parseFlag(
    const std::string& token,
    const std::string& source_name,
    std::size_t line_number) {
  if (token == "0") {
    return false;
  }
  if (token == "1") {
    return true;
  }
  throw errorAt(source_name, line_number, "expected boolean flag 0 or 1");
}

void requireTokenCount(
    const std::vector<std::string>& tokens,
    std::size_t expected,
    const std::string& source_name,
    std::size_t line_number) {
  if (tokens.size() != expected) {
    std::ostringstream message;
    message << "expected " << expected << " fields, found " << tokens.size();
    throw errorAt(source_name, line_number, message.str());
  }
}

void recordKey(
    std::set<std::string>& seen,
    const std::string& key,
    const std::string& source_name,
    std::size_t line_number) {
  if (!seen.insert(key).second) {
    throw errorAt(source_name, line_number, "duplicate setting '" + key + "'");
  }
}

template <typename Parser>
auto parseFile(
    const std::string& filename,
    const char* purpose,
    Parser parser) -> decltype(parser(
        std::declval<std::istream&>(), std::declval<const std::string&>())) {
  std::ifstream input(filename);
  if (!input.is_open()) {
    throw std::runtime_error(
        "cannot open " + std::string(purpose) + " file '" + filename + "'");
  }
  return parser(input, filename);
}

const std::map<std::string, DoubleMember>& doubleSettings() {
  static const std::map<std::string, DoubleMember> settings = {
      {"canSeePointEpsilon",
       &ControllerConfiguration::can_see_point_epsilon},
      {"laserScanRadianIncrement",
       &ControllerConfiguration::laser_scan_radian_increment},
      {"robotFootPrint", &ControllerConfiguration::robot_footprint},
      {"bufferForRobot", &ControllerConfiguration::robot_footprint_buffer},
      {"maxLaserRange", &ControllerConfiguration::max_laser_range},
      {"maxForwardActionBuffer",
       &ControllerConfiguration::max_forward_action_buffer},
      {"maxForwardActionSweepAngle",
       &ControllerConfiguration::max_forward_action_sweep_angle},
      {"highwayDistanceThreshold",
       &ControllerConfiguration::highway_distance_threshold},
      {"highwayTimeThreshold",
       &ControllerConfiguration::highway_time_threshold},
      {"highwayDecisionThreshold",
       &ControllerConfiguration::highway_decision_threshold},
  };
  return settings;
}

const std::map<std::string, BoolMember>& booleanSettings() {
  static const std::map<std::string, BoolMember> settings = {
      {"trailsOn", &ControllerConfiguration::trails_on},
      {"conveyorsOn", &ControllerConfiguration::conveyors_on},
      {"regionsOn", &ControllerConfiguration::regions_on},
      {"doorsOn", &ControllerConfiguration::doors_on},
      {"hallwaysOn", &ControllerConfiguration::hallways_on},
      {"barrsOn", &ControllerConfiguration::barriers_on},
      {"aStarOn", &ControllerConfiguration::a_star_on},
      {"highwaysOn", &ControllerConfiguration::highways_on},
      {"frontiersOn", &ControllerConfiguration::frontiers_on},
      {"outofhereOn", &ControllerConfiguration::out_of_here_on},
      {"doorwayOn", &ControllerConfiguration::doorway_on},
      {"findawayOn", &ControllerConfiguration::find_a_way_on},
      {"behindOn", &ControllerConfiguration::behind_on},
      {"dontgobackOn", &ControllerConfiguration::dont_go_back_on},
  };
  return settings;
}

const std::map<std::string, PlannerBoolMember>& plannerSettings() {
  static const std::map<std::string, PlannerBoolMember> settings = {
      {"distance", &PlannerConfiguration::distance},
      {"smooth", &PlannerConfiguration::smooth},
      {"novel", &PlannerConfiguration::novel},
      {"density", &PlannerConfiguration::density},
      {"risk", &PlannerConfiguration::risk},
      {"flow", &PlannerConfiguration::flow},
      {"combined", &PlannerConfiguration::combined},
      {"CUSUM", &PlannerConfiguration::cusum},
      {"discount", &PlannerConfiguration::discount},
      {"explore", &PlannerConfiguration::explore},
      {"spatial", &PlannerConfiguration::spatial},
      {"hallwayer", &PlannerConfiguration::hallwayer},
      {"trailer", &PlannerConfiguration::trailer},
      {"barrier", &PlannerConfiguration::barrier},
      {"conveys", &PlannerConfiguration::conveys},
      {"safe", &PlannerConfiguration::safe},
      {"skeleton", &PlannerConfiguration::skeleton},
      {"hallwayskel", &PlannerConfiguration::hallway_skeleton},
  };
  return settings;
}

void validateControllerConfiguration(
    const ControllerConfiguration& configuration,
    const std::string& source_name) {
  if (configuration.task_decision_limit <= 0) {
    throw errorAt(source_name, 0, "decisionlimit must be greater than zero");
  }
  if (configuration.plan_limit <= 0) {
    throw errorAt(source_name, 0, "planLimit must be greater than zero");
  }
  if (configuration.move_actions.empty() ||
      configuration.rotate_actions.empty()) {
    throw errorAt(source_name, 0, "move and rotate actions must not be empty");
  }
  if (configuration.move_actions.size() > 300 ||
      configuration.rotate_actions.size() > 300) {
    throw errorAt(source_name, 0, "action lists may contain at most 300 values");
  }

  const auto validate_actions = [&source_name](
      const std::vector<double>& actions,
      const std::string& name) {
    if (!std::all_of(actions.begin(), actions.end(), [](double value) {
        return std::isfinite(value) && value > 0.0;
      })) {
      throw errorAt(
        source_name, 0,
        name + " actions must contain only finite positive values");
    }
    if (!std::is_sorted(actions.begin(), actions.end()) ||
        std::adjacent_find(actions.begin(), actions.end()) != actions.end()) {
      throw errorAt(
        source_name, 0, name + " actions must be strictly increasing");
    }
  };
  validate_actions(configuration.move_actions, "move");
  validate_actions(configuration.rotate_actions, "rotate");

  if (configuration.can_see_point_epsilon < 0.0 ||
      configuration.laser_scan_radian_increment <= 0.0 ||
      configuration.robot_footprint <= 0.0 ||
      configuration.robot_footprint_buffer < 0.0 ||
      configuration.max_laser_range <= 0.0 ||
      configuration.max_forward_action_buffer < 0.0 ||
      configuration.max_forward_action_sweep_angle <= 0.0 ||
      configuration.max_forward_action_sweep_angle > std::acos(-1.0) ||
      configuration.highway_distance_threshold < 0.0 ||
      configuration.highway_time_threshold < 0.0 ||
      configuration.highway_decision_threshold < 0.0) {
    throw errorAt(
      source_name, 0,
      "safety thresholds must be finite and within their documented ranges");
  }
  if (configuration.a_star_on &&
      !configuration.planners.skeleton &&
      !configuration.planners.hallway_skeleton) {
    throw errorAt(
      source_name, 0,
      "aStarOn requires skeleton or hallwayskel planner construction");
  }
}

void normalizeLegacyActions(std::vector<double>& actions)
{
  if (!actions.empty() && actions.front() == 0.0) {
    actions.erase(actions.begin());
  }
}

void validateMapContents(
    const std::string& map_file,
    const MapDimensions& dimensions)
{
  std::ifstream input(map_file);
  if (!input.is_open()) {
    throw std::runtime_error("cannot open map file '" + map_file + "'");
  }
  const std::string xml{
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>()};
  const std::regex coordinate(
    R"coordinate(p_([xy])\s*=\s*"([^"]+)")coordinate");
  for (std::sregex_iterator it(xml.begin(), xml.end(), coordinate), end;
      it != end; ++it) {
    const double value = parseDouble((*it)[2].str(), map_file, 0);
    const double upper =
      (*it)[1].str() == "x" ? dimensions.length : dimensions.height;
    if (value < 0.0 || value > upper) {
      throw std::runtime_error(
        "map coordinate " + (*it)[1].str() + "=" +
        (*it)[2].str() + " lies outside configured dimensions");
    }
  }
}

}  // namespace

ControllerConfiguration parseControllerConfiguration(
    std::istream& input,
    const std::string& source_name) {
  ControllerConfiguration configuration;
  std::set<std::string> seen;
  std::string line;
  std::size_t line_number = 0;

  while (std::getline(input, line)) {
    ++line_number;
    const std::vector<std::string> tokens = tokensFrom(line);
    if (tokens.empty()) {
      continue;
    }

    const std::string& key = tokens.front();
    recordKey(seen, key, source_name, line_number);

    if (key == "move" || key == "rotate") {
      if (tokens.size() < 2) {
        throw errorAt(
            source_name, line_number, key + " requires at least one action");
      }
      std::vector<double>& actions =
          key == "move" ? configuration.move_actions
                        : configuration.rotate_actions;
      for (std::size_t index = 1; index < tokens.size(); ++index) {
        actions.push_back(
            parseDouble(tokens[index], source_name, line_number));
      }
      continue;
    }

    requireTokenCount(tokens, 2, source_name, line_number);
    const auto double_setting = doubleSettings().find(key);
    if (double_setting != doubleSettings().end()) {
      configuration.*(double_setting->second) =
          parseDouble(tokens[1], source_name, line_number);
      continue;
    }
    const auto boolean_setting = booleanSettings().find(key);
    if (boolean_setting != booleanSettings().end()) {
      configuration.*(boolean_setting->second) =
          parseFlag(tokens[1], source_name, line_number);
      continue;
    }
    const auto planner_setting = plannerSettings().find(key);
    if (planner_setting != plannerSettings().end()) {
      configuration.planners.*(planner_setting->second) =
          parseFlag(tokens[1], source_name, line_number);
      continue;
    }
    if (key == "decisionlimit") {
      configuration.task_decision_limit =
          parseInt(tokens[1], source_name, line_number);
      continue;
    }
    if (key == "planLimit") {
      configuration.plan_limit = parseInt(tokens[1], source_name, line_number);
      continue;
    }
    throw errorAt(source_name, line_number, "unknown setting '" + key + "'");
  }

  std::set<std::string> required = {"move", "rotate", "decisionlimit", "planLimit"};
  for (const auto& setting : doubleSettings()) {
    required.insert(setting.first);
  }
  for (const auto& setting : booleanSettings()) {
    required.insert(setting.first);
  }
  for (const auto& setting : plannerSettings()) {
    required.insert(setting.first);
  }
  for (const std::string& key : required) {
    if (seen.count(key) == 0) {
      throw errorAt(source_name, 0, "missing required setting '" + key + "'");
    }
  }

  normalizeLegacyActions(configuration.move_actions);
  normalizeLegacyActions(configuration.rotate_actions);
  validateControllerConfiguration(configuration, source_name);
  return configuration;
}

MapDimensions parseMapDimensions(
    std::istream& input,
    const std::string& source_name) {
  MapDimensions dimensions;
  bool found = false;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const std::vector<std::string> tokens = tokensFrom(line);
    if (tokens.empty()) {
      continue;
    }
    if (found) {
      throw errorAt(source_name, line_number, "expected a single dimensions row");
    }
    requireTokenCount(tokens, 3, source_name, line_number);
    dimensions.length = parseInt(tokens[0], source_name, line_number);
    dimensions.height = parseInt(tokens[1], source_name, line_number);
    dimensions.granularity =
        parseDouble(tokens[2], source_name, line_number);
    found = true;
  }
  if (!found) {
    throw errorAt(source_name, 0, "missing map dimensions");
  }
  if (dimensions.length <= 0 || dimensions.height <= 0 ||
      dimensions.granularity <= 0.0) {
    throw errorAt(
        source_name, 0, "map dimensions and granularity must be positive");
  }
  return dimensions;
}

std::vector<AdvisorConfiguration> parseAdvisorConfigurations(
    std::istream& input,
    const std::string& source_name) {
  std::vector<AdvisorConfiguration> advisors;
  std::set<std::string> names;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const std::vector<std::string> tokens = tokensFrom(line);
    if (tokens.empty()) {
      continue;
    }
    requireTokenCount(tokens, 8, source_name, line_number);

    AdvisorConfiguration advisor;
    advisor.name = tokens[0];
    advisor.description = tokens[1];
    if (!names.insert(advisor.name).second) {
      throw errorAt(
          source_name, line_number, "duplicate advisor '" + advisor.name + "'");
    }
    if (tokens[2] == "t") {
      advisor.active = true;
    } else if (tokens[2] == "f") {
      advisor.active = false;
    } else {
      throw errorAt(
          source_name, line_number, "advisor state must be 't' or 'f'");
    }
    advisor.weight = parseDouble(tokens[3], source_name, line_number);
    for (std::size_t index = 0; index < advisor.parameters.size(); ++index) {
      advisor.parameters[index] =
          parseDouble(tokens[index + 4], source_name, line_number);
    }
    advisors.push_back(std::move(advisor));
  }
  if (advisors.empty()) {
    throw errorAt(source_name, 0, "at least one advisor is required");
  }
  return advisors;
}

std::vector<TaskConfiguration> parseTaskConfigurations(
    std::istream& input,
    const std::string& source_name) {
  std::vector<TaskConfiguration> tasks;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const std::vector<std::string> tokens = tokensFrom(line);
    if (tokens.empty()) {
      continue;
    }
    requireTokenCount(tokens, 2, source_name, line_number);
    TaskConfiguration task;
    task.x = parseDouble(tokens[0], source_name, line_number);
    task.y = parseDouble(tokens[1], source_name, line_number);
    tasks.push_back(task);
  }
  if (tasks.empty()) {
    throw errorAt(source_name, 0, "at least one task is required");
  }
  return tasks;
}

Configuration loadConfiguration(const ConfigurationFiles& files) {
  Configuration configuration;
  configuration.controller = parseFile(
      files.parameters, "parameter", parseControllerConfiguration);
  configuration.map_dimensions =
      parseFile(files.dimensions, "dimensions", parseMapDimensions);
  configuration.advisors =
      parseFile(files.advisors, "advisor", parseAdvisorConfigurations);
  configuration.tasks =
      parseFile(files.tasks, "task", parseTaskConfigurations);

  configuration.map_file = files.map;
  validateConfiguration(configuration);
  return configuration;
}

Configuration loadStructuredConfiguration(
    ControllerConfiguration controller,
    MapDimensions map_dimensions,
    std::vector<AdvisorConfiguration> advisors,
    const std::string& tasks_file,
    const std::string& map_file) {
  Configuration configuration;
  configuration.controller = std::move(controller);
  configuration.map_dimensions = map_dimensions;
  configuration.advisors = std::move(advisors);
  configuration.tasks =
      parseFile(tasks_file, "task", parseTaskConfigurations);
  configuration.map_file = map_file;
  validateConfiguration(configuration);
  return configuration;
}

void validateConfiguration(const Configuration& configuration) {
  validateControllerConfiguration(configuration.controller, "configuration");
  if (configuration.map_dimensions.length <= 0 ||
      configuration.map_dimensions.height <= 0 ||
      !std::isfinite(configuration.map_dimensions.granularity) ||
      configuration.map_dimensions.granularity <= 0.0) {
    throw std::runtime_error(
      "configuration: map dimensions and granularity must be positive");
  }
  if (configuration.advisors.empty()) {
    throw std::runtime_error("configuration: at least one advisor is required");
  }
  if (std::none_of(
      configuration.advisors.begin(),
      configuration.advisors.end(),
      [](const AdvisorConfiguration& advisor) { return advisor.active; })) {
    throw std::runtime_error(
      "configuration: at least one decision-producing advisor must be active");
  }
  std::set<std::string> advisor_names;
  for (const AdvisorConfiguration& advisor : configuration.advisors) {
    if (advisor.name.empty()) {
      throw std::runtime_error("configuration: advisor names must not be empty");
    }
    if (!advisor_names.insert(advisor.name).second) {
      throw std::runtime_error(
        "configuration: duplicate advisor '" + advisor.name + "'");
    }
    if (!std::isfinite(advisor.weight) || advisor.weight < 0.0 ||
        !std::all_of(
          advisor.parameters.begin(),
          advisor.parameters.end(),
          [](double value) { return std::isfinite(value); })) {
      throw std::runtime_error(
        "configuration: advisor '" + advisor.name +
        "' has an invalid weight or parameter");
    }
  }
  if (configuration.tasks.empty()) {
    throw std::runtime_error("configuration: at least one task is required");
  }
  for (const TaskConfiguration& task : configuration.tasks) {
    if (!std::isfinite(task.x) || !std::isfinite(task.y) ||
        task.x < 0.0 || task.y < 0.0 ||
        task.x > configuration.map_dimensions.length ||
        task.y > configuration.map_dimensions.height) {
      throw std::runtime_error(
        "configuration: task coordinate lies outside configured dimensions");
    }
  }
  validateMapContents(configuration.map_file, configuration.map_dimensions);
}

}  // namespace config
}  // namespace semaforr
