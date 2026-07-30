#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <regex>
#include <semaforr/config/Configuration.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace semaforr::config {
namespace {

std::uint64_t fnv1a(std::string_view value) {
  std::uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char byte : value) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::runtime_error errorAt(const std::string& source, std::size_t line,
                           const std::string& message) {
  return std::runtime_error(
      source + (line == 0U ? ": " : ":" + std::to_string(line) + ": ") +
      message);
}

double parseFiniteDouble(const std::string& token, const std::string& source,
                         std::size_t line) {
  std::size_t consumed = 0U;
  double value = 0.0;
  try {
    value = std::stod(token, &consumed);
  } catch (const std::exception&) {
    throw errorAt(source, line, "invalid number '" + token + "'");
  }
  if (consumed != token.size() || !std::isfinite(value)) {
    throw errorAt(source, line, "invalid number '" + token + "'");
  }
  return value;
}

std::vector<TaskConfiguration> parseTasks(const std::string& filename) {
  std::ifstream input(filename);
  if (!input.is_open()) {
    throw std::runtime_error("cannot open task file '" + filename + "'");
  }
  std::vector<TaskConfiguration> tasks;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(input, line)) {
    ++line_number;
    line = line.substr(0U, line.find('#'));
    std::istringstream row(line);
    std::string x;
    std::string y;
    std::string extra;
    if (!(row >> x)) {
      continue;
    }
    if (!(row >> y) || row >> extra) {
      throw errorAt(filename, line_number,
                    "task row must contain exactly x and y");
    }
    tasks.push_back({parseFiniteDouble(x, filename, line_number),
                     parseFiniteDouble(y, filename, line_number)});
  }
  if (tasks.empty()) {
    throw errorAt(filename, 0U, "at least one task is required");
  }
  return tasks;
}

void validateActions(const std::vector<double>& actions,
                     const std::string& name) {
  if (actions.empty()) {
    throw std::runtime_error("configuration: " + name +
                             " actions must not be empty");
  }
  if (actions.size() > 300U) {
    throw std::runtime_error("configuration: " + name +
                             " actions may contain at most 300 values");
  }
  if (!std::all_of(actions.begin(), actions.end(), [](double value) {
        return std::isfinite(value) && value > 0.0;
      })) {
    throw std::runtime_error(
        "configuration: " + name +
        " actions must contain only finite positive values");
  }
  if (!std::is_sorted(actions.begin(), actions.end()) ||
      std::adjacent_find(actions.begin(), actions.end()) != actions.end()) {
    throw std::runtime_error("configuration: " + name +
                             " actions must be strictly increasing");
  }
}

void validateNavigation(const NavigationConfiguration& configuration) {
  if (configuration.task_decision_limit <= 0) {
    throw std::runtime_error(
        "configuration: mission.decision_limit must be positive");
  }
  validateActions(configuration.move_actions, "move");
  validateActions(configuration.rotate_actions, "rotate");

  const std::array<double, 7U> safety{
      configuration.can_see_point_epsilon,
      configuration.laser_scan_radian_increment,
      configuration.robot_footprint,
      configuration.robot_footprint_buffer,
      configuration.max_laser_range,
      configuration.max_forward_action_buffer,
      configuration.max_forward_action_sweep_angle};
  if (!std::all_of(safety.begin(), safety.end(),
                   [](double value) { return std::isfinite(value); }) ||
      configuration.can_see_point_epsilon < 0.0 ||
      configuration.laser_scan_radian_increment <= 0.0 ||
      configuration.robot_footprint <= 0.0 ||
      configuration.robot_footprint_buffer < 0.0 ||
      configuration.max_laser_range <= 0.0 ||
      configuration.max_forward_action_buffer < 0.0 ||
      configuration.max_forward_action_sweep_angle <= 0.0 ||
      configuration.max_forward_action_sweep_angle > std::acos(-1.0)) {
    throw std::runtime_error(
        "configuration: safety thresholds must be finite and within "
        "their documented ranges");
  }

  if (configuration.a_star_on && !configuration.planners.skeleton) {
    throw std::runtime_error(
        "configuration: features.astar requires the skeleton planner");
  }
  const bool has_crowd_planner = configuration.planners.density ||
                                 configuration.planners.risk ||
                                 configuration.planners.flow;
  if (has_crowd_planner && !configuration.planners.skeleton) {
    throw std::runtime_error(
        "configuration: density, risk, and flow planners require skeleton");
  }
  if (has_crowd_planner && !configuration.crowd_learning.enabled) {
    throw std::runtime_error(
        "configuration: crowd-cost planners require social.learning.enabled");
  }

  const auto& crowd = configuration.crowd_learning;
  const bool known_estimator =
      crowd.estimator == "count_exposure" || crowd.estimator == "count" ||
      crowd.estimator == "discounted_count" || crowd.estimator == "discount" ||
      crowd.estimator == "cusum" || crowd.estimator == "bayes_cusum" ||
      crowd.estimator == "thompson" || crowd.estimator == "count_thompson";
  if (crowd.enabled &&
      (crowd.frame.empty() || !known_estimator ||
       !std::isfinite(crowd.resolution_m) || crowd.resolution_m <= 0.0 ||
       !std::isfinite(crowd.origin_x_m) || !std::isfinite(crowd.origin_y_m) ||
       !std::isfinite(crowd.discount_factor) || crowd.discount_factor <= 0.0 ||
       crowd.discount_factor > 1.0 ||
       !std::isfinite(crowd.minimum_update_period_s) ||
       crowd.minimum_update_period_s < 0.0 ||
       !std::isfinite(crowd.encounter_radius_m) ||
       crowd.encounter_radius_m <= 0.0 ||
       !std::isfinite(crowd.minimum_flow_speed_mps) ||
       crowd.minimum_flow_speed_mps < 0.0 ||
       !std::isfinite(crowd.confidence_exposures) ||
       crowd.confidence_exposures <= 0.0 ||
       !std::isfinite(crowd.cusum_increase) || crowd.cusum_increase <= 0.0 ||
       !std::isfinite(crowd.cusum_decrease) || crowd.cusum_decrease >= 0.0 ||
       !std::isfinite(crowd.cusum_threshold) || crowd.cusum_threshold <= 0.0)) {
    throw std::runtime_error(
        "configuration: crowd learning has an unknown estimator or "
        "an out-of-range value");
  }
}

void validateMap(const std::string& map_file, const MapDimensions& dimensions) {
  std::ifstream input(map_file);
  if (!input.is_open()) {
    throw std::runtime_error("cannot open map file '" + map_file + "'");
  }
  const std::string xml{std::istreambuf_iterator<char>(input),
                        std::istreambuf_iterator<char>()};
  const std::regex coordinate(
      R"coordinate(p_([xy])\s*=\s*"([^"]+)")coordinate");
  for (std::sregex_iterator iterator(xml.begin(), xml.end(), coordinate), end;
       iterator != end; ++iterator) {
    const double value = parseFiniteDouble((*iterator)[2].str(), map_file, 0U);
    const double upper = (*iterator)[1].str() == "x"
                             ? static_cast<double>(dimensions.length)
                             : static_cast<double>(dimensions.height);
    if (value < 0.0 || value > upper) {
      throw std::runtime_error("map coordinate " + (*iterator)[1].str() + "=" +
                               (*iterator)[2].str() +
                               " lies outside configured dimensions");
    }
  }
}

}  // namespace

std::string_view toString(AblationProfile profile) noexcept {
  switch (profile) {
    case AblationProfile::Full:
      return "full";
    case AblationProfile::TierOneOnly:
      return "tier1_only";
    case AblationProfile::TierOneTierThree:
      return "tier1_tier3";
    case AblationProfile::TierThreeOnly:
      return "tier3_only";
    case AblationProfile::TierOneTierTwoTierThree:
      return "tier1_tier2_tier3";
    case AblationProfile::NoInitialExploration:
      return "no_initial_exploration";
    case AblationProfile::NoOpportunisticExploration:
      return "no_opportunistic_exploration";
    case AblationProfile::NoSpatialModel:
      return "no_spatial_model";
    case AblationProfile::NoSocial:
      return "no_social";
    case AblationProfile::Custom:
      return "custom";
  }
  return "custom";
}

AblationProfile ablationProfileFromString(const std::string& value) {
  for (const AblationProfile profile :
       {AblationProfile::Full, AblationProfile::TierOneOnly,
        AblationProfile::TierOneTierThree, AblationProfile::TierThreeOnly,
        AblationProfile::TierOneTierTwoTierThree,
        AblationProfile::NoInitialExploration,
        AblationProfile::NoOpportunisticExploration,
        AblationProfile::NoSpatialModel, AblationProfile::NoSocial,
        AblationProfile::Custom}) {
    if (value == toString(profile)) return profile;
  }
  throw std::runtime_error("unknown experiment profile '" + value + "'");
}

void applyAblationProfile(Configuration& configuration) {
  auto& experiment = configuration.experiment;
  switch (experiment.profile) {
    case AblationProfile::Full:
    case AblationProfile::TierOneTierTwoTierThree:
      experiment.tiers = {true, true, true};
      break;
    case AblationProfile::TierOneOnly:
      experiment.tiers = {true, false, false};
      break;
    case AblationProfile::TierOneTierThree:
      experiment.tiers = {true, false, true};
      break;
    case AblationProfile::TierThreeOnly:
      experiment.tiers = {false, false, true};
      break;
    case AblationProfile::NoInitialExploration:
      experiment.initial_exploration = {};
      break;
    case AblationProfile::NoOpportunisticExploration:
      experiment.opportunistic_exploration = false;
      break;
    case AblationProfile::NoSpatialModel:
      configuration.navigation.trails_on = false;
      configuration.navigation.conveyors_on = false;
      configuration.navigation.regions_on = false;
      configuration.navigation.doors_on = false;
      configuration.navigation.hallways_on = false;
      configuration.navigation.barriers_on = false;
      configuration.navigation.a_star_on = false;
      configuration.navigation.planners = {};
      break;
    case AblationProfile::NoSocial:
      experiment.social_enabled = false;
      configuration.navigation.crowd_learning.enabled = false;
      configuration.navigation.planners.density = false;
      configuration.navigation.planners.risk = false;
      configuration.navigation.planners.flow = false;
      for (auto& advisor : configuration.advisors) {
        if (advisor.name == "social_navigation" ||
            advisor.name == "crowd_avoid" || advisor.name == "risk_avoid" ||
            advisor.name == "flow_follow") {
          advisor.active = false;
        }
      }
      break;
    case AblationProfile::Custom:
      break;
  }
}

std::string configurationFingerprint(const Configuration& configuration) {
  std::ostringstream canonical;
  canonical << std::setprecision(17) << toString(configuration.experiment.profile)
            << '|' << configuration.experiment.tiers.tier_one << '|'
            << configuration.experiment.tiers.tier_two << '|'
            << configuration.experiment.tiers.tier_three << '|'
            << configuration.experiment.initial_exploration.enabled << '|'
            << configuration.experiment.initial_exploration.observation_budget
            << '|' << configuration.experiment.opportunistic_exploration << '|'
            << configuration.experiment.social_enabled << '|'
            << configuration.map_file << '|' << configuration.map_dimensions.length
            << '|' << configuration.map_dimensions.height << '|'
            << configuration.map_dimensions.granularity;
  for (const double value : configuration.navigation.move_actions)
    canonical << "|m:" << value;
  for (const double value : configuration.navigation.rotate_actions)
    canonical << "|r:" << value;
  canonical << '|' << configuration.navigation.trails_on << '|'
            << configuration.navigation.conveyors_on << '|'
            << configuration.navigation.regions_on << '|'
            << configuration.navigation.doors_on << '|'
            << configuration.navigation.hallways_on << '|'
            << configuration.navigation.barriers_on << '|'
            << configuration.navigation.a_star_on << '|'
            << configuration.navigation.crowd_learning.enabled;
  for (const auto& advisor : configuration.advisors)
    canonical << "|a:" << advisor.name << ':' << advisor.active << ':'
              << advisor.weight;
  for (const auto& task : configuration.tasks)
    canonical << "|t:" << task.x << ':' << task.y;
  std::ostringstream encoded;
  encoded << std::hex << std::setw(16) << std::setfill('0')
          << fnv1a(canonical.str());
  return encoded.str();
}

std::vector<std::string> componentManifest(
    const Configuration& configuration) {
  std::vector<std::string> result{"hard_safety:obstacle_clearance",
                                  "phase:target_navigation"};
  const auto& experiment = configuration.experiment;
  if (experiment.initial_exploration.enabled)
    result.push_back("phase:initial_exploration");
  if (experiment.tiers.tier_one) result.push_back("tier:tier_one");
  if (experiment.tiers.tier_two) result.push_back("tier:tier_two");
  if (experiment.tiers.tier_three) result.push_back("tier:tier_three");
  const auto add_feature = [&result](bool enabled, std::string name) {
    if (enabled) result.push_back("spatial:" + std::move(name));
  };
  add_feature(configuration.navigation.trails_on, "trails");
  add_feature(configuration.navigation.conveyors_on, "conveyors");
  add_feature(configuration.navigation.regions_on, "regions");
  add_feature(configuration.navigation.doors_on, "doors");
  add_feature(configuration.navigation.hallways_on, "hallways");
  add_feature(configuration.navigation.barriers_on, "barriers");
  for (const auto& advisor : configuration.advisors)
    if (advisor.active) result.push_back("advisor:" + advisor.name);
  std::sort(result.begin(), result.end());
  return result;
}

Configuration loadStructuredConfiguration(
    NavigationConfiguration navigation, MapDimensions map_dimensions,
    std::vector<AdvisorConfiguration> advisors, const std::string& tasks_file,
    const std::string& map_file) {
  Configuration configuration;
  configuration.navigation = std::move(navigation);
  configuration.map_dimensions = map_dimensions;
  configuration.advisors = std::move(advisors);
  configuration.tasks = parseTasks(tasks_file);
  configuration.map_file = map_file;
  applyAblationProfile(configuration);
  validateConfiguration(configuration);
  return configuration;
}

void validateConfiguration(const Configuration& configuration) {
  validateNavigation(configuration.navigation);
  if (configuration.experiment.initial_exploration.enabled &&
      configuration.experiment.initial_exploration.observation_budget == 0U) {
    throw std::runtime_error(
        "configuration: enabled initial exploration requires a positive "
        "observation budget");
  }
  if (!configuration.experiment.tiers.tier_one &&
      !configuration.experiment.tiers.tier_two &&
      !configuration.experiment.tiers.tier_three) {
    throw std::runtime_error(
        "configuration: at least one cognitive tier must be enabled");
  }
  if (configuration.map_dimensions.length <= 0 ||
      configuration.map_dimensions.height <= 0 ||
      !std::isfinite(configuration.map_dimensions.granularity) ||
      configuration.map_dimensions.granularity <= 0.0) {
    throw std::runtime_error(
        "configuration: map dimensions and granularity must be positive");
  }

  if (configuration.experiment.tiers.tier_three &&
      configuration.advisors.empty()) {
    throw std::runtime_error("configuration: at least one advisor is required");
  }
  if (configuration.experiment.tiers.tier_three &&
      std::none_of(
          configuration.advisors.begin(), configuration.advisors.end(),
          [](const AdvisorConfiguration& advisor) { return advisor.active; })) {
    throw std::runtime_error(
        "configuration: at least one decision-producing advisor must be "
        "active");
  }
  const std::set<std::string> registered_advisors{
      "goal_progress",      "goal_progress_linear", "clearance",
      "clearance_rotation", "exploration",          "social_navigation",
      "crowd_avoid",        "risk_avoid",           "flow_follow"};
  std::set<std::string> names;
  for (const AdvisorConfiguration& advisor : configuration.advisors) {
    if (!names.insert(advisor.name).second) {
      throw std::runtime_error("configuration: duplicate advisor '" +
                               advisor.name + "'");
    }
    if (!registered_advisors.contains(advisor.name)) {
      throw std::runtime_error("configuration: unknown advisor '" +
                               advisor.name + "'");
    }
    if (!std::isfinite(advisor.weight) || advisor.weight < 0.0 ||
        !std::all_of(advisor.parameters.begin(), advisor.parameters.end(),
                     [](double value) { return std::isfinite(value); })) {
      throw std::runtime_error("configuration: advisor '" + advisor.name +
                               "' has an invalid weight or parameter");
    }
  }

  if (configuration.tasks.empty()) {
    throw std::runtime_error("configuration: at least one task is required");
  }
  for (const TaskConfiguration& task : configuration.tasks) {
    if (!std::isfinite(task.x) || !std::isfinite(task.y) || task.x < 0.0 ||
        task.y < 0.0 || task.x > configuration.map_dimensions.length ||
        task.y > configuration.map_dimensions.height) {
      throw std::runtime_error(
          "configuration: task coordinate lies outside configured dimensions");
    }
  }
  validateMap(configuration.map_file, configuration.map_dimensions);
}

}  // namespace semaforr::config
