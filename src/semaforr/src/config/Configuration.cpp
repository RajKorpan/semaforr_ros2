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
      experiment.tiers.tier_one = true;
      experiment.tiers.tier_two = true;
      experiment.tiers.tier_three = true;
      break;
    case AblationProfile::TierOneOnly:
      experiment.tiers.tier_one = true;
      experiment.tiers.tier_two = false;
      experiment.tiers.tier_three = false;
      experiment.reactive_exploration_enabled = false;
      break;
    case AblationProfile::TierOneTierThree:
      experiment.tiers.tier_one = true;
      experiment.tiers.tier_two = false;
      experiment.tiers.tier_three = true;
      experiment.reactive_exploration_enabled = false;
      break;
    case AblationProfile::TierThreeOnly:
      experiment.tiers.tier_one = false;
      experiment.tiers.tier_two = false;
      experiment.tiers.tier_three = true;
      experiment.reactive_exploration_enabled = false;
      break;
    case AblationProfile::NoInitialExploration:
      experiment.initial_exploration = {};
      break;
    case AblationProfile::NoOpportunisticExploration:
      experiment.opportunistic_exploration = false;
      for (auto& advisor : configuration.advisors) {
        if (advisor.name == "exploration" || advisor.name == "novelty" ||
            advisor.name == "curiosity" ||
            advisor.name == "spatial_learner" ||
            advisor.name == "enfilade" || advisor.name == "visual_scan")
          advisor.active = false;
      }
      break;
    case AblationProfile::NoSpatialModel:
      configuration.navigation.trails_on = false;
      configuration.navigation.conveyors_on = false;
      configuration.navigation.regions_on = false;
      configuration.navigation.doors_on = false;
      configuration.navigation.hallways_on = false;
      configuration.navigation.barriers_on = false;
      configuration.navigation.a_star_on = false;
      configuration.navigation.known_grid_on = false;
      configuration.navigation.inclusion_grid_on = false;
      configuration.navigation.highways_on = false;
      configuration.navigation.circumstances_on = false;
      experiment.reactive_exploration_enabled = false;
      configuration.navigation.planners.distance = false;
      configuration.navigation.planners.density = false;
      configuration.navigation.planners.risk = false;
      configuration.navigation.planners.flow = false;
      configuration.navigation.planners.skeleton = false;
      configuration.navigation.planners.highway = false;
      break;
    case AblationProfile::NoSocial:
      experiment.social_enabled = false;
      experiment.social = {};
      experiment.social.enabled = false;
      experiment.social.observations = false;
      experiment.social.learning = false;
      experiment.social.advisors = false;
      experiment.social.planners = false;
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
  if (!experiment.social.enabled || !experiment.social_enabled) {
    experiment.social_enabled = false;
    experiment.social.enabled = false;
    experiment.social.observations = false;
    experiment.social.learning = false;
    experiment.social.advisors = false;
    experiment.social.planners = false;
    configuration.navigation.crowd_learning.enabled = false;
    configuration.navigation.planners.density = false;
    configuration.navigation.planners.risk = false;
    configuration.navigation.planners.flow = false;
    for (auto& advisor : configuration.advisors) {
      if (advisor.name == "social_navigation" ||
          advisor.name == "crowd_avoid" || advisor.name == "risk_avoid" ||
          advisor.name == "flow_follow")
        advisor.active = false;
    }
  }
}

std::string configurationFingerprint(const Configuration& configuration) {
  std::ostringstream canonical;
  canonical << std::setprecision(17) << toString(configuration.experiment.profile)
            << '|' << configuration.experiment.random_seed
            << '|' << configuration.experiment.tiers.tier_one << '|'
            << configuration.experiment.tiers.tier_two << '|'
            << configuration.experiment.tiers.tier_three << '|'
            << configuration.experiment.initial_exploration.enabled << '|'
            << configuration.experiment.initial_exploration.observation_budget
            << '|' << configuration.experiment.initial_exploration.strategy
            << '|' << configuration.experiment.initial_exploration.time_limit_s
            << '|'
            << configuration.experiment.initial_exploration.decision_budget
            << '|'
            << configuration.experiment.initial_exploration.minimum_clearance_m
            << '|'
            << configuration.experiment.initial_exploration
                   .heading_tolerance_rad
            << '|'
            << configuration.experiment.initial_exploration
                   .candidate_completion_distance_m
            << '|'
            << configuration.experiment.initial_exploration
                   .cue_similarity_radius_m
            << '|'
            << configuration.experiment.initial_exploration
                   .passage_grid_resolution_m
            << '|'
            << configuration.experiment.initial_exploration
                   .minimum_bundle_beams
            << '|' << configuration.experiment.reactive_exploration_enabled
            << '|' << configuration.experiment.opportunistic_exploration << '|'
            << configuration.experiment.social.enabled << '|'
            << configuration.experiment.social.observations << '|'
            << configuration.experiment.social.learning << '|'
            << configuration.experiment.social.advisors << '|'
            << configuration.experiment.social.planners << '|'
            << configuration.experiment.safety_envelope.enabled << '|'
            << configuration.experiment.safety_envelope
                   .sensor_freshness_timeout_s
            << '|'
            << configuration.map_file << '|' << configuration.map_dimensions.length
            << '|' << configuration.map_dimensions.height << '|'
            << configuration.map_dimensions.granularity;
  for (const double value : configuration.navigation.move_actions)
    canonical << "|m:" << value;
  for (const double value : configuration.navigation.rotate_actions)
    canonical << "|r:" << value;
  for (const auto& rule : configuration.experiment.tiers.tier_one_rules)
    canonical << "|t1:" << rule;
  for (const auto& planner : configuration.experiment.tiers.reactive_planners)
    canonical << "|rx:" << planner;
  canonical << '|' << configuration.navigation.trails_on << '|'
            << configuration.navigation.conveyors_on << '|'
            << configuration.navigation.regions_on << '|'
            << configuration.navigation.doors_on << '|'
            << configuration.navigation.hallways_on << '|'
            << configuration.navigation.barriers_on << '|'
            << configuration.navigation.a_star_on << '|'
            << configuration.navigation.known_grid_on << '|'
            << configuration.navigation.inclusion_grid_on << '|'
            << configuration.navigation.highways_on << '|'
            << configuration.navigation.circumstances_on << '|'
            << configuration.navigation.planners.distance << '|'
            << configuration.navigation.planners.skeleton << '|'
            << configuration.navigation.planners.highway << '|'
            << configuration.navigation.planners.density << '|'
            << configuration.navigation.planners.risk << '|'
            << configuration.navigation.planners.flow << '|'
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
  for (const auto& rule : experiment.tiers.tier_one_rules)
    if (experiment.tiers.tier_one) result.push_back("tier1:" + rule);
  for (const auto& planner : experiment.tiers.reactive_planners)
    if (experiment.tiers.tier_one)
      result.push_back("reactive:" + planner);
  if (experiment.reactive_exploration_enabled)
    result.push_back("exploration:lle");
  const auto add_feature = [&result](bool enabled, std::string name) {
    if (enabled) result.push_back("spatial:" + std::move(name));
  };
  add_feature(configuration.navigation.trails_on, "trails");
  add_feature(configuration.navigation.conveyors_on, "conveyors");
  add_feature(configuration.navigation.regions_on, "regions");
  add_feature(configuration.navigation.doors_on, "doors");
  add_feature(configuration.navigation.hallways_on, "hallways");
  add_feature(configuration.navigation.barriers_on, "barriers");
  add_feature(configuration.navigation.known_grid_on, "known_grid");
  add_feature(configuration.navigation.inclusion_grid_on, "inclusion_grid");
  add_feature(configuration.navigation.highways_on, "highways");
  add_feature(configuration.navigation.circumstances_on, "circumstances");
  const auto add_planner = [&result](bool enabled, std::string name) {
    if (enabled) result.push_back("planner:" + std::move(name));
  };
  add_planner(configuration.navigation.planners.distance, "distance");
  add_planner(configuration.navigation.planners.skeleton, "skeleton");
  add_planner(configuration.navigation.planners.highway, "highway");
  add_planner(configuration.navigation.planners.density, "density");
  add_planner(configuration.navigation.planners.risk, "risk");
  add_planner(configuration.navigation.planners.flow, "flow");
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
  const auto& experiment = configuration.experiment;
  if (experiment.initial_exploration.enabled &&
      (experiment.initial_exploration.strategy != "hle" ||
       !std::isfinite(experiment.initial_exploration.time_limit_s) ||
       experiment.initial_exploration.time_limit_s <= 0.0 ||
       experiment.initial_exploration.decision_budget == 0U ||
       !(experiment.initial_exploration.minimum_clearance_m > 0.0) ||
       !(experiment.initial_exploration.heading_tolerance_rad > 0.0) ||
       !(experiment.initial_exploration.candidate_completion_distance_m >
         0.0) ||
       !(experiment.initial_exploration.cue_similarity_radius_m > 0.0) ||
       !(experiment.initial_exploration.passage_grid_resolution_m > 0.0) ||
       experiment.initial_exploration.minimum_bundle_beams == 0U)) {
    throw std::runtime_error(
        "configuration: HLE requires strategy 'hle' and positive typed "
        "clearance, heading, candidate, grid, bundle, time, and decision "
        "parameters; observation_budget may be zero");
  }
  if (!experiment.target_navigation.enabled && !configuration.tasks.empty())
    throw std::runtime_error(
        "configuration: target navigation cannot be disabled when mission "
        "tasks are configured");
  const std::vector<std::string> tier_one_order{
      "victory", "avoid_obstacles", "not_opposite", "enforcer", "thru",
      "behind", "out", "low_level_exploration", "forward", "precedent"};
  std::size_t previous = 0U;
  bool first_rule = true;
  std::set<std::string> configured_rules;
  for (const auto& rule : experiment.tiers.tier_one_rules) {
    const auto found =
        std::find(tier_one_order.begin(), tier_one_order.end(), rule);
    if (found == tier_one_order.end())
      throw std::runtime_error("configuration: unknown Tier-1 rule '" + rule +
                               "'");
    if (!configured_rules.insert(rule).second)
      throw std::runtime_error("configuration: duplicate Tier-1 rule '" +
                               rule + "'");
    const std::size_t position =
        static_cast<std::size_t>(found - tier_one_order.begin());
    if (!first_rule && position <= previous)
      throw std::runtime_error(
          "configuration: Tier-1 rules must preserve dissertation order");
    first_rule = false;
    previous = position;
  }
  const std::set<std::string> registered_reactive{
      "thru", "behind", "out", "low_level_exploration"};
  std::set<std::string> configured_reactive;
  for (const auto& planner : experiment.tiers.reactive_planners) {
    if (!registered_reactive.contains(planner))
      throw std::runtime_error(
          "configuration: unknown reactive planner '" + planner + "'");
    if (!configured_reactive.insert(planner).second)
      throw std::runtime_error(
          "configuration: duplicate reactive planner '" + planner + "'");
  }
  if (!experiment.safety_envelope.enabled)
    throw std::runtime_error(
        "configuration: safety.command_envelope.enabled is an invariant "
        "platform boundary and must remain true for every cognitive ablation");
  if (!std::isfinite(
          experiment.safety_envelope.sensor_freshness_timeout_s) ||
      experiment.safety_envelope.sensor_freshness_timeout_s <= 0.0)
    throw std::runtime_error(
        "configuration: safety.sensor_freshness_timeout_s must be finite "
        "and positive");
  if (experiment.reactive_exploration_enabled &&
      (!experiment.tiers.tier_one ||
       !configuration.navigation.inclusion_grid_on ||
       !experiment.tiers.tier_two ||
       !configured_reactive.contains("low_level_exploration") ||
       !configured_rules.contains("low_level_exploration") ||
       !(configuration.navigation.planners.distance ||
         configuration.navigation.planners.skeleton ||
         configuration.navigation.planners.highway ||
         configuration.navigation.planners.density ||
         configuration.navigation.planners.risk ||
         configuration.navigation.planners.flow)))
    throw std::runtime_error(
        "configuration: LLE requires Tier 1, the inclusion grid, Tier 2 replanning, "
        "'low_level_exploration' in reactive planners, and at least one "
        "enabled global replanning strategy");
  if (experiment.reactive_exploration_enabled &&
      experiment.reactive_exploration_strategy != "lle")
    throw std::runtime_error(
        "configuration: reactive exploration strategy must be 'lle'");
  if (configuration.navigation.planners.highway &&
      !configuration.navigation.highways_on)
    throw std::runtime_error(
        "configuration: HighwayPlan requires the highway graph");
  if (configuration.navigation.highways_on &&
      !experiment.initial_exploration.enabled &&
      configuration.navigation.loaded_highway_model.empty())
    throw std::runtime_error(
        "configuration: the highway graph requires HLE output or "
        "features.loaded_highway_model");
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
      "crowd_avoid", "risk_avoid", "flow_follow", "avoid_revisit",
      "prefer_regions", "prefer_highways", "prefer_doors", "follow_trails",
      "big_step", "elbow_room", "novelty", "go_around", "greedy",
      "curiosity", "enfilade", "visual_scan", "convey", "enter", "exit",
      "trailer", "unlikely", "access", "crossroads", "follow",
      "least_angle", "spatial_learner", "stay"};
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
    if (advisor.active &&
        ((advisor.name == "prefer_regions" &&
          !configuration.navigation.regions_on) ||
         (advisor.name == "prefer_highways" &&
          !configuration.navigation.highways_on) ||
         (advisor.name == "prefer_doors" &&
          !configuration.navigation.doors_on) ||
         (advisor.name == "follow_trails" &&
          !configuration.navigation.trails_on)))
      throw std::runtime_error("configuration: advisor '" + advisor.name +
                               "' requires its spatial representation");
    const bool social_advisor =
        advisor.name == "social_navigation" || advisor.name == "crowd_avoid" ||
        advisor.name == "risk_avoid" || advisor.name == "flow_follow";
    if (advisor.active && social_advisor &&
        (!experiment.social.enabled || !experiment.social.advisors))
      throw std::runtime_error(
          "configuration: social advisor '" + advisor.name +
          "' requires social.enabled and social.advisors.enabled");
  }
  const bool crowd_planner = configuration.navigation.planners.density ||
                             configuration.navigation.planners.risk ||
                             configuration.navigation.planners.flow;
  if (crowd_planner &&
      (!experiment.social.enabled || !experiment.social.planners))
    throw std::runtime_error(
        "configuration: crowd planners require social.enabled and "
        "social.planners.enabled");
  if (configuration.navigation.crowd_learning.enabled &&
      (!experiment.social.enabled || !experiment.social.learning))
    throw std::runtime_error(
        "configuration: crowd learning requires social.enabled and "
        "social.learning.enabled");

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
