#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <semaforr/config/navigation_configuration.hpp>
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
  const auto& circumstance = configuration.circumstances;
  const bool circumstance_finite =
      std::isfinite(circumstance.setting_resolution_m) &&
      std::isfinite(circumstance.setting_radius_m) &&
      std::isfinite(circumstance.assignment_confidence_threshold) &&
      std::isfinite(circumstance.similarity_l1_threshold) &&
      std::isfinite(circumstance.accuracy_threshold) &&
      std::isfinite(circumstance.action_confidence_threshold) &&
      std::isfinite(circumstance.distance_bin_base_m);
  if (!circumstance_finite || circumstance.setting_resolution_m <= 0.0 ||
      circumstance.setting_radius_m <= 0.0 ||
      circumstance.minimum_cluster_size == 0U ||
      circumstance.minimum_cluster_size > 1000000U ||
      circumstance.reclustering_threshold == 0U ||
      circumstance.reclustering_threshold > 1000000U ||
      circumstance.minimum_case_evidence == 0U ||
      circumstance.minimum_case_evidence > 1000000U ||
      circumstance.assignment_confidence_threshold < 0.0 ||
      circumstance.assignment_confidence_threshold > 1.0 ||
      circumstance.similarity_l1_threshold <= 0.0 ||
      circumstance.accuracy_threshold < 0.0 ||
      circumstance.accuracy_threshold > 1.0 ||
      circumstance.action_confidence_threshold < 0.0 ||
      circumstance.action_confidence_threshold > 1.0 ||
      circumstance.distance_bin_base_m <= 0.0 ||
      circumstance.angle_bin_count == 0U ||
      circumstance.angle_bin_count > 360U)
    throw std::runtime_error(
        "configuration: circumstance normalization, clustering, confidence, "
        "accuracy, and evidence thresholds are outside valid ranges");
  const std::set<std::string> selection_policies{
      "single", "minimum_normalized_cost", "range_vote", "pareto_then_vote",
      "shortest_valid"};
  if (!selection_policies.contains(configuration.planners.selection_policy))
    throw std::runtime_error(
        "configuration: planners.selection_policy must be single, "
        "minimum_normalized_cost, range_vote, pareto_then_vote, or "
        "shortest_valid");
  const std::size_t enabled_planner_count =
      static_cast<std::size_t>(configuration.planners.distance) +
      configuration.planners.sensor_distance +
      configuration.planners.density + configuration.planners.risk +
      configuration.planners.flow + configuration.planners.region +
      configuration.planners.hallway + configuration.planners.trail +
      configuration.planners.conveyor + configuration.planners.skeleton +
      configuration.planners.highway;
  if (configuration.planners.selection_policy == "single" &&
      enabled_planner_count != 1U)
    throw std::runtime_error(
        "configuration: selection policy 'single' requires exactly one "
        "enabled planner");
  if (configuration.planners.region && !configuration.regions_on)
    throw std::runtime_error(
        "configuration: RegionPlan requires the region representation");
  if (configuration.planners.hallway && !configuration.hallways_on)
    throw std::runtime_error(
        "configuration: HallwayPlan requires the hallway representation");
  if (configuration.planners.trail && !configuration.trails_on)
    throw std::runtime_error(
        "configuration: TrailPlan requires the trail representation");
  if (configuration.planners.conveyor && !configuration.conveyors_on)
    throw std::runtime_error(
        "configuration: ConveyorPlan requires the conveyor representation");

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

}  // namespace

std::string_view toString(BehaviorMode mode) noexcept {
  switch (mode) {
    case BehaviorMode::Compatibility:
      return "compatibility";
    case BehaviorMode::Modernized:
      return "modernized";
  }
  return "modernized";
}

BehaviorMode behaviorModeFromString(const std::string& value) {
  if (value == "compatibility") return BehaviorMode::Compatibility;
  if (value == "modernized") return BehaviorMode::Modernized;
  throw std::invalid_argument(
      "experiment.behavior_mode must be 'compatibility' or 'modernized'");
}

std::string_view toString(MapOperatingMode mode) noexcept {
  return mode == MapOperatingMode::MapEnabled ? "map_enabled" : "mapless";
}

MapOperatingMode mapOperatingModeFromString(const std::string& value) {
  if (value == "mapless") return MapOperatingMode::Mapless;
  if (value == "map_enabled") return MapOperatingMode::MapEnabled;
  throw std::invalid_argument(
      "map.mode must be 'mapless' or 'map_enabled'");
}

std::string_view toString(MapLoadFailurePolicy policy) noexcept {
  return policy == MapLoadFailurePolicy::DisableMap ? "disable_map"
                                                     : "fail_startup";
}

MapLoadFailurePolicy mapLoadFailurePolicyFromString(const std::string& value) {
  if (value == "fail_startup") return MapLoadFailurePolicy::FailStartup;
  if (value == "disable_map") return MapLoadFailurePolicy::DisableMap;
  throw std::invalid_argument(
      "map.on_load_failure must be 'fail_startup' or 'disable_map'");
}

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
    case AblationProfile::PurelyReactive:
      return "purely_reactive";
    case AblationProfile::Original:
      return "original";
    case AblationProfile::Doors:
      return "doors";
    case AblationProfile::LeastAngle:
      return "least_angle";
    case AblationProfile::Access:
      return "access";
    case AblationProfile::Tentative:
      return "tentative";
    case AblationProfile::Hallways:
      return "hallways";
    case AblationProfile::ShortestPath:
      return "shortest_path";
    case AblationProfile::CostGraph:
      return "cost_graph";
    case AblationProfile::Wander:
      return "wander";
    case AblationProfile::Deliberator:
      return "deliberator";
    case AblationProfile::ForwardOnly:
      return "forward_only";
    case AblationProfile::GlobalExploration:
      return "global_exploration";
    case AblationProfile::LocalExploration:
      return "local_exploration";
    case AblationProfile::Highway:
      return "highway";
    case AblationProfile::Circumstances:
      return "circumstances";
    case AblationProfile::Naive:
      return "naive";
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
        AblationProfile::PurelyReactive, AblationProfile::Original,
        AblationProfile::Doors, AblationProfile::LeastAngle,
        AblationProfile::Access, AblationProfile::Tentative,
        AblationProfile::Hallways, AblationProfile::ShortestPath,
        AblationProfile::CostGraph, AblationProfile::Wander,
        AblationProfile::Deliberator, AblationProfile::ForwardOnly,
        AblationProfile::GlobalExploration,
        AblationProfile::LocalExploration, AblationProfile::Highway,
        AblationProfile::Circumstances, AblationProfile::Naive,
        AblationProfile::Custom}) {
    if (value == toString(profile)) return profile;
  }
  throw std::runtime_error("unknown experiment profile '" + value + "'");
}

void applyAblationProfile(Configuration& configuration) {
  auto& experiment = configuration.experiment;
  const auto set_advisors = [&](std::initializer_list<std::string_view> names) {
    for (auto& advisor : configuration.advisors) advisor.active = false;
    for (const auto name : names) {
      auto advisor = std::find_if(
          configuration.advisors.begin(), configuration.advisors.end(),
          [&](const auto& value) { return value.name == name; });
      if (advisor == configuration.advisors.end())
        configuration.advisors.push_back(
            {std::string(name), std::string(name), true, 1.0, {}});
      else
        advisor->active = true;
    }
  };
  const auto set_planners = [&](std::initializer_list<std::string_view> names) {
    auto& planners = configuration.navigation.planners;
    planners.distance = planners.density = planners.risk = planners.flow =
        planners.region = planners.hallway = planners.trail =
            planners.conveyor = planners.skeleton = planners.highway = false;
    for (const auto name : names) {
      if (name == "distance") planners.distance = true;
      if (name == "region") planners.region = true;
      if (name == "hallway") planners.hallway = true;
      if (name == "trail") planners.trail = true;
      if (name == "conveyor") planners.conveyor = true;
      if (name == "skeleton") planners.skeleton = true;
      if (name == "highway") planners.highway = true;
    }
  };
  const auto original_advisors = [&] {
    set_advisors({"big_step", "elbow_room", "novelty", "go_around",
                  "greedy", "convey", "enter", "exit", "trailer",
                  "unlikely"});
  };
  const auto disable_spatial_model = [&] {
    configuration.navigation.trails_on = false;
    configuration.navigation.conveyors_on = false;
    configuration.navigation.regions_on = false;
    configuration.navigation.doors_on = false;
    configuration.navigation.hallways_on = false;
    configuration.navigation.barriers_on = false;
    configuration.navigation.known_grid_on = false;
    configuration.navigation.inclusion_grid_on = false;
    configuration.navigation.highways_on = false;
    configuration.navigation.circumstances_on = false;
  };
  const auto evaluation_baseline = [&] {
    experiment.tiers = {};
    experiment.tiers.tier_one = true;
    experiment.tiers.tier_two = false;
    experiment.tiers.tier_three = true;
    experiment.tiers.tier_one_rules = {"victory", "avoid_obstacles",
                                       "not_opposite"};
    experiment.tiers.reactive_planners.clear();
    experiment.initial_exploration.enabled = false;
    experiment.reactive_exploration_enabled = false;
    experiment.opportunistic_exploration = false;
    experiment.social_enabled = false;
    experiment.social = {};
    experiment.social.enabled = false;
    configuration.navigation.crowd_learning.enabled = false;
    configuration.navigation.trails_on = true;
    configuration.navigation.conveyors_on = true;
    configuration.navigation.regions_on = true;
    configuration.navigation.doors_on = false;
    configuration.navigation.hallways_on = false;
    configuration.navigation.highways_on = false;
    configuration.navigation.circumstances_on = false;
    set_planners({});
  };
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
            advisor.name == "curiosity" || advisor.name == "spatial_learner" ||
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
      std::erase(experiment.tiers.tier_one_rules, "precedent");
      experiment.reactive_exploration_enabled = false;
      configuration.navigation.planners.distance = false;
      configuration.navigation.planners.density = false;
      configuration.navigation.planners.risk = false;
      configuration.navigation.planners.flow = false;
      configuration.navigation.planners.region = false;
      configuration.navigation.planners.hallway = false;
      configuration.navigation.planners.trail = false;
      configuration.navigation.planners.conveyor = false;
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
    case AblationProfile::PurelyReactive:
      evaluation_baseline();
      disable_spatial_model();
      set_advisors({"random"});
      break;
    case AblationProfile::Original:
      evaluation_baseline();
      original_advisors();
      break;
    case AblationProfile::Doors:
      evaluation_baseline();
      original_advisors();
      configuration.navigation.doors_on = true;
      break;
    case AblationProfile::LeastAngle:
    case AblationProfile::Access:
    case AblationProfile::Tentative:
    case AblationProfile::Hallways: {
      evaluation_baseline();
      configuration.navigation.doors_on = true;
      std::vector<std::string_view> names{
          "big_step", "elbow_room", "novelty", "go_around", "greedy",
          "convey", "enter", "exit", "trailer", "unlikely",
          "least_angle"};
      if (experiment.profile == AblationProfile::Access ||
          experiment.profile == AblationProfile::Tentative ||
          experiment.profile == AblationProfile::Hallways)
        names.push_back("access");
      if (experiment.profile == AblationProfile::Tentative ||
          experiment.profile == AblationProfile::Hallways) {
        names.insert(names.end(), {"curiosity", "enfilade", "visual_scan",
                                   "spatial_learner"});
      }
      if (experiment.profile == AblationProfile::Hallways) {
        configuration.navigation.hallways_on = true;
        names.insert(names.end(), {"crossroads", "follow", "stay"});
      }
      for (auto& advisor : configuration.advisors) advisor.active = false;
      for (const auto name : names) {
        auto advisor = std::find_if(
            configuration.advisors.begin(), configuration.advisors.end(),
            [&](const auto& value) { return value.name == name; });
        if (advisor == configuration.advisors.end())
          configuration.advisors.push_back(
              {std::string(name), std::string(name), true, 1.0, {}});
        else
          advisor->active = true;
      }
      break;
    }
    case AblationProfile::ShortestPath:
    case AblationProfile::CostGraph:
      evaluation_baseline();
      configuration.navigation.doors_on = true;
      configuration.navigation.hallways_on = true;
      experiment.tiers.tier_one_rules.push_back("enforcer");
      experiment.tiers.tier_two = true;
      if (experiment.profile == AblationProfile::ShortestPath) {
        set_planners({"distance"});
        set_advisors({"big_step", "elbow_room", "novelty", "go_around",
                      "greedy", "convey", "enter", "exit", "trailer",
                      "unlikely", "curiosity", "enfilade", "visual_scan"});
      } else {
        set_planners(
            {"distance", "conveyor", "hallway", "region", "trail"});
        set_advisors({"big_step", "elbow_room", "novelty", "go_around",
                      "greedy", "convey", "enter", "exit", "trailer",
                      "unlikely", "curiosity", "enfilade", "visual_scan",
                      "access", "crossroads", "follow", "least_angle",
                      "spatial_learner", "stay"});
      }
      break;
    case AblationProfile::Wander:
    case AblationProfile::Deliberator:
    case AblationProfile::ForwardOnly:
    case AblationProfile::GlobalExploration:
    case AblationProfile::LocalExploration:
    case AblationProfile::Highway:
      evaluation_baseline();
      configuration.navigation.doors_on = true;
      configuration.navigation.hallways_on = true;
      original_advisors();
      experiment.tiers.tier_one_rules.insert(
          experiment.tiers.tier_one_rules.end(),
          {"enforcer", "thru", "behind", "out", "forward"});
      experiment.tiers.reactive_planners = {"thru", "behind", "out"};
      if (experiment.profile == AblationProfile::ForwardOnly) {
        experiment.tiers.tier_one_rules = {"victory", "avoid_obstacles",
                                           "not_opposite", "enforcer",
                                           "forward"};
        experiment.tiers.reactive_planners.clear();
      }
      if (experiment.profile != AblationProfile::Wander) {
        experiment.tiers.tier_two = true;
        set_planners({"skeleton"});
      }
      if (experiment.profile != AblationProfile::Deliberator) {
        experiment.initial_exploration.enabled = true;
      }
      if (experiment.profile == AblationProfile::LocalExploration ||
          experiment.profile == AblationProfile::Highway) {
        experiment.reactive_exploration_enabled = true;
        const auto forward = std::find(
            experiment.tiers.tier_one_rules.begin(),
            experiment.tiers.tier_one_rules.end(), "forward");
        experiment.tiers.tier_one_rules.insert(forward,
                                               "low_level_exploration");
        experiment.tiers.reactive_planners.push_back(
            "low_level_exploration");
      }
      if (experiment.profile == AblationProfile::Highway) {
        configuration.navigation.highways_on = true;
        configuration.navigation.planners.highway = true;
      }
      break;
    case AblationProfile::Circumstances:
      evaluation_baseline();
      configuration.navigation.doors_on = true;
      configuration.navigation.circumstances_on = true;
      experiment.tiers.tier_two = true;
      experiment.tiers.tier_one_rules.push_back("enforcer");
      experiment.tiers.tier_one_rules.push_back("precedent");
      set_planners({"skeleton"});
      set_advisors({"big_step", "elbow_room", "novelty", "go_around",
                    "greedy", "convey", "enter", "exit", "trailer",
                    "unlikely", "least_angle"});
      break;
    case AblationProfile::Naive:
      evaluation_baseline();
      disable_spatial_model();
      set_advisors({"greedy"});
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
  canonical
      << std::setprecision(17)
      << toString(configuration.experiment.behavior_mode) << '|'
      << toString(configuration.experiment.profile)
      << '|' << configuration.experiment.random_seed << '|'
      << configuration.experiment.tiers.tier_one << '|'
      << configuration.experiment.tiers.tier_two << '|'
      << configuration.experiment.tiers.tier_three << '|'
      << configuration.experiment.initial_exploration.enabled << '|'
      << configuration.experiment.initial_exploration.observation_budget << '|'
      << configuration.experiment.initial_exploration.strategy << '|'
      << configuration.experiment.initial_exploration.time_limit_s << '|'
      << configuration.experiment.initial_exploration.decision_budget << '|'
      << configuration.experiment.initial_exploration.minimum_clearance_m << '|'
      << configuration.experiment.initial_exploration.heading_tolerance_rad
      << '|'
      << configuration.experiment.initial_exploration
             .candidate_completion_distance_m
      << '|'
      << configuration.experiment.initial_exploration.cue_similarity_radius_m
      << '|'
      << configuration.experiment.initial_exploration.passage_grid_resolution_m
      << '|'
      << configuration.experiment.initial_exploration.minimum_bundle_beams
      << '|' << configuration.experiment.reactive_exploration_enabled << '|'
      << configuration.experiment.opportunistic_exploration << '|'
      << configuration.experiment.social.enabled << '|'
      << configuration.experiment.social.observations << '|'
      << configuration.experiment.social.learning << '|'
      << configuration.experiment.social.advisors << '|'
      << configuration.experiment.social.planners << '|'
      << configuration.experiment.safety_envelope.enabled << '|'
      << configuration.experiment.safety_envelope.sensor_freshness_timeout_s
      << '|' << toString(configuration.static_map.mode) << '|'
      << toString(configuration.static_map.failure_policy) << '|'
      << configuration.static_map.path << '|'
      << configuration.static_map.origin_x_m << '|'
      << configuration.static_map.origin_y_m << '|'
      << configuration.static_map.occupancy_resolution_m << '|'
      << configuration.static_map.obstacle_inflation_m << '|'
      << configuration.static_map.map_based_planning_enabled << '|'
      << configuration.map_dimensions.length << '|'
      << configuration.map_dimensions.height << '|'
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
            << configuration.navigation.sensed_occupancy_on << '|'
            << configuration.navigation.inclusion_grid_on << '|'
            << configuration.navigation.highways_on << '|'
            << configuration.navigation.circumstances_on << '|'
            << configuration.navigation.circumstances.setting_resolution_m
            << '|'
            << configuration.navigation.circumstances.setting_radius_m << '|'
            << configuration.navigation.circumstances.minimum_cluster_size
            << '|'
            << configuration.navigation.circumstances
                   .assignment_confidence_threshold
            << '|'
            << configuration.navigation.circumstances
                   .similarity_l1_threshold
            << '|'
            << configuration.navigation.circumstances
                   .reclustering_threshold
            << '|'
            << configuration.navigation.circumstances.minimum_case_evidence
            << '|'
            << configuration.navigation.circumstances.accuracy_threshold
            << '|'
            << configuration.navigation.circumstances
                   .action_confidence_threshold
            << '|'
            << configuration.navigation.circumstances.distance_bin_base_m
            << '|'
            << configuration.navigation.circumstances.angle_bin_count << '|'
            << configuration.navigation.grids.extent_policy << '|'
            << configuration.navigation.grids.free_observations_to_clear
            << '|'
            << configuration.navigation.grids.dynamic_expiry_observations
            << '|' << configuration.navigation.grids.map_unknown_policy << '|'
            << configuration.navigation.grids.sensor_unknown_policy << '|'
            << configuration.navigation.planners.distance << '|'
            << configuration.navigation.planners.sensor_distance << '|'
            << configuration.navigation.planners.skeleton << '|'
            << configuration.navigation.planners.highway << '|'
            << configuration.navigation.planners.density << '|'
            << configuration.navigation.planners.risk << '|'
            << configuration.navigation.planners.flow << '|'
            << configuration.navigation.planners.region << '|'
            << configuration.navigation.planners.hallway << '|'
            << configuration.navigation.planners.trail << '|'
            << configuration.navigation.planners.conveyor << '|'
            << configuration.navigation.planners.selection_policy << '|'
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

std::vector<std::string> componentManifest(const Configuration& configuration) {
  std::vector<std::string> result{
      "behavior_mode:" +
          std::string(toString(configuration.experiment.behavior_mode)),
      "hard_safety:obstacle_clearance", "phase:target_navigation"};
  result.push_back("map_mode:" +
                   std::string(toString(configuration.static_map.mode)));
  if (configuration.static_map.mode == MapOperatingMode::MapEnabled)
    result.push_back("map:requested");
  const auto& experiment = configuration.experiment;
  if (experiment.initial_exploration.enabled)
    result.push_back("phase:initial_exploration");
  if (experiment.tiers.tier_one) result.push_back("tier:tier_one");
  if (experiment.tiers.tier_two) result.push_back("tier:tier_two");
  if (experiment.tiers.tier_three) result.push_back("tier:tier_three");
  for (const auto& rule : experiment.tiers.tier_one_rules)
    if (experiment.tiers.tier_one) result.push_back("tier1:" + rule);
  for (const auto& planner : experiment.tiers.reactive_planners)
    if (experiment.tiers.tier_one) result.push_back("reactive:" + planner);
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
  add_feature(configuration.navigation.sensed_occupancy_on,
              "sensed_occupancy");
  add_feature(configuration.navigation.inclusion_grid_on, "inclusion_grid");
  add_feature(configuration.navigation.highways_on, "highways");
  add_feature(configuration.navigation.circumstances_on, "circumstances");
  const auto add_planner = [&result](bool enabled, std::string name) {
    if (enabled) result.push_back("planner:" + std::move(name));
  };
  add_planner(configuration.navigation.planners.distance, "distance");
  add_planner(configuration.navigation.planners.sensor_distance,
              "sensor_distance");
  add_planner(configuration.navigation.planners.skeleton, "skeleton");
  add_planner(configuration.navigation.planners.highway, "highway");
  add_planner(configuration.navigation.planners.density, "density");
  add_planner(configuration.navigation.planners.risk, "risk");
  add_planner(configuration.navigation.planners.flow, "flow");
  add_planner(configuration.navigation.planners.region, "region");
  add_planner(configuration.navigation.planners.hallway, "hallway");
  add_planner(configuration.navigation.planners.trail, "trail");
  add_planner(configuration.navigation.planners.conveyor, "conveyor");
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
  configuration.static_map.path = map_file;
  configuration.static_map.mode = map_file.empty() ? MapOperatingMode::Mapless
                                                    : MapOperatingMode::MapEnabled;
  applyAblationProfile(configuration);
  validateConfiguration(configuration);
  return configuration;
}

void validateConfiguration(const Configuration& configuration) {
  validateNavigation(configuration.navigation);
  const auto& experiment = configuration.experiment;
  if (experiment.behavior_mode == BehaviorMode::Compatibility) {
    throw std::runtime_error(
        "configuration: experiment.behavior_mode 'compatibility' is reserved "
        "but not operational; unresolved fidelity blockers include trail, "
        "conveyor, region, door/exit, hallway, region-skeleton, HLE, exact "
        "tier ordering, Enforcer action selection, and Why explanations. Use "
        "'modernized' until the compatibility acceptance suite is enabled");
  }
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
      "victory", "avoid_obstacles", "not_opposite", "enforcer",
      "thru",    "behind",          "out",          "low_level_exploration",
      "forward", "precedent"};
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
      throw std::runtime_error("configuration: duplicate Tier-1 rule '" + rule +
                               "'");
    const std::size_t position =
        static_cast<std::size_t>(found - tier_one_order.begin());
    if (!first_rule && position <= previous)
      throw std::runtime_error(
          "configuration: Tier-1 rules must preserve the configured order");
    first_rule = false;
    previous = position;
  }
  const std::set<std::string> registered_reactive{"thru", "behind", "out",
                                                  "low_level_exploration"};
  std::set<std::string> configured_reactive;
  for (const auto& planner : experiment.tiers.reactive_planners) {
    if (!registered_reactive.contains(planner))
      throw std::runtime_error("configuration: unknown reactive planner '" +
                               planner + "'");
    if (!configured_reactive.insert(planner).second)
      throw std::runtime_error("configuration: duplicate reactive planner '" +
                               planner + "'");
  }
  if (!experiment.safety_envelope.enabled)
    throw std::runtime_error(
        "configuration: safety.command_envelope.enabled is an invariant "
        "platform boundary and must remain true for every cognitive ablation");
  if (experiment.tiers.tier_one && configured_rules.contains("precedent") &&
      !configuration.navigation.circumstances_on)
    throw std::runtime_error(
        "configuration: Precedent requires the circumstances representation");
  if (!std::isfinite(experiment.safety_envelope.sensor_freshness_timeout_s) ||
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
         configuration.navigation.planners.flow ||
         configuration.navigation.planners.region ||
         configuration.navigation.planners.hallway ||
         configuration.navigation.planners.trail ||
         configuration.navigation.planners.conveyor)))
    throw std::runtime_error(
        "configuration: LLE requires Tier 1, the inclusion grid, Tier 2 "
        "replanning, "
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
  const std::set<std::string> registered_advisors{"random",
                                                  "goal_progress",
                                                  "goal_progress_linear",
                                                  "clearance",
                                                  "clearance_rotation",
                                                  "exploration",
                                                  "social_navigation",
                                                  "crowd_avoid",
                                                  "risk_avoid",
                                                  "flow_follow",
                                                  "avoid_revisit",
                                                  "prefer_regions",
                                                  "prefer_highways",
                                                  "prefer_doors",
                                                  "follow_trails",
                                                  "big_step",
                                                  "elbow_room",
                                                  "novelty",
                                                  "go_around",
                                                  "greedy",
                                                  "curiosity",
                                                  "enfilade",
                                                  "visual_scan",
                                                  "convey",
                                                  "enter",
                                                  "exit",
                                                  "trailer",
                                                  "unlikely",
                                                  "access",
                                                  "crossroads",
                                                  "follow",
                                                  "least_angle",
                                                  "spatial_learner",
                                                  "stay"};
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
    const bool missing_spatial_representation =
        (advisor.name == "prefer_regions" || advisor.name == "exit")
            ? !configuration.navigation.regions_on
        : advisor.name == "prefer_highways"
            ? !configuration.navigation.highways_on
        : advisor.name == "prefer_doors" ? !configuration.navigation.doors_on
        : (advisor.name == "follow_trails" || advisor.name == "trailer")
            ? !configuration.navigation.trails_on
        : advisor.name == "convey" ? !configuration.navigation.conveyors_on
        : (advisor.name == "follow" || advisor.name == "crossroads" ||
           advisor.name == "stay")
            ? !configuration.navigation.hallways_on
        : advisor.name == "enter" ? !configuration.navigation.regions_on
        : (advisor.name == "access" || advisor.name == "unlikely")
            ? (!configuration.navigation.regions_on ||
               !configuration.navigation.doors_on)
        : advisor.name == "least_angle" ? !configuration.navigation.regions_on
        : advisor.name == "spatial_learner"
            ? (!configuration.navigation.inclusion_grid_on ||
               !configuration.navigation.regions_on ||
               !configuration.navigation.conveyors_on)
            : false;
    if (advisor.active && missing_spatial_representation)
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

  const auto& map = configuration.static_map;
  if (!std::isfinite(map.origin_x_m) || !std::isfinite(map.origin_y_m) ||
      !std::isfinite(map.occupancy_resolution_m) ||
      map.occupancy_resolution_m <= 0.0 ||
      !std::isfinite(map.obstacle_inflation_m) ||
      map.obstacle_inflation_m < 0.0 ||
      configuration.map_dimensions.length <= 0 ||
      configuration.map_dimensions.height <= 0 ||
      !std::isfinite(configuration.map_dimensions.granularity) ||
      configuration.map_dimensions.granularity <= 0.0)
    throw std::runtime_error(
        "configuration: map bounds, origin, occupancy resolution, inflation, "
        "and granularity are invalid");
  if (map.mode == MapOperatingMode::MapEnabled && map.path.empty())
    throw std::runtime_error(
        "configuration: map-enabled operation requires map.path");
  const bool map_planner = configuration.navigation.planners.distance ||
                           configuration.navigation.planners.density ||
                           configuration.navigation.planners.risk ||
                           configuration.navigation.planners.flow;
  const std::set<std::string> unknown_policies{
      "prohibited", "high_cost", "within_sensor_range", "exploration_only"};
  const auto& grids = configuration.navigation.grids;
  if ((grids.extent_policy != "fixed" && grids.extent_policy != "expand") ||
      grids.free_observations_to_clear == 0U ||
      grids.free_observations_to_clear >
          std::numeric_limits<std::uint16_t>::max() ||
      grids.dynamic_expiry_observations == 0U ||
      !unknown_policies.contains(grids.map_unknown_policy) ||
      !unknown_policies.contains(grids.sensor_unknown_policy) ||
      !std::isfinite(grids.localization_uncertainty_m) ||
      grids.localization_uncertainty_m < 0.0 ||
      !std::isfinite(grids.turning_footprint_margin_m) ||
      grids.turning_footprint_margin_m < 0.0 ||
      !std::isfinite(grids.dynamic_obstacle_margin_m) ||
      grids.dynamic_obstacle_margin_m < 0.0 ||
      !std::isfinite(grids.unknown_cost_multiplier) ||
      grids.unknown_cost_multiplier < 1.0)
    throw std::runtime_error(
        "configuration: grid evidence thresholds, unknown-space policies, "
        "inflation margins, and unknown cost are invalid");
  if (configuration.navigation.planners.sensor_distance &&
      !configuration.navigation.sensed_occupancy_on)
    throw std::runtime_error(
        "configuration: sensor_distance requires features.sensed_occupancy");
  if (map_planner &&
      (map.mode != MapOperatingMode::MapEnabled ||
       !map.map_based_planning_enabled))
    throw std::runtime_error(
        "configuration: static-map grid and crowd planners require "
        "map.mode=map_enabled, map.planning.enabled=true, and successfully "
        "loaded map occupancy");

  if (configuration.tasks.empty()) {
    throw std::runtime_error("configuration: at least one task is required");
  }
  for (const TaskConfiguration& task : configuration.tasks) {
    if (!std::isfinite(task.x) || !std::isfinite(task.y) ||
        task.x < map.origin_x_m || task.y < map.origin_y_m ||
        task.x > map.origin_x_m + configuration.map_dimensions.length ||
        task.y > map.origin_y_m + configuration.map_dimensions.height) {
      throw std::runtime_error(
          "configuration: task coordinate lies outside configured dimensions");
    }
  }
}

}  // namespace semaforr::config
