#ifndef SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP
#define SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP

#include <cstdint>
#include <optional>
#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/decision/enforcer.hpp>
#include <semaforr/decision/hard_safety_filter.hpp>
#include <semaforr/decision/mission_manager.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/domain/world_model.hpp>
#include <semaforr/exploration/exploration_coordinator.hpp>
#include <semaforr/navigation/navigation_phase.hpp>
#include <semaforr/planning/planning_coordinator.hpp>
#include <semaforr/planning/reactive_planner.hpp>
#include <semaforr/social/crowd_field_learner.hpp>
#include <semaforr/spatial/spatial_learning_coordinator.hpp>
#include <string>
#include <vector>

namespace semaforr::decision {

class NavigationEngine {
 public:
  NavigationEngine(domain::WorldModel& world,
                   const domain::ActionSpace& action_space,
                   DecisionCoordinator& decisions, MissionManager& mission,
                   planning::PlanningCoordinator& planning,
                   spatial::SpatialLearningCoordinator& learning,
                   social::CrowdFieldLearner* crowd_learning = nullptr,
                   domain::Distance goal_tolerance = domain::Distance(0.5),
                   HardSafetyFilter* hard_safety = nullptr,
                   navigation::NavigationPhaseCoordinator* phases = nullptr,
                   std::string configuration_fingerprint = {},
                   std::vector<std::string> component_manifest = {},
                   std::vector<std::string> reactive_planners =
                       {"thru", "behind", "out"},
                   bool low_level_exploration_enabled = true,
                   bool enforcer_enabled = true,
                   exploration::HighLevelExplorationConfiguration
                       hle_configuration = {});

  void observe(const domain::RobotObservation& observation);
  DecisionResult decide();
  DecisionResult decide(const domain::RobotObservation& observation);
  bool missionComplete() noexcept;
  navigation::NavigationPhase phase() const noexcept;

 private:
  std::vector<domain::Action> candidates() const;
  std::optional<std::string> preparePlan(MissionStep step);

  domain::WorldModel& world_;
  const domain::ActionSpace& action_space_;
  DecisionCoordinator& decisions_;
  MissionManager& mission_;
  planning::PlanningCoordinator& planning_;
  spatial::SpatialLearningCoordinator& learning_;
  social::CrowdFieldLearner* crowd_learning_;
  HardSafetyFilter* hard_safety_;
  navigation::NavigationPhaseCoordinator owned_phases_;
  navigation::NavigationPhaseCoordinator* phases_;
  std::string configuration_fingerprint_;
  std::vector<std::string> component_manifest_;
  exploration::ExplorationCoordinator exploration_;
  Enforcer enforcer_;
  planning::ReactivePlannerCoordinator reactive_;
  planning::LowLevelExplorer lle_;
  bool low_level_exploration_enabled_;
  bool enforcer_enabled_;
  domain::Distance goal_tolerance_;
  std::optional<domain::RobotObservation> observation_;
  std::vector<std::string> pending_phase_events_;
  std::uint64_t decision_sequence_{0U};
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP
