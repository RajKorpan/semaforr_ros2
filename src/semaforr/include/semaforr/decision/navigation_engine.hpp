#ifndef SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP
#define SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP

#include <vector>

#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/decision/mission_manager.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/domain/world_model.hpp>
#include <semaforr/planning/planning_coordinator.hpp>
#include <semaforr/spatial/spatial_learning_coordinator.hpp>
#include <semaforr/social/crowd_field_learner.hpp>

namespace semaforr::decision {

class NavigationEngine {
public:
  NavigationEngine(
    domain::WorldModel& world,
    const domain::ActionSpace& action_space,
    DecisionCoordinator& decisions,
    MissionManager& mission,
    planning::PlanningCoordinator& planning,
    spatial::SpatialLearningCoordinator& learning,
    social::CrowdFieldLearner* crowd_learning = nullptr);

  DecisionResult decide(const domain::RobotObservation& observation);

private:
  std::vector<domain::Action> candidates() const;

  domain::WorldModel& world_;
  const domain::ActionSpace& action_space_;
  DecisionCoordinator& decisions_;
  MissionManager& mission_;
  planning::PlanningCoordinator& planning_;
  spatial::SpatialLearningCoordinator& learning_;
  social::CrowdFieldLearner* crowd_learning_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP
