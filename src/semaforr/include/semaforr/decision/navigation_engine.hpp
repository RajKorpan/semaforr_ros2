#ifndef SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP
#define SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP

#include <cstdint>
#include <optional>
#include <semaforr/decision/decision_coordinator.hpp>
#include <semaforr/decision/mission_manager.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/domain/world_model.hpp>
#include <semaforr/planning/planning_coordinator.hpp>
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
                   domain::Distance goal_tolerance = domain::Distance(0.5));

  void observe(const domain::RobotObservation& observation);
  DecisionResult decide();
  DecisionResult decide(const domain::RobotObservation& observation);
  bool missionComplete() const noexcept;

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
  domain::Distance goal_tolerance_;
  std::optional<domain::RobotObservation> observation_;
  std::uint64_t decision_sequence_{0U};
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_NAVIGATION_ENGINE_HPP
