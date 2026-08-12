#ifndef SEMAFORR_DECISION_MISSION_MANAGER_HPP
#define SEMAFORR_DECISION_MISSION_MANAGER_HPP

#include <semaforr/domain/mission.hpp>
#include <vector>

namespace semaforr::decision {

enum class MissionStep { Ready, ActivatedTask, SkippedTask, Complete };

class MissionManager {
 public:
  explicit MissionManager(domain::Mission& mission) : mission_(mission) {}

  MissionStep prepareDecision();
  void recordDecision();
  bool completeActiveTask();
  bool skipActiveTask();
  void installPlan(std::vector<domain::Point2D> plan);
  void prependPlan(std::vector<domain::Point2D> prefix);
  void clearPlan();
  bool advanceWaypoint(const domain::Pose2D& pose, domain::Distance tolerance);
  bool complete() const noexcept;

 private:
  domain::Mission& mission_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_MISSION_MANAGER_HPP
