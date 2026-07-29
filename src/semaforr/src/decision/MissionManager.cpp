#include <semaforr/decision/mission_manager.hpp>

namespace semaforr::decision {

MissionStep MissionManager::prepareDecision()
{
  if (!mission_.active()) {
    if (!mission_.activate_next()) {
      return MissionStep::Complete;
    }
    return MissionStep::ActivatedTask;
  }
  if (mission_.decisions_for_active() >= mission_.decision_limit()) {
    mission_.skip_active();
    if (!mission_.activate_next()) {
      return MissionStep::Complete;
    }
    return MissionStep::SkippedTask;
  }
  return MissionStep::Ready;
}

void MissionManager::recordDecision()
{
  mission_.record_decision();
}

bool MissionManager::completeActiveTask()
{
  return mission_.complete_active();
}

bool MissionManager::skipActiveTask()
{
  return mission_.skip_active();
}

bool MissionManager::complete() const noexcept
{
  return mission_.finished();
}

}  // namespace semaforr::decision
