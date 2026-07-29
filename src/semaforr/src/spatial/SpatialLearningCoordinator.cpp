#include <semaforr/spatial/spatial_learning_coordinator.hpp>

#include <stdexcept>

namespace semaforr::spatial {

void SpatialLearningCoordinator::addStep(LearningStep step)
{
  if (!step) {
    throw std::invalid_argument("spatial learning step must not be empty");
  }
  steps_.push_back(std::move(step));
}

void SpatialLearningCoordinator::process(domain::WorldModel& world) const
{
  for (const auto& step : steps_) {
    step(world);
  }
}

}  // namespace semaforr::spatial
