#ifndef SEMAFORR_SPATIAL_SPATIAL_LEARNING_COORDINATOR_HPP
#define SEMAFORR_SPATIAL_SPATIAL_LEARNING_COORDINATOR_HPP

#include <functional>
#include <vector>

#include <semaforr/domain/world_model.hpp>

namespace semaforr::spatial {

class SpatialLearningCoordinator {
public:
  using LearningStep = std::function<void(domain::WorldModel&)>;

  void addStep(LearningStep step);
  void process(domain::WorldModel& world) const;
  std::size_t stepCount() const noexcept { return steps_.size(); }

private:
  std::vector<LearningStep> steps_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_SPATIAL_LEARNING_COORDINATOR_HPP
