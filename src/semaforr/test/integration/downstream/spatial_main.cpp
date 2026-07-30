#include <semaforr/spatial/spatial_learning_coordinator.hpp>

int main() {
  const auto learning =
      semaforr::spatial::SpatialLearningCoordinator::defaults();
  return learning.learnerCount() == 9U ? 0 : 1;
}
