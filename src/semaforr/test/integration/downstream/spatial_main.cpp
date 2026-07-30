#include <semaforr/spatial/spatial_learning_coordinator.hpp>

int main() {
  const auto learning =
      semaforr::spatial::SpatialLearningCoordinator::defaults();
  return learning.learnerCount() == 10U ? 0 : 1;
}
