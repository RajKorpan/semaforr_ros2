#include <semaforr/spatial/spatial_learning_coordinator.hpp>

int main() {
  const auto learning =
      semaforr::spatial::SpatialLearningCoordinator::defaults();
  return learning.learnerCount() == 7U ? 0 : 1;
}
