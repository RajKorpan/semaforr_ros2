#ifndef SEMAFORR_DECISION_ENFORCER_HPP
#define SEMAFORR_DECISION_ENFORCER_HPP

#include <cstddef>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <vector>

namespace semaforr::decision {

class Enforcer {
 public:
  std::vector<domain::Point2D> operationalize(
      const planning::HierarchicalPlan& plan) const;
  std::size_t activeStep(const planning::HierarchicalPlan& plan,
                         const domain::Pose2D& pose,
                         domain::Distance tolerance) const noexcept;
};

}  // namespace semaforr::decision

#endif
