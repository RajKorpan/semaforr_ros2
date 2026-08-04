#ifndef SEMAFORR_DECISION_ENFORCER_HPP
#define SEMAFORR_DECISION_ENFORCER_HPP

#include <cstddef>
#include <semaforr/decision/rules.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <vector>

namespace semaforr::decision {

class Enforcer final : public PlanOperationalizer {
 public:
  std::string_view name() const noexcept override { return "enforcer"; }
  std::vector<domain::Point2D> operationalize(
      const planning::HierarchicalPlan& plan) const override;
  std::size_t activeStep(const planning::HierarchicalPlan& plan,
                         const domain::Pose2D& pose,
                         domain::Distance tolerance) const noexcept;
  std::optional<domain::Point2D> operationalizeNext(
      planning::HierarchicalPlan& plan, const domain::SpatialModel& spatial,
      const domain::Pose2D& pose, domain::Distance tolerance) const override;
};

}  // namespace semaforr::decision

#endif
