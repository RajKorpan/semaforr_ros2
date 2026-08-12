#ifndef SEMAFORR_DECISION_ENFORCER_HPP
#define SEMAFORR_DECISION_ENFORCER_HPP

#include <cstddef>
#include <semaforr/decision/rules.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/observation.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <semaforr/planning/traversability.hpp>
#include <span>
#include <string>
#include <vector>

namespace semaforr::decision {

enum class EnforcerMode { Grid, Model };
enum class EnforcementStatus { Mandated, Complete, CannotOperationalize,
                               Invalid, Stale };

struct LocalActionPrediction {
  domain::Action action;
  domain::Pose2D predicted_pose;
  double progress_m = 0.0;
  bool selected = false;
};

struct PlanEnforcementContext {
  const domain::SpatialModel& spatial;
  const domain::CrowdModel* crowd = nullptr;
  const domain::StaticMap* static_map = nullptr;
  const domain::Pose2D& pose;
  const domain::ActionSpace& action_space;
  std::span<const domain::Action> viable_actions;
  planning::TraversabilityConfiguration traversability;
  domain::Distance tolerance{0.5};
  std::optional<domain::TaskId> task_id;
};

struct PlanEnforcementResult {
  EnforcementStatus status{EnforcementStatus::CannotOperationalize};
  EnforcerMode mode{EnforcerMode::Grid};
  std::optional<domain::Action> action;
  std::optional<domain::Point2D> operational_target;
  std::size_t step_index = 0U;
  std::size_t skipped_elements = 0U;
  double lookahead_m = 0.0;
  std::string step_type;
  std::string supporting_id;
  std::string reason_code;
  std::string validation_evidence;
  std::optional<std::string> shortcut;
  std::optional<std::string> repair;
  domain::DependencyRevisions dependency_revisions;
  std::vector<LocalActionPrediction> candidates;
};

class LocalActionEvaluator {
 public:
  PlanEnforcementResult evaluate(PlanEnforcementResult result,
                                 const PlanEnforcementContext& context) const;
};

class GridPlanEnforcer {
 public:
  PlanEnforcementResult enforce(planning::HierarchicalPlan& plan,
                                const PlanEnforcementContext& context) const;
};

class ModelPlanEnforcer {
 public:
  PlanEnforcementResult enforce(planning::HierarchicalPlan& plan,
                                const PlanEnforcementContext& context) const;
};

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
  PlanEnforcementResult enforce(planning::HierarchicalPlan& plan,
                                const PlanEnforcementContext& context) const;

 private:
  GridPlanEnforcer grid_;
  ModelPlanEnforcer model_;
};

}  // namespace semaforr::decision

#endif
