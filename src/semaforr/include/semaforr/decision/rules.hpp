#ifndef SEMAFORR_DECISION_RULES_HPP
#define SEMAFORR_DECISION_RULES_HPP

#include <optional>
#include <semaforr/decision/context.hpp>
#include <semaforr/decision/decision_result.hpp>
#include <semaforr/planning/hierarchical_plan.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::decision {

struct Decision {
  domain::Action action;
  std::string rule;
  std::string explanation;
};

class MandatoryRule {
 public:
  virtual ~MandatoryRule() = default;
  virtual std::string_view name() const noexcept { return "mandatory_rule"; }
  virtual std::vector<std::string_view> dependencies() const { return {}; }
  virtual std::optional<Decision> evaluate(
      const DecisionContext& context) const = 0;
};

class VetoRule {
 public:
  virtual ~VetoRule() = default;
  virtual std::string_view name() const noexcept { return "veto_rule"; }
  virtual std::vector<std::string_view> dependencies() const { return {}; }
  virtual std::vector<Veto> evaluate(const DecisionContext& context) const = 0;
};

class PlanOperationalizer {
 public:
  virtual ~PlanOperationalizer() = default;
  virtual std::string_view name() const noexcept = 0;
  virtual std::vector<domain::Point2D> operationalize(
      const planning::HierarchicalPlan&) const = 0;
  virtual std::optional<domain::Point2D> operationalizeNext(
      planning::HierarchicalPlan&, const domain::SpatialModel&,
      const domain::Pose2D&, domain::Distance) const = 0;
};

struct ReplanningRequest {
  bool requested = false;
  std::string reason;
};

class ReplanningTrigger {
 public:
  virtual ~ReplanningTrigger() = default;
  virtual std::string_view name() const noexcept = 0;
  virtual ReplanningRequest evaluateReplan(const DecisionContext&) const = 0;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_RULES_HPP
