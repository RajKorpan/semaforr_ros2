#ifndef SEMAFORR_DECISION_TIER_REGISTRY_HPP
#define SEMAFORR_DECISION_TIER_REGISTRY_HPP

#include <semaforr/decision/advisor.hpp>
#include <semaforr/decision/registry.hpp>
#include <semaforr/decision/rules.hpp>
#include <semaforr/planning/reactive_planner.hpp>
#include <semaforr/decision/enforcer.hpp>
#include <stdexcept>

namespace semaforr::decision {

class VictoryRule final : public MandatoryRule {
 public:
  explicit VictoryRule(domain::Distance tolerance = domain::Distance(0.5),
                       domain::ActionSpace action_space =
                           domain::ActionSpace({0.25}, {0.2}))
      : tolerance_(tolerance), action_space_(std::move(action_space)) {}
  std::string_view name() const noexcept override { return "Victory"; }
  std::vector<std::string_view> dependencies() const override {
    return {"active_task", "robot_pose"};
  }
  std::optional<Decision> evaluate(
      const DecisionContext& context) const override;

 private:
  domain::Distance tolerance_;
  domain::ActionSpace action_space_;
};

class ForwardRule final : public VetoRule {
 public:
  ForwardRule(domain::ActionSpace action_space,
              double visited_grid_resolution_m = 1.0)
      : action_space_(std::move(action_space)),
        visited_grid_resolution_m_(visited_grid_resolution_m) {
    if (!(visited_grid_resolution_m_ > 0.0))
      throw std::invalid_argument(
          "Forward visited-grid resolution must be positive");
  }
  std::string_view name() const noexcept override { return "Forward"; }
  std::vector<std::string_view> dependencies() const override {
    return {"active_waypoint", "robot_pose"};
  }
  std::vector<Veto> evaluate(
      const DecisionContext& context) const override;

 private:
  void synchronizeVisitedGrid(const domain::WorldModel&) const;

  domain::ActionSpace action_space_;
  double visited_grid_resolution_m_;
  mutable std::optional<domain::TaskId> task_id_;
  mutable std::size_t history_cursor_ = 0U;
  mutable std::vector<domain::Point2D> visited_plan_positions_;
};

class NotOppositeRule final : public VetoRule {
 public:
  explicit NotOppositeRule(
      domain::ActionSpace action_space = domain::ActionSpace({0.25}, {0.2}),
      double orientation_tolerance_rad = 0.05)
      : action_space_(std::move(action_space)),
        orientation_tolerance_rad_(orientation_tolerance_rad) {}
  std::string_view name() const noexcept override { return "NotOpposite"; }
  std::vector<std::string_view> dependencies() const override {
    return {"navigation_history"};
  }
  std::vector<Veto> evaluate(
      const DecisionContext& context) const override;

 private:
  domain::ActionSpace action_space_;
  double orientation_tolerance_rad_;
};

struct PrecedentConfiguration {
  std::size_t minimum_case_evidence = 10U;
  double accuracy_threshold = 0.75;
  double action_confidence_threshold = 0.25;
};

class PrecedentRule final : public VetoRule {
 public:
  explicit PrecedentRule(
      domain::ActionSpace action_space = domain::ActionSpace({0.25}, {0.2}),
      PrecedentConfiguration configuration = {});
  std::string_view name() const noexcept override { return "Precedent"; }
  std::vector<std::string_view> dependencies() const override {
    return {"circumstances"};
  }
  std::vector<Veto> evaluate(const DecisionContext&) const override;

 private:
  domain::ActionSpace action_space_;
  PrecedentConfiguration configuration_;
};

enum class SpatialAdvisorObjective {
  AvoidRevisit,
  PreferRegions,
  PreferHighways,
  PreferDoors,
  FollowTrails
};

class SpatialAdvisor final : public Advisor {
 public:
  SpatialAdvisor(std::string name, SpatialAdvisorObjective objective,
                 domain::ActionSpace action_space, double weight = 1.0);
  std::string_view name() const noexcept override { return name_; }
  std::vector<std::string_view> dependencies() const override;
  AdvisorEvaluation evaluate(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const override;

 private:
  std::string name_;
  SpatialAdvisorObjective objective_;
  domain::ActionSpace action_space_;
  double weight_;
};

class TierOneRegistry {
 public:
  enum class Kind { Mandatory, Veto, PlanOperationalizer, ReactivePlanner,
                    ReplanningTrigger };
  using MandatoryFactory =
      std::function<std::unique_ptr<MandatoryRule>()>;
  using VetoFactory = std::function<std::unique_ptr<VetoRule>()>;
  using OperationalizerFactory =
      std::function<std::unique_ptr<PlanOperationalizer>()>;
  using ReactiveFactory =
      std::function<std::unique_ptr<planning::ReactivePlanner>()>;
  void registerMandatory(std::string name, MandatoryFactory factory);
  void registerVeto(std::string name, VetoFactory factory);
  void registerOperationalizer(std::string name,
                               OperationalizerFactory factory);
  void registerReactive(std::string name, ReactiveFactory factory,
                        bool replanning_trigger = false);
  Kind kind(std::string_view name) const;
  std::unique_ptr<MandatoryRule> createMandatory(
      std::string_view name) const;
  std::unique_ptr<VetoRule> createVeto(std::string_view name) const;
  std::unique_ptr<PlanOperationalizer> createOperationalizer(
      std::string_view name) const;
  std::unique_ptr<planning::ReactivePlanner> createReactive(
      std::string_view name) const;

 private:
  std::unordered_map<std::string, MandatoryFactory> mandatory_;
  std::unordered_map<std::string, VetoFactory> veto_;
  std::unordered_map<std::string, OperationalizerFactory> operationalizers_;
  std::unordered_map<std::string, ReactiveFactory> reactive_;
  std::unordered_map<std::string, Kind> kinds_;
};

void registerTierFactories(TierOneRegistry& tier_one,
                           AdvisorRegistry& tier_three,
                           const domain::ActionSpace& action_space,
                           double robot_radius_m = 0.25,
                           double obstacle_buffer_m = 0.1,
                           PrecedentConfiguration precedent = {});

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_TIER_REGISTRY_HPP
