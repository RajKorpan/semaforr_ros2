#ifndef SEMAFORR_DECISION_RESTORED_TIERS_HPP
#define SEMAFORR_DECISION_RESTORED_TIERS_HPP

#include <semaforr/decision/advisor.hpp>
#include <semaforr/decision/registry.hpp>
#include <semaforr/decision/rules.hpp>
#include <semaforr/planning/reactive_planner.hpp>
#include <semaforr/decision/enforcer.hpp>

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
              double heading_tolerance_rad = 0.15)
      : action_space_(std::move(action_space)),
        heading_tolerance_rad_(heading_tolerance_rad) {}
  std::string_view name() const noexcept override { return "Forward"; }
  std::vector<std::string_view> dependencies() const override {
    return {"active_waypoint", "robot_pose"};
  }
  std::vector<Veto> evaluate(
      const DecisionContext& context) const override;

 private:
  domain::ActionSpace action_space_;
  double heading_tolerance_rad_;
};

class NotOppositeRule final : public VetoRule {
 public:
  std::string_view name() const noexcept override { return "NotOpposite"; }
  std::vector<std::string_view> dependencies() const override {
    return {"navigation_history"};
  }
  std::vector<Veto> evaluate(
      const DecisionContext& context) const override;
};

class PrecedentRule final : public VetoRule {
 public:
  std::string_view name() const noexcept override { return "Precedent"; }
  std::vector<std::string_view> dependencies() const override {
    return {"circumstances"};
  }
  std::vector<Veto> evaluate(const DecisionContext&) const override {
    return {};
  }
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

void registerRestoredTierFactories(TierOneRegistry& tier_one,
                                   AdvisorRegistry& tier_three,
                                   const domain::ActionSpace& action_space,
                                   double robot_radius_m = 0.25,
                                   double obstacle_buffer_m = 0.1);

}  // namespace semaforr::decision

#endif
