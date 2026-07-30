#ifndef SEMAFORR_DECISION_RESTORED_TIERS_HPP
#define SEMAFORR_DECISION_RESTORED_TIERS_HPP

#include <semaforr/decision/advisor.hpp>
#include <semaforr/decision/registry.hpp>
#include <semaforr/decision/rules.hpp>

namespace semaforr::decision {

class VictoryRule final : public MandatoryRule {
 public:
  explicit VictoryRule(domain::Distance tolerance = domain::Distance(0.5))
      : tolerance_(tolerance) {}
  std::string_view name() const noexcept override { return "Victory"; }
  std::vector<std::string_view> dependencies() const override {
    return {"active_task", "robot_pose"};
  }
  std::optional<Decision> evaluate(
      const DecisionContext& context) const override;

 private:
  domain::Distance tolerance_;
};

class ForwardRule final : public MandatoryRule {
 public:
  ForwardRule(domain::ActionSpace action_space,
              double heading_tolerance_rad = 0.15)
      : action_space_(std::move(action_space)),
        heading_tolerance_rad_(heading_tolerance_rad) {}
  std::string_view name() const noexcept override { return "Forward"; }
  std::vector<std::string_view> dependencies() const override {
    return {"active_waypoint", "robot_pose"};
  }
  std::optional<Decision> evaluate(
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
  using MandatoryFactory =
      std::function<std::unique_ptr<MandatoryRule>()>;
  using VetoFactory = std::function<std::unique_ptr<VetoRule>()>;
  void registerMandatory(std::string name, MandatoryFactory factory);
  void registerVeto(std::string name, VetoFactory factory);
  std::unique_ptr<MandatoryRule> createMandatory(
      std::string_view name) const;
  std::unique_ptr<VetoRule> createVeto(std::string_view name) const;

 private:
  std::unordered_map<std::string, MandatoryFactory> mandatory_;
  std::unordered_map<std::string, VetoFactory> veto_;
};

void registerRestoredTierFactories(TierOneRegistry& tier_one,
                                   AdvisorRegistry& tier_three,
                                   const domain::ActionSpace& action_space);

}  // namespace semaforr::decision

#endif
