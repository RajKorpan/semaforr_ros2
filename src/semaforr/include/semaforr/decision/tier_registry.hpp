/**
 * @file tier_registry.hpp
 * @brief Tier registry responsibilities.
 *
 * @details This file defines tier registry behavior for tiered decision making
 * and action arbitration. It centers on `VictoryRule`, `ForwardRule`,
 * `NotOppositeRule`, `PrecedentConfiguration`, `PrecedentRule`,
 * `SpatialAdvisorObjective`, `SpatialAdvisor`, `TierOneRegistry`. Its
 * package-relative location is
 * `include/semaforr/decision/tier_registry.hpp`.
 */
#ifndef SEMAFORR_DECISION_TIER_REGISTRY_HPP
#define SEMAFORR_DECISION_TIER_REGISTRY_HPP

#include <cstdint>
#include <semaforr/decision/advisor.hpp>
#include <semaforr/decision/enforcer.hpp>
#include <semaforr/decision/registry.hpp>
#include <semaforr/decision/rules.hpp>
#include <semaforr/planning/reactive_planner.hpp>
#include <set>
#include <stdexcept>

namespace semaforr::decision {

/**
 * @brief Encapsulates victory rule state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class VictoryRule final : public MandatoryRule {
 public:
  /**
   * @brief Performs the victory rule operation for this subsystem.
   *
   * Arguments:
   * - @p tolerance: Supplies tolerance input to the operation.
   * - @p action_space: Supplies action space input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit VictoryRule(
      domain::Distance tolerance = domain::Distance(0.5),
      domain::ActionSpace action_space = domain::ActionSpace({0.25}, {0.2}))
      : tolerance_(tolerance), action_space_(std::move(action_space)) {}
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string_view name() const noexcept override { return "Victory"; }
  /**
   * @brief Performs the dependencies operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<std::string_view>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<std::string_view> dependencies() const override {
    return {"active_task", "robot_pose", "laser", "action_space"};
  }
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   *
   * Returns:
   * - `std::optional<Decision>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<Decision> evaluate(
      const DecisionContext& context) const override;

 private:
  domain::Distance tolerance_;
  domain::ActionSpace action_space_;
};

/**
 * @brief Encapsulates forward rule state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class ForwardRule final : public VetoRule {
 public:
  /**
   * @brief Performs the forward rule operation for this subsystem.
   *
   * Arguments:
   * - @p action_space: Supplies action space input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit ForwardRule(domain::ActionSpace action_space)
      : action_space_(std::move(action_space)) {}
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string_view name() const noexcept override { return "Forward"; }
  /**
   * @brief Performs the dependencies operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<std::string_view>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<std::string_view> dependencies() const override {
    return {"active_waypoint", "robot_pose", "decision_history"};
  }
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   *
   * Returns:
   * - `std::vector<Veto>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<Veto> evaluate(const DecisionContext& context) const override;

 private:
  /**
   * @brief Performs the synchronize visited grid operation for this
   * subsystem.
   *
   * Arguments:
   * - @p WorldModel: Supplies world model input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void synchronizeVisitedGrid(const domain::WorldModel&) const;

  domain::ActionSpace action_space_;
  mutable std::optional<domain::TaskId> task_id_;
  mutable std::size_t decision_cursor_ = 0U;
  using VisitedCell = std::pair<std::int64_t, std::int64_t>;
  /**
   * @brief Performs the visited cell operation for this subsystem.
   *
   * Arguments:
   * - @p Point2D: Supplies point2 d input to the operation.
   *
   * Returns:
   * - `VisitedCell` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  VisitedCell visitedCell(domain::Point2D) const noexcept;
  mutable std::set<VisitedCell> visited_cells_;
};

/**
 * @brief Encapsulates not opposite rule state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class NotOppositeRule final : public VetoRule {
 public:
  /**
   * @brief Performs the not opposite rule operation for this subsystem.
   *
   * Arguments:
   * - @p action_space: Supplies action space input to the operation.
   * - @p orientation_tolerance_rad: Supplies orientation tolerance rad
   * input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit NotOppositeRule(
      domain::ActionSpace action_space = domain::ActionSpace({0.25}, {0.2}),
      double orientation_tolerance_rad = 0.05)
      : action_space_(std::move(action_space)),
        orientation_tolerance_rad_(orientation_tolerance_rad) {}
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string_view name() const noexcept override { return "NotOpposite"; }
  /**
   * @brief Performs the dependencies operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<std::string_view>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<std::string_view> dependencies() const override {
    return {"navigation_history"};
  }
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   *
   * Returns:
   * - `std::vector<Veto>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<Veto> evaluate(const DecisionContext& context) const override;

 private:
  domain::ActionSpace action_space_;
  double orientation_tolerance_rad_;
};

/**
 * @brief Encapsulates precedent configuration state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
struct PrecedentConfiguration {
  std::size_t minimum_case_evidence = 10U;
  std::size_t minimum_action_evidence = 5U;
  double minimum_assignment_confidence = 0.95;
  double accuracy_threshold = 0.75;
  double action_confidence_threshold = 0.25;
};

/**
 * @brief Encapsulates precedent rule state and behavior for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class PrecedentRule final : public VetoRule {
 public:
  /**
   * @brief Performs the precedent rule operation for this subsystem.
   *
   * Arguments:
   * - @p action_space: Supplies action space input to the operation.
   * - @p configuration: Supplies configuration input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit PrecedentRule(
      domain::ActionSpace action_space = domain::ActionSpace({0.25}, {0.2}),
      PrecedentConfiguration configuration = {});
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string_view name() const noexcept override { return "Precedent"; }
  /**
   * @brief Performs the dependencies operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<std::string_view>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<std::string_view> dependencies() const override {
    return {"circumstances"};
  }
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `std::vector<Veto>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<Veto> evaluate(const DecisionContext&) const override;
  /**
   * @brief Performs the last reason operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string lastReason() const override { return last_reason_; }

 private:
  domain::ActionSpace action_space_;
  PrecedentConfiguration configuration_;
  mutable std::string last_reason_;
};

/**
 * @brief Enumerates the supported spatial advisor objective values used by
 * this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
enum class SpatialAdvisorObjective {
  AvoidRevisit,
  PreferRegions,
  PreferHighways,
  PreferDoors,
  FollowTrails
};

/**
 * @brief Encapsulates spatial advisor state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class SpatialAdvisor final : public Advisor {
 public:
  /**
   * @brief Performs the spatial advisor operation for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   * - @p objective: Supplies objective input to the operation.
   * - @p action_space: Supplies action space input to the operation.
   * - @p weight: Supplies weight input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  SpatialAdvisor(std::string name, SpatialAdvisorObjective objective,
                 domain::ActionSpace action_space, double weight = 1.0);
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string_view name() const noexcept override { return name_; }
  /**
   * @brief Performs the dependencies operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::vector<std::string_view>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::vector<std::string_view> dependencies() const override;
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p context: Supplies context input to the operation.
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - `AdvisorEvaluation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  AdvisorEvaluation evaluate(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const override;

 private:
  std::string name_;
  SpatialAdvisorObjective objective_;
  domain::ActionSpace action_space_;
  double weight_;
};

/**
 * @brief Encapsulates tier one registry state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class TierOneRegistry {
 public:
  /**
   * @brief Enumerates the supported kind values used by this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - Not applicable to this declaration.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  enum class Kind {
    Mandatory,
    Veto,
    PlanOperationalizer,
    ReactivePlanner,
    ReplanningTrigger
  };
  using MandatoryFactory = std::function<std::unique_ptr<MandatoryRule>()>;
  using VetoFactory = std::function<std::unique_ptr<VetoRule>()>;
  using OperationalizerFactory =
      std::function<std::unique_ptr<PlanOperationalizer>()>;
  using ReactiveFactory =
      std::function<std::unique_ptr<planning::ReactivePlanner>()>;
  /**
   * @brief Registers mandatory for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   * - @p factory: Supplies factory input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void registerMandatory(std::string name, MandatoryFactory factory);
  /**
   * @brief Registers veto for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   * - @p factory: Supplies factory input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void registerVeto(std::string name, VetoFactory factory);
  /**
   * @brief Registers operationalizer for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   * - @p factory: Supplies factory input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void registerOperationalizer(std::string name,
                               OperationalizerFactory factory);
  /**
   * @brief Registers reactive for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   * - @p factory: Supplies factory input to the operation.
   * - @p replanning_trigger: Supplies replanning trigger input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void registerReactive(std::string name, ReactiveFactory factory,
                        bool replanning_trigger = false);
  /**
   * @brief Performs the kind operation for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   *
   * Returns:
   * - `Kind` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Kind kind(std::string_view name) const;
  /**
   * @brief Creates mandatory for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   *
   * Returns:
   * - `std::unique_ptr<MandatoryRule>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::unique_ptr<MandatoryRule> createMandatory(std::string_view name) const;
  /**
   * @brief Creates veto for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   *
   * Returns:
   * - `std::unique_ptr<VetoRule>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::unique_ptr<VetoRule> createVeto(std::string_view name) const;
  /**
   * @brief Creates operationalizer for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   *
   * Returns:
   * - `std::unique_ptr<PlanOperationalizer>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::unique_ptr<PlanOperationalizer> createOperationalizer(
      std::string_view name) const;
  /**
   * @brief Creates reactive for this subsystem.
   *
   * Arguments:
   * - @p name: Supplies name input to the operation.
   *
   * Returns:
   * - `std::unique_ptr<planning::ReactivePlanner>` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::unique_ptr<planning::ReactivePlanner> createReactive(
      std::string_view name) const;

 private:
  std::unordered_map<std::string, MandatoryFactory> mandatory_;
  std::unordered_map<std::string, VetoFactory> veto_;
  std::unordered_map<std::string, OperationalizerFactory> operationalizers_;
  std::unordered_map<std::string, ReactiveFactory> reactive_;
  std::unordered_map<std::string, Kind> kinds_;
};

/**
 * @brief Registers tier factories for this subsystem.
 *
 * Arguments:
 * - @p tier_one: Supplies tier one input to the operation.
 * - @p tier_three: Supplies tier three input to the operation.
 * - @p action_space: Supplies action space input to the operation.
 * - @p robot_radius_m: Supplies robot radius m input to the operation.
 * - @p obstacle_buffer_m: Supplies obstacle buffer m input to the
 * operation.
 * - @p precedent: Supplies precedent input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void registerTierFactories(TierOneRegistry& tier_one,
                           AdvisorRegistry& tier_three,
                           const domain::ActionSpace& action_space,
                           double robot_radius_m = 0.25,
                           double obstacle_buffer_m = 0.1,
                           PrecedentConfiguration precedent = {});

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_TIER_REGISTRY_HPP
