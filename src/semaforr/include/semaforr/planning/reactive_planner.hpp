/**
 * @file reactive_planner.hpp
 * @brief Reactive planner responsibilities.
 *
 * @details This file defines reactive planner behavior for path planning and
 * hierarchical plan construction. It centers on `ReactiveStatus`,
 * `InterruptionReason`, `ReactiveCompletionReason`,
 * `LowLevelExplorationState`, `TriggerEvaluation`, `ReactivePlanUpdate`,
 * `ReactiveRequest`, `ReactiveResult`. Its package-relative location is
 * `include/semaforr/planning/reactive_planner.hpp`.
 */
#ifndef SEMAFORR_PLANNING_REACTIVE_PLANNER_HPP
#define SEMAFORR_PLANNING_REACTIVE_PLANNER_HPP

#include <cstdint>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <semaforr/decision/context.hpp>
#include <semaforr/decision/rules.hpp>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::planning {

/**
 * @brief Enumerates the supported reactive status values used by this
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
enum class ReactiveStatus { NotApplicable, Action, InstallPlan, RequestReplan };
/**
 * @brief Enumerates the supported interruption reason values used by this
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
enum class InterruptionReason {
  TargetSensed,
  NewPlanAvailable,
  SensorLost,
  MissionChanged,
  Disabled
};
/**
 * @brief Enumerates the supported reactive completion reason values used by
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
enum class ReactiveCompletionReason {
  None,
  TargetSensed,
  NewPlanAvailable,
  CandidateExhausted,
  NoCandidates,
  BudgetExceeded,
  SensorLost,
  MissionChanged
};
/**
 * @brief Enumerates the supported low level exploration state values used
 * by this subsystem.
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
enum class LowLevelExplorationState {
  DetectMissingGuidance,
  AssembleCandidateRays,
  RankByTargetRelevance,
  PlanToCandidateStart,
  PursueCandidate,
  CheckConnectivity,
  Complete
};

/**
 * @brief Encapsulates trigger evaluation state and behavior for this
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
struct TriggerEvaluation {
  bool triggered = false;
  std::string rationale;
};

/**
 * @brief Encapsulates reactive plan update state and behavior for this
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
struct ReactivePlanUpdate {
  ReactiveStatus status = ReactiveStatus::NotApplicable;
  std::optional<domain::Action> action;
  LowLevelExplorationState state =
      LowLevelExplorationState::DetectMissingGuidance;
  ReactiveCompletionReason completion_reason = ReactiveCompletionReason::None;
  std::optional<std::uint64_t> candidate_id;
  std::string explanation;
  std::vector<domain::Point2D> prepend_waypoints;
  std::optional<domain::LearnedTrail> learned_recovery_trail;

  /**
   * @brief Performs the reactive plan update operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactivePlanUpdate() = default;
  /**
   * @brief Performs the reactive plan update operation for this subsystem.
   *
   * Arguments:
   * - @p update_status: Supplies update status input to the operation.
   * - @p update_action: Supplies update action input to the operation.
   * - @p update_state: Supplies update state input to the operation.
   * - @p reason: Supplies reason input to the operation.
   * - @p update_candidate_id: Supplies update candidate id input to the
   * operation.
   * - @p update_explanation: Supplies update explanation input to the
   * operation.
   * - @p waypoints: Supplies waypoints input to the operation.
   * - @p recovery_trail: Supplies recovery trail input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactivePlanUpdate(
      ReactiveStatus update_status, std::optional<domain::Action> update_action,
      LowLevelExplorationState update_state =
          LowLevelExplorationState::DetectMissingGuidance,
      ReactiveCompletionReason reason = ReactiveCompletionReason::None,
      std::optional<std::uint64_t> update_candidate_id = std::nullopt,
      std::string update_explanation = {},
      std::vector<domain::Point2D> waypoints = {},
      std::optional<domain::LearnedTrail> recovery_trail = std::nullopt)
      : status(update_status),
        action(update_action),
        state(update_state),
        completion_reason(reason),
        candidate_id(update_candidate_id),
        explanation(std::move(update_explanation)),
        prepend_waypoints(std::move(waypoints)),
        learned_recovery_trail(std::move(recovery_trail)) {}
};

/**
 * @brief Encapsulates reactive request state and behavior for this
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
struct ReactiveRequest {
  const domain::WorldModel& world;
  const domain::ActionSpace& action_space;
  std::span<const domain::Action> viable_actions{};
};

/**
 * @brief Encapsulates reactive result state and behavior for this
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
struct ReactiveResult {
  ReactiveStatus status = ReactiveStatus::NotApplicable;
  std::optional<domain::Action> action;
  std::string planner;
  std::string explanation;
  ReactiveCompletionReason completion_reason = ReactiveCompletionReason::None;
  std::vector<domain::Point2D> prepend_waypoints;
  std::optional<domain::LearnedTrail> learned_recovery_trail;

  /**
   * @brief Performs the reactive result operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactiveResult() = default;
  /**
   * @brief Performs the reactive result operation for this subsystem.
   *
   * Arguments:
   * - @p result_status: Supplies result status input to the operation.
   * - @p result_action: Supplies result action input to the operation.
   * - @p result_planner: Supplies result planner input to the operation.
   * - @p result_explanation: Supplies result explanation input to the
   * operation.
   * - @p reason: Supplies reason input to the operation.
   * - @p waypoints: Supplies waypoints input to the operation.
   * - @p recovery_trail: Supplies recovery trail input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactiveResult(
      ReactiveStatus result_status, std::optional<domain::Action> result_action,
      std::string result_planner, std::string result_explanation,
      ReactiveCompletionReason reason,
      std::vector<domain::Point2D> waypoints = {},
      std::optional<domain::LearnedTrail> recovery_trail = std::nullopt)
      : status(result_status),
        action(result_action),
        planner(std::move(result_planner)),
        explanation(std::move(result_explanation)),
        completion_reason(reason),
        prepend_waypoints(std::move(waypoints)),
        learned_recovery_trail(std::move(recovery_trail)) {}
};

/**
 * @brief Encapsulates reactive planner state and behavior for this
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
class ReactivePlanner {
 public:
  /**
   * @brief Performs the reactive planner operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual ~ReactivePlanner() = default;
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
  virtual std::string_view name() const noexcept = 0;
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
  virtual std::vector<std::string_view> dependencies() const = 0;
  /**
   * @brief Evaluates trigger for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `TriggerEvaluation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual TriggerEvaluation evaluateTrigger(
      const decision::DecisionContext&) const = 0;
  /**
   * @brief Updates package content for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `ReactivePlanUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual ReactivePlanUpdate update(const decision::DecisionContext&) = 0;
  /**
   * @brief Performs the cancel operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual void cancel(InterruptionReason) = 0;
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - `ReactiveResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactiveResult evaluate(const ReactiveRequest&);
};

/**
 * @brief Encapsulates thru state and behavior for this subsystem.
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
class Thru final : public ReactivePlanner {
 public:
  /**
   * @brief Performs the thru operation for this subsystem.
   *
   * Arguments:
   * - @p decision_budget: Supplies decision budget input to the operation.
   * - @p desired_step_m: Supplies desired step m input to the operation.
   * - @p endpoint_tolerance_m: Supplies endpoint tolerance m input to the
   * operation.
   * - @p beam_neighborhood_half_width: Supplies beam neighborhood half
   * width input to the operation.
   * - @p minimum_clear_beams: Supplies minimum clear beams input to the
   * operation.
   * - @p openness_bundle_beams: Supplies openness bundle beams input to the
   * operation.
   * - @p corridor_half_width_m: Supplies corridor half width m input to the
   * operation.
   * - @p corridor_longitudinal_tolerance_m: Supplies corridor longitudinal
   * tolerance m input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit Thru(std::size_t decision_budget = 20U, double desired_step_m = 0.8,
                double endpoint_tolerance_m = 0.75,
                std::size_t beam_neighborhood_half_width = 2U,
                std::size_t minimum_clear_beams = 3U,
                std::size_t openness_bundle_beams = 5U,
                double corridor_half_width_m = 0.25,
                double corridor_longitudinal_tolerance_m = 0.5);
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
  std::string_view name() const noexcept override { return "Thru"; }
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
    return {"active_waypoint", "laser"};
  }
  /**
   * @brief Evaluates trigger for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `TriggerEvaluation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  TriggerEvaluation evaluateTrigger(
      const decision::DecisionContext&) const override;
  /**
   * @brief Updates package content for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `ReactivePlanUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactivePlanUpdate update(const decision::DecisionContext&) override;
  /**
   * @brief Performs the cancel operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void cancel(InterruptionReason) override;

 private:
  /**
   * @brief Encapsulates sensed objective state and behavior for this
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
  struct SensedObjective {
    domain::Point2D point;
    std::size_t ray_index{0U};
    double distance_m{0.0};
    bool mission_target{false};
  };
  /**
   * @brief Encapsulates endpoint choice state and behavior for this
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
  struct EndpointChoice {
    domain::Point2D point;
    std::string side;
  };

  /**
   * @brief Performs the sensed objective operation for this subsystem.
   *
   * Arguments:
   * - @p WorldModel: Supplies world model input to the operation.
   *
   * Returns:
   * - `std::optional<SensedObjective>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<SensedObjective> sensedObjective(
      const domain::WorldModel&) const;
  /**
   * @brief Performs the choose endpoint operation for this subsystem.
   *
   * Arguments:
   * - @p WorldModel: Supplies world model input to the operation.
   * - @p argument_2: Supplies argument 2 input to the operation.
   *
   * Returns:
   * - `std::optional<EndpointChoice>` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::optional<EndpointChoice> chooseEndpoint(const domain::WorldModel&,
                                               const SensedObjective&) const;
  std::optional<domain::Point2D> endpoint_;
  std::optional<domain::TaskId> mission_id_;
  std::string selected_side_;
  std::string objective_kind_;
  std::size_t decisions_ = 0U;
  std::size_t decision_budget_;
  double desired_step_m_;
  double endpoint_tolerance_m_;
  std::size_t beam_neighborhood_half_width_;
  std::size_t minimum_clear_beams_;
  std::size_t openness_bundle_beams_;
  double corridor_half_width_m_;
  double corridor_longitudinal_tolerance_m_;
};

/**
 * @brief Encapsulates behind state and behavior for this subsystem.
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
class Behind final : public ReactivePlanner {
 public:
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
  std::string_view name() const noexcept override { return "Behind"; }
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
    return {"active_waypoint", "laser", "navigation_history"};
  }
  /**
   * @brief Evaluates trigger for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `TriggerEvaluation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  TriggerEvaluation evaluateTrigger(
      const decision::DecisionContext&) const override;
  /**
   * @brief Updates package content for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `ReactivePlanUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactivePlanUpdate update(const decision::DecisionContext&) override;
  /**
   * @brief Performs the cancel operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void cancel(InterruptionReason) override {}
};

/**
 * @brief Encapsulates out state and behavior for this subsystem.
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
class Out final : public ReactivePlanner {
 public:
  /**
   * @brief Performs the out operation for this subsystem.
   *
   * Arguments:
   * - @p coverage_threshold: Supplies coverage threshold input to the
   * operation.
   * - @p covered_fraction: Supplies covered fraction input to the
   * operation.
   * - @p maximum_new_cells: Supplies maximum new cells input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit Out(std::size_t coverage_threshold = 4U,
               double covered_fraction = 0.75,
               std::size_t maximum_new_cells = 1U);
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
  std::string_view name() const noexcept override { return "Out"; }
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
    return {"recovery_state", "navigation_history", "path_history", "laser"};
  }
  /**
   * @brief Evaluates trigger for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `TriggerEvaluation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  TriggerEvaluation evaluateTrigger(
      const decision::DecisionContext&) const override;
  /**
   * @brief Updates package content for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `ReactivePlanUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactivePlanUpdate update(const decision::DecisionContext&) override;
  /**
   * @brief Performs the cancel operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void cancel(InterruptionReason) override;

 private:
  /**
   * @brief Enumerates the supported state values used by this subsystem.
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
  enum class State { Idle, Survey };
  /**
   * @brief Resets package content for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void reset() noexcept;
  /**
   * @brief Builds escape for this subsystem.
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
  void buildEscape(const domain::WorldModel&);

  State state_ = State::Idle;
  std::optional<domain::TaskId> mission_id_;
  std::size_t rotations_ = 0U;
  std::vector<domain::Point2D> escape_points_;
  std::optional<domain::LearnedTrail> recovery_trail_;
  std::size_t coverage_threshold_;
  double covered_fraction_;
  std::size_t maximum_new_cells_;
};

/**
 * @brief Encapsulates reactive planner coordinator state and behavior for
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
class ReactivePlannerCoordinator {
 public:
  /**
   * @brief Encapsulates evaluation state and behavior for this subsystem.
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
  struct Evaluation {
    ReactiveResult result;
    std::vector<decision::DecisionCycleEvent> trace;
  };
  /**
   * @brief Performs the reactive planner coordinator operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactivePlannerCoordinator();
  /**
   * @brief Performs the reactive planner coordinator operation for this
   * subsystem.
   *
   * Arguments:
   * - @p planners: Supplies planners input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit ReactivePlannerCoordinator(
      std::vector<std::unique_ptr<ReactivePlanner>> planners);
  /**
   * @brief Performs the add operation for this subsystem.
   *
   * Arguments:
   * - @p planner: Supplies planner input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void add(std::unique_ptr<ReactivePlanner> planner);
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p request: Supplies request input to the operation.
   *
   * Returns:
   * - `ReactiveResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactiveResult evaluate(const ReactiveRequest& request);
  /**
   * @brief Evaluates detailed for this subsystem.
   *
   * Arguments:
   * - @p request: Supplies request input to the operation.
   * - @p viable_actions: Supplies viable actions input to the operation.
   *
   * Returns:
   * - `Evaluation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  Evaluation evaluateDetailed(const ReactiveRequest& request,
                              std::span<const domain::Action> viable_actions);
  /**
   * @brief Performs the cancel all operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void cancelAll(InterruptionReason);

 private:
  std::vector<std::unique_ptr<ReactivePlanner>> planners_;
};

/**
 * @brief Enumerates the supported llecandidate source values used by this
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
enum class LLECandidateSource {
  UnfinishedHle,
  CurrentTargetObservation,
  RegionVisibility,
  InclusionGap,
  IncludedRelocation
};

/**
 * @brief Enumerates the supported candidate start plan outcome values used
 * by this subsystem.
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
enum class CandidateStartPlanOutcome {
  NotAttempted,
  AlreadySatisfied,
  Succeeded,
  Failed,
  Invalidated
};

/**
 * @brief Encapsulates llecandidate state and behavior for this subsystem.
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
struct LLECandidate {
  std::uint64_t id = 0U;
  LLECandidateSource source = LLECandidateSource::CurrentTargetObservation;
  domain::Point2D start;
  domain::Point2D target;
  double target_relevance = 0.0;
  bool validated_cue = false;
};

/**
 * @brief Enumerates the supported llebehavior policy values used by this
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
enum class LLEBehaviorPolicy { Compatibility, Modernized };

/**
 * @brief Encapsulates low level exploration configuration state and
 * behavior for this subsystem.
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
struct LowLevelExplorationConfiguration {
  LLEBehaviorPolicy behavior_policy{LLEBehaviorPolicy::Modernized};
  bool stalled_history_extension{true};
  std::size_t history_window{4U};
  double progress_threshold_m{0.1};
  std::size_t decision_budget{64U};
  double minimum_cue_length_m{2.0};
  double target_cue_tolerance_m{5.0};
  std::size_t cue_waypoint_count{20U};
  double closest_target_bin_m{1.0};
  std::uint32_t random_seed{0U};
};

/**
 * @brief Encapsulates low level explorer state and behavior for this
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
class LowLevelExplorer final : public ReactivePlanner,
                               public decision::ReplanningTrigger {
 public:
  /**
   * @brief Performs the low level explorer operation for this subsystem.
   *
   * Arguments:
   * - @p history_window: Supplies history window input to the operation.
   * - @p progress_threshold_m: Supplies progress threshold m input to the
   * operation.
   * - @p decision_budget: Supplies decision budget input to the operation.
   * - @p minimum_cue_length_m: Supplies minimum cue length m input to the
   * operation.
   * - @p target_cue_tolerance_m: Supplies target cue tolerance m input to
   * the operation.
   * - @p cue_waypoint_count: Supplies cue waypoint count input to the
   * operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit LowLevelExplorer(std::size_t history_window = 4U,
                            double progress_threshold_m = 0.1,
                            std::size_t decision_budget = 64U,
                            double minimum_cue_length_m = 2.0,
                            double target_cue_tolerance_m = 5.0,
                            std::size_t cue_waypoint_count = 20U);
  /**
   * @brief Performs the low level explorer operation for this subsystem.
   *
   * Arguments:
   * - @p configuration: Supplies configuration input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  explicit LowLevelExplorer(LowLevelExplorationConfiguration configuration);
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
  std::string_view name() const noexcept override { return "LLE"; }
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
    return {"active_target", "laser", "regions", "inclusion_grid",
            "unfinished_hle_candidates"};
  }
  /**
   * @brief Evaluates trigger for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `TriggerEvaluation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  TriggerEvaluation evaluateTrigger(
      const decision::DecisionContext&) const override;
  /**
   * @brief Evaluates replan for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `decision::ReplanningRequest` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  decision::ReplanningRequest evaluateReplan(
      const decision::DecisionContext&) const override;
  /**
   * @brief Updates package content for this subsystem.
   *
   * Arguments:
   * - @p DecisionContext: Supplies decision context input to the operation.
   *
   * Returns:
   * - `ReactivePlanUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactivePlanUpdate update(const decision::DecisionContext&) override;
  /**
   * @brief Performs the cancel operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void cancel(InterruptionReason) override;
  /**
   * @brief Evaluates package content for this subsystem.
   *
   * Arguments:
   * - @p request: Supplies request input to the operation.
   *
   * Returns:
   * - `ReactiveResult` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactiveResult evaluate(const ReactiveRequest& request);
  /**
   * @brief Performs the state operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `LowLevelExplorationState` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  LowLevelExplorationState state() const noexcept { return state_; }
  /**
   * @brief Performs the completion reason operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `ReactiveCompletionReason` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactiveCompletionReason completionReason() const noexcept {
    return completion_reason_;
  }
  /**
   * @brief Performs the candidates operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<LLECandidate>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<LLECandidate>& candidates() const noexcept {
    return ranked_candidates_;
  }
  /**
   * @brief Performs the last trigger operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const TriggerEvaluation&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const TriggerEvaluation& lastTrigger() const noexcept {
    return last_trigger_;
  }
  /**
   * @brief Performs the last trigger reason code operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::string&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::string& lastTriggerReasonCode() const noexcept {
    return last_trigger_reason_code_;
  }
  /**
   * @brief Performs the candidate start plan outcome operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `CandidateStartPlanOutcome` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  CandidateStartPlanOutcome candidateStartPlanOutcome() const noexcept {
    return start_plan_outcome_;
  }
  /**
   * @brief Performs the candidate start plan reason operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::string&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::string& candidateStartPlanReason() const noexcept {
    return start_plan_reason_;
  }
  /**
   * @brief Performs the candidate start plan operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<domain::Point2D>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<domain::Point2D>& candidateStartPlan() const noexcept {
    return start_connection_waypoints_;
  }
  /**
   * @brief Performs the candidate start diagnostics operation for this
   * subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<std::string>&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<std::string>& candidateStartDiagnostics() const noexcept {
    return candidate_start_diagnostics_;
  }
  /**
   * @brief Performs the cue waypoints operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<domain::Point2D>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const std::vector<domain::Point2D>& cueWaypoints() const noexcept {
    return cue_waypoints_;
  }

 private:
  /**
   * @brief Performs the assemble candidates operation for this subsystem.
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
  void assembleCandidates(const domain::WorldModel&);
  /**
   * @brief Performs the append current view candidates operation for this
   * subsystem.
   *
   * Arguments:
   * - @p WorldModel: Supplies world model input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool appendCurrentViewCandidates(const domain::WorldModel&);
  /**
   * @brief Performs the install candidate waypoints operation for this
   * subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void installCandidateWaypoints(const LLECandidate&);
  /**
   * @brief Performs the select fallback operation for this subsystem.
   *
   * Arguments:
   * - @p candidates: Supplies candidates input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void selectFallback(std::vector<LLECandidate> candidates);
  /**
   * @brief Constructs candidate start for this subsystem.
   *
   * Arguments:
   * - @p WorldModel: Supplies world model input to the operation.
   * - @p argument_2: Supplies argument 2 input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool planCandidateStart(const domain::WorldModel&, const LLECandidate&);
  /**
   * @brief Performs the candidate start plan valid operation for this
   * subsystem.
   *
   * Arguments:
   * - @p WorldModel: Supplies world model input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool candidateStartPlanValid(const domain::WorldModel&) const;
  /**
   * @brief Performs the advance candidate operation for this subsystem.
   *
   * Arguments:
   * - @p WorldModel: Supplies world model input to the operation.
   *
   * Returns:
   * - `bool` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  bool advanceCandidate(const domain::WorldModel&);
  /**
   * @brief Performs the included cell count operation for this subsystem.
   *
   * Arguments:
   * - @p WorldModel: Supplies world model input to the operation.
   *
   * Returns:
   * - `std::size_t` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::size_t includedCellCount(const domain::WorldModel&) const noexcept;
  /**
   * @brief Performs the action toward operation for this subsystem.
   *
   * Arguments:
   * - @p Pose2D: Supplies pose2 d input to the operation.
   * - @p Point2D: Supplies point2 d input to the operation.
   * - @p ActionSpace: Supplies action space input to the operation.
   *
   * Returns:
   * - `domain::Action` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  domain::Action actionToward(const domain::Pose2D&, domain::Point2D,
                              const domain::ActionSpace&) const;
  /**
   * @brief Performs the complete operation for this subsystem.
   *
   * Arguments:
   * - @p argument_1: Supplies argument 1 input to the operation.
   * - @p string: Supplies string input to the operation.
   * - @p argument_3: Supplies argument 3 input to the operation.
   *
   * Returns:
   * - `ReactivePlanUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  ReactivePlanUpdate complete(ReactiveCompletionReason, std::string,
                              ReactiveStatus = ReactiveStatus::NotApplicable);

  std::size_t history_window_;
  double progress_threshold_m_;
  std::size_t decision_budget_;
  LowLevelExplorationState state_ =
      LowLevelExplorationState::DetectMissingGuidance;
  ReactiveCompletionReason completion_reason_ = ReactiveCompletionReason::None;
  std::vector<LLECandidate> ranked_candidates_;
  std::size_t candidate_cursor_ = 0U;
  std::size_t decisions_ = 0U;
  std::optional<domain::TaskId> mission_id_;
  std::vector<domain::Point2D> plan_at_start_;
  std::size_t waypoint_index_at_start_ = 0U;
  domain::DependencyRevisions source_revisions_;
  std::uint64_t next_candidate_id_ = 1U;
  double minimum_cue_length_m_;
  double target_cue_tolerance_m_;
  std::size_t cue_waypoint_count_;
  std::vector<domain::Point2D> cue_waypoints_;
  std::size_t waypoint_cursor_ = 0U;
  std::size_t lost_waypoint_cycles_ = 0U;
  std::vector<domain::Point2D> start_connection_waypoints_;
  std::size_t start_connection_cursor_ = 0U;
  domain::Revision start_connection_inclusion_revision_ = 0U;
  bool start_connection_uses_inclusion_{false};
  CandidateStartPlanOutcome start_plan_outcome_ =
      CandidateStartPlanOutcome::NotAttempted;
  std::string start_plan_reason_{"not_attempted"};
  std::vector<std::string> candidate_start_diagnostics_;
  std::size_t initial_included_cells_ = 0U;
  LowLevelExplorationConfiguration configuration_;
  mutable TriggerEvaluation last_trigger_;
  mutable std::string last_trigger_reason_code_{"none"};
  std::mt19937 random_;
};

/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p argument_1: Supplies argument 1 input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(ReactiveCompletionReason) noexcept;
/**
 * @brief Converts string for this subsystem.
 *
 * Arguments:
 * - @p argument_1: Supplies argument 1 input to the operation.
 *
 * Returns:
 * - `std::string_view` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::string_view toString(LowLevelExplorationState) noexcept;

}  // namespace semaforr::planning

#endif
