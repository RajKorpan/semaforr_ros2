#ifndef SEMAFORR_DECISION_DECISION_RESULT_HPP
#define SEMAFORR_DECISION_DECISION_RESULT_HPP

#include <cstdint>
#include <optional>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/action_execution.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/navigation/navigation_phase.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace semaforr::decision {

enum class DecisionSource {
  MandatoryRule,
  TierThreeAdvisor,
  Planner,
  Exploration,
  Fallback,
  SafeStop
};

enum class DecisionTier {
  TierOne,
  TierTwo,
  TierThree,
  Exploration,
  Fallback,
  SafeStop
};

enum class ActionOutcome {
  Pending,
  Completed,
  TimedOut,
  OdometryReset,
  ClockReset,
  Cancelled,
  SensorLost,
  Shutdown,
  PartialMovement,
  NoMovement,
  SafetyInterrupted,
  ControllerRejected,
  ControllerFailure,
  GoalPreempted,
  NavigationModeTransition
};

struct Veto {
  domain::Action action;
  std::string rule;
  std::string explanation;

  bool operator==(const Veto&) const = default;
};

struct DecisionCycleEvent {
  std::size_t order{0U};
  std::string tier;
  std::string component;
  std::vector<domain::Action> input_actions;
  std::optional<domain::Action> mandate;
  std::vector<Veto> vetoes;
  std::string outcome;
  bool returned_to_earlier_tier{false};
  std::optional<DecisionTier> final_attribution;

  DecisionCycleEvent() = default;
  DecisionCycleEvent(
      std::size_t event_order, std::string event_tier,
      std::string event_component, std::vector<domain::Action> inputs,
      std::optional<domain::Action> event_mandate,
      std::vector<Veto> event_vetoes, std::string event_outcome = {},
      bool returned = false,
      std::optional<DecisionTier> attribution = std::nullopt)
      : order(event_order),
        tier(std::move(event_tier)),
        component(std::move(event_component)),
        input_actions(std::move(inputs)),
        mandate(event_mandate),
        vetoes(std::move(event_vetoes)),
        outcome(std::move(event_outcome)),
        returned_to_earlier_tier(returned),
        final_attribution(attribution) {}

  bool operator==(const DecisionCycleEvent&) const = default;
};

struct AdvisorContribution {
  std::string advisor;
  domain::Action action;
  double raw_score{0.0};
  double weight{1.0};
  double weighted_score{0.0};
  std::string explanation;
  std::size_t model_revision_used{0U};

  bool operator==(const AdvisorContribution&) const = default;
};

struct TaskDiagnostic {
  std::uint64_t task_index{0U};
  std::uint64_t decision_count{0U};
  domain::Point2D target;
  std::optional<domain::Point2D> waypoint;

  bool operator==(const TaskDiagnostic&) const = default;
};

struct DecisionResult {
  std::uint64_t sequence{0U};
  domain::DecisionId decision_id{0U};
  domain::ActionId action_id{0U};
  domain::Pose2D robot_pose;
  navigation::NavigationPhase navigation_phase{
      navigation::NavigationPhase::TargetNavigation};
  std::string configuration_fingerprint;
  std::vector<std::string> component_manifest;
  std::vector<std::string> phase_events;
  std::optional<TaskDiagnostic> task;
  std::vector<domain::Action> candidates;
  domain::Action action{domain::Action::pause()};
  DecisionSource source{DecisionSource::SafeStop};
  DecisionTier tier{DecisionTier::SafeStop};
  std::string selected_policy;
  std::vector<Veto> vetoes;
  std::vector<AdvisorContribution> contributions;
  std::vector<DecisionCycleEvent> decision_cycle;
  std::optional<std::string> planner;
  double decision_latency_s{0.0};
  double planning_latency_s{0.0};
  double model_update_cost_s{0.0};
  std::uint64_t allocation_count{0U};
  std::uint64_t allocation_bytes{0U};
  std::uint64_t covered_cells{0U};
  ActionOutcome action_outcome{ActionOutcome::Pending};
  double action_duration_s{0.0};
  double action_progress{0.0};
  double action_target{0.0};
  std::string outcome_detail;
};

std::string_view toString(DecisionSource source) noexcept;
std::string_view toString(DecisionTier tier) noexcept;
std::string_view toString(ActionOutcome outcome) noexcept;

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_DECISION_RESULT_HPP
