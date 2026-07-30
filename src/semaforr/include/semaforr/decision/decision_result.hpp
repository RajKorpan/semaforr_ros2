#ifndef SEMAFORR_DECISION_DECISION_RESULT_HPP
#define SEMAFORR_DECISION_DECISION_RESULT_HPP

#include <cstdint>
#include <optional>
#include <semaforr/domain/action.hpp>
#include <semaforr/domain/geometry.hpp>
#include <semaforr/navigation/navigation_phase.hpp>
#include <string>
#include <string_view>
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
  Shutdown
};

struct Veto {
  domain::Action action;
  std::string rule;
  std::string explanation;

  bool operator==(const Veto&) const = default;
};

struct AdvisorContribution {
  std::string advisor;
  domain::Action action;
  double raw_score{0.0};
  double weight{1.0};
  double weighted_score{0.0};
  std::string explanation;

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
  domain::Pose2D robot_pose;
  navigation::NavigationPhase navigation_phase{
      navigation::NavigationPhase::TargetNavigation};
  std::string configuration_fingerprint;
  std::vector<std::string> component_manifest;
  std::optional<TaskDiagnostic> task;
  std::vector<domain::Action> candidates;
  domain::Action action{domain::Action::pause()};
  DecisionSource source{DecisionSource::SafeStop};
  DecisionTier tier{DecisionTier::SafeStop};
  std::string selected_policy;
  std::vector<Veto> vetoes;
  std::vector<AdvisorContribution> contributions;
  std::optional<std::string> planner;
  double decision_latency_s{0.0};
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
