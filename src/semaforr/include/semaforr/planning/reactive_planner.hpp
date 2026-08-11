#ifndef SEMAFORR_PLANNING_REACTIVE_PLANNER_HPP
#define SEMAFORR_PLANNING_REACTIVE_PLANNER_HPP

#include <cstdint>
#include <memory>
#include <optional>
#include <queue>
#include <semaforr/decision/context.hpp>
#include <semaforr/decision/rules.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::planning {

enum class ReactiveStatus { NotApplicable, Action, RequestReplan };
enum class InterruptionReason { TargetSensed, NewPlanAvailable, SensorLost,
                                MissionChanged, Disabled };
enum class ReactiveCompletionReason {
  None, TargetSensed, NewPlanAvailable, CandidateExhausted, NoCandidates,
  BudgetExceeded, SensorLost, MissionChanged
};
enum class LowLevelExplorationState {
  DetectMissingGuidance, AssembleCandidateRays, RankByTargetRelevance,
  PlanToCandidateStart, PursueCandidate, CheckConnectivity, Complete
};

struct TriggerEvaluation {
  bool triggered = false;
  std::string rationale;
};

struct ReactivePlanUpdate {
  ReactiveStatus status = ReactiveStatus::NotApplicable;
  std::optional<domain::Action> action;
  LowLevelExplorationState state =
      LowLevelExplorationState::DetectMissingGuidance;
  ReactiveCompletionReason completion_reason = ReactiveCompletionReason::None;
  std::optional<std::uint64_t> candidate_id;
  std::string explanation;
};

struct ReactiveRequest {
  const domain::WorldModel& world;
  const domain::ActionSpace& action_space;
};

struct ReactiveResult {
  ReactiveStatus status = ReactiveStatus::NotApplicable;
  std::optional<domain::Action> action;
  std::string planner;
  std::string explanation;
  ReactiveCompletionReason completion_reason = ReactiveCompletionReason::None;
};

class ReactivePlanner {
 public:
  virtual ~ReactivePlanner() = default;
  virtual std::string_view name() const noexcept = 0;
  virtual std::vector<std::string_view> dependencies() const = 0;
  virtual TriggerEvaluation evaluateTrigger(
      const decision::DecisionContext&) const = 0;
  virtual ReactivePlanUpdate update(const decision::DecisionContext&) = 0;
  virtual void cancel(InterruptionReason) = 0;
  ReactiveResult evaluate(const ReactiveRequest&);
};

class Thru final : public ReactivePlanner {
 public:
  explicit Thru(std::size_t decision_budget = 20U,
                double desired_step_m = 0.8,
                double endpoint_tolerance_m = 0.75);
  std::string_view name() const noexcept override { return "Thru"; }
  std::vector<std::string_view> dependencies() const override {
    return {"active_waypoint", "laser"};
  }
  TriggerEvaluation evaluateTrigger(
      const decision::DecisionContext&) const override;
  ReactivePlanUpdate update(const decision::DecisionContext&) override;
  void cancel(InterruptionReason) override;

 private:
  std::optional<domain::Point2D> chooseEndpoint(
      const domain::WorldModel&) const;
  std::optional<domain::Point2D> endpoint_;
  std::optional<domain::TaskId> mission_id_;
  std::size_t decisions_ = 0U;
  std::size_t decision_budget_;
  double desired_step_m_;
  double endpoint_tolerance_m_;
};

class Behind final : public ReactivePlanner {
 public:
  std::string_view name() const noexcept override { return "Behind"; }
  std::vector<std::string_view> dependencies() const override {
    return {"active_waypoint", "laser", "navigation_history"};
  }
  TriggerEvaluation evaluateTrigger(
      const decision::DecisionContext&) const override;
  ReactivePlanUpdate update(const decision::DecisionContext&) override;
  void cancel(InterruptionReason) override {}
};

class Out final : public ReactivePlanner {
 public:
  explicit Out(std::size_t coverage_threshold = 4U,
               double covered_fraction = 0.75,
               std::size_t maximum_new_cells = 1U);
  std::string_view name() const noexcept override { return "Out"; }
  std::vector<std::string_view> dependencies() const override {
    return {"recovery_state", "known_grid"};
  }
  TriggerEvaluation evaluateTrigger(
      const decision::DecisionContext&) const override;
  ReactivePlanUpdate update(const decision::DecisionContext&) override;
  void cancel(InterruptionReason) override;

 private:
  enum class State { Idle, Survey, Escape };
  void reset() noexcept;
  void buildEscape(const domain::WorldModel&);

  State state_ = State::Idle;
  std::optional<domain::TaskId> mission_id_;
  std::size_t rotations_ = 0U;
  std::size_t baseline_known_cells_ = 0U;
  std::vector<domain::Point2D> escape_points_;
  std::size_t escape_cursor_ = 0U;
  std::size_t coverage_threshold_;
  double covered_fraction_;
  std::size_t maximum_new_cells_;
};

class ReactivePlannerCoordinator {
 public:
  ReactivePlannerCoordinator();
  explicit ReactivePlannerCoordinator(
      std::vector<std::unique_ptr<ReactivePlanner>> planners);
  void add(std::unique_ptr<ReactivePlanner> planner);
  ReactiveResult evaluate(const ReactiveRequest& request);
  void cancelAll(InterruptionReason);

 private:
  std::vector<std::unique_ptr<ReactivePlanner>> planners_;
};

enum class LLECandidateSource {
  UnfinishedHle, CurrentTargetObservation, RegionVisibility, InclusionGap
};

struct LLECandidate {
  std::uint64_t id = 0U;
  LLECandidateSource source = LLECandidateSource::CurrentTargetObservation;
  domain::Point2D start;
  domain::Point2D target;
  double target_relevance = 0.0;
  bool validated_cue = false;
};

class LowLevelExplorer final : public ReactivePlanner,
                               public decision::ReplanningTrigger {
 public:
  explicit LowLevelExplorer(std::size_t history_window = 4U,
                            double progress_threshold_m = 0.1,
                            std::size_t decision_budget = 64U,
                            double minimum_cue_length_m = 2.0,
                            double target_cue_tolerance_m = 5.0,
                            std::size_t cue_waypoint_count = 20U);
  std::string_view name() const noexcept override { return "LLE"; }
  std::vector<std::string_view> dependencies() const override {
    return {"active_target", "laser", "regions", "inclusion_grid",
            "unfinished_hle_candidates"};
  }
  TriggerEvaluation evaluateTrigger(
      const decision::DecisionContext&) const override;
  decision::ReplanningRequest evaluateReplan(
      const decision::DecisionContext&) const override;
  ReactivePlanUpdate update(const decision::DecisionContext&) override;
  void cancel(InterruptionReason) override;
  ReactiveResult evaluate(const ReactiveRequest& request);
  LowLevelExplorationState state() const noexcept { return state_; }
  ReactiveCompletionReason completionReason() const noexcept {
    return completion_reason_;
  }
  const std::vector<LLECandidate>& candidates() const noexcept {
    return ranked_candidates_;
  }

 private:
  void assembleCandidates(const domain::WorldModel&);
  bool appendCurrentViewCandidates(const domain::WorldModel&);
  void installCandidateWaypoints(const LLECandidate&);
  std::size_t includedCellCount(const domain::WorldModel&) const noexcept;
  domain::Action actionToward(const domain::Pose2D&, domain::Point2D,
                              const domain::ActionSpace&) const;
  ReactivePlanUpdate complete(ReactiveCompletionReason, std::string,
                              ReactiveStatus = ReactiveStatus::NotApplicable);

  std::size_t history_window_;
  double progress_threshold_m_;
  std::size_t decision_budget_;
  LowLevelExplorationState state_ =
      LowLevelExplorationState::DetectMissingGuidance;
  ReactiveCompletionReason completion_reason_ =
      ReactiveCompletionReason::None;
  std::vector<LLECandidate> ranked_candidates_;
  std::size_t candidate_cursor_ = 0U;
  std::size_t decisions_ = 0U;
  std::optional<domain::TaskId> mission_id_;
  domain::DependencyRevisions source_revisions_;
  std::uint64_t next_candidate_id_ = 1U;
  double minimum_cue_length_m_;
  double target_cue_tolerance_m_;
  std::size_t cue_waypoint_count_;
  std::vector<domain::Point2D> cue_waypoints_;
  std::size_t waypoint_cursor_ = 0U;
  std::size_t lost_waypoint_cycles_ = 0U;
  std::size_t initial_included_cells_ = 0U;
};

std::string_view toString(ReactiveCompletionReason) noexcept;
std::string_view toString(LowLevelExplorationState) noexcept;

}  // namespace semaforr::planning

#endif
