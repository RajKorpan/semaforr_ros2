#ifndef SEMAFORR_WHY_WHY_SYSTEM_HPP
#define SEMAFORR_WHY_WHY_SYSTEM_HPP

#include <functional>
#include <map>
#include <optional>
#include <semaforr_msgs/msg/decision_record.hpp>
#include <semaforr_msgs/msg/explanation_question.hpp>
#include <semaforr_msgs/msg/explanation_response.hpp>
#include <vector>

namespace semaforr::why {

using DecisionRecord = semaforr_msgs::msg::DecisionRecord;
using ExplanationQuestion = semaforr_msgs::msg::ExplanationQuestion;
using ExplanationResponse = semaforr_msgs::msg::ExplanationResponse;

class ExplanationTraceStore {
public:
  void record(const DecisionRecord &record);
  const DecisionRecord *latestDecision() const noexcept;
  const DecisionRecord *decision(std::uint64_t id) const noexcept;
  const DecisionRecord *action(std::uint64_t id) const noexcept;
  const DecisionRecord *execution(std::uint64_t id) const noexcept;
  const DecisionRecord *task(std::uint64_t id) const noexcept;
  const DecisionRecord *currentTask() const noexcept;
  const DecisionRecord *plan(std::uint64_t id) const noexcept;
  const DecisionRecord *planningEpisode(std::uint64_t id) const noexcept;
  std::vector<std::uint64_t> decisionIds() const;

private:
  std::map<std::uint64_t, DecisionRecord> decisions_;
  std::map<std::uint64_t, std::uint64_t> action_to_decision_;
  std::map<std::uint64_t, std::uint64_t> execution_to_decision_;
  std::map<std::uint64_t, std::uint64_t> task_to_decision_;
  std::map<std::uint64_t, std::uint64_t> plan_to_decision_;
  std::map<std::uint64_t, std::uint64_t> episode_to_decision_;
};

class UnifiedWhySystem {
public:
  using HypotheticalEvaluator =
      std::function<DecisionRecord(const geometry_msgs::msg::Pose2D &)>;
  using RouteEvaluator = std::function<std::vector<double>(
      const std::vector<geometry_msgs::msg::Point> &,
      const std::vector<std::string> &)>;

  void record(const DecisionRecord &record) { traces_.record(record); }
  void setHypotheticalEvaluator(HypotheticalEvaluator evaluator) {
    hypothetical_evaluator_ = std::move(evaluator);
  }
  void setRouteEvaluator(RouteEvaluator evaluator) {
    route_evaluator_ = std::move(evaluator);
  }
  ExplanationResponse answer(const ExplanationQuestion &question) const;
  const ExplanationTraceStore &traces() const noexcept { return traces_; }

private:
  const DecisionRecord *
  resolveDecision(const ExplanationQuestion &question) const noexcept;
  ExplanationResponse baseResponse(const ExplanationQuestion &question,
                                   const DecisionRecord *record) const;
  ExplanationResponse explainDecision(const ExplanationQuestion &,
                                      const DecisionRecord &) const;
  ExplanationResponse explainCounterfactual(const ExplanationQuestion &,
                                            const DecisionRecord &) const;
  ExplanationResponse explainDecisionConfidence(const ExplanationQuestion &,
                                                const DecisionRecord &) const;
  ExplanationResponse explainPlan(const ExplanationQuestion &,
                                  const DecisionRecord &) const;
  ExplanationResponse explainAlternativePlan(const ExplanationQuestion &,
                                             const DecisionRecord &) const;
  ExplanationResponse explainPlanConfidence(const ExplanationQuestion &,
                                            const DecisionRecord &) const;
  ExplanationResponse explainRoute(const ExplanationQuestion &,
                                   const DecisionRecord &) const;
  ExplanationResponse comparePlan(const ExplanationQuestion &,
                                  const DecisionRecord &) const;
  ExplanationResponse explainHypothetical(const ExplanationQuestion &) const;

  mutable std::uint64_t next_explanation_id_{1U};
  ExplanationTraceStore traces_;
  HypotheticalEvaluator hypothetical_evaluator_;
  RouteEvaluator route_evaluator_;
};

} // namespace semaforr::why

#endif
