#ifndef SEMAFORR_DECISION_DECISION_RESULT_HPP
#define SEMAFORR_DECISION_DECISION_RESULT_HPP

#include <optional>
#include <string>
#include <vector>

#include <semaforr/domain/action.hpp>

namespace semaforr::decision {

enum class DecisionSource {
  MandatoryRule,
  TierThreeAdvisor,
  Planner,
  Exploration,
  Fallback,
  SafeStop
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

// Compatibility diagnostics keep the legacy log format available while callers
// migrate to the typed veto and contribution collections above.
struct DecisionDiagnostics {
  double legacy_tier{0.0};
  std::string veto_summary;
  std::string advisor_summary;
  std::string advisor_comments;
  std::string advisor_influence;
  double planning_seconds{0.0};
  double learning_seconds{0.0};
  double graphing_seconds{0.0};
  std::string planner_comments;
};

struct DecisionResult {
  domain::Action action{domain::Action::pause()};
  DecisionSource source{DecisionSource::SafeStop};
  std::vector<Veto> vetoes;
  std::vector<AdvisorContribution> contributions;
  std::optional<std::string> planner;
  DecisionDiagnostics diagnostics;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_DECISION_RESULT_HPP
