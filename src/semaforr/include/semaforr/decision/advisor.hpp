#ifndef SEMAFORR_DECISION_ADVISOR_HPP
#define SEMAFORR_DECISION_ADVISOR_HPP

#include <semaforr/decision/context.hpp>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::decision {

struct ActionScore {
  domain::Action action;
  double raw_score{0.0};
};

struct AdvisorEvaluation {
  bool participated{false};
  std::vector<ActionScore> scores;
  double weight{1.0};
  std::string explanation;
};

class Advisor {
 public:
  virtual ~Advisor() = default;
  virtual std::string_view name() const noexcept = 0;
  virtual std::vector<std::string_view> dependencies() const { return {}; }
  virtual AdvisorEvaluation evaluate(
      const DecisionContext& context,
      std::span<const domain::Action> candidates) const = 0;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_ADVISOR_HPP
