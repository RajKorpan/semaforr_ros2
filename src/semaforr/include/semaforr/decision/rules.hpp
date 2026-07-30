#ifndef SEMAFORR_DECISION_RULES_HPP
#define SEMAFORR_DECISION_RULES_HPP

#include <optional>
#include <semaforr/decision/context.hpp>
#include <semaforr/decision/decision_result.hpp>
#include <string>
#include <vector>

namespace semaforr::decision {

struct Decision {
  domain::Action action;
  std::string rule;
  std::string explanation;
};

class MandatoryRule {
 public:
  virtual ~MandatoryRule() = default;
  virtual std::optional<Decision> evaluate(
      const DecisionContext& context) const = 0;
};

class VetoRule {
 public:
  virtual ~VetoRule() = default;
  virtual std::vector<Veto> evaluate(const DecisionContext& context) const = 0;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_RULES_HPP
