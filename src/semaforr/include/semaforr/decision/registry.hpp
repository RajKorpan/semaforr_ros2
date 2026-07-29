#ifndef SEMAFORR_DECISION_REGISTRY_HPP
#define SEMAFORR_DECISION_REGISTRY_HPP

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>

#include <semaforr/decision/advisor.hpp>
#include <semaforr/planning/planner.hpp>

namespace semaforr::decision {

class AdvisorRegistry {
public:
  using Factory = std::function<std::unique_ptr<Advisor>()>;

  void registerFactory(std::string name, Factory factory)
  {
    if (name.empty() || !factory) {
      throw std::invalid_argument("advisor registration requires a name and factory");
    }
    if (!factories_.emplace(std::move(name), std::move(factory)).second) {
      throw std::invalid_argument("advisor name is already registered");
    }
  }

  std::unique_ptr<Advisor> create(std::string_view name) const
  {
    const auto found = factories_.find(std::string(name));
    if (found == factories_.end()) {
      throw std::invalid_argument(
        "unknown advisor type '" + std::string(name) + "'");
    }
    return found->second();
  }

private:
  std::unordered_map<std::string, Factory> factories_;
};

class PlannerRegistry {
public:
  using Factory = std::function<std::unique_ptr<planning::Planner>()>;

  void registerFactory(std::string name, Factory factory)
  {
    if (name.empty() || !factory) {
      throw std::invalid_argument("planner registration requires a name and factory");
    }
    if (!factories_.emplace(std::move(name), std::move(factory)).second) {
      throw std::invalid_argument("planner name is already registered");
    }
  }

  std::unique_ptr<planning::Planner> create(std::string_view name) const
  {
    const auto found = factories_.find(std::string(name));
    if (found == factories_.end()) {
      throw std::invalid_argument(
        "unknown planner type '" + std::string(name) + "'");
    }
    return found->second();
  }

private:
  std::unordered_map<std::string, Factory> factories_;
};

}  // namespace semaforr::decision

#endif  // SEMAFORR_DECISION_REGISTRY_HPP
