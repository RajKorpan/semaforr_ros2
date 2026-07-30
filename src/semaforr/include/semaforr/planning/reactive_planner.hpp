#ifndef SEMAFORR_PLANNING_REACTIVE_PLANNER_HPP
#define SEMAFORR_PLANNING_REACTIVE_PLANNER_HPP

#include <optional>
#include <memory>
#include <semaforr/domain/world_model.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::planning {

enum class ReactiveStatus { NotApplicable, Action, RequestReplan };

struct ReactiveRequest {
  const domain::WorldModel& world;
  const domain::ActionSpace& action_space;
};

struct ReactiveResult {
  ReactiveStatus status = ReactiveStatus::NotApplicable;
  std::optional<domain::Action> action;
  std::string planner;
  std::string explanation;
};

class ReactivePlanner {
 public:
  virtual ~ReactivePlanner() = default;
  virtual std::string_view name() const noexcept = 0;
  virtual std::vector<std::string_view> dependencies() const = 0;
  virtual ReactiveResult evaluate(const ReactiveRequest& request) const = 0;
};

class Thru final : public ReactivePlanner {
 public:
  std::string_view name() const noexcept override { return "Thru"; }
  std::vector<std::string_view> dependencies() const override {
    return {"active_waypoint", "laser"};
  }
  ReactiveResult evaluate(const ReactiveRequest& request) const override;
};

class Behind final : public ReactivePlanner {
 public:
  std::string_view name() const noexcept override { return "Behind"; }
  std::vector<std::string_view> dependencies() const override {
    return {"active_waypoint"};
  }
  ReactiveResult evaluate(const ReactiveRequest& request) const override;
};

class Out final : public ReactivePlanner {
 public:
  std::string_view name() const noexcept override { return "Out"; }
  std::vector<std::string_view> dependencies() const override {
    return {"recovery_state", "inclusion_grid"};
  }
  ReactiveResult evaluate(const ReactiveRequest& request) const override;
};

class ReactivePlannerCoordinator {
 public:
  ReactivePlannerCoordinator();
  void add(std::unique_ptr<ReactivePlanner> planner);
  ReactiveResult evaluate(const ReactiveRequest& request) const;

 private:
  std::vector<std::unique_ptr<ReactivePlanner>> planners_;
};

class LowLevelExplorer {
 public:
  explicit LowLevelExplorer(std::size_t history_window = 4U,
                            double progress_threshold_m = 0.1);
  ReactiveResult evaluate(const ReactiveRequest& request) const;

 private:
  std::size_t history_window_;
  double progress_threshold_m_;
  mutable std::optional<std::size_t> last_missing_knowledge_revision_;
};

}  // namespace semaforr::planning

#endif
