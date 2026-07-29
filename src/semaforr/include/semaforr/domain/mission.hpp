#ifndef SEMAFORR_DOMAIN_MISSION_HPP
#define SEMAFORR_DOMAIN_MISSION_HPP

#include <cstddef>
#include <deque>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <semaforr/domain/geometry.hpp>

namespace semaforr::domain {

using TaskId = std::size_t;

struct NavigationTask {
  TaskId id = 0U;
  Point2D target;
};

class Mission {
public:
  explicit Mission(
    std::vector<NavigationTask> tasks = {},
    std::size_t decision_limit = 1U)
    : decision_limit_(decision_limit)
  {
    if (decision_limit_ == 0U) {
      throw std::invalid_argument("mission decision limit must be positive");
    }
    for (NavigationTask& task : tasks) {
      pending_.push_back(std::move(task));
    }
  }

  bool activate_next()
  {
    if (active_ || pending_.empty()) {
      return false;
    }
    active_ = std::move(pending_.front());
    pending_.pop_front();
    decisions_for_active_ = 0U;
    return true;
  }

  bool record_decision()
  {
    if (!active_) {
      return false;
    }
    ++decisions_for_active_;
    return decisions_for_active_ <= decision_limit_;
  }

  bool complete_active()
  {
    if (!active_) {
      return false;
    }
    completed_.push_back(std::move(*active_));
    active_.reset();
    decisions_for_active_ = 0U;
    return true;
  }

  bool skip_active()
  {
    if (!active_) {
      return false;
    }
    skipped_.push_back(std::move(*active_));
    active_.reset();
    decisions_for_active_ = 0U;
    return true;
  }

  const std::deque<NavigationTask>& pending() const noexcept { return pending_; }
  const std::optional<NavigationTask>& active() const noexcept { return active_; }
  const std::vector<NavigationTask>& completed() const noexcept
  {
    return completed_;
  }
  const std::vector<NavigationTask>& skipped() const noexcept { return skipped_; }
  std::size_t decision_limit() const noexcept { return decision_limit_; }
  std::size_t decisions_for_active() const noexcept
  {
    return decisions_for_active_;
  }
  bool finished() const noexcept { return pending_.empty() && !active_; }

private:
  std::deque<NavigationTask> pending_;
  std::optional<NavigationTask> active_;
  std::vector<NavigationTask> completed_;
  std::vector<NavigationTask> skipped_;
  std::size_t decision_limit_;
  std::size_t decisions_for_active_ = 0U;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_MISSION_HPP
