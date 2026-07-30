#ifndef SEMAFORR_PLANNING_PLANNER_HPP
#define SEMAFORR_PLANNING_PLANNER_HPP

#include <semaforr/domain/geometry.hpp>
#include <semaforr/domain/world_model.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace semaforr::planning {

enum class PlanStatus { Success, NoPath, InvalidRequest, PlannerUnavailable };

struct PlanningRequest {
  domain::Pose2D start;
  domain::Point2D goal;
  const domain::SpatialModel* spatial_model{nullptr};
  const domain::CrowdModel* crowd_model{nullptr};
};

struct PlanResult {
  PlanStatus status{PlanStatus::PlannerUnavailable};
  std::vector<domain::Point2D> path;
  double cost_m{0.0};
  std::string explanation;

  bool succeeded() const noexcept { return status == PlanStatus::Success; }
};

class Planner {
 public:
  virtual ~Planner() = default;
  virtual PlanResult plan(const PlanningRequest& request) = 0;
  virtual std::string_view name() const noexcept = 0;
};

}  // namespace semaforr::planning

#endif  // SEMAFORR_PLANNING_PLANNER_HPP
