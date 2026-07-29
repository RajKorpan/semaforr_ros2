/*
 * Tier-two planning implementation.
 */

#include <semaforr/decision/DecisionTier.hpp>
#include <semaforr/decision/Arbitration.hpp>
#include "DecisionTierFactory.h"

#include <semaforr/decision/Beliefs.hpp>
#include <semaforr/decision/Tier1Advisor.hpp>
#include <semaforr/navigation/PathPlanner.hpp>

#include <algorithm>
#include <cmath>
#include <list>
#include <limits>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <utility>
#include <vector>

namespace semaforr {
namespace decision {
namespace {

class DefaultTierTwoDecision final : public TierTwoDecision {
public:
  explicit DefaultTierTwoDecision(TierTwoDependencies dependencies)
    : dependencies_(dependencies) {}

  TierTwoResult plan(Position current, bool select_next_task) override {
    TierTwoResult result;
    Beliefs& beliefs = dependencies_.beliefs;
    std::vector<std::list<int>> plans;
    std::vector<std::string> planner_names;

    if (select_next_task) {
      beliefs.getAgentState()->setCurrentTask(
        beliefs.getAgentState()->getNextTask());
    }

    timeval clock_value;
    gettimeofday(&clock_value, nullptr);
    const double start_time =
      clock_value.tv_sec + (clock_value.tv_usec / 1000000.0);

    bool plan_created = false;
    for (const auto& owned_planner : dependencies_.planners) {
      PathPlanner* planner = owned_planner.get();
      planner->setPosHistory(beliefs.getAgentState()->getAllTrace());
      const std::vector<std::vector<CartesianPoint>> trails_trace =
        beliefs.getSpatialModel()->getTrails()->getTrailsPoints();
      planner->setSpatialModel(
        beliefs.getSpatialModel()->getConveyors(),
        beliefs.getSpatialModel()->getRegionList()->getRegions(),
        beliefs.getSpatialModel()->getDoors()->getDoors(),
        trails_trace,
        beliefs.getSpatialModel()->getHallways()->getHallways());

      if (dependencies_.highway_finished >= 1 ||
          dependencies_.frontier_finished >= 1) {
        planner->setPassageGrid(
          beliefs.getAgentState()->getPassageGrid(),
          beliefs.getAgentState()->getPassageGraphNodes(),
          beliefs.getAgentState()->getPassageGraph(),
          beliefs.getAgentState()->getAveragePassage());
        beliefs.getAgentState()->getCurrentTask()->setPassageValues(
          beliefs.getAgentState()->getPassageGrid(),
          beliefs.getAgentState()->getPassageGraphNodes(),
          beliefs.getAgentState()->getPassageGraphEdges(),
          beliefs.getAgentState()->getPassageGraph(),
          beliefs.getAgentState()->getAveragePassage(),
          beliefs.getAgentState()->getGraphTrails(),
          beliefs.getAgentState()->getGraphThroughIntersections(),
          beliefs.getAgentState()->getGraphIntersectionTrails());
      }
      if (dependencies_.tier_one_advisor.localExplorationStarted()) {
        planner->setCoverageGrid(
          dependencies_.tier_one_advisor.getLocalExploreCoverage());
      }

      const std::vector<std::list<int>> multiple_plans =
        beliefs.getAgentState()->getPlansWaypoints(
          current, planner, dependencies_.a_star_enabled);
      for (const std::list<int>& candidate : multiple_plans) {
        if (!candidate.empty()) {
          plans.push_back(candidate);
          planner_names.push_back(planner->getName());
          plan_created = true;
        }
      }
    }

    if (plan_created) {
      std::vector<std::vector<double>> plan_costs;
      for (const auto& owned_planner : dependencies_.planners) {
        PathPlanner* planner = owned_planner.get();
        std::vector<double> planner_costs;
        for (const std::list<int>& candidate : plans) {
          double cost = 0.0;
          if (planner->getName() != "skeleton" &&
              planner->getName() != "hallwayskel") {
            cost = planner->calcPathCost(candidate);
          }
          planner_costs.push_back(cost);
        }
        plan_costs.push_back(planner_costs);
      }

      std::vector<std::vector<double>> normalized_costs;
      for (const std::vector<double>& planner_costs : plan_costs) {
        double maximum = -std::numeric_limits<double>::infinity();
        double minimum = std::numeric_limits<double>::infinity();
        bool has_finite_cost = false;
        for (const double cost : planner_costs) {
          if (std::isfinite(cost)) {
            maximum = std::max(maximum, cost);
            minimum = std::min(minimum, cost);
            has_finite_cost = true;
          }
        }
        const double range = maximum - minimum;
        const bool uniform_costs =
          has_finite_cost && maximum == minimum;
        const bool valid_range =
          has_finite_cost && maximum != minimum &&
          std::isfinite(range) && range > 0.0;
        const double normalization_factor =
          valid_range ? range / 10.0 : 0.0;
        std::vector<double> normalized;
        for (const double cost : planner_costs) {
          if (!std::isfinite(cost) ||
              (!uniform_costs && !valid_range)) {
            normalized.push_back(
              std::numeric_limits<double>::infinity());
          } else if (valid_range) {
            const double normalized_cost =
              (cost - minimum) / normalization_factor;
            normalized.push_back(
              std::isfinite(normalized_cost)
                ? normalized_cost
                : std::numeric_limits<double>::infinity());
          } else {
            normalized.push_back(0.0);
          }
        }
        normalized_costs.push_back(normalized);
      }

      std::vector<double> total_costs;
      for (std::size_t index = 0; index < plans.size(); ++index) {
        double cost = 0.0;
        bool valid_cost = true;
        for (const std::vector<double>& normalized : normalized_costs) {
          const double component = normalized.at(index);
          if (!std::isfinite(component) ||
              !std::isfinite(cost + component)) {
            valid_cost = false;
          } else {
            cost += component;
          }
        }
        const double recorded_cost = valid_cost
          ? cost
          : std::numeric_limits<double>::infinity();
        total_costs.push_back(recorded_cost);
      }

      const PlanArbitrationResult selection =
        selectLowestCostPlan(total_costs);
      if (selection.selected) {
        const std::string& selected_planner =
          planner_names.at(selection.index);
        result.chosen_planner = selected_planner;
        for (const std::string& planner_name : planner_names) {
          if (planner_name != selected_planner) {
            result.chosen_planner += ">" + planner_name;
          }
        }

        for (const auto& owned_planner : dependencies_.planners) {
          PathPlanner* planner = owned_planner.get();
          if (planner->getName() == selected_planner) {
            beliefs.getAgentState()->setCurrentWaypoints(
              current,
              beliefs.getAgentState()->getCurrentLaserEndpoints(),
              planner,
              dependencies_.a_star_enabled,
              plans.at(selection.index),
              beliefs.getSpatialModel()->getRegionList()->getRegions());
            if (planner->getName() == "hallwayskel") {
              if (beliefs.getAgentState()
                    ->getCurrentTask()
                    ->getSkeletonWaypoint()
                    .getCreator() == 0) {
                result.chosen_planner = "skeletonhall>hallwayskel";
              } else {
                result.chosen_planner = "hallwayskel>skeletonhall";
              }
            } else if (planner->getName() == "skeleton") {
              result.chosen_planner = "skeleton>distance";
            }
            result.plan_selected = true;
            break;
          }
        }
      }
    }

    for (const auto& owned_planner : dependencies_.planners) {
      PathPlanner* planner = owned_planner.get();
      planner->resetPath();
      planner->resetOrigPath();
    }

    gettimeofday(&clock_value, nullptr);
    const double end_time =
      clock_value.tv_sec + (clock_value.tv_usec / 1000000.0);
    result.computation_time_seconds = end_time - start_time;
    return result;
  }

private:
  TierTwoDependencies dependencies_;
};

}  // namespace

std::unique_ptr<TierTwoDecision> makeTierTwoDecision(
  TierTwoDependencies dependencies) {
  return std::make_unique<DefaultTierTwoDecision>(dependencies);
}

}  // namespace decision
}  // namespace semaforr
