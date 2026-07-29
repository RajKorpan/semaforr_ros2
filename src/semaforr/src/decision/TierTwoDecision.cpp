/*
 * Tier-two planning implementation.
 */

#include <semaforr/decision/DecisionTier.h>
#include "DecisionTierFactory.h"

#include <semaforr/decision/Beliefs.h>
#include <semaforr/decision/Tier1Advisor.h>
#include <semaforr/navigation/PathPlanner.h>

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <list>
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
        plans.push_back(candidate);
        planner_names.push_back(planner->getName());
        if (!candidate.empty()) {
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
        const double maximum =
          *std::max_element(planner_costs.begin(), planner_costs.end());
        const double minimum =
          *std::min_element(planner_costs.begin(), planner_costs.end());
        const double normalization_factor = (maximum - minimum) / 10.0;
        std::vector<double> normalized;
        for (const double cost : planner_costs) {
          if (maximum != minimum) {
            normalized.push_back(
              (cost - minimum) / normalization_factor);
          } else {
            normalized.push_back(0.0);
          }
        }
        normalized_costs.push_back(normalized);
      }

      std::stringstream planner_comments;
      std::vector<double> total_costs;
      for (std::size_t index = 0; index < plans.size(); ++index) {
        double cost = 0.0;
        planner_comments << planner_names[index] << " ";
        for (const std::vector<double>& normalized : normalized_costs) {
          cost += normalized.at(index);
          planner_comments << normalized.at(index) << " ";
        }
        planner_comments << cost << ";";
        total_costs.push_back(cost);
      }

      double minimum_cost = 100000.0;
      for (const double cost : total_costs) {
        if (cost < minimum_cost) {
          minimum_cost = cost;
        }
      }

      std::vector<std::string> best_plan_names;
      std::vector<int> best_plan_indices;
      for (std::size_t index = 0; index < total_costs.size(); ++index) {
        if (total_costs[index] == minimum_cost) {
          best_plan_names.push_back(planner_names[index]);
          best_plan_indices.push_back(static_cast<int>(index));
        }
      }

      std::srand(std::time(nullptr));
      const int random_number =
        std::rand() % static_cast<int>(best_plan_indices.size());
      result.chosen_planner = best_plan_names.at(random_number);
      for (const std::string& planner_name : planner_names) {
        if (planner_name != best_plan_names.at(random_number)) {
          result.chosen_planner += ">" + planner_name;
        }
      }

      for (const auto& owned_planner : dependencies_.planners) {
        PathPlanner* planner = owned_planner.get();
        if (planner->getName() == best_plan_names.at(random_number)) {
          beliefs.getAgentState()->setCurrentWaypoints(
            current,
            beliefs.getAgentState()->getCurrentLaserEndpoints(),
            planner,
            dependencies_.a_star_enabled,
            plans.at(best_plan_indices.at(random_number)),
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
          break;
        }
      }
      result.plan_selected = true;
      result.planner_comments = planner_comments.str();
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
