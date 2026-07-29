#ifndef SEMAFORR_DECISION_DECISION_TIER_FACTORY_H
#define SEMAFORR_DECISION_DECISION_TIER_FACTORY_H

#include <memory>
#include <vector>

#include <semaforr/decision/DecisionTier.h>

class Beliefs;
class PathPlanner;
class Tier1Advisor;
class Tier3Advisor;

namespace semaforr {
namespace decision {

struct TierOneDependencies {
  Beliefs& beliefs;
  Tier1Advisor& advisor;
  bool doorway_enabled;
  bool behind_enabled;
  bool get_out_enabled;
  bool find_a_way_enabled;
  bool dont_go_back_enabled;
  int& highway_finished;
  int& frontier_finished;
};

struct TierTwoDependencies {
  Beliefs& beliefs;
  Tier1Advisor& tier_one_advisor;
  std::vector<std::unique_ptr<PathPlanner>>& planners;
  bool a_star_enabled;
  int& highway_finished;
  int& frontier_finished;
};

struct TierThreeDependencies {
  std::vector<std::unique_ptr<Tier3Advisor>>& advisors;
};

std::unique_ptr<TierOneDecision> makeTierOneDecision(
  TierOneDependencies dependencies);
std::unique_ptr<TierTwoDecision> makeTierTwoDecision(
  TierTwoDependencies dependencies);
std::unique_ptr<TierThreeDecision> makeTierThreeDecision(
  TierThreeDependencies dependencies);

}  // namespace decision
}  // namespace semaforr

#endif  // SEMAFORR_DECISION_DECISION_TIER_FACTORY_H
