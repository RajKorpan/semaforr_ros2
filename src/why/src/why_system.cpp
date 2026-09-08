#include <why/why_system.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace semaforr::why {
namespace {

constexpr double kPi = 3.14159265358979323846;

using Action = semaforr_msgs::msg::DecisionAction;
using PlanCandidate = semaforr_msgs::msg::PlanCandidateDiagnostic;

bool sameAction(const Action &left, const Action &right) {
  return left.type == right.type &&
         left.magnitude_index == right.magnitude_index;
}

std::string actionText(const Action &action) {
  switch (action.type) {
  case Action::FORWARD:
    return "move forward";
  case Action::TURN_RIGHT:
    return "turn right";
  case Action::TURN_LEFT:
    return "turn left";
  case Action::PAUSE:
    return "pause";
  default:
    return "stop";
  }
}

std::string joinNatural(const std::vector<std::string> &clauses) {
  if (clauses.empty())
    return {};
  if (clauses.size() == 1U)
    return clauses.front();
  std::ostringstream text;
  for (std::size_t index = 0U; index < clauses.size(); ++index) {
    if (index != 0U)
      text << (index + 1U == clauses.size() ? " and " : ", ");
    text << clauses[index];
  }
  return text.str();
}

std::pair<std::string, std::string>
advisorRationale(std::string_view advisor, std::string_view fallback) {
  static const std::map<std::string_view, std::pair<std::string, std::string>>
      translations{
          {"BigStep", {"take a big step", "take a small step"}},
          {"ElbowRoom", {"stay away from that wall", "go close to that wall"}},
          {"Novelty", {"go somewhere new", "go somewhere I've been"}},
          {"GoAround", {"get around this wall", "turn towards this wall"}},
          {"Greedy", {"get close to our target", "go farther from our target"}},
          {"Curiosity",
           {"go somewhere I've never been", "go somewhere I've been before"}},
          {"Enfilade", {"go back to where I was", "leave where I was"}},
          {"VisualScan",
           {"turn to see something new",
            "look in a direction I've already faced"}},
          {"Convey", {"go somewhere familiar", "go somewhere unfamiliar"}},
          {"Enter", {"go to our target's area", "leave our target's area"}},
          {"Exit",
           {"leave since our target isn't here",
            "stay although our target isn't here"}},
          {"Trailer",
           {"follow a familiar route that may lead to our target",
            "leave a familiar route that may lead to our target"}},
          {"Unlikely", {"stay out of a dead end", "go toward a dead end"}},
          {"Access",
           {"go to an area I've often been to",
            "leave an area I've often been to"}},
          {"Crossroads",
           {"go to a hallway I've often been to",
            "leave a hallway I've often been to"}},
          {"Follow",
           {"follow this hallway that may approach our target",
            "leave this hallway that may approach our target"}},
          {"LeastAngle",
           {"leave this area in the direction of our target",
            "leave this area not in the direction of our target"}},
          {"SpatialLearner",
           {"learn more about the world", "go somewhere I already know about"}},
          {"Stay", {"stay in this hallway", "leave this hallway"}}};
  const auto found = translations.find(advisor);
  if (found != translations.end())
    return found->second;
  const std::string rationale = fallback.empty()
                                    ? "follow this advisor's rationale"
                                    : std::string(fallback);
  return {rationale, "avoid " + rationale};
}

std::optional<std::string> relativeSupportClause(
    const semaforr_msgs::msg::AdvisorContribution &contribution) {
  const double rho = contribution.relative_support;
  if (rho > -0.75 && rho <= 0.75)
    return std::nullopt;
  const auto rationale =
      advisorRationale(contribution.advisor, contribution.explanation);
  if (rho <= -1.5)
    return "I really don't want to " + rationale.second;
  if (rho <= -0.75)
    return "I don't want to " + rationale.second;
  if (rho <= 1.5)
    return "I want to " + rationale.first;
  return "I really want to " + rationale.first;
}

int agreementLevel(double gamma) {
  if (gamma > 0.45)
    return 0;
  if (gamma > 0.25)
    return 1;
  return 2;
}

std::string agreementPhrase(double gamma) {
  if (gamma > 0.45)
    return "my reasons conflict";
  if (gamma > 0.25)
    return "I've only got a few reasons for it";
  return "I've got many reasons for it";
}

int supportLevel(double zeta) {
  if (zeta <= 0.75)
    return 0;
  if (zeta <= 1.5)
    return 1;
  return 2;
}

std::string supportPhrase(double zeta) {
  if (zeta <= 0.75)
    return "I don't really want to do this much more than some other action";
  if (zeta <= 1.5)
    return "I somewhat want to do this much more than some other action";
  return "I really want to do this much more than some other action";
}

int confidenceLevel(double lambda) {
  if (lambda <= 0.0375)
    return 0;
  if (lambda <= 0.375)
    return 1;
  return 2;
}

std::string confidencePhrase(double lambda) {
  if (lambda <= 0.0375)
    return "not";
  if (lambda <= 0.375)
    return "only somewhat";
  return "really";
}

const PlanCandidate *selectedPlan(const DecisionRecord &record) {
  if (!record.has_plan)
    return nullptr;
  const auto found = std::find_if(
      record.planning_candidates.begin(), record.planning_candidates.end(),
      [&](const auto &candidate) {
        return candidate.plan_id == record.active_plan_id;
      });
  return found == record.planning_candidates.end() ? nullptr : &*found;
}

double pathLength(const std::vector<geometry_msgs::msg::Point> &path) {
  double result = 0.0;
  for (std::size_t index = 1U; index < path.size(); ++index)
    result += std::hypot(path[index].x - path[index - 1U].x,
                         path[index].y - path[index - 1U].y);
  return result;
}

int allocentricDirectionLabel(double angle) {
  const double normalized = std::remainder(angle, 2.0 * kPi);
  if (normalized >= -7.0 * kPi / 8.0 && normalized < -5.0 * kPi / 8.0)
    return 2;
  if (normalized >= -5.0 * kPi / 8.0 && normalized < -3.0 * kPi / 8.0)
    return 3;
  if (normalized >= -3.0 * kPi / 8.0 && normalized < -kPi / 8.0)
    return 4;
  if (normalized >= -kPi / 8.0 && normalized < kPi / 8.0)
    return 5;
  if (normalized >= kPi / 8.0 && normalized < 3.0 * kPi / 8.0)
    return 6;
  if (normalized >= 3.0 * kPi / 8.0 && normalized < 5.0 * kPi / 8.0)
    return 7;
  if (normalized >= 5.0 * kPi / 8.0 && normalized < 7.0 * kPi / 8.0)
    return 8;
  return 1;
}

std::string turnPhrase(int difference) {
  static const std::array<std::string_view, 8> phrases{
      "go straight", "turn left a little", "turn left",  "turn hard left",
      "turn around", "turn hard right",    "turn right", "turn right a little"};
  return std::string(phrases.at(static_cast<std::size_t>(difference)));
}

std::string distanceCategory(double distance) {
  double upper = 1.0;
  if (distance <= 1.0) {
    upper = 1.0;
  } else if (distance <= 10.0) {
    upper = 2.0 * std::ceil(distance / 2.0);
  } else if (distance <= 50.0) {
    upper = 5.0 * std::ceil(distance / 5.0);
  } else if (distance <= 110.0) {
    upper = 10.0 * std::ceil(distance / 10.0);
  } else {
    return "more than 110 meters";
  }
  std::ostringstream text;
  text << static_cast<int>(upper) << (upper == 1.0 ? " meter" : " meters");
  return text.str();
}

struct RouteLocation {
  geometry_msgs::msg::Point point;
  bool highway_intersection{false};
};

struct RoutePhrase {
  std::string text;
  double distance_m{0.0};
  double turn_rad{0.0};
  bool travel{false};
};

bool samePoint(const geometry_msgs::msg::Point &left,
               const geometry_msgs::msg::Point &right) {
  return std::hypot(left.x - right.x, left.y - right.y) <= 1.0e-6;
}

std::vector<RouteLocation> planLocations(const DecisionRecord &record,
                                         const PlanCandidate &plan) {
  std::vector<RouteLocation> result;
  geometry_msgs::msg::Point robot;
  robot.x = record.robot_pose.x;
  robot.y = record.robot_pose.y;
  result.push_back({robot, false});
  const bool model_plan =
      plan.metadata.plan_type == "model-based" || plan.plan_family == "model" ||
      plan.planner == "SkeletonPlan" || plan.planner == "HighwayPlan";
  if (model_plan) {
    for (const auto &step : plan.typed_steps) {
      const bool omit_highway_structure = step.step_type == "highway" ||
                                          step.step_type == "highway_entry" ||
                                          step.step_type == "highway_exit";
      if (omit_highway_structure)
        continue;
      std::optional<geometry_msgs::msg::Point> representative;
      if (step.has_target)
        representative = step.target;
      else if (!step.geometry.empty())
        representative = step.geometry.back();
      if (!representative)
        continue;
      const bool intersection = step.step_type == "intersection";
      if (samePoint(result.back().point, *representative)) {
        result.back().highway_intersection |= intersection;
      } else {
        result.push_back({*representative, intersection});
      }
    }
  }
  if (!model_plan || result.size() == 1U) {
    for (const auto &point : plan.geometry)
      if (!samePoint(result.back().point, point))
        result.push_back({point, false});
  } else if (!plan.geometry.empty() &&
             !samePoint(result.back().point, plan.geometry.back())) {
    result.push_back({plan.geometry.back(), false});
  }
  return result;
}

std::string describeRoute(const DecisionRecord &record,
                          const PlanCandidate &plan, bool alternative,
                          ExplanationResponse &response) {
  const auto locations = planLocations(record, plan);
  response.route.clear();
  for (const auto &location : locations)
    response.route.push_back(location.point);
  if (locations.size() < 2U)
    return alternative ? "No usable alternative route geometry is recorded."
                       : "No usable selected route geometry is recorded.";
  struct Segment {
    double length{0.0};
    double bearing{0.0};
  };
  std::vector<Segment> segments;
  for (std::size_t index = 1U; index < locations.size(); ++index) {
    const double dx = locations[index].point.x - locations[index - 1U].point.x;
    const double dy = locations[index].point.y - locations[index - 1U].point.y;
    const double length = std::hypot(dx, dy);
    if (length > 1.0e-6)
      segments.push_back({length, std::atan2(dy, dx)});
  }
  if (segments.empty())
    return alternative ? "No usable alternative route geometry is recorded."
                       : "No usable selected route geometry is recorded.";
  const bool highway_plan =
      plan.planner == "HighwayPlan" || plan.plan_family == "highway";
  std::vector<RoutePhrase> phrases;
  phrases.push_back({"go straight", segments.front().length, 0.0, true});
  for (std::size_t index = 1U; index < segments.size(); ++index) {
    const int previous =
        allocentricDirectionLabel(segments[index - 1U].bearing);
    const int current = allocentricDirectionLabel(segments[index].bearing);
    const int difference = (current - previous + 8) % 8;
    const double turn = std::remainder(
        segments[index].bearing - segments[index - 1U].bearing, 2.0 * kPi);
    if (difference != 0) {
      std::string turn_text = turnPhrase(difference);
      if (highway_plan && index < locations.size() &&
          locations[index].highway_intersection)
        turn_text += " at an intersection";
      phrases.push_back({std::move(turn_text), 0.0, turn, false});
    }
    phrases.push_back({"go straight", segments[index].length, 0.0, true});
  }
  std::vector<RoutePhrase> collapsed;
  for (const auto &phrase : phrases) {
    if (phrase.travel && !collapsed.empty() && collapsed.back().travel) {
      collapsed.back().distance_m += phrase.distance_m;
    } else {
      collapsed.push_back(phrase);
    }
  }
  response.exact_distances_m.clear();
  response.exact_turn_angles_rad.clear();
  response.direction_categories.clear();
  response.distance_categories.clear();
  std::vector<std::string> clauses;
  for (const auto &phrase : collapsed) {
    response.exact_distances_m.push_back(phrase.distance_m);
    response.exact_turn_angles_rad.push_back(phrase.turn_rad);
    response.direction_categories.push_back(phrase.text);
    response.distance_categories.push_back(
        phrase.travel ? distanceCategory(phrase.distance_m) : "");
    clauses.push_back(phrase.travel ? phrase.text + " about " +
                                          response.distance_categories.back()
                                    : phrase.text);
  }
  std::ostringstream text;
  text << (alternative ? "We could " : "We will ");
  if (clauses.size() == 1U) {
    text << clauses.front();
  } else {
    for (std::size_t index = 0U; index < clauses.size(); ++index) {
      if (index != 0U)
        text << (index + 1U == clauses.size() ? ", and " : ", ");
      text << clauses[index];
    }
  }
  text << " to reach our target.";
  return text.str();
}

enum class ObjectiveFamily { Distance, Affordance, Freespace, Unsupported };

ObjectiveFamily objectiveFamily(std::string_view objective) {
  if (objective == "distance")
    return ObjectiveFamily::Distance;
  if (objective == "conveyor" || objective == "hallway" ||
      objective == "region" || objective == "trail")
    return ObjectiveFamily::Affordance;
  if (objective == "skeleton_distance" || objective == "highway_distance")
    return ObjectiveFamily::Freespace;
  return ObjectiveFamily::Unsupported;
}

std::optional<double> objectiveCost(const PlanCandidate &plan,
                                    std::string_view objective) {
  const auto found =
      std::find(plan.objectives.begin(), plan.objectives.end(), objective);
  if (found == plan.objectives.end())
    return std::nullopt;
  const auto index =
      static_cast<std::size_t>(std::distance(plan.objectives.begin(), found));
  if (index >= plan.raw_costs.size())
    return std::nullopt;
  return plan.raw_costs[index];
}

double preferredObjectiveDifference(ObjectiveFamily family,
                                    double preferred_cost, double other_cost) {
  return family == ObjectiveFamily::Distance ? other_cost - preferred_cost
                                             : preferred_cost - other_cost;
}

int differenceMagnitudeLevel(ObjectiveFamily family, double difference) {
  if (family == ObjectiveFamily::Distance) {
    if (difference <= 1.0)
      return 0;
    if (difference <= 10.0)
      return 1;
    return 2;
  }
  if (family == ObjectiveFamily::Affordance) {
    if (difference <= -150.0)
      return 2;
    if (difference <= -25.0)
      return 1;
    return 0;
  }
  if (difference <= -25.0)
    return 2;
  if (difference <= -5.0)
    return 1;
  return 0;
}

std::string magnitudePhrase(int level) {
  if (level == 2)
    return "a lot";
  if (level == 1)
    return "somewhat";
  return "a bit";
}

std::pair<std::string, std::string>
objectiveComparators(std::string_view objective, std::string_view fallback) {
  if (objective == "distance")
    return {"shorter", "longer"};
  if (objective == "conveyor")
    return {"better at going through well-traveled areas",
            "worse at going through well-traveled areas"};
  if (objective == "hallway")
    return {"better at following hallways", "worse at following hallways"};
  if (objective == "region")
    return {"better at going through open areas",
            "worse at going through open areas"};
  if (objective == "trail")
    return {"better at following ways we've gone before",
            "worse at following ways we've gone before"};
  if (objective == "skeleton_distance")
    return {"better at going through open areas",
            "worse at going through open areas"};
  if (objective == "highway_distance")
    return {"better at following long hallways",
            "worse at following long hallways"};
  return {"better at " + std::string(fallback),
          "worse at " + std::string(fallback)};
}

std::string planConfidenceCategory(int robot_magnitude,
                                   int comparison_magnitude) {
  static const std::array<std::array<std::string_view, 3>, 3> table{
      {{{"only somewhat", "not", "not"}},
       {{"really", "only somewhat", "not"}},
       {{"really", "really", "only somewhat"}}}};
  return std::string(table.at(static_cast<std::size_t>(robot_magnitude))
                         .at(static_cast<std::size_t>(comparison_magnitude)));
}

std::uint8_t inferredQuestionType(const ExplanationQuestion &question) {
  if (question.natural_language_question.empty() &&
      question.question_type <= ExplanationQuestion::COMBINED)
    return question.question_type;
  std::string text = question.natural_language_question;
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char value) {
                   return static_cast<char>(std::tolower(value));
                 });
  if (text.find("another way") != std::string::npos)
    return ExplanationQuestion::ALTERNATIVE_PLAN;
  if (text.find("getting there") != std::string::npos ||
      text.find("how are we") != std::string::npos)
    return ExplanationQuestion::ROUTE_DESCRIPTION;
  if (text.find("plan") != std::string::npos &&
      text.find("sure") != std::string::npos)
    return ExplanationQuestion::PLAN_CONFIDENCE;
  if (text.find("better than") != std::string::npos)
    return ExplanationQuestion::COMPARE_PLAN;
  if (text.find("why not") != std::string::npos ||
      text.find("instead") != std::string::npos)
    return ExplanationQuestion::WHY_NOT_ACTION;
  if (text.find("if you were") != std::string::npos)
    return ExplanationQuestion::HYPOTHETICAL_POSE;
  if (text.find("why") != std::string::npos &&
      (text.find("route") != std::string::npos ||
       text.find("highway") != std::string::npos ||
       text.find("region") != std::string::npos))
    return ExplanationQuestion::COMBINED;
  if (text.find("plan") != std::string::npos)
    return ExplanationQuestion::WHY_PLAN;
  if (text.find("sure") != std::string::npos)
    return ExplanationQuestion::DECISION_CONFIDENCE;
  return ExplanationQuestion::WHY_DECISION;
}

std::string tierOnePhrase(const DecisionRecord &record) {
  const std::string &policy = record.selected_policy;
  if (policy.find("Victory") != std::string::npos)
    return "Victory selected it because the target was directly reachable";
  if (policy.find("Enforcer") != std::string::npos) {
    if (record.enforcer_mode == "grid") {
      std::ostringstream text;
      text << "GridPlanEnforcer selected it at path index "
           << record.active_plan_step;
      if (record.has_operational_target)
        text << " toward waypoint (" << record.operational_target.x << ", "
             << record.operational_target.y << ')';
      if (!record.enforcer_reason.empty())
        text << " using " << record.enforcer_reason;
      return text.str();
    }
    std::string step = "model step";
    if (const auto *plan = selectedPlan(record);
        plan && record.active_plan_step < plan->typed_steps.size()) {
      const auto &typed = plan->typed_steps[record.active_plan_step];
      step = typed.step_type + " " + std::to_string(typed.primary_entity_id);
    }
    return "ModelPlanEnforcer selected it to operationalize " + step +
           (record.enforcer_reason.empty()
                ? std::string{}
                : " using " + record.enforcer_reason);
  }
  if (policy.find("Thru") != std::string::npos)
    return "Thru retained control to pass through a tight opening";
  if (policy.find("Behind") != std::string::npos)
    return "Behind retained control to recover a target or waypoint outside "
           "the current view";
  if (policy.find("Out") != std::string::npos)
    return "Out retained control to escape the recently known confined area";
  if (policy.find("LLE") != std::string::npos)
    return "LLE retained control because target-directed planning lacked "
           "sufficient learned connectivity";
  if (policy.find("HLE") != std::string::npos ||
      policy.find("hle:") != std::string::npos)
    return "HLE selected it while pursuing the current passage candidate";
  if (policy.find("AvoidObstacles") != std::string::npos)
    return "AvoidObstacles selected it from motions with sufficient observed "
           "clearance";
  if (policy.find("NotOpposite") != std::string::npos)
    return "NotOpposite selected it after applying the execution-confirmed "
           "orientation-history constraint";
  if (policy.find("Forward") != std::string::npos)
    return "Forward selected it to avoid returning to footprint-sized visited "
           "space";
  if (policy.find("Precedent") != std::string::npos)
    return "Precedent selected it using the recorded high-confidence "
           "circumstance evidence";
  return "the recorded Tier-1 component mandated it";
}

} // namespace

void ExplanationTraceStore::record(const DecisionRecord &record) {
  decisions_.insert_or_assign(record.decision_id, record);
  action_to_decision_[record.action_id] = record.decision_id;
  if (record.execution_id != 0U)
    execution_to_decision_[record.execution_id] = record.decision_id;
  if (record.has_task)
    task_to_decision_[record.task.task_index] = record.decision_id;
  if (record.has_plan)
    plan_to_decision_[record.active_plan_id] = record.decision_id;
  if (record.has_planning_episode)
    episode_to_decision_[record.planning_episode_id] = record.decision_id;
}

const DecisionRecord *ExplanationTraceStore::latestDecision() const noexcept {
  return decisions_.empty() ? nullptr : &decisions_.rbegin()->second;
}

const DecisionRecord *
ExplanationTraceStore::decision(std::uint64_t id) const noexcept {
  const auto found = decisions_.find(id);
  return found == decisions_.end() ? nullptr : &found->second;
}

const DecisionRecord *
ExplanationTraceStore::action(std::uint64_t id) const noexcept {
  const auto found = action_to_decision_.find(id);
  return found == action_to_decision_.end() ? nullptr : decision(found->second);
}

const DecisionRecord *
ExplanationTraceStore::execution(std::uint64_t id) const noexcept {
  const auto found = execution_to_decision_.find(id);
  return found == execution_to_decision_.end() ? nullptr
                                               : decision(found->second);
}

const DecisionRecord *
ExplanationTraceStore::task(std::uint64_t id) const noexcept {
  const auto found = task_to_decision_.find(id);
  return found == task_to_decision_.end() ? nullptr : decision(found->second);
}

const DecisionRecord *ExplanationTraceStore::currentTask() const noexcept {
  return task_to_decision_.empty()
             ? nullptr
             : decision(task_to_decision_.rbegin()->second);
}

const DecisionRecord *
ExplanationTraceStore::plan(std::uint64_t id) const noexcept {
  const auto found = plan_to_decision_.find(id);
  return found == plan_to_decision_.end() ? nullptr : decision(found->second);
}

const DecisionRecord *
ExplanationTraceStore::planningEpisode(std::uint64_t id) const noexcept {
  const auto found = episode_to_decision_.find(id);
  return found == episode_to_decision_.end() ? nullptr
                                             : decision(found->second);
}

std::vector<std::uint64_t> ExplanationTraceStore::decisionIds() const {
  std::vector<std::uint64_t> result;
  for (const auto &[id, record] : decisions_) {
    static_cast<void>(record);
    result.push_back(id);
  }
  return result;
}

const DecisionRecord *UnifiedWhySystem::resolveDecision(
    const ExplanationQuestion &question) const noexcept {
  if (question.has_decision_id)
    return traces_.decision(question.decision_id);
  if (question.has_action_id)
    return traces_.action(question.action_id);
  if (question.has_execution_id)
    return traces_.execution(question.execution_id);
  if (question.has_plan_id)
    return traces_.plan(question.plan_id);
  if (question.has_planning_episode_id)
    return traces_.planningEpisode(question.planning_episode_id);
  if (question.has_task_id)
    return traces_.task(question.task_id);
  return traces_.latestDecision();
}

ExplanationResponse
UnifiedWhySystem::baseResponse(const ExplanationQuestion &question,
                               const DecisionRecord *record) const {
  ExplanationResponse response;
  response.explanation_id = next_explanation_id_++;
  response.question_id = question.question_id;
  response.question_type = inferredQuestionType(question);
  response.found = record != nullptr;
  if (!record)
    return response;
  response.has_decision_id = true;
  response.decision_id = record->decision_id;
  response.has_action_id = true;
  response.action_id = record->action_id;
  response.has_execution_id = record->execution_id != 0U;
  response.execution_id = record->execution_id;
  response.has_task_id = record->has_task;
  response.task_id = record->has_task ? record->task.task_index : 0U;
  response.has_plan_id = record->has_plan;
  response.plan_id = record->active_plan_id;
  response.plan_revision = record->active_plan_revision;
  response.confidence_category = record->decision_confidence_category;
  response.gini_agreement = record->decision_gamma;
  response.standardized_support = record->decision_zeta;
  response.relative_support = record->decision_lambda;
  response.source_provenance = record->source_provenance;
  if (const auto *plan = selectedPlan(*record)) {
    response.referenced_model_revisions = plan->dependency_revisions;
    for (const auto &dependency : plan->metadata.representation_dependencies)
      if (std::find(response.source_provenance.begin(),
                    response.source_provenance.end(),
                    dependency) == response.source_provenance.end())
        response.source_provenance.push_back(dependency);
    if (plan->static_map_contributed &&
        std::find(response.source_provenance.begin(),
                  response.source_provenance.end(),
                  "static_map") == response.source_provenance.end())
      response.source_provenance.push_back("static_map");
  }
  return response;
}

ExplanationResponse
UnifiedWhySystem::explainDecision(const ExplanationQuestion &question,
                                  const DecisionRecord &record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "decision";
  std::ostringstream text;
  text << "Action " << record.action_id << " was "
       << (record.action_lifecycle_status.empty()
               ? "selected"
               : record.action_lifecycle_status)
       << ": " << actionText(record.selected_action) << ". ";
  if (record.has_execution_result) {
    text << "The controller actually reached (" << record.execution_final_pose.x
         << ", " << record.execution_final_pose.y << ") after translating "
         << record.distance_achieved_m << " m and rotating "
         << record.rotation_achieved_rad << " rad";
    if (!record.execution_cancellation_reason.empty())
      text << "; terminal detail: " << record.execution_cancellation_reason;
    text << ". ";
  }
  if (record.selected_tier == DecisionRecord::TIER_ONE) {
    response.primary_reasoning_source = "tier_one";
    text << tierOnePhrase(record) << '.';
  } else if (record.selected_tier == DecisionRecord::TIER_THREE) {
    response.primary_reasoning_source = "tier_three_voting";
    std::vector<const semaforr_msgs::msg::AdvisorContribution *> supportive;
    std::vector<const semaforr_msgs::msg::AdvisorContribution *> opposing;
    for (const auto &contribution : record.advisor_contributions) {
      if (!sameAction(contribution.action, record.selected_action))
        continue;
      response.referenced_advisors.push_back(contribution.advisor);
      response.structured_facts.push_back(
          contribution.advisor +
          ":raw=" + std::to_string(contribution.raw_score) +
          ",normalized=" + std::to_string(contribution.normalized_score) +
          ",mean=" + std::to_string(contribution.advisor_mean) +
          ",standard_deviation=" +
          std::to_string(contribution.advisor_standard_deviation) +
          ",relative_support=" + std::to_string(contribution.relative_support) +
          ",weight=" + std::to_string(contribution.weight) +
          ",weighted_contribution=" +
          std::to_string(contribution.weighted_score) +
          ",final_total=" + std::to_string(contribution.final_total));
      if (contribution.relative_support > 0.75)
        supportive.push_back(&contribution);
      else if (contribution.relative_support <= -0.75)
        opposing.push_back(&contribution);
    }
    const auto retainStrongestBand = [](auto &contributions, bool support) {
      const bool has_strong = std::any_of(
          contributions.begin(), contributions.end(), [&](const auto *item) {
            return support ? item->relative_support > 1.5
                           : item->relative_support <= -1.5;
          });
      if (has_strong)
        std::erase_if(contributions, [&](const auto *item) {
          return support ? item->relative_support <= 1.5
                         : item->relative_support > -1.5;
        });
    };
    retainStrongestBand(supportive, true);
    retainStrongestBand(opposing, false);
    std::vector<std::string> support_clauses;
    std::vector<std::string> opposition_clauses;
    for (const auto *contribution : supportive)
      if (const auto clause = relativeSupportClause(*contribution))
        support_clauses.push_back(*clause);
    for (const auto *contribution : opposing)
      if (const auto clause = relativeSupportClause(*contribution))
        opposition_clauses.push_back(*clause);
    if (!opposition_clauses.empty())
      text << "Although " << joinNatural(opposition_clauses) << ", ";
    text << "I decided to " << actionText(record.selected_action);
    if (!support_clauses.empty())
      text << " because " << joinNatural(support_clauses);
    else
      text << " because it had the greatest total Tier-3 comment strength";
    text << '.';
    if (record.circumstance_match_available) {
      text << " The setting matched a " << record.circumstance_learning_mode
           << " circumstance with assignment confidence "
           << record.circumstance_assignment_confidence << ".";
      if (record.circumstance_weighting_applied) {
        const auto selected = std::find_if(
            record.tier_three_action_totals.begin(),
            record.tier_three_action_totals.end(), [&](const auto &total) {
              return sameAction(total.action, record.selected_action);
            });
        if (selected != record.tier_three_action_totals.end())
          text << " Historical execution evidence ("
               << selected->circumstance_action_evidence
               << " effective outcomes, confidence "
               << selected->circumstance_action_confidence
               << ") multiplied its base total "
               << selected->pre_circumstance_total << " by "
               << selected->circumstance_multiplier << " to produce "
               << selected->post_circumstance_total << ".";
        text << (record.circumstance_weighting_changed_winner
                     ? " This changed the Tier-3 winner."
                     : " This did not change the Tier-3 winner.");
      } else if (!record.circumstance_reason.empty()) {
        text << " Circumstance weighting remained neutral: "
             << record.circumstance_reason << ".";
      }
    }
  } else if (record.selected_tier == DecisionRecord::EXPLORATION) {
    response.primary_reasoning_source = "initial_exploration";
    text << "The exploration state machine selected it.";
  } else {
    response.primary_reasoning_source = record.selected_policy;
    text << "The recorded policy was " << record.selected_policy << '.';
  }
  if (record.has_plan) {
    response.referenced_planners.push_back(record.selected_planner);
    response.referenced_plan_steps.push_back(record.active_plan_step);
    text << " The active plan was " << record.active_plan_id << " revision "
         << record.active_plan_revision;
    if (record.has_operational_target)
      text << ", operationalized at (" << record.operational_target.x << ", "
           << record.operational_target.y << ')';
    text << '.';
    if (!record.plan_status.empty())
      text << " Its recorded status was " << record.plan_status << '.';
    for (const auto &event : record.plan_execution_events) {
      response.structured_facts.push_back("plan_event=" + event);
      text << " Plan event: " << event << '.';
    }
  }
  const auto precedent = std::find_if(
      record.decision_cycle.begin(), record.decision_cycle.end(),
      [](const auto &event) { return event.component == "Precedent"; });
  if (precedent != record.decision_cycle.end() &&
      precedent->reason_code.find("precedent:abstained:") == 0U)
    text << " Precedent abstained ("
         << precedent->reason_code.substr(
                std::string("precedent:abstained:").size())
         << "), so learned experience did not remove an action.";
  if (!record.social_input_source.empty() ||
      !record.social_input_status.empty()) {
    const std::string status = record.social_input_status.empty()
                                   ? "unknown"
                                   : record.social_input_status;
    response.structured_facts.push_back("social_input_source=" +
                                        record.social_input_source);
    response.structured_facts.push_back("social_prediction_source=" +
                                        record.social_prediction_source);
    response.structured_facts.push_back("social_input_status=" + status);
    response.structured_facts.push_back(
        "crowd_revisions=live:" + std::to_string(record.live_social_revision) +
        ",density:" + std::to_string(record.crowd_density_revision) +
        ",risk:" + std::to_string(record.crowd_risk_revision) +
        ",flow:" + std::to_string(record.crowd_flow_revision));
    if (status == "ready") {
      text << " Social evidence came from " << record.social_input_source;
      if (!record.social_prediction_source.empty())
        text << " with " << record.social_prediction_source << " predictions";
      text << ".";
      if (record.formation_evidence_participated)
        text << " Formation evidence participated in this decision.";
    } else {
      text << " Social input was degraded (" << status
           << "); stale or missing live crowd evidence was not treated as "
              "current.";
    }
  }
  response.natural_language_response = text.str();
  return response;
}

ExplanationResponse
UnifiedWhySystem::explainCounterfactual(const ExplanationQuestion &question,
                                        const DecisionRecord &record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "counterfactual_action";
  if (!question.has_alternative_action) {
    response.natural_language_response =
        "No alternative action was supplied for the comparison.";
    return response;
  }
  const auto &alternative = question.alternative_action;
  const bool generated = std::any_of(
      record.candidates.begin(), record.candidates.end(),
      [&](const auto &action) { return sameAction(action, alternative); });
  if (!generated) {
    response.primary_reasoning_source = "candidate_generation";
    response.natural_language_response =
        actionText(alternative) + " was not generated in that decision cycle.";
    return response;
  }
  const auto veto = std::find_if(
      record.vetoes.begin(), record.vetoes.end(),
      [&](const auto &item) { return sameAction(item.action, alternative); });
  if (veto != record.vetoes.end()) {
    response.primary_reasoning_source = veto->rule;
    response.structured_facts = {veto->reason_code, veto->rejection_kind,
                                 veto->explanation_category};
    response.natural_language_response =
        actionText(alternative) + " was removed by " + veto->rule + " as " +
        veto->explanation_category +
        ", not merely because it had a lower "
        "preference. The recorded reason was " +
        veto->reason_code + '.';
    return response;
  }
  const bool viable = std::any_of(
      record.viable_actions.begin(), record.viable_actions.end(),
      [&](const auto &action) { return sameAction(action, alternative); });
  if (!viable) {
    response.primary_reasoning_source = "not_viable";
    response.natural_language_response =
        actionText(alternative) +
        " was generated but was not in the recorded viable action set. No "
        "safety or cognitive cause was recorded, so I will not invent one.";
    return response;
  }
  const bool tied = std::any_of(
      record.tier_three_tie_candidates.begin(),
      record.tier_three_tie_candidates.end(),
      [&](const auto &action) { return sameAction(action, alternative); });
  if (tied) {
    response.primary_reasoning_source = "tie_breaking";
    response.natural_language_response =
        actionText(alternative) +
        " tied with the selected action; the recorded " +
        record.tier_three_tie_policy +
        " tie rule and seeded random choice "
        "selected " +
        actionText(record.selected_action) + '.';
    return response;
  }
  double alternative_total = 0.0, selected_total = 0.0;
  for (const auto &total : record.tier_three_action_totals) {
    if (sameAction(total.action, alternative))
      alternative_total = total.total;
    if (sameAction(total.action, record.selected_action))
      selected_total = total.total;
  }
  response.primary_reasoning_source =
      record.has_plan && record.selected_tier == DecisionRecord::TIER_ONE
          ? "plan_enforcement"
          : "lower_preference";
  response.structured_facts = {
      "alternative_total=" + std::to_string(alternative_total),
      "selected_total=" + std::to_string(selected_total)};
  std::ostringstream rationale;
  rationale << actionText(alternative) << " remained viable";
  if (response.primary_reasoning_source == "plan_enforcement") {
    rationale << " but was inconsistent with the action mandated for active "
                 "plan step "
              << record.active_plan_step;
  } else {
    rationale << " but had lower recorded support (" << alternative_total
              << " versus " << selected_total << ')';
  }
  rationale << ". It was not classified as unsafe.";
  const auto alternative_trace = std::find_if(
      record.tier_three_action_totals.begin(),
      record.tier_three_action_totals.end(),
      [&](const auto &total) { return sameAction(total.action, alternative); });
  if (alternative_trace != record.tier_three_action_totals.end() &&
      record.circumstance_weighting_applied)
    rationale << " Circumstance evidence changed its base support from "
              << alternative_trace->pre_circumstance_total << " by multiplier "
              << alternative_trace->circumstance_multiplier << " to "
              << alternative_trace->post_circumstance_total << ".";
  for (const auto &contribution : record.advisor_contributions) {
    if (!sameAction(contribution.action, alternative))
      continue;
    response.referenced_advisors.push_back(contribution.advisor);
    response.structured_facts.push_back(
        contribution.advisor +
        ":raw=" + std::to_string(contribution.raw_score) +
        ",relative=" + std::to_string(contribution.relative_support) +
        ",contribution=" + std::to_string(contribution.weighted_score));
  }
  response.natural_language_response = rationale.str();
  return response;
}

ExplanationResponse UnifiedWhySystem::explainDecisionConfidence(
    const ExplanationQuestion &question, const DecisionRecord &record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "decision_confidence";
  if (record.selected_tier != DecisionRecord::TIER_THREE) {
    response.primary_reasoning_source = "recorded_tier_one_confidence";
    response.natural_language_response =
        "Decision confidence is recorded by the deciding Tier-1 component; "
        "the Chapter 5 Tier-3 equations do not apply to this decision.";
    return response;
  }
  response.primary_reasoning_source = "chapter_five_equations_5_2_to_5_4";
  const double gamma = record.decision_gamma;
  const double zeta = record.decision_zeta;
  const double lambda = record.decision_lambda;
  response.confidence_value = lambda;
  response.gini_agreement = gamma;
  response.standardized_support = zeta;
  response.relative_support = lambda;
  response.confidence_category = confidencePhrase(lambda);
  response.structured_facts = {
      "selected_comment_sum=" +
          std::to_string(record.decision_selected_comment_sum),
      "advisor_count=" + std::to_string(record.decision_advisor_count),
      "normalized_support_proportion=" +
          std::to_string(record.decision_normalized_support_proportion),
      "gamma=" + std::to_string(gamma),
      "action_total_mean=" + std::to_string(record.decision_action_total_mean),
      "action_total_standard_deviation=" +
          std::to_string(record.decision_action_total_standard_deviation),
      "zeta=" + std::to_string(zeta),
      "lambda=" + std::to_string(lambda)};
  for (const auto &total : record.tier_three_action_totals)
    response.structured_facts.push_back(
        "action_total=" + actionText(total.action) + ':' +
        std::to_string(total.chapter_five_comment_total));
  const int gamma_level = agreementLevel(gamma);
  const int zeta_level = supportLevel(zeta);
  const int lambda_level = confidenceLevel(lambda);
  std::ostringstream text;
  text << "I'm " << confidencePhrase(lambda) << " sure in my decision because ";
  if (gamma_level == lambda_level && zeta_level == lambda_level) {
    text << agreementPhrase(gamma) << ". " << supportPhrase(zeta) << '.';
  } else if (gamma_level == lambda_level) {
    text << agreementPhrase(gamma) << '.';
  } else if (zeta_level == lambda_level) {
    text << supportPhrase(zeta) << '.';
  } else {
    const std::string lower =
        gamma_level < zeta_level ? agreementPhrase(gamma) : supportPhrase(zeta);
    const std::string higher =
        gamma_level > zeta_level ? agreementPhrase(gamma) : supportPhrase(zeta);
    text << "even though " << higher << ", " << lower << '.';
  }
  text << " (gamma=" << gamma << ", zeta=" << zeta << ", lambda=" << lambda
       << ").";
  if (record.circumstance_match_available)
    text << " Circumstance assignment confidence was "
         << record.circumstance_assignment_confidence << " under model "
         << record.circumstance_model_version << " ("
         << record.circumstance_learning_mode << "). "
         << record.circumstance_reason << '.';
  response.natural_language_response = text.str();
  return response;
}

ExplanationResponse
UnifiedWhySystem::explainPlan(const ExplanationQuestion &question,
                              const DecisionRecord &record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "plan_selection";
  const auto *selected = selectedPlan(record);
  if (!selected) {
    response.found = false;
    response.natural_language_response =
        "No recorded planning candidate matches the requested plan.";
    return response;
  }
  response.primary_reasoning_source = "tier_two_selection";
  response.referenced_planners.push_back(selected->planner);
  response.referenced_model_revisions = selected->dependency_revisions;
  std::ostringstream text;
  text << "Plan " << selected->plan_id << " was generated by "
       << selected->planner << " to "
       << selected->metadata.objective_description << ". It is a "
       << selected->metadata.plan_type << " plan using ";
  for (std::size_t index = 0U;
       index < selected->metadata.representation_dependencies.size(); ++index) {
    if (index != 0U)
      text << ", ";
    text << selected->metadata.representation_dependencies[index];
  }
  text << ". Tier 2 compared " << record.planning_candidates.size()
       << " candidate plans; its final vote was " << selected->summed_score
       << ".";
  if (record.planning_tie_candidates.size() > 1U)
    text << " It tied with other planners and "
         << record.planning_tie_break_reason << ".";
  for (const auto &candidate : record.planning_candidates) {
    response.referenced_planners.push_back(candidate.planner);
    text << ' ' << candidate.planner << " total=" << candidate.summed_score
         << " [";
    for (std::size_t index = 0U; index < candidate.objectives.size(); ++index) {
      if (index != 0U)
        text << ", ";
      text << candidate.objectives[index]
           << " raw=" << candidate.raw_costs[index]
           << " normalized=" << candidate.normalized_costs[index];
    }
    text << "].";
  }
  response.natural_language_response = text.str();
  return response;
}

ExplanationResponse
UnifiedWhySystem::explainAlternativePlan(const ExplanationQuestion &question,
                                         const DecisionRecord &record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "alternative_plan";
  const PlanCandidate *alternative = nullptr;
  for (const auto &candidate : record.planning_candidates) {
    if (candidate.plan_id == record.active_plan_id)
      continue;
    if (question.has_alternative_plan_id &&
        candidate.plan_id != question.alternative_plan_id)
      continue;
    if (!alternative || candidate.summed_score < alternative->summed_score)
      alternative = &candidate;
  }
  if (!alternative) {
    response.found = false;
    response.natural_language_response =
        "No recorded alternative plan exists; I did not silently replan.";
    return response;
  }
  response.has_plan_id = true;
  response.plan_id = alternative->plan_id;
  response.primary_reasoning_source = "chapter_five_recorded_alternative";
  response.referenced_planners.push_back(alternative->planner);
  response.referenced_model_revisions = alternative->dependency_revisions;
  for (const auto &step : alternative->typed_steps)
    response.referenced_plan_steps.push_back(step.step_id);
  response.natural_language_response =
      describeRoute(record, *alternative, true, response);
  return response;
}

ExplanationResponse
UnifiedWhySystem::explainPlanConfidence(const ExplanationQuestion &question,
                                        const DecisionRecord &record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "plan_confidence";
  response.primary_reasoning_source = "chapter_five_table_5_10";
  const auto *selected = selectedPlan(record);
  if (!selected) {
    response.found = false;
    response.natural_language_response = "No selected plan trace is available.";
    return response;
  }
  const PlanCandidate *comparison = nullptr;
  for (const auto &candidate : record.planning_candidates) {
    if (candidate.plan_id == selected->plan_id)
      continue;
    if (question.has_alternative_plan_id &&
        candidate.plan_id != question.alternative_plan_id)
      continue;
    if (!comparison || candidate.summed_score < comparison->summed_score)
      comparison = &candidate;
  }
  if (!comparison) {
    response.found = false;
    response.primary_reasoning_source = "comparison_plan_unavailable";
    response.confidence_category = "unavailable";
    response.natural_language_response =
        "Plan confidence is unavailable because no valid recorded comparison "
        "plan exists; I did not fabricate one.";
    return response;
  }
  const std::string robot_objective = selected->metadata.objective_name;
  const std::string comparison_objective = comparison->metadata.objective_name;
  const auto robot_family = objectiveFamily(robot_objective);
  const auto comparison_family = objectiveFamily(comparison_objective);
  const auto robot_selected_cost = objectiveCost(*selected, robot_objective);
  const auto robot_comparison_cost =
      objectiveCost(*comparison, robot_objective);
  const auto comparison_selected_cost =
      objectiveCost(*selected, comparison_objective);
  const auto comparison_comparison_cost =
      objectiveCost(*comparison, comparison_objective);
  if (robot_family == ObjectiveFamily::Unsupported ||
      comparison_family == ObjectiveFamily::Unsupported ||
      !robot_selected_cost || !robot_comparison_cost ||
      !comparison_selected_cost || !comparison_comparison_cost) {
    response.found = false;
    response.primary_reasoning_source = "comparison_objectives_unavailable";
    response.confidence_category = "unavailable";
    response.natural_language_response =
        "Plan confidence is unavailable because both recorded plans cannot "
        "be evaluated under the two required Chapter 5 objectives.";
    return response;
  }
  const double robot_difference = preferredObjectiveDifference(
      robot_family, *robot_selected_cost, *robot_comparison_cost);
  const double comparison_difference = preferredObjectiveDifference(
      comparison_family, *comparison_comparison_cost,
      *comparison_selected_cost);
  const int robot_magnitude =
      differenceMagnitudeLevel(robot_family, robot_difference);
  const int comparison_magnitude =
      differenceMagnitudeLevel(comparison_family, comparison_difference);
  const std::string robot_translation = magnitudePhrase(robot_magnitude);
  const std::string comparison_translation =
      magnitudePhrase(comparison_magnitude);
  response.confidence_category =
      planConfidenceCategory(robot_magnitude, comparison_magnitude);
  response.confidence_value = 0.0;
  response.referenced_planners = {selected->planner, comparison->planner};
  response.structured_facts = {"robot_objective=" + robot_objective,
                               "comparison_objective=" + comparison_objective,
                               "robot_plan_robot_objective_cost=" +
                                   std::to_string(*robot_selected_cost),
                               "comparison_plan_robot_objective_cost=" +
                                   std::to_string(*robot_comparison_cost),
                               "robot_plan_comparison_objective_cost=" +
                                   std::to_string(*comparison_selected_cost),
                               "comparison_plan_comparison_objective_cost=" +
                                   std::to_string(*comparison_comparison_cost),
                               "D_R=" + std::to_string(robot_difference),
                               "D_H=" + std::to_string(comparison_difference),
                               "M(D_R)=" + robot_translation,
                               "M(D_H)=" + comparison_translation,
                               "N(M(D_R))=" + robot_translation,
                               "N(M(D_H))=" + comparison_translation,
                               "N(M(D_R,D_H))=" + response.confidence_category};
  const auto robot_comparators = objectiveComparators(
      robot_objective, selected->metadata.objective_description);
  const auto comparison_comparators = objectiveComparators(
      comparison_objective, comparison->metadata.objective_description);
  std::ostringstream text;
  text << "I'm " << response.confidence_category << " sure because ";
  if (response.confidence_category == "really") {
    text << "my plan is " << robot_translation << ' ' << robot_comparators.first
         << " and only " << comparison_translation << ' '
         << comparison_comparators.second << " than the comparison plan.";
  } else if (response.confidence_category == "only somewhat") {
    text << "even though my plan is " << robot_translation << ' '
         << robot_comparators.first << ", it is also " << comparison_translation
         << ' ' << comparison_comparators.second
         << " than the comparison plan.";
  } else {
    text << "my plan is " << comparison_translation << ' '
         << comparison_comparators.second << " and only " << robot_translation
         << ' ' << robot_comparators.first << " than the comparison plan.";
  }
  response.natural_language_response = text.str();
  return response;
}

ExplanationResponse
UnifiedWhySystem::explainRoute(const ExplanationQuestion &question,
                               const DecisionRecord &record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "route_description";
  const auto *selected = selectedPlan(record);
  if (!selected) {
    response.found = false;
    response.natural_language_response = "No selected route is recorded.";
    return response;
  }
  response.primary_reasoning_source = "chapter_five_route_description";
  response.referenced_planners.push_back(selected->planner);
  for (const auto &step : selected->typed_steps)
    response.referenced_plan_steps.push_back(step.step_id);
  response.natural_language_response =
      describeRoute(record, *selected, false, response);
  return response;
}

ExplanationResponse
UnifiedWhySystem::comparePlan(const ExplanationQuestion &question,
                              const DecisionRecord &record) const {
  auto response = baseResponse(question, &record);
  response.explanation_category = "plan_comparison";
  const auto *selected = selectedPlan(record);
  if (!selected) {
    response.found = false;
    response.natural_language_response = "No selected plan is recorded.";
    return response;
  }
  const PlanCandidate *alternative = nullptr;
  if (question.has_alternative_plan_id) {
    for (const auto &candidate : record.planning_candidates)
      if (candidate.plan_id == question.alternative_plan_id)
        alternative = &candidate;
  }
  if (alternative) {
    response.primary_reasoning_source = "recorded_plan_cost_matrix";
    const auto count =
        std::min({selected->objectives.size(), selected->raw_costs.size(),
                  alternative->raw_costs.size()});
    for (std::size_t index = 0U; index < count; ++index) {
      response.structured_facts.push_back(
          selected->objectives[index] +
          ":robot=" + std::to_string(selected->raw_costs[index]) +
          ",alternative=" + std::to_string(alternative->raw_costs[index]));
    }
    response.natural_language_response =
        "The selected " + selected->planner + " plan scored " +
        std::to_string(selected->summed_score) + " versus " +
        std::to_string(alternative->summed_score) + " for " +
        alternative->planner +
        ". The lower total won; the objective arrays "
        "in the response trace preserve the tradeoffs rather than claiming "
        "unqualified superiority.";
    response.referenced_planners = {selected->planner, alternative->planner};
    return response;
  }
  if (!question.user_route.empty()) {
    if (!route_evaluator_) {
      response.found = false;
      response.primary_reasoning_source = "route_evaluator_unavailable";
      response.natural_language_response =
          "A user route was supplied, but no immutable objective evaluator is "
          "available. I will not invent crowd or learned-model costs after "
          "the fact.";
      return response;
    }
    const auto costs =
        route_evaluator_(question.user_route, selected->objectives);
    response.primary_reasoning_source = "immutable_route_evaluator";
    for (std::size_t index = 0U; index < costs.size(); ++index)
      response.structured_facts.push_back(selected->objectives.at(index) + "=" +
                                          std::to_string(costs[index]));
    response.natural_language_response =
        "The supplied route was evaluated under the same recorded objectives. "
        "The structured facts report each tradeoff; the selected route length "
        "was " +
        std::to_string(pathLength(selected->geometry)) + " metres.";
    return response;
  }
  response.found = false;
  response.natural_language_response =
      "Supply a recorded alternative plan ID or a user route for comparison.";
  return response;
}

ExplanationResponse UnifiedWhySystem::explainHypothetical(
    const ExplanationQuestion &question) const {
  auto response = baseResponse(question, nullptr);
  response.hypothetical = true;
  response.explanation_category = "hypothetical_pose";
  if (!question.has_hypothetical_pose || !hypothetical_evaluator_) {
    response.found = false;
    response.natural_language_response =
        "Hypothetical evaluation requires a pose and an immutable evaluator.";
    return response;
  }
  const auto hypothetical = hypothetical_evaluator_(question.hypothetical_pose);
  response = explainDecision(question, hypothetical);
  response.hypothetical = true;
  response.explanation_category = "hypothetical_pose";
  response.primary_reasoning_source = "immutable_world_snapshot";
  response.natural_language_response =
      "Hypothetically, without changing navigation state, " +
      response.natural_language_response;
  return response;
}

ExplanationResponse
UnifiedWhySystem::answer(const ExplanationQuestion &question) const {
  const auto type = inferredQuestionType(question);
  if (type == ExplanationQuestion::HYPOTHETICAL_POSE)
    return explainHypothetical(question);
  const auto *record = resolveDecision(question);
  if (!record) {
    auto response = baseResponse(question, nullptr);
    response.natural_language_response =
        "No matching historical decision or plan trace was found.";
    return response;
  }
  switch (type) {
  case ExplanationQuestion::WHY_DECISION:
    return explainDecision(question, *record);
  case ExplanationQuestion::DECISION_CONFIDENCE:
    return explainDecisionConfidence(question, *record);
  case ExplanationQuestion::WHY_NOT_ACTION:
    return explainCounterfactual(question, *record);
  case ExplanationQuestion::WHY_PLAN:
    return explainPlan(question, *record);
  case ExplanationQuestion::COMPARE_PLAN:
    return comparePlan(question, *record);
  case ExplanationQuestion::ALTERNATIVE_PLAN:
    return explainAlternativePlan(question, *record);
  case ExplanationQuestion::PLAN_CONFIDENCE:
    return explainPlanConfidence(question, *record);
  case ExplanationQuestion::ROUTE_DESCRIPTION:
    return explainRoute(question, *record);
  case ExplanationQuestion::COMBINED: {
    auto decision = explainDecision(question, *record);
    const auto route = explainRoute(question, *record);
    if (route.found) {
      decision.explanation_category = "combined_action_and_plan";
      decision.natural_language_response +=
          " " + route.natural_language_response;
      decision.referenced_plan_steps = route.referenced_plan_steps;
    }
    return decision;
  }
  default:
    break;
  }
  return explainDecision(question, *record);
}

} // namespace semaforr::why
