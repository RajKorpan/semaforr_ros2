#include <algorithm>
#include <cmath>
#include <iomanip>
#include <semaforr/validation/experiment_metrics.hpp>
#include <semaforr/spatial/coverage.hpp>
#include <sstream>
#include <stdexcept>

namespace semaforr::validation {
namespace {

void requireMeasurement(double value, const char* name) {
  if (!std::isfinite(value) || value < 0.0)
    throw std::invalid_argument(std::string(name) +
                                " must be finite and nonnegative");
}

std::string escape(std::string_view value) {
  std::string result;
  for (const char character : value) {
    if (character == '\\' || character == '"') result.push_back('\\');
    result.push_back(character);
  }
  return result;
}

}  // namespace

ExperimentMetricsCollector::ExperimentMetricsCollector(
    std::string scenario, std::string profile, std::size_t freespace_cells)
    : scenario_(std::move(scenario)),
      profile_(std::move(profile)),
      freespace_cells_(freespace_cells) {
  if (scenario_.empty() || profile_.empty() || freespace_cells_ == 0U)
    throw std::invalid_argument("experiment identity and freespace are required");
}

void ExperimentMetricsCollector::record(
    const ExperimentObservation& observation) {
  requireMeasurement(observation.runtime_s, "runtime");
  requireMeasurement(observation.planning_latency_s, "planning latency");
  requireMeasurement(observation.model_update_cost_s, "model update cost");
  if (observation.covered_cells &&
      *observation.covered_cells > freespace_cells_)
    throw std::invalid_argument("covered cells exceed scenario freespace");
  observations_.push_back(observation);
}

void ExperimentMetricsCollector::recordTargetOutcome(bool succeeded) {
  ++targets_attempted_;
  if (succeeded) ++targets_succeeded_;
}

ExperimentSummary ExperimentMetricsCollector::summary() const {
  ExperimentSummary result;
  result.scenario = scenario_;
  result.profile = profile_;
  result.targets_attempted = targets_attempted_;
  result.targets_succeeded = targets_succeeded_;
  result.decisions = observations_.size();
  result.success_rate = targets_attempted_ == 0U
                            ? 0.0
                            : static_cast<double>(targets_succeeded_) /
                                  static_cast<double>(targets_attempted_);
  std::optional<domain::Pose2D> previous_pose;
  std::size_t latest_coverage = 0U;
  double decision_latency_total = 0.0;
  for (const auto& observation : observations_) {
    if (result.configuration_fingerprint.empty())
      result.configuration_fingerprint =
          observation.decision.configuration_fingerprint;
    const bool exploring = observation.decision.navigation_phase ==
                           navigation::NavigationPhase::InitialExploration;
    if (previous_pose) {
      const double distance = domain::distance(
                                  previous_pose->position,
                                  observation.decision.robot_pose.position)
                                  .meters();
      if (exploring)
        result.exploration_distance_m += distance;
      else
        result.target_distance_m += distance;
    }
    previous_pose = observation.decision.robot_pose;
    if (exploring)
      result.exploration_runtime_s += observation.runtime_s;
    else
      result.target_runtime_s += observation.runtime_s;
    result.planning_latency_s += observation.planning_latency_s;
    result.model_update_cost_s += observation.model_update_cost_s;
    result.allocation_count += observation.allocations.count;
    result.allocation_bytes += observation.allocations.bytes;
    decision_latency_total += observation.decision.decision_latency_s;
    result.maximum_decision_latency_s = std::max(
        result.maximum_decision_latency_s,
        observation.decision.decision_latency_s);
    if (observation.covered_cells)
      latest_coverage = *observation.covered_cells;
    const std::string intervention =
        observation.decision.selected_policy.empty()
            ? std::string(decision::toString(observation.decision.tier))
            : observation.decision.selected_policy;
    ++result.intervention_counts[intervention];
  }
  result.total_distance_m =
      result.exploration_distance_m + result.target_distance_m;
  result.total_runtime_s =
      result.exploration_runtime_s + result.target_runtime_s;
  result.mean_decision_latency_s = observations_.empty()
                                       ? 0.0
                                       : decision_latency_total /
                                             observations_.size();
  result.coverage = static_cast<double>(latest_coverage) /
                    static_cast<double>(freespace_cells_);
  for (const auto& [name, count] : result.intervention_counts)
    result.intervention_frequency[name] =
        observations_.empty()
            ? 0.0
            : static_cast<double>(count) /
                  static_cast<double>(observations_.size());
  return result;
}

std::string ExperimentMetricsCollector::serialize() const {
  const auto value = summary();
  std::ostringstream output;
  output << std::setprecision(17) << "{\"schema_version\":1"
         << ",\"scenario\":\"" << escape(value.scenario) << "\""
         << ",\"profile\":\"" << escape(value.profile) << "\""
         << ",\"configuration_fingerprint\":\""
         << escape(value.configuration_fingerprint) << "\""
         << ",\"targets_attempted\":" << value.targets_attempted
         << ",\"targets_succeeded\":" << value.targets_succeeded
         << ",\"success_rate\":" << value.success_rate
         << ",\"decisions\":" << value.decisions
         << ",\"distance_m\":{\"exploration\":"
         << value.exploration_distance_m << ",\"targets\":"
         << value.target_distance_m << ",\"total\":"
         << value.total_distance_m << "}"
         << ",\"runtime_s\":{\"exploration\":"
         << value.exploration_runtime_s << ",\"targets\":"
         << value.target_runtime_s << ",\"total\":"
         << value.total_runtime_s << "}"
         << ",\"decision_latency_s\":{\"mean\":"
         << value.mean_decision_latency_s << ",\"maximum\":"
         << value.maximum_decision_latency_s << "}"
         << ",\"planning_latency_s\":" << value.planning_latency_s
         << ",\"model_update_cost_s\":" << value.model_update_cost_s
         << ",\"allocations\":{\"count\":" << value.allocation_count
         << ",\"bytes\":" << value.allocation_bytes << "}"
         << ",\"coverage\":" << value.coverage
         << ",\"intervention_frequency\":{";
  bool first = true;
  for (const auto& [name, frequency] : value.intervention_frequency) {
    if (!first) output << ',';
    first = false;
    output << '"' << escape(name) << "\":" << frequency;
  }
  output << "}}";
  return output.str();
}

std::size_t ExperimentMetricsCollector::coveredCells(
    const domain::SpatialModel& model) {
  return spatial::representedCoverageCells(model);
}

}  // namespace semaforr::validation
