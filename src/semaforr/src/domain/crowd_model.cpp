/**
 * @file crowd_model.cpp
 * @brief Crowd model responsibilities.
 *
 * @details This file implements crowd model behavior for ROS-independent domain
 * state and value types. It records the declarations, settings, fixtures,
 * or guidance needed by that responsibility. Its package-relative location
 * is `src/domain/crowd_model.cpp`.
 */
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <istream>
#include <limits>
#include <ostream>
#include <semaforr/domain/crowd_model.hpp>
#include <stdexcept>
#include <string>

namespace semaforr::domain {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr const char* kSerializationMagic = "SEMAFORR_CROWD_FIELD_V1";

/**
 * @brief Performs the finite nonnegative operation for this subsystem.
 *
 * Arguments:
 * - @p value: Supplies value input to the operation.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool finiteNonnegative(double value) noexcept {
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

/**
 * @brief Performs the crowd flow direction angle operation for this
 * subsystem.
 *
 * Arguments:
 * - @p direction: Supplies direction input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double crowdFlowDirectionAngle(CrowdFlowDirection direction) noexcept {
  return static_cast<double>(static_cast<std::size_t>(direction)) * (kPi / 4.0);
}

/**
 * @brief Reports whether evidence for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool CrowdFieldCell::hasEvidence() const noexcept {
  return visibility_exposures > 0.0 || risk_experiences > 0.0;
}

/**
 * @brief Performs the finite operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool CrowdFieldCell::finite() const noexcept {
  return finiteNonnegative(density) &&
         finiteNonnegative(learned_encounter_risk) &&
         std::all_of(directional_flow.begin(), directional_flow.end(),
                     finiteNonnegative) &&
         finiteNonnegative(visibility_exposures) &&
         finiteNonnegative(pedestrian_hits) &&
         finiteNonnegative(risk_encounters) &&
         finiteNonnegative(risk_experiences) && last_updated.count() >= 0 &&
         std::isfinite(confidence) && confidence >= 0.0 && confidence <= 1.0;
}

/**
 * @brief Validates package content for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void CrowdFieldSnapshot::validate() const {
  geometry.validate();
  if (cells.size() != geometry.cellCount()) {
    throw std::invalid_argument(
        "crowd field cell count does not match grid geometry");
  }
  if (generated_at.count() < 0 || estimator.empty()) {
    throw std::invalid_argument(
        "crowd field timestamp must be nonnegative and estimator must be "
        "named");
  }
  if (!std::all_of(cells.begin(), cells.end(),
                   [](const CrowdFieldCell& cell) { return cell.finite(); })) {
    throw std::invalid_argument(
        "crowd field contains invalid or non-finite values");
  }
}

/**
 * @brief Performs the available operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `bool` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
bool CrowdFieldSnapshot::available() const noexcept {
  return version > 0U && std::any_of(cells.begin(), cells.end(),
                                     [](const CrowdFieldCell& cell) {
                                       return cell.hasEvidence();
                                     });
}

/**
 * @brief Performs the sample operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 * - @p now: Supplies now input to the operation.
 * - @p maximum_age: Supplies maximum age input to the operation.
 *
 * Returns:
 * - `std::optional<CrowdFieldSample>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<CrowdFieldSample> CrowdFieldSnapshot::sample(
    Point2D point, SocialTimestamp now,
    std::chrono::nanoseconds maximum_age) const noexcept {
  const auto cell_index = geometry.index(point);
  if (!cell_index || *cell_index >= cells.size() ||
      !cells[*cell_index].hasEvidence()) {
    return std::nullopt;
  }
  bool stale = false;
  if (maximum_age > std::chrono::nanoseconds::zero() &&
      now > cells[*cell_index].last_updated) {
    stale = now - cells[*cell_index].last_updated > maximum_age;
  }
  return CrowdFieldSample{geometry.center(*cell_index), cells[*cell_index],
                          stale};
}

/**
 * @brief Serializes package content for this subsystem.
 *
 * Arguments:
 * - @p output: Supplies output input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void CrowdFieldSnapshot::save(std::ostream& output) const {
  validate();
  output << kSerializationMagic << '\n'
         << std::quoted(geometry.frame_id) << ' ' << std::setprecision(17)
         << geometry.widthMeters() << ' ' << geometry.heightMeters() << ' '
         << geometry.resolution_m << ' ' << geometry.origin.x_m << ' '
         << geometry.origin.y_m << '\n'
         << generated_at.count() << ' ' << version << ' '
         << std::quoted(estimator) << ' ' << cells.size() << '\n';
  for (const auto& cell : cells) {
    output << cell.density << ' ' << cell.learned_encounter_risk;
    for (const double flow : cell.directional_flow) {
      output << ' ' << flow;
    }
    output << ' ' << cell.visibility_exposures << ' ' << cell.pedestrian_hits
           << ' ' << cell.risk_encounters << ' ' << cell.risk_experiences << ' '
           << cell.last_updated.count() << ' ' << cell.confidence << '\n';
  }
  if (!output) {
    throw std::runtime_error("failed to serialize crowd field");
  }
}

/**
 * @brief Loads package content for this subsystem.
 *
 * Arguments:
 * - @p input: Supplies input input to the operation.
 *
 * Returns:
 * - `CrowdFieldSnapshot` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
CrowdFieldSnapshot CrowdFieldSnapshot::load(std::istream& input) {
  std::string magic;
  std::getline(input, magic);
  if (magic != kSerializationMagic) {
    throw std::invalid_argument("unsupported crowd field serialization");
  }
  CrowdFieldSnapshot result;
  std::size_t cell_count = 0U;
  std::int64_t generated = 0;
  std::string frame;
  double width_m = 0.0, height_m = 0.0, resolution_m = 0.0;
  double origin_x_m = 0.0, origin_y_m = 0.0;
  input >> std::quoted(frame) >> width_m >> height_m >> resolution_m >>
      origin_x_m >> origin_y_m >> generated >> result.version >>
      std::quoted(result.estimator) >> cell_count;
  result.geometry = {std::move(frame), width_m,    height_m,
                     resolution_m,     origin_x_m, origin_y_m};
  result.generated_at = SocialTimestamp(generated);
  result.cells.resize(cell_count);
  for (auto& cell : result.cells) {
    std::int64_t updated = 0;
    input >> cell.density >> cell.learned_encounter_risk;
    for (double& flow : cell.directional_flow) {
      input >> flow;
    }
    input >> cell.visibility_exposures >> cell.pedestrian_hits >>
        cell.risk_encounters >> cell.risk_experiences >> updated >>
        cell.confidence;
    cell.last_updated = SocialTimestamp(updated);
  }
  if (!input) {
    throw std::invalid_argument("crowd field serialization is truncated");
  }
  result.validate();
  return result;
}

/**
 * @brief Records mutation for this subsystem.
 *
 * Arguments:
 * - @p dependency: Supplies dependency input to the operation.
 * - @p summary: Supplies summary input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void CrowdModel::recordMutation(ModelDependency dependency,
                                std::string summary) {
  const Revision revision = ++revisions_[dependency];
  mutation_history_.push_back({++mutation_sequence_, dependency, revision,
                               std::chrono::steady_clock::now(),
                               std::move(summary)});
}

/**
 * @brief Updates input diagnostics for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void CrowdModel::updateInputDiagnostics(const CrowdObservation& observation) {
  input_source_ =
      observation.provenance.empty() ? "unknown" : observation.provenance;
  formation_evidence_available_ = !observation.formations.empty();
  formation_evidence_participated_ = false;
  bool gst = false;
  bool fallback = false;
  bool missing = false;
  for (const auto& pedestrian : observation.pedestrians) {
    gst = gst || pedestrian.prediction_source == "gst";
    fallback = fallback || pedestrian.prediction_source == "constant_velocity";
    missing = missing || pedestrian.prediction_source.empty() ||
              pedestrian.prediction_source == "none";
  }
  if (observation.pedestrians.empty()) {
    prediction_source_ = "not_applicable";
    input_status_ = "ready_empty";
  } else if (gst && !fallback && !missing) {
    prediction_source_ = "gst";
    input_status_ = "ready";
  } else if (fallback && !gst && !missing) {
    prediction_source_ = "constant_velocity";
    input_status_ = "degraded_fallback_prediction";
  } else if (gst || fallback) {
    prediction_source_ = "mixed";
    input_status_ = "degraded_mixed_prediction";
  } else {
    prediction_source_ = "none";
    input_status_ = "degraded_missing_prediction";
  }
}

/**
 * @brief Updates package content for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 * - @p history_limit: Supplies history limit input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void CrowdModel::update(CrowdObservation observation,
                        std::size_t history_limit) {
  observation.validate();
  const bool changed =
      !observations_.current() || *observations_.current() != observation;
  observations_.update(std::move(observation), history_limit);
  updateInputDiagnostics(*observations_.current());
  if (changed)
    recordMutation(ModelDependency::LiveCrowdObservation,
                   "validated live social observation updated");
}

/**
 * @brief Performs the replace current operation for this subsystem.
 *
 * Arguments:
 * - @p observation: Supplies observation input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void CrowdModel::replaceCurrent(CrowdObservation observation) {
  observation.validate();
  const bool changed =
      !observations_.current() || *observations_.current() != observation;
  observations_.replaceCurrent(std::move(observation));
  updateInputDiagnostics(*observations_.current());
  if (changed)
    recordMutation(ModelDependency::LiveCrowdObservation,
                   "live social observation replaced");
}

/**
 * @brief Clears current for this subsystem.
 *
 * Arguments:
 * - @p status: Supplies status input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void CrowdModel::clearCurrent(std::string status) {
  const bool changed = observations_.current().has_value();
  observations_.clearCurrent();
  prediction_source_ = "none";
  input_status_ = status.empty() ? "unavailable" : std::move(status);
  formation_evidence_available_ = false;
  formation_evidence_participated_ = false;
  if (changed)
    recordMutation(ModelDependency::LiveCrowdObservation,
                   "live social observation cleared: " + input_status_);
}

/**
 * @brief Sets learned for this subsystem.
 *
 * Arguments:
 * - @p snapshot: Supplies snapshot input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void CrowdModel::setLearned(CrowdFieldSnapshot snapshot) {
  snapshot.validate();
  const bool geometry_changed = learned_.geometry != snapshot.geometry ||
                                learned_.cells.size() != snapshot.cells.size();
  const auto layerChanged = [&](auto projection) {
    if (geometry_changed) return true;
    for (std::size_t index = 0U; index < snapshot.cells.size(); ++index)
      if (projection(learned_.cells[index]) !=
          projection(snapshot.cells[index]))
        return true;
    return false;
  };
  const auto record = [&](ModelDependency dependency, bool changed) {
    if (!changed) return;
    recordMutation(dependency, "crowd field layer changed");
  };
  record(ModelDependency::CrowdDensity, layerChanged([](const auto& cell) {
           return std::array{cell.density, cell.visibility_exposures,
                             cell.pedestrian_hits, cell.confidence};
         }));
  record(ModelDependency::CrowdRisk, layerChanged([](const auto& cell) {
           return std::array{cell.learned_encounter_risk, cell.risk_encounters,
                             cell.risk_experiences};
         }));
  record(ModelDependency::CrowdFlow, layerChanged([](const auto& cell) {
           return std::pair{cell.directional_flow, cell.confidence};
         }));
  learned_ = std::move(snapshot);
}

/**
 * @brief Performs the status operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `CrowdModelStatus` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
CrowdModelStatus CrowdModel::status() const noexcept {
  const bool live = current().has_value();
  const bool learned = learnedAvailable();
  if (live && learned) return CrowdModelStatus::LiveAndLearned;
  if (live) return CrowdModelStatus::LiveOnly;
  if (learned) return CrowdModelStatus::LearnedOnly;
  return CrowdModelStatus::Unavailable;
}

/**
 * @brief Performs the learned at operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 * - @p now: Supplies now input to the operation.
 * - @p maximum_age: Supplies maximum age input to the operation.
 *
 * Returns:
 * - `std::optional<CrowdFieldSample>` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
std::optional<CrowdFieldSample> CrowdModel::learnedAt(
    Point2D point, SocialTimestamp now,
    std::chrono::nanoseconds maximum_age) const noexcept {
  return learned_.sample(point, now, maximum_age);
}

/**
 * @brief Performs the density at operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double CrowdModel::densityAt(Point2D point) const noexcept {
  const auto sample = learnedAt(point);
  return sample && !sample->stale ? sample->cell.density : 0.0;
}

/**
 * @brief Performs the learned encounter risk at operation for this
 * subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double CrowdModel::learnedEncounterRiskAt(Point2D point) const noexcept {
  const auto sample = learnedAt(point);
  return sample && !sample->stale ? sample->cell.learned_encounter_risk : 0.0;
}

/**
 * @brief Performs the visibility exposures at operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double CrowdModel::visibilityExposuresAt(Point2D point) const noexcept {
  const auto sample = learnedAt(point);
  return sample ? sample->cell.visibility_exposures : 0.0;
}

/**
 * @brief Performs the risk experiences at operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double CrowdModel::riskExperiencesAt(Point2D point) const noexcept {
  const auto sample = learnedAt(point);
  return sample ? sample->cell.risk_experiences : 0.0;
}

/**
 * @brief Performs the flow observation at operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double CrowdModel::flowObservationAt(Point2D point) const noexcept {
  const auto sample = learnedAt(point);
  if (!sample) return 0.0;
  double total = 0.0;
  for (const double flow : sample->cell.directional_flow) {
    total += flow;
  }
  return total;
}

/**
 * @brief Performs the flow alignment at operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 * - @p travel_direction: Supplies travel direction input to the operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double CrowdModel::flowAlignmentAt(Point2D point,
                                   Angle travel_direction) const noexcept {
  const auto sample = learnedAt(point);
  if (!sample || sample->stale) return 0.0;
  double alignment = 0.0;
  for (std::size_t index = 0U; index < kCrowdFlowDirectionCount; ++index) {
    alignment += sample->cell.directional_flow[index] *
                 std::cos(crowdFlowDirectionAngle(
                              static_cast<CrowdFlowDirection>(index)) -
                          travel_direction.radians());
  }
  return alignment;
}

/**
 * @brief Performs the predictive collision risk at operation for this
 * subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 * - @p gaussian_variance_m2: Supplies gaussian variance m2 input to the
 * operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double CrowdModel::predictiveCollisionRiskAt(
    Point2D point, double gaussian_variance_m2) const noexcept {
  if (!current() || !std::isfinite(gaussian_variance_m2) ||
      gaussian_variance_m2 <= 0.0) {
    return 0.0;
  }
  double risk = 0.0;
  for (const auto& pedestrian : current()->pedestrians) {
    const auto add = [&](Point2D position) {
      const double dx = point.x_m - position.x_m;
      const double dy = point.y_m - position.y_m;
      risk = std::max(
          risk, pedestrian.confidence * std::exp(-(dx * dx + dy * dy) /
                                                 (2.0 * gaussian_variance_m2)));
    };
    add(pedestrian.position);
    for (const auto& prediction : pedestrian.predicted_trajectory) {
      add(prediction.position);
    }
  }
  return risk;
}

/**
 * @brief Performs the navigation risk at operation for this subsystem.
 *
 * Arguments:
 * - @p point: Supplies point input to the operation.
 * - @p gaussian_variance_m2: Supplies gaussian variance m2 input to the
 * operation.
 *
 * Returns:
 * - `double` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
double CrowdModel::navigationRiskAt(
    Point2D point, double gaussian_variance_m2) const noexcept {
  return std::max(learnedEncounterRiskAt(point),
                  predictiveCollisionRiskAt(point, gaussian_variance_m2));
}

}  // namespace semaforr::domain
