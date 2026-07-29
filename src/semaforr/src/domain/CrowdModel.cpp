#include <semaforr/domain/crowd_model.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <istream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>

namespace semaforr::domain {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr const char* kSerializationMagic = "SEMAFORR_CROWD_FIELD_V1";

bool finiteNonnegative(double value) noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

void GridGeometry::validate() const
{
  if (frame_id.empty()) {
    throw std::invalid_argument("crowd grid frame must not be empty");
  }
  if (!std::isfinite(width_m) || width_m <= 0.0 ||
      !std::isfinite(height_m) || height_m <= 0.0 ||
      !std::isfinite(resolution_m) || resolution_m <= 0.0 ||
      !std::isfinite(origin_x_m) || !std::isfinite(origin_y_m)) {
    throw std::invalid_argument(
      "crowd grid dimensions, resolution, and origin must be finite; "
      "dimensions and resolution must be positive");
  }
  if (cellCount() > 10'000'000U) {
    throw std::invalid_argument("crowd grid contains too many cells");
  }
}

std::size_t GridGeometry::columns() const
{
  return static_cast<std::size_t>(std::ceil(width_m / resolution_m));
}

std::size_t GridGeometry::rows() const
{
  return static_cast<std::size_t>(std::ceil(height_m / resolution_m));
}

std::size_t GridGeometry::cellCount() const
{
  const auto column_count = columns();
  const auto row_count = rows();
  if (column_count != 0U &&
      row_count > std::numeric_limits<std::size_t>::max() / column_count) {
    throw std::overflow_error("crowd grid cell count overflow");
  }
  return column_count * row_count;
}

std::optional<std::size_t> GridGeometry::index(Point2D point) const noexcept
{
  if (!point.finite() || !std::isfinite(resolution_m) ||
      resolution_m <= 0.0) {
    return std::nullopt;
  }
  const double relative_x = point.x_m - origin_x_m;
  const double relative_y = point.y_m - origin_y_m;
  if (relative_x < 0.0 || relative_y < 0.0 ||
      relative_x >= width_m || relative_y >= height_m) {
    return std::nullopt;
  }
  const auto column =
    static_cast<std::size_t>(std::floor(relative_x / resolution_m));
  const auto row =
    static_cast<std::size_t>(std::floor(relative_y / resolution_m));
  if (column >= columns() || row >= rows()) {
    return std::nullopt;
  }
  return row * columns() + column;
}

Point2D GridGeometry::center(std::size_t index_value) const
{
  if (index_value >= cellCount()) {
    throw std::out_of_range("crowd grid cell index is out of range");
  }
  const std::size_t column = index_value % columns();
  const std::size_t row = index_value / columns();
  return {
    origin_x_m + (static_cast<double>(column) + 0.5) * resolution_m,
    origin_y_m + (static_cast<double>(row) + 0.5) * resolution_m};
}

double crowdFlowDirectionAngle(CrowdFlowDirection direction) noexcept
{
  return static_cast<double>(static_cast<std::size_t>(direction)) *
    (kPi / 4.0);
}

bool CrowdFieldCell::hasEvidence() const noexcept
{
  return visibility_exposures > 0.0 || risk_experiences > 0.0;
}

bool CrowdFieldCell::finite() const noexcept
{
  return finiteNonnegative(density) &&
    finiteNonnegative(learned_encounter_risk) &&
    std::all_of(
      directional_flow.begin(), directional_flow.end(),
      finiteNonnegative) &&
    finiteNonnegative(visibility_exposures) &&
    finiteNonnegative(pedestrian_hits) &&
    finiteNonnegative(risk_encounters) &&
    finiteNonnegative(risk_experiences) &&
    last_updated.count() >= 0 &&
    std::isfinite(confidence) && confidence >= 0.0 && confidence <= 1.0;
}

void CrowdFieldSnapshot::validate() const
{
  geometry.validate();
  if (cells.size() != geometry.cellCount()) {
    throw std::invalid_argument(
      "crowd field cell count does not match grid geometry");
  }
  if (generated_at.count() < 0 || estimator.empty()) {
    throw std::invalid_argument(
      "crowd field timestamp must be nonnegative and estimator must be named");
  }
  if (!std::all_of(
      cells.begin(), cells.end(),
      [](const CrowdFieldCell& cell) { return cell.finite(); })) {
    throw std::invalid_argument(
      "crowd field contains invalid or non-finite values");
  }
}

bool CrowdFieldSnapshot::available() const noexcept
{
  return version > 0U &&
    std::any_of(
      cells.begin(), cells.end(),
      [](const CrowdFieldCell& cell) { return cell.hasEvidence(); });
}

std::optional<CrowdFieldSample> CrowdFieldSnapshot::sample(
  Point2D point,
  SocialTimestamp now,
  std::chrono::nanoseconds maximum_age) const noexcept
{
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
  return CrowdFieldSample{
    geometry.center(*cell_index), cells[*cell_index], stale};
}

void CrowdFieldSnapshot::save(std::ostream& output) const
{
  validate();
  output << kSerializationMagic << '\n'
         << std::quoted(geometry.frame_id) << ' '
         << std::setprecision(17)
         << geometry.width_m << ' ' << geometry.height_m << ' '
         << geometry.resolution_m << ' ' << geometry.origin_x_m << ' '
         << geometry.origin_y_m << '\n'
         << generated_at.count() << ' ' << version << ' '
         << std::quoted(estimator) << ' ' << cells.size() << '\n';
  for (const auto& cell : cells) {
    output << cell.density << ' ' << cell.learned_encounter_risk;
    for (const double flow : cell.directional_flow) {
      output << ' ' << flow;
    }
    output << ' ' << cell.visibility_exposures
           << ' ' << cell.pedestrian_hits
           << ' ' << cell.risk_encounters
           << ' ' << cell.risk_experiences
           << ' ' << cell.last_updated.count()
           << ' ' << cell.confidence << '\n';
  }
  if (!output) {
    throw std::runtime_error("failed to serialize crowd field");
  }
}

CrowdFieldSnapshot CrowdFieldSnapshot::load(std::istream& input)
{
  std::string magic;
  std::getline(input, magic);
  if (magic != kSerializationMagic) {
    throw std::invalid_argument("unsupported crowd field serialization");
  }
  CrowdFieldSnapshot result;
  std::size_t cell_count = 0U;
  std::int64_t generated = 0;
  input >> std::quoted(result.geometry.frame_id)
        >> result.geometry.width_m >> result.geometry.height_m
        >> result.geometry.resolution_m >> result.geometry.origin_x_m
        >> result.geometry.origin_y_m
        >> generated >> result.version >> std::quoted(result.estimator)
        >> cell_count;
  result.generated_at = SocialTimestamp(generated);
  result.cells.resize(cell_count);
  for (auto& cell : result.cells) {
    std::int64_t updated = 0;
    input >> cell.density >> cell.learned_encounter_risk;
    for (double& flow : cell.directional_flow) {
      input >> flow;
    }
    input >> cell.visibility_exposures >> cell.pedestrian_hits
          >> cell.risk_encounters >> cell.risk_experiences
          >> updated >> cell.confidence;
    cell.last_updated = SocialTimestamp(updated);
  }
  if (!input) {
    throw std::invalid_argument("crowd field serialization is truncated");
  }
  result.validate();
  return result;
}

void CrowdModel::setLearned(CrowdFieldSnapshot snapshot)
{
  snapshot.validate();
  learned_ = std::move(snapshot);
}

CrowdModelStatus CrowdModel::status() const noexcept
{
  const bool live = current().has_value();
  const bool learned = learnedAvailable();
  if (live && learned) return CrowdModelStatus::LiveAndLearned;
  if (live) return CrowdModelStatus::LiveOnly;
  if (learned) return CrowdModelStatus::LearnedOnly;
  return CrowdModelStatus::Unavailable;
}

std::optional<CrowdFieldSample> CrowdModel::learnedAt(
  Point2D point,
  SocialTimestamp now,
  std::chrono::nanoseconds maximum_age) const noexcept
{
  return learned_.sample(point, now, maximum_age);
}

double CrowdModel::densityAt(Point2D point) const noexcept
{
  const auto sample = learnedAt(point);
  return sample && !sample->stale ? sample->cell.density : 0.0;
}

double CrowdModel::learnedEncounterRiskAt(Point2D point) const noexcept
{
  const auto sample = learnedAt(point);
  return sample && !sample->stale
    ? sample->cell.learned_encounter_risk : 0.0;
}

double CrowdModel::visibilityExposuresAt(Point2D point) const noexcept
{
  const auto sample = learnedAt(point);
  return sample ? sample->cell.visibility_exposures : 0.0;
}

double CrowdModel::riskExperiencesAt(Point2D point) const noexcept
{
  const auto sample = learnedAt(point);
  return sample ? sample->cell.risk_experiences : 0.0;
}

double CrowdModel::flowObservationAt(Point2D point) const noexcept
{
  const auto sample = learnedAt(point);
  if (!sample) return 0.0;
  double total = 0.0;
  for (const double flow : sample->cell.directional_flow) {
    total += flow;
  }
  return total;
}

double CrowdModel::flowAlignmentAt(
  Point2D point,
  Angle travel_direction) const noexcept
{
  const auto sample = learnedAt(point);
  if (!sample || sample->stale) return 0.0;
  double alignment = 0.0;
  for (std::size_t index = 0U;
       index < kCrowdFlowDirectionCount; ++index) {
    alignment += sample->cell.directional_flow[index] * std::cos(
      crowdFlowDirectionAngle(
        static_cast<CrowdFlowDirection>(index)) -
      travel_direction.radians());
  }
  return alignment;
}

double CrowdModel::predictiveCollisionRiskAt(
  Point2D point,
  double gaussian_variance_m2) const noexcept
{
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
        risk,
        pedestrian.confidence *
          std::exp(-(dx * dx + dy * dy) /
            (2.0 * gaussian_variance_m2)));
    };
    add(pedestrian.position);
    for (const auto& prediction : pedestrian.predicted_trajectory) {
      add(prediction.position);
    }
  }
  return risk;
}

double CrowdModel::navigationRiskAt(
  Point2D point,
  double gaussian_variance_m2) const noexcept
{
  return std::max(
    learnedEncounterRiskAt(point),
    predictiveCollisionRiskAt(point, gaussian_variance_m2));
}

}  // namespace semaforr::domain
