#ifndef SEMAFORR_DOMAIN_SOCIAL_HPP
#define SEMAFORR_DOMAIN_SOCIAL_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <semaforr/domain/geometry.hpp>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace semaforr::domain {

using SocialTimestamp = std::chrono::nanoseconds;

struct PredictedPosition {
  Point2D position;
  SocialTimestamp predicted_at{};

  bool operator==(const PredictedPosition&) const = default;
};

struct PedestrianObservation {
  std::string id;
  Point2D position;
  Point2D velocity_mps;
  std::vector<PredictedPosition> predicted_trajectory;
  double confidence{0.0};
  std::array<double, 4> position_covariance{};
  std::string prediction_source{"none"};
  std::optional<std::size_t> formation_index;

  void validate(SocialTimestamp observed_at) const {
    if (id.empty()) {
      throw std::invalid_argument("pedestrian ID must not be empty");
    }
    if (!position.finite() || !velocity_mps.finite()) {
      throw std::invalid_argument(
          "pedestrian position and velocity must be finite");
    }
    if (!std::isfinite(confidence) || confidence < 0.0 || confidence > 1.0) {
      throw std::invalid_argument(
          "pedestrian confidence must be within [0, 1]");
    }
    if (!std::all_of(position_covariance.begin(), position_covariance.end(),
                     [](double value) { return std::isfinite(value); }) ||
        position_covariance[0] < 0.0 || position_covariance[3] < 0.0 ||
        std::abs(position_covariance[1] - position_covariance[2]) > 1.0e-9 ||
        position_covariance[0] * position_covariance[3] -
                position_covariance[1] * position_covariance[2] <
            -1.0e-12) {
      throw std::invalid_argument(
          "pedestrian covariance must be finite, symmetric, and "
          "positive semidefinite");
    }

    SocialTimestamp previous = observed_at;
    for (const auto& prediction : predicted_trajectory) {
      if (!prediction.position.finite()) {
        throw std::invalid_argument(
            "predicted pedestrian positions must be finite");
      }
      if (prediction.predicted_at <= previous) {
        throw std::invalid_argument(
            "prediction timestamps must be strictly increasing and future");
      }
      previous = prediction.predicted_at;
    }
  }

  bool operator==(const PedestrianObservation&) const = default;
};

struct FormationObservation {
  std::vector<std::string> member_ids;
  std::string formation_type;
  Point2D center;
  double confidence{0.0};

  void validate() const {
    if (member_ids.empty() || formation_type.empty() || !center.finite() ||
        !std::isfinite(confidence) || confidence < 0.0 || confidence > 1.0) {
      throw std::invalid_argument("invalid social formation observation");
    }
    std::unordered_set<std::string> identifiers;
    for (const auto& id : member_ids) {
      if (id.empty() || !identifiers.insert(id).second) {
        throw std::invalid_argument(
            "formation members must have unique nonempty IDs");
      }
    }
  }

  bool operator==(const FormationObservation&) const = default;
};

struct CrowdObservation {
  std::string frame_id;
  SocialTimestamp observed_at{};
  std::chrono::nanoseconds data_age{};
  std::string provenance;
  std::vector<PedestrianObservation> pedestrians;
  std::vector<FormationObservation> formations;

  void validate() const {
    if (frame_id.empty()) {
      throw std::invalid_argument("social observation frame must not be empty");
    }
    if (observed_at.count() < 0 || data_age.count() < 0) {
      throw std::invalid_argument(
          "social observation timestamps and age must be non-negative");
    }
    std::unordered_set<std::string> identifiers;
    for (const auto& pedestrian : pedestrians) {
      pedestrian.validate(observed_at);
      if (!identifiers.insert(pedestrian.id).second) {
        throw std::invalid_argument(
            "social observation contains duplicate pedestrian ID '" +
            pedestrian.id + "'");
      }
    }
    for (const auto& formation : formations) {
      formation.validate();
      for (const auto& id : formation.member_ids) {
        if (!identifiers.contains(id)) {
          throw std::invalid_argument(
              "formation references unknown pedestrian ID '" + id + "'");
        }
      }
    }
    for (const auto& pedestrian : pedestrians) {
      if (pedestrian.formation_index &&
          *pedestrian.formation_index >= formations.size()) {
        throw std::invalid_argument(
            "pedestrian formation index is outside the formation array");
      }
    }
  }

  bool usable(std::chrono::nanoseconds maximum_age,
              double minimum_confidence = 0.0) const noexcept {
    if (!fresh(maximum_age) || pedestrians.empty()) {
      return false;
    }
    return std::any_of(pedestrians.begin(), pedestrians.end(),
                       [minimum_confidence](const auto& pedestrian) {
                         return std::isfinite(pedestrian.confidence) &&
                                pedestrian.confidence >= minimum_confidence;
                       });
  }

  // A valid empty observation is useful negative evidence for the learned
  // crowd field even though no live social advisor should participate.
  bool fresh(std::chrono::nanoseconds maximum_age) const noexcept {
    return !frame_id.empty() && data_age >= std::chrono::nanoseconds::zero() &&
           data_age <= maximum_age;
  }

  bool operator==(const CrowdObservation&) const = default;
};

class CrowdState {
 public:
  void update(CrowdObservation observation, std::size_t history_limit = 100U) {
    observation.validate();
    current_ = std::move(observation);
    history_.push_back(*current_);
    if (history_.size() > history_limit) {
      history_.erase(history_.begin(),
                     history_.begin() + static_cast<std::ptrdiff_t>(
                                            history_.size() - history_limit));
    }
  }

  void clearCurrent() noexcept { current_.reset(); }

  void replaceCurrent(CrowdObservation observation) {
    observation.validate();
    current_ = std::move(observation);
  }

  const std::optional<CrowdObservation>& current() const noexcept {
    return current_;
  }

  const std::vector<CrowdObservation>& history() const noexcept {
    return history_;
  }

  bool hasValidData(std::chrono::nanoseconds maximum_age,
                    double minimum_confidence = 0.0) const noexcept {
    return current_ && current_->usable(maximum_age, minimum_confidence);
  }

 private:
  std::optional<CrowdObservation> current_;
  std::vector<CrowdObservation> history_;
};

}  // namespace semaforr::domain

#endif  // SEMAFORR_DOMAIN_SOCIAL_HPP
