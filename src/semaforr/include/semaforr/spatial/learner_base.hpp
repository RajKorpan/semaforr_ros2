#ifndef SEMAFORR_SPATIAL_LEARNER_BASE_HPP
#define SEMAFORR_SPATIAL_LEARNER_BASE_HPP

#include <semaforr/spatial/learner.hpp>
#include <string>
#include <utility>
#include <vector>

namespace semaforr::spatial {

class SpatialLearnerBase : public SpatialLearner {
 public:
  SpatialLearnerBase(SpatialRepresentation representation, std::string name,
                     UpdateMode mode, ObservationContract contract);

  void observe(const NavigationEpisode& episode) final;
  void rebuild() final;
  SpatialModelUpdate snapshot() const final;
  SharedSpatialSnapshot sharedSnapshot() const final;

  SpatialRepresentation representation() const noexcept final {
    return update_.representation;
  }
  std::string_view name() const noexcept final { return update_.learner; }
  const ObservationContract& contract() const noexcept final {
    return contract_;
  }

 protected:
  virtual void onObserve(const NavigationEpisode& episode) = 0;
  virtual void onRebuild() = 0;

  void publish(SpatialPayload payload, ModelStatus status,
               std::string diagnostic = {});
  void markIncomplete(std::string diagnostic);
  const std::vector<NavigationEpisode>& episodes() const noexcept {
    return episodes_;
  }

 private:
  ObservationContract contract_;
  SpatialModelUpdate update_;
  SharedSpatialSnapshot published_;
  mutable SharedSpatialSnapshot metadata_snapshot_;
  std::string published_payload_signature_;
  std::vector<NavigationEpisode> episodes_;
};

}  // namespace semaforr::spatial

#endif  // SEMAFORR_SPATIAL_LEARNER_BASE_HPP
