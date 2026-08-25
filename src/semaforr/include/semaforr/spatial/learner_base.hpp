/**
 * @file learner_base.hpp
 * @brief Learner base responsibilities.
 *
 * @details This file defines learner base behavior for learned spatial
 * representations and their lifecycle. It centers on `SpatialLearnerBase`.
 * Its package-relative location is
 * `include/semaforr/spatial/learner_base.hpp`.
 */
#ifndef SEMAFORR_SPATIAL_LEARNER_BASE_HPP
#define SEMAFORR_SPATIAL_LEARNER_BASE_HPP

#include <semaforr/spatial/learner.hpp>
#include <string>
#include <utility>
#include <vector>

namespace semaforr::spatial {

/**
 * @brief Encapsulates spatial learner base state and behavior for this
 * subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - Not applicable to this declaration.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
class SpatialLearnerBase : public SpatialLearner {
 public:
  /**
   * @brief Performs the spatial learner base operation for this subsystem.
   *
   * Arguments:
   * - @p representation: Supplies representation input to the operation.
   * - @p name: Supplies name input to the operation.
   * - @p mode: Supplies mode input to the operation.
   * - @p contract: Supplies contract input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  SpatialLearnerBase(SpatialRepresentation representation, std::string name,
                     UpdateMode mode, ObservationContract contract);

  /**
   * @brief Processes package content for this subsystem.
   *
   * Arguments:
   * - @p episode: Supplies episode input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void observe(const NavigationEpisode& episode) final;
  /**
   * @brief Performs the rebuild operation for this subsystem.
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
  void rebuild() final;
  /**
   * @brief Performs the snapshot operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `SpatialModelUpdate` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  SpatialModelUpdate snapshot() const final;
  /**
   * @brief Performs the shared snapshot operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `SharedSpatialSnapshot` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  SharedSpatialSnapshot sharedSnapshot() const final;

  /**
   * @brief Performs the representation operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `SpatialRepresentation` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  SpatialRepresentation representation() const noexcept final {
    return update_.representation;
  }
  /**
   * @brief Performs the name operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `std::string_view` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  std::string_view name() const noexcept final { return update_.learner; }
  /**
   * @brief Performs the contract operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const ObservationContract&` containing the operation result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  const ObservationContract& contract() const noexcept final {
    return contract_;
  }

 protected:
  /**
   * @brief Performs the on observe operation for this subsystem.
   *
   * Arguments:
   * - @p episode: Supplies episode input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  virtual void onObserve(const NavigationEpisode& episode) = 0;
  /**
   * @brief Performs the on rebuild operation for this subsystem.
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
  virtual void onRebuild() = 0;

  /**
   * @brief Publishes package content for this subsystem.
   *
   * Arguments:
   * - @p payload: Supplies payload input to the operation.
   * - @p status: Supplies status input to the operation.
   * - @p diagnostic: Supplies diagnostic input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void publish(SpatialPayload payload, ModelStatus status,
               std::string diagnostic = {});
  /**
   * @brief Performs the mark incomplete operation for this subsystem.
   *
   * Arguments:
   * - @p diagnostic: Supplies diagnostic input to the operation.
   *
   * Returns:
   * - No value; effects are applied to owned state or outputs.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
  void markIncomplete(std::string diagnostic);
  /**
   * @brief Performs the episodes operation for this subsystem.
   *
   * Arguments:
   * - None.
   *
   * Returns:
   * - `const std::vector<NavigationEpisode>&` containing the operation
   * result.
   *
   * Exceptions:
   * - None documented; validation or dependency failures may propagate.
   */
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
