/**
 * @file allocation_probe.hpp
 * @brief Allocation probe responsibilities.
 *
 * @details This file defines allocation probe behavior for replay, experimental
 * validation, and performance measurement. It centers on
 * `AllocationSnapshot`. Its package-relative location is
 * `include/semaforr/validation/allocation_probe.hpp`.
 */
#ifndef SEMAFORR_VALIDATION_ALLOCATION_PROBE_HPP
#define SEMAFORR_VALIDATION_ALLOCATION_PROBE_HPP

#include <cstddef>

namespace semaforr::validation {

/**
 * @brief Encapsulates allocation snapshot state and behavior for this
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
struct AllocationSnapshot {
  std::size_t count = 0U;
  std::size_t bytes = 0U;
};

/**
 * @brief Performs the allocation snapshot operation for this subsystem.
 *
 * Arguments:
 * - None.
 *
 * Returns:
 * - `AllocationSnapshot` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
AllocationSnapshot allocationSnapshot() noexcept;
/**
 * @brief Performs the allocation difference operation for this subsystem.
 *
 * Arguments:
 * - @p before: Supplies before input to the operation.
 * - @p after: Supplies after input to the operation.
 *
 * Returns:
 * - `AllocationSnapshot` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
AllocationSnapshot allocationDifference(AllocationSnapshot before,
                                        AllocationSnapshot after) noexcept;

namespace detail {
/**
 * @brief Records allocation for this subsystem.
 *
 * Arguments:
 * - @p bytes: Supplies bytes input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void recordAllocation(std::size_t bytes) noexcept;
}  // namespace detail

}  // namespace semaforr::validation

#endif  // SEMAFORR_VALIDATION_ALLOCATION_PROBE_HPP
