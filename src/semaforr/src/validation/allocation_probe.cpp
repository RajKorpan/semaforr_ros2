/**
 * @file allocation_probe.cpp
 * @brief Allocation probe responsibilities.
 *
 * @details This file implements allocation probe behavior for replay,
 * experimental validation, and performance measurement. It records the
 * declarations, settings, fixtures, or guidance needed by that responsibility.
 * Its package-relative location is `src/validation/allocation_probe.cpp`.
 */
#include <atomic>
#include <cstdlib>
#include <new>
#include <semaforr/validation/allocation_probe.hpp>

namespace {
std::atomic<std::size_t> allocation_count{0U};
std::atomic<std::size_t> allocation_bytes{0U};
}  // namespace

namespace semaforr::validation {

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
AllocationSnapshot allocationSnapshot() noexcept {
  return {allocation_count.load(std::memory_order_relaxed),
          allocation_bytes.load(std::memory_order_relaxed)};
}

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
                                        AllocationSnapshot after) noexcept {
  return {after.count >= before.count ? after.count - before.count : 0U,
          after.bytes >= before.bytes ? after.bytes - before.bytes : 0U};
}

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
void recordAllocation(std::size_t bytes) noexcept {
  allocation_count.fetch_add(1U, std::memory_order_relaxed);
  allocation_bytes.fetch_add(bytes, std::memory_order_relaxed);
}
}  // namespace detail

}  // namespace semaforr::validation

/**
 * @brief Performs the operator new operation for this subsystem.
 *
 * Arguments:
 * - @p size: Supplies size input to the operation.
 *
 * Returns:
 * - `void*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void* operator new(std::size_t size) {
  if (void* memory = std::malloc(size)) {
    semaforr::validation::detail::recordAllocation(size);
    return memory;
  }
  throw std::bad_alloc();
}

/**
 * @brief Performs the operator new operation for this subsystem.
 *
 * Arguments:
 * - @p size: Supplies size input to the operation.
 *
 * Returns:
 * - `void*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void* operator new[](std::size_t size) { return ::operator new(size); }

/**
 * @brief Performs the operator new operation for this subsystem.
 *
 * Arguments:
 * - @p size: Supplies size input to the operation.
 * - @p alignment: Supplies alignment input to the operation.
 *
 * Returns:
 * - `void*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void* operator new(std::size_t size, std::align_val_t alignment) {
  void* memory = nullptr;
  if (posix_memalign(&memory, static_cast<std::size_t>(alignment), size) != 0)
    throw std::bad_alloc();
  semaforr::validation::detail::recordAllocation(size);
  return memory;
}

/**
 * @brief Performs the operator new operation for this subsystem.
 *
 * Arguments:
 * - @p size: Supplies size input to the operation.
 * - @p alignment: Supplies alignment input to the operation.
 *
 * Returns:
 * - `void*` containing the operation result.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void* operator new[](std::size_t size, std::align_val_t alignment) {
  return ::operator new(size, alignment);
}

/**
 * @brief Releases dynamically allocated memory for this subsystem.
 *
 * Arguments:
 * - @p memory: Supplies memory input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void operator delete(void* memory) noexcept { std::free(memory); }
/**
 * @brief Releases dynamically allocated memory for this subsystem.
 *
 * Arguments:
 * - @p memory: Supplies memory input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void operator delete[](void* memory) noexcept { std::free(memory); }
/**
 * @brief Releases dynamically allocated memory for this subsystem.
 *
 * Arguments:
 * - @p memory: Supplies memory input to the operation.
 * - @p size_t: Supplies size t input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
/**
 * @brief Releases dynamically allocated memory for this subsystem.
 *
 * Arguments:
 * - @p memory: Supplies memory input to the operation.
 * - @p size_t: Supplies size t input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void operator delete[](void* memory, std::size_t) noexcept {
  std::free(memory);
}
/**
 * @brief Releases dynamically allocated memory for this subsystem.
 *
 * Arguments:
 * - @p memory: Supplies memory input to the operation.
 * - @p align_val_t: Supplies align val t input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void operator delete(void* memory, std::align_val_t) noexcept {
  std::free(memory);
}
/**
 * @brief Releases dynamically allocated memory for this subsystem.
 *
 * Arguments:
 * - @p memory: Supplies memory input to the operation.
 * - @p align_val_t: Supplies align val t input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void operator delete[](void* memory, std::align_val_t) noexcept {
  std::free(memory);
}
/**
 * @brief Releases dynamically allocated memory for this subsystem.
 *
 * Arguments:
 * - @p memory: Supplies memory input to the operation.
 * - @p size_t: Supplies size t input to the operation.
 * - @p align_val_t: Supplies align val t input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void operator delete(void* memory, std::size_t, std::align_val_t) noexcept {
  std::free(memory);
}
/**
 * @brief Releases dynamically allocated memory for this subsystem.
 *
 * Arguments:
 * - @p memory: Supplies memory input to the operation.
 * - @p size_t: Supplies size t input to the operation.
 * - @p align_val_t: Supplies align val t input to the operation.
 *
 * Returns:
 * - No value; effects are applied to owned state or outputs.
 *
 * Exceptions:
 * - None documented; validation or dependency failures may propagate.
 */
void operator delete[](void* memory, std::size_t, std::align_val_t) noexcept {
  std::free(memory);
}
