#ifndef SEMAFORR_VALIDATION_ALLOCATION_PROBE_HPP
#define SEMAFORR_VALIDATION_ALLOCATION_PROBE_HPP

#include <cstddef>

namespace semaforr::validation {

struct AllocationSnapshot {
  std::size_t count = 0U;
  std::size_t bytes = 0U;
};

AllocationSnapshot allocationSnapshot() noexcept;
AllocationSnapshot allocationDifference(AllocationSnapshot before,
                                        AllocationSnapshot after) noexcept;

namespace detail {
void recordAllocation(std::size_t bytes) noexcept;
}

}  // namespace semaforr::validation

#endif  // SEMAFORR_VALIDATION_ALLOCATION_PROBE_HPP
