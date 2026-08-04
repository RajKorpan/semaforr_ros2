#include <atomic>
#include <cstdlib>
#include <new>
#include <semaforr/validation/allocation_probe.hpp>

namespace {
std::atomic<std::size_t> allocation_count{0U};
std::atomic<std::size_t> allocation_bytes{0U};
}

namespace semaforr::validation {

AllocationSnapshot allocationSnapshot() noexcept {
  return {allocation_count.load(std::memory_order_relaxed),
          allocation_bytes.load(std::memory_order_relaxed)};
}

AllocationSnapshot allocationDifference(AllocationSnapshot before,
                                        AllocationSnapshot after) noexcept {
  return {after.count >= before.count ? after.count - before.count : 0U,
          after.bytes >= before.bytes ? after.bytes - before.bytes : 0U};
}

namespace detail {
void recordAllocation(std::size_t bytes) noexcept {
  allocation_count.fetch_add(1U, std::memory_order_relaxed);
  allocation_bytes.fetch_add(bytes, std::memory_order_relaxed);
}
}  // namespace detail

}  // namespace semaforr::validation

void* operator new(std::size_t size) {
  if (void* memory = std::malloc(size)) {
    semaforr::validation::detail::recordAllocation(size);
    return memory;
  }
  throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
  return ::operator new(size);
}

void* operator new(std::size_t size, std::align_val_t alignment) {
  void* memory = nullptr;
  if (posix_memalign(&memory, static_cast<std::size_t>(alignment), size) != 0)
    throw std::bad_alloc();
  semaforr::validation::detail::recordAllocation(size);
  return memory;
}

void* operator new[](std::size_t size, std::align_val_t alignment) {
  return ::operator new(size, alignment);
}

void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept {
  std::free(memory);
}
void operator delete(void* memory, std::align_val_t) noexcept {
  std::free(memory);
}
void operator delete[](void* memory, std::align_val_t) noexcept {
  std::free(memory);
}
void operator delete(void* memory, std::size_t, std::align_val_t) noexcept {
  std::free(memory);
}
void operator delete[](void* memory, std::size_t,
                       std::align_val_t) noexcept {
  std::free(memory);
}
