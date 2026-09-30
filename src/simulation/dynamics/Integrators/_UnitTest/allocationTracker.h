/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef INTEGRATOR_TEST_ALLOCATION_TRACKER_H
#define INTEGRATOR_TEST_ALLOCATION_TRACKER_H

#include <array>
#include <cstddef>
#include <cstdint>

namespace integrator_allocation_test {

/** @brief Allocation and deallocation APIs distinguished by the test counters. */
enum class AllocationApi : std::size_t
{
    CppNew,
    CppDelete,
    CppAlignedNew,
    CppAlignedDelete,
    Malloc,
    Calloc,
    Realloc,
    Free,
    AlignedAlloc,
    PosixMemalign,
    Memalign,
    Count
};

/** @brief Copy of the intercepted allocation and deallocation API call counts. */
struct AllocationSnapshot
{
    std::array<std::uint64_t, static_cast<std::size_t>(AllocationApi::Count)> calls{}; //!< Call counts by API.

    /**
     * @brief Read the count for one intercepted API.
     * @param api Allocation or deallocation API to inspect; AllocationApi::Count is not a valid index.
     * @return Number of recorded calls to the selected API.
     */
    std::uint64_t operator[](AllocationApi api) const noexcept { return this->calls[static_cast<std::size_t>(api)]; }

    /**
     * @brief Sum allocation API calls, including realloc calls.
     * @return Total count of allocation API calls in this snapshot.
     */
    std::uint64_t allocationCalls() const noexcept;
    /**
     * @brief Sum delete and free API calls.
     * @return Total count of deallocation API calls in this snapshot.
     */
    std::uint64_t deallocationCalls() const noexcept;
    /**
     * @brief Sum allocation and deallocation API calls.
     * @return Total count of all intercepted API calls in this snapshot.
     */
    std::uint64_t totalCalls() const noexcept;
};

/** @brief Clear all recorded allocation and deallocation counts. */
void
resetAllocationCounts() noexcept;
/**
 * @brief Copy the current allocation and deallocation counts.
 * @return Snapshot of the recorded API call counts.
 */
AllocationSnapshot
allocationSnapshot() noexcept;
/**
 * @brief Report whether C allocation calls can be intercepted on this build.
 * @return True when the C allocator interception backend is enabled.
 */
bool
cAllocatorInterceptionAvailable() noexcept;
/**
 * @brief Report whether aligned C allocation calls can be intercepted on this build.
 * @return True when aligned C allocator interception is enabled.
 */
bool
alignedAllocatorInterceptionAvailable() noexcept;
/**
 * @brief Describe the allocator interception backend and any coverage limitation.
 * @return Pointer to a string literal describing the active backend.
 */
const char*
allocatorInterceptionDescription() noexcept;

/** @brief Nestable scope that enables allocation tracking on the current thread. */
class ScopedAllocationTracking
{
  public:
    /** @brief Enter an allocation-tracking scope without clearing existing counts. */
    ScopedAllocationTracking() noexcept;
    /** @brief Leave this scope, preserving any enclosing tracking scope. */
    ~ScopedAllocationTracking();

    ScopedAllocationTracking(const ScopedAllocationTracking&) = delete;
    ScopedAllocationTracking& operator=(const ScopedAllocationTracking&) = delete;
};

/**
 * @brief Reset the counters and track allocation calls made while invoking a callable.
 * @tparam Callable Type of the callable to invoke without arguments.
 * @param callable Operation to observe on the current thread.
 * @return Snapshot of allocation and deallocation counts after the callable returns.
 */
template<typename Callable>
AllocationSnapshot
trackAllocations(Callable&& callable)
{
    resetAllocationCounts();
    {
        ScopedAllocationTracking tracking;
        callable();
    }
    return allocationSnapshot();
}

/** @brief Nestable scope that suspends allocation tracking on the current thread. */
class ScopedAllocationSuspension
{
  public:
    /** @brief Suspend tracking without changing the recorded counts. */
    ScopedAllocationSuspension() noexcept;
    /** @brief Restore the enclosing tracking and suspension state. */
    ~ScopedAllocationSuspension();

    ScopedAllocationSuspension(const ScopedAllocationSuspension&) = delete;
    ScopedAllocationSuspension& operator=(const ScopedAllocationSuspension&) = delete;
};

namespace detail {
void
recordAllocationCall(AllocationApi api) noexcept;
bool
allocationTrackingActive() noexcept;
void
enterCppAllocation() noexcept;
void
leaveCppAllocation() noexcept;
}

} // namespace integrator_allocation_test

#endif
