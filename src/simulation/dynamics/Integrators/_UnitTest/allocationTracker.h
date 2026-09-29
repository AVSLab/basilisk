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

namespace integrator_test {

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

struct AllocationSnapshot
{
    std::array<std::uint64_t, static_cast<std::size_t>(AllocationApi::Count)> calls{};

    std::uint64_t operator[](AllocationApi api) const noexcept { return this->calls[static_cast<std::size_t>(api)]; }

    std::uint64_t allocationCalls() const noexcept;
    std::uint64_t deallocationCalls() const noexcept;
    std::uint64_t totalCalls() const noexcept;
};

void
resetAllocationCounts() noexcept;
AllocationSnapshot
allocationSnapshot() noexcept;
bool
cAllocatorInterceptionAvailable() noexcept;
bool
alignedAllocatorInterceptionAvailable() noexcept;
const char*
allocatorInterceptionDescription() noexcept;

class ScopedAllocationTracking
{
  public:
    ScopedAllocationTracking() noexcept;
    ~ScopedAllocationTracking();

    ScopedAllocationTracking(const ScopedAllocationTracking&) = delete;
    ScopedAllocationTracking& operator=(const ScopedAllocationTracking&) = delete;
};

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

class ScopedAllocationSuspension
{
  public:
    ScopedAllocationSuspension() noexcept;
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

} // namespace integrator_test

#endif
