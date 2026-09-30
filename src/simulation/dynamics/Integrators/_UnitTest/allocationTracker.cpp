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

#include "allocationTracker.h"

#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <new>

#if defined(__APPLE__)
#include <malloc/malloc.h>
#elif defined(_WIN32)
#include <malloc.h>
#endif

namespace {
constexpr std::size_t apiCount = static_cast<std::size_t>(integrator_allocation_test::AllocationApi::Count);

std::array<std::atomic<std::uint64_t>, apiCount> callCounts{};

thread_local std::uint32_t trackingDepth = 0;
thread_local std::uint32_t suspensionDepth = 0;
thread_local std::uint32_t cppAllocationDepth = 0;

std::size_t
apiIndex(integrator_allocation_test::AllocationApi api) noexcept
{
    return static_cast<std::size_t>(api);
}

class ScopedCppAllocation
{
  public:
    ScopedCppAllocation() noexcept { integrator_allocation_test::detail::enterCppAllocation(); }

    ~ScopedCppAllocation() { integrator_allocation_test::detail::leaveCppAllocation(); }
};

void*
allocateWithAlignment(std::size_t size, std::size_t alignment)
{
    void* memory = nullptr;
#if defined(_WIN32)
    memory = _aligned_malloc(size, alignment);
#elif defined(__APPLE__)
    memory = malloc_zone_memalign(malloc_default_zone(), alignment, size);
#else
    if (::posix_memalign(&memory, alignment, size) != 0) {
        memory = nullptr;
    }
#endif
    return memory;
}

void
freeAligned(void* memory) noexcept
{
#if defined(_WIN32)
    _aligned_free(memory);
#elif defined(__APPLE__)
    if (memory != nullptr) {
        malloc_zone_t* zone = malloc_zone_from_ptr(memory);
        malloc_zone_free(zone == nullptr ? malloc_default_zone() : zone, memory);
    }
#else
    std::free(memory);
#endif
}
}

namespace integrator_allocation_test {

std::uint64_t
AllocationSnapshot::allocationCalls() const noexcept
{
    return (*this)[AllocationApi::CppNew] + (*this)[AllocationApi::CppAlignedNew] + (*this)[AllocationApi::Malloc] +
           (*this)[AllocationApi::Calloc] + (*this)[AllocationApi::Realloc] + (*this)[AllocationApi::AlignedAlloc] +
           (*this)[AllocationApi::PosixMemalign] + (*this)[AllocationApi::Memalign];
}

std::uint64_t
AllocationSnapshot::deallocationCalls() const noexcept
{
    return (*this)[AllocationApi::CppDelete] + (*this)[AllocationApi::CppAlignedDelete] + (*this)[AllocationApi::Free];
}

std::uint64_t
AllocationSnapshot::totalCalls() const noexcept
{
    return this->allocationCalls() + this->deallocationCalls();
}

void
resetAllocationCounts() noexcept
{
    for (auto& count : callCounts) {
        count.store(0, std::memory_order_relaxed);
    }
}

AllocationSnapshot
allocationSnapshot() noexcept
{
    AllocationSnapshot snapshot;
    for (std::size_t index = 0; index < apiCount; ++index) {
        snapshot.calls[index] = callCounts[index].load(std::memory_order_relaxed);
    }
    return snapshot;
}

bool
cAllocatorInterceptionAvailable() noexcept
{
#if defined(__APPLE__) || defined(BASILISK_TEST_LINKER_WRAP_ALLOCATORS)
    return true;
#else
    return false;
#endif
}

bool
alignedAllocatorInterceptionAvailable() noexcept
{
    return cAllocatorInterceptionAvailable();
}

const char*
allocatorInterceptionDescription() noexcept
{
#if defined(__APPLE__)
    return "macOS dyld interposition with malloc-zone forwarding";
#elif defined(BASILISK_TEST_LINKER_WRAP_ALLOCATORS)
    return "ELF linker --wrap interception";
#elif defined(_WIN32)
    return "partial coverage: C++ operators only; MSVC CRT allocation interposition unavailable";
#else
    return "partial coverage: C++ operators only; no supported C allocator interposition";
#endif
}

ScopedAllocationTracking::ScopedAllocationTracking() noexcept
{
    ++trackingDepth;
}

ScopedAllocationTracking::~ScopedAllocationTracking()
{
    --trackingDepth;
}

ScopedAllocationSuspension::ScopedAllocationSuspension() noexcept
{
    ++suspensionDepth;
}

ScopedAllocationSuspension::~ScopedAllocationSuspension()
{
    --suspensionDepth;
}

namespace detail {
void
recordAllocationCall(AllocationApi api) noexcept
{
    if (allocationTrackingActive()) {
        callCounts[apiIndex(api)].fetch_add(1, std::memory_order_relaxed);
    }
}

bool
allocationTrackingActive() noexcept
{
    return trackingDepth != 0 && suspensionDepth == 0 && cppAllocationDepth == 0;
}

void
enterCppAllocation() noexcept
{
    ++cppAllocationDepth;
}

void
leaveCppAllocation() noexcept
{
    --cppAllocationDepth;
}
}

} // namespace integrator_allocation_test

void*
operator new(std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::CppNew);
    ScopedCppAllocation guard;
    if (void* memory = std::malloc(size == 0 ? 1 : size)) {
        return memory;
    }
    throw std::bad_alloc();
}

void*
operator new[](std::size_t size)
{
    return ::operator new(size);
}

void
operator delete(void* memory) noexcept
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::CppDelete);
    ScopedCppAllocation guard;
    std::free(memory);
}

void
operator delete[](void* memory) noexcept
{
    ::operator delete(memory);
}

void
operator delete(void* memory, std::size_t) noexcept
{
    ::operator delete(memory);
}

void
operator delete[](void* memory, std::size_t) noexcept
{
    ::operator delete(memory);
}

void*
operator new(std::size_t size, const std::nothrow_t&) noexcept
{
    try {
        return ::operator new(size);
    } catch (...) {
        return nullptr;
    }
}

void*
operator new[](std::size_t size, const std::nothrow_t& tag) noexcept
{
    return ::operator new(size, tag);
}

void
operator delete(void* memory, const std::nothrow_t&) noexcept
{
    ::operator delete(memory);
}

void
operator delete[](void* memory, const std::nothrow_t&) noexcept
{
    ::operator delete(memory);
}

#if defined(__cpp_aligned_new)
void*
operator new(std::size_t size, std::align_val_t alignment)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::CppAlignedNew);
    ScopedCppAllocation guard;
    if (void* memory = allocateWithAlignment(size == 0 ? 1 : size, static_cast<std::size_t>(alignment))) {
        return memory;
    }
    throw std::bad_alloc();
}

void*
operator new[](std::size_t size, std::align_val_t alignment)
{
    return ::operator new(size, alignment);
}

void
operator delete(void* memory, std::align_val_t) noexcept
{
    integrator_allocation_test::detail::recordAllocationCall(
      integrator_allocation_test::AllocationApi::CppAlignedDelete);
    ScopedCppAllocation guard;
    freeAligned(memory);
}

void
operator delete[](void* memory, std::align_val_t alignment) noexcept
{
    ::operator delete(memory, alignment);
}

void
operator delete(void* memory, std::size_t, std::align_val_t alignment) noexcept
{
    ::operator delete(memory, alignment);
}

void
operator delete[](void* memory, std::size_t, std::align_val_t alignment) noexcept
{
    ::operator delete(memory, alignment);
}

void*
operator new(std::size_t size, std::align_val_t alignment, const std::nothrow_t&) noexcept
{
    try {
        return ::operator new(size, alignment);
    } catch (...) {
        return nullptr;
    }
}

void*
operator new[](std::size_t size, std::align_val_t alignment, const std::nothrow_t& tag) noexcept
{
    return ::operator new(size, alignment, tag);
}

void
operator delete(void* memory, std::align_val_t alignment, const std::nothrow_t&) noexcept
{
    ::operator delete(memory, alignment);
}

void
operator delete[](void* memory, std::align_val_t alignment, const std::nothrow_t&) noexcept
{
    ::operator delete(memory, alignment);
}
#endif

#if defined(__APPLE__)
extern "C" void*
basilisk_test_malloc(std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::Malloc);
    return malloc_zone_malloc(malloc_default_zone(), size);
}

extern "C" void*
basilisk_test_calloc(std::size_t count, std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::Calloc);
    return malloc_zone_calloc(malloc_default_zone(), count, size);
}

extern "C" void*
basilisk_test_realloc(void* memory, std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::Realloc);
    malloc_zone_t* zone = memory == nullptr ? malloc_default_zone() : malloc_zone_from_ptr(memory);
    return malloc_zone_realloc(zone == nullptr ? malloc_default_zone() : zone, memory, size);
}

extern "C" void
basilisk_test_free(void* memory)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::Free);
    if (memory != nullptr) {
        malloc_zone_t* zone = malloc_zone_from_ptr(memory);
        malloc_zone_free(zone == nullptr ? malloc_default_zone() : zone, memory);
    }
}

extern "C" void*
basilisk_test_aligned_alloc(std::size_t alignment, std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::AlignedAlloc);
    if (alignment == 0 || size % alignment != 0) {
        errno = EINVAL;
        return nullptr;
    }
    return malloc_zone_memalign(malloc_default_zone(), alignment, size);
}

extern "C" int
basilisk_test_posix_memalign(void** result, std::size_t alignment, std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::PosixMemalign);
    if (result == nullptr || alignment < sizeof(void*) || (alignment & (alignment - 1)) != 0) {
        return EINVAL;
    }
    *result = malloc_zone_memalign(malloc_default_zone(), alignment, size);
    return *result == nullptr ? ENOMEM : 0;
}

#define BASILISK_DYLD_INTERPOSE(replacement, replacee)                                                                 \
    __attribute__((used)) static struct                                                                                \
    {                                                                                                                  \
        const void* replacement;                                                                                       \
        const void* replacee;                                                                                          \
    } basilisk_interpose_##replacee                                                                                    \
      __attribute__((section("__DATA,__interpose"))) = { reinterpret_cast<const void*>(replacement),                   \
                                                         reinterpret_cast<const void*>(replacee) }

BASILISK_DYLD_INTERPOSE(basilisk_test_malloc, malloc);
BASILISK_DYLD_INTERPOSE(basilisk_test_calloc, calloc);
BASILISK_DYLD_INTERPOSE(basilisk_test_realloc, realloc);
BASILISK_DYLD_INTERPOSE(basilisk_test_free, free);
BASILISK_DYLD_INTERPOSE(basilisk_test_aligned_alloc, aligned_alloc);
BASILISK_DYLD_INTERPOSE(basilisk_test_posix_memalign, posix_memalign);

#elif defined(BASILISK_TEST_LINKER_WRAP_ALLOCATORS)
extern "C"
{
    void* __real_malloc(std::size_t);
    void* __real_calloc(std::size_t, std::size_t);
    void* __real_realloc(void*, std::size_t);
    void __real_free(void*);
    void* __real_aligned_alloc(std::size_t, std::size_t);
    int __real_posix_memalign(void**, std::size_t, std::size_t);
    void* __real_memalign(std::size_t, std::size_t);
}

extern "C" void*
__wrap_malloc(std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::Malloc);
    return __real_malloc(size);
}

extern "C" void*
__wrap_calloc(std::size_t count, std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::Calloc);
    return __real_calloc(count, size);
}

extern "C" void*
__wrap_realloc(void* memory, std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::Realloc);
    return __real_realloc(memory, size);
}

extern "C" void
__wrap_free(void* memory)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::Free);
    __real_free(memory);
}

extern "C" void*
__wrap_aligned_alloc(std::size_t alignment, std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::AlignedAlloc);
    return __real_aligned_alloc(alignment, size);
}

extern "C" int
__wrap_posix_memalign(void** result, std::size_t alignment, std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::PosixMemalign);
    return __real_posix_memalign(result, alignment, size);
}

extern "C" void*
__wrap_memalign(std::size_t alignment, std::size_t size)
{
    integrator_allocation_test::detail::recordAllocationCall(integrator_allocation_test::AllocationApi::Memalign);
    return __real_memalign(alignment, size);
}
#endif
