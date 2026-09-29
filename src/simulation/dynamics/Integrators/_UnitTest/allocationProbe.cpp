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

#include "allocationProbe.h"

#include <cstdlib>
#include <new>

extern "C" void*
basilisk_allocation_probe_new(std::size_t size)
{
    return ::operator new(size);
}

extern "C" void
basilisk_allocation_probe_delete(void* memory)
{
    ::operator delete(memory);
}

extern "C" void*
basilisk_allocation_probe_aligned_new(std::size_t size, std::size_t alignment)
{
#if defined(__cpp_aligned_new)
    return ::operator new(size, static_cast<std::align_val_t>(alignment));
#else
    (void)size;
    (void)alignment;
    return nullptr;
#endif
}

extern "C" void
basilisk_allocation_probe_aligned_delete(void* memory, std::size_t alignment)
{
#if defined(__cpp_aligned_new)
    ::operator delete(memory, static_cast<std::align_val_t>(alignment));
#else
    (void)memory;
    (void)alignment;
#endif
}

extern "C" void*
basilisk_allocation_probe_malloc(std::size_t size)
{
    return std::malloc(size);
}

extern "C" void*
basilisk_allocation_probe_calloc(std::size_t count, std::size_t size)
{
    return std::calloc(count, size);
}

extern "C" void*
basilisk_allocation_probe_realloc(void* memory, std::size_t size)
{
    return std::realloc(memory, size);
}

extern "C" void
basilisk_allocation_probe_free(void* memory)
{
    std::free(memory);
}

extern "C" void*
basilisk_allocation_probe_aligned_alloc(std::size_t alignment, std::size_t size)
{
#if defined(_WIN32)
    (void)alignment;
    (void)size;
    return nullptr;
#else
    return std::aligned_alloc(alignment, size);
#endif
}

extern "C" int
basilisk_allocation_probe_posix_memalign(void** result, std::size_t alignment, std::size_t size)
{
#if defined(_WIN32)
    (void)result;
    (void)alignment;
    (void)size;
    return -1;
#else
    return ::posix_memalign(result, alignment, size);
#endif
}
