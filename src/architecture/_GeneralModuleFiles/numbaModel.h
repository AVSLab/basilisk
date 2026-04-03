/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#pragma once
#include <cstdint>
#include <vector>
#include "sys_model.h"
#include "architecture/messaging/msgHeader.h"

/*! One slot per registered ReadFunctor.  All three addresses are stable for
    the lifetime of the Python attribute (subscribeTo copies values in-place). */
struct ReaderSlot {
    void**   payloadPtrAddr;  //!< address of ReadFunctor::payloadPointer
    uint8_t* linkedAddr;      //!< address of ReadFunctor::initialized
    void*    dummy;           //!< zero-filled fallback used when not linked
};

class NumbaModel : virtual public SysModel
{
public:
    //! Register a reader functor so UpdateState can refresh its payload pointer dynamically
    void addReaderSlot(uintptr_t payloadPtrAddr, uintptr_t linkedAddr, uintptr_t dummyAddr);

    void addWritePayloadPtr(uintptr_t ptr);
    void addWriteHeaderPtr(uintptr_t ptr);
    void addUserPointer(uintptr_t ptr);
    void setStateUpdateFunc(uintptr_t funcPtr);

    //! Return the raw address of the readLinked_ data buffer (registered once as a user pointer)
    uintptr_t getReadLinkedPtr() const;

    /*! Assemble the flat allPtrs_ array from the three staging vectors.
        Must be called from Python Reset() after _nbm_compile() has finished
        registering all reader/writer/user slots. */
    void finalizeAllPtrs();

    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

public:
    BSKLogger* bskLogger;

protected:
    std::vector<ReaderSlot> readerSlots_;    //!< one per registered read functor
    std::vector<uint8_t>    readLinked_;     //!< refreshed every tick; exposed as user pointer
    size_t                  readCount_ = 0;  //!< number of read entries in allPtrs_
    // Staging vectors - populated by add* calls, cleared by finalizeAllPtrs()
    std::vector<void*>      writePayloads_;
    std::vector<void*>      userPointers_;
    // Flat pointer array passed to the cfunc: [reads | writes | user]
    std::vector<void*>      allPtrs_;
    std::vector<MsgHeader*> writeHeaders_;
    void (*stateUpdateFunc_)(void**, uint64_t) = nullptr;
};
