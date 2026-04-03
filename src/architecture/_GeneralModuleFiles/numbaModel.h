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

#pragma once
#include <cstdint>
#include <vector>
#include "sys_model.h"
#include "architecture/messaging/msgHeader.h"

/*! @brief Bookkeeping for one registered ReadFunctor.
 *
 * All three addresses remain valid for the lifetime of the corresponding
 * Python attribute because ``subscribeTo`` updates the functor in place.
 */
struct ReaderSlot {
    void**   payloadPtrAddr;  //!< Address of ``ReadFunctor::payloadPointer``.
    uint8_t* linkedAddr;      //!< Address of ``ReadFunctor::initialized``.
    void*    dummy;           //!< Zero-filled fallback payload used when not linked.
};

/*! @brief Base class for Python-defined Numba modules.
 *
 * ``NumbaModel`` lets Python subclasses register message readers, message
 * writers, and auxiliary buffers, then exposes them to a compiled Numba
 * callback through a flat pointer array.  The Python layer owns the
 * ``UpdateStateImpl`` implementation, while this C++ class refreshes the
 * read pointers, updates message headers, and invokes the compiled callback
 * every simulation tick.
 */
class NumbaModel : virtual public SysModel
{
public:
    /*! Register a reader functor for dynamic payload refresh.
     *
     * @param payloadPtrAddr Address of the functor payload pointer storage.
     * @param linkedAddr Address of the functor linked-flag storage.
     * @param dummyAddr Address of the fallback payload used when unlinked.
     */
    void addReaderSlot(uintptr_t payloadPtrAddr, uintptr_t linkedAddr, uintptr_t dummyAddr);

    /*! Register one writable payload pointer in the flattened pointer table.
     *
     * @param ptr Raw payload pointer address.
     */
    void addWritePayloadPtr(uintptr_t ptr);

    /*! Register one writable message-header pointer.
     *
     * @param ptr Raw message-header pointer address.
     */
    void addWriteHeaderPtr(uintptr_t ptr);

    /*! Register one user-owned pointer in the flattened pointer table.
     *
     * @param ptr Raw user pointer address.
     */
    void addUserPointer(uintptr_t ptr);

    /*! Store the compiled Numba update function pointer.
     *
     * @param funcPtr Raw function pointer returned by Numba.
     */
    void setStateUpdateFunc(uintptr_t funcPtr);

    /*! Return the raw address of the ``readLinked_`` backing buffer.
     *
     * This buffer is typically registered once as a user pointer so the
     * compiled callback can inspect reader linkage state.
     */
    uintptr_t getReadLinkedPtr() const;

    /*! Assemble the flattened pointer table consumed by the compiled callback.
     *
     * This must be called from Python ``Reset()`` after ``_nbm_compile()``
     * finishes registering all reader, writer, and user pointers.
     */
    void finalizeAllPtrs();

    /*! Prepare the model for a new simulation run.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void Reset(uint64_t CurrentSimNanos) override;

    /*! Refresh registered pointers and invoke the compiled callback.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void UpdateState(uint64_t CurrentSimNanos) override;

public:
    BSKLogger bskLogger;  //!< Logger proxy exposed to Python-side Numba modules.

protected:
    std::vector<ReaderSlot> readerSlots_;    //!< One entry per registered reader functor.
    std::vector<uint8_t>    readLinked_;     //!< Per-reader linkage flags refreshed each tick.
    size_t                  readCount_ = 0;  //!< Number of reader entries stored in ``allPtrs_``.
    std::vector<void*>      writePayloads_;  //!< Staged writable payload pointers awaiting finalization.
    std::vector<void*>      userPointers_;   //!< Staged user pointers appended after readers and writers.
    std::vector<void*>      allPtrs_;        //!< Flat pointer table laid out as ``[reads | writes | user]``.
    std::vector<MsgHeader*> writeHeaders_;   //!< Headers corresponding to registered writable payloads.
    void (*stateUpdateFunc_)(void**, uint64_t) = nullptr;  //!< Compiled Numba state update callback.
};
