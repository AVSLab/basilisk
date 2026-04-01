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

#include "numbaModel.h"

void NumbaModel::addReaderSlot(uintptr_t payloadPtrAddr, uintptr_t linkedAddr, uintptr_t dummyAddr) {
    ReaderSlot slot;
    slot.payloadPtrAddr = reinterpret_cast<void**>(payloadPtrAddr);
    slot.linkedAddr     = reinterpret_cast<uint8_t*>(linkedAddr);
    slot.dummy          = reinterpret_cast<void*>(dummyAddr);
    readerSlots_.push_back(slot);
    readLinked_.push_back(0);
}

void NumbaModel::addWritePayloadPtr(uintptr_t ptr) {
    writePayloads_.push_back(reinterpret_cast<void*>(ptr));
}

void NumbaModel::addWriteHeaderPtr(uintptr_t ptr) {
    writeHeaders_.push_back(reinterpret_cast<MsgHeader*>(ptr));
}

void NumbaModel::addUserPointer(uintptr_t ptr) {
    userPointers_.push_back(reinterpret_cast<void*>(ptr));
}

uintptr_t NumbaModel::getReadLinkedPtr() const {
    return reinterpret_cast<uintptr_t>(readLinked_.data());
}

void NumbaModel::setStateUpdateFunc(uintptr_t funcPtr) {
    stateUpdateFunc_ = reinterpret_cast<void(*)(void**, uint64_t)>(funcPtr);
}

void NumbaModel::finalizeAllPtrs() {
    readCount_ = readerSlots_.size();
    allPtrs_.clear();
    // Read section: initialise with dummy pointers; refreshed per tick.
    for (auto& slot : readerSlots_)
        allPtrs_.push_back(slot.dummy);
    // Write section: stable payload pointers set at Reset time.
    for (auto* p : writePayloads_)
        allPtrs_.push_back(p);
    // User section: stable user-data pointers (memory, moduleID, rng, …).
    for (auto* p : userPointers_)
        allPtrs_.push_back(p);
    // Release staging vectors - no longer needed after assembly.
    writePayloads_.clear();
    writePayloads_.shrink_to_fit();
    userPointers_.clear();
    userPointers_.shrink_to_fit();
}

void NumbaModel::Reset(uint64_t /*CurrentSimNanos*/) {
    readerSlots_.clear();
    readLinked_.clear();
    allPtrs_.clear();
    writePayloads_.clear();
    writeHeaders_.clear();
    userPointers_.clear();
    readCount_ = 0;
    stateUpdateFunc_ = nullptr;
}

void NumbaModel::UpdateState(uint64_t CurrentSimNanos) {
    if (!stateUpdateFunc_) return;

    // Refresh the read section of allPtrs_ and the linked flags array.
    // This makes re-wiring (subscribeTo after Reset) transparent to the cfunc.
    for (size_t i = 0; i < readCount_; i++) {
        uint8_t linked = *readerSlots_[i].linkedAddr;
        readLinked_[i] = linked;
        allPtrs_[i] = linked
            ? *readerSlots_[i].payloadPtrAddr
            : readerSlots_[i].dummy;
    }

    stateUpdateFunc_(allPtrs_.data(), CurrentSimNanos);

    for (MsgHeader* h : writeHeaders_) {
        h->isWritten   = 1;
        h->timeWritten = CurrentSimNanos;
        h->moduleID    = this->moduleID;
    }
}
