/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "architecture/messaging/blank_storage.h"
#include <cstring>

/*!
 * Constructor for BlankStorage
 */
BlankStorage::BlankStorage()
{
    this->StorageBuffer = NULL;
    this->BufferStorageSize = 0;
}

/*!
 * Destructor for BlankStorage
 */
BlankStorage::~BlankStorage()
{
    if(this->StorageBuffer != NULL)
    {
        delete [] this->StorageBuffer;
        this->StorageBuffer = NULL;
    }
    this->BufferStorageSize = 0;
}

/*!
 * Initialize a BlankStorage with some non-blank storage
 * @param BlankStorage& mainCopy
 */
BlankStorage::BlankStorage(const BlankStorage &mainCopy)
{
    this->BufferStorageSize = 0;
    this->StorageBuffer= NULL;
    this->IncreaseStorage(mainCopy.GetCurrentSize());
    if(this->BufferStorageSize > 0)
    {
        memcpy(this->StorageBuffer, mainCopy.StorageBuffer, this->BufferStorageSize);
    }
}

/*!
 * Kill the BlankStorage()
 * @return void
 */
void BlankStorage::ClearStorage()
{
    if(this->StorageBuffer != NULL)
    {
        delete [] this->StorageBuffer;
    }
    return;
}

/*!
 * Copy StorageBuffer data into a new block of memory of size NewVolume
 * Also delete the old memory block
 * @param uint64_t NewVolume total volume in bytes of StorageBuffer to increase to
 * @return void
 */
void BlankStorage::IncreaseStorage(uint64_t NewVolume)
{
    if(NewVolume <= this->BufferStorageSize)
    {
        return;
    }
    
    uint8_t *NewBuffer = new uint8_t[NewVolume];
    uint8_t *OldBuffer = this->StorageBuffer;
    memset(NewBuffer, '\0', NewVolume);
    if(this->BufferStorageSize > 0 && this->StorageBuffer != NULL)
    {
        memcpy(NewBuffer, this->StorageBuffer, this->BufferStorageSize);
    }
    this->BufferStorageSize = NewVolume;
    this->StorageBuffer = NewBuffer;
    if(OldBuffer != NULL) {
        delete[] OldBuffer;
    }
}

