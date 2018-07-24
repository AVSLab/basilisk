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

BlankStorage :: BlankStorage()
{
    StorageBuffer = NULL;
    BufferStorageSize = 0;
    
}

BlankStorage::~BlankStorage()
{
    if(StorageBuffer != NULL)
    {
        delete [] StorageBuffer;
        StorageBuffer = NULL;
    }
    BufferStorageSize = 0;
}


BlankStorage :: BlankStorage(const BlankStorage &mainCopy)
{
    BufferStorageSize = 0;
    StorageBuffer= NULL;
    IncreaseStorage(mainCopy.GetCurrentSize());
    if(BufferStorageSize > 0)
    {
        memcpy(StorageBuffer, mainCopy.StorageBuffer, BufferStorageSize);
    }
}

void BlankStorage::ClearStorage()
{
    if(StorageBuffer != NULL)
    {
        delete [] StorageBuffer;
    }
    return;
}

void BlankStorage::IncreaseStorage(uint64_t NewVolume)
{
    if(NewVolume <= BufferStorageSize)
    {
        return;
    }
    
    uint8_t *NewBuffer = new uint8_t[NewVolume];
    uint8_t *OldBuffer = StorageBuffer;
    memset(NewBuffer, '\0', NewVolume);
    if(BufferStorageSize > 0 && StorageBuffer != NULL)
    {
        memcpy(NewBuffer, StorageBuffer, BufferStorageSize);
    }
    BufferStorageSize = NewVolume;
    StorageBuffer = NewBuffer;
    if(OldBuffer != NULL)
    {
        delete [] OldBuffer;
    }
    
}

