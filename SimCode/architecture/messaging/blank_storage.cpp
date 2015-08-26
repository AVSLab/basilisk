
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

