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

#ifndef _BlankStorage_HH_
#define _BlankStorage_HH_

#include <vector>
#include <stdint.h>
#include "utilities/bskLog.h"

#ifdef _WIN32
class __declspec(dllexport) BlankStorage
#else
class BlankStorage
#endif
{
public:
    BlankStorage();  //! -- The memory space for a process message buffer and an integer with the size
    ~BlankStorage();  //! -- destruction
    BlankStorage(const BlankStorage &mainCopy);  //! -- Initialize with some already written memory
    void IncreaseStorage(uint64_t NewVolume);  //! -- Copy to new memory of size NewVolume bytes
    void ClearStorage();  //! -- null out the BlankStorage
    uint64_t GetCurrentSize() const {return(this->BufferStorageSize);}  //! -- size in bytes of the StorageBuffer

public:
    uint8_t* StorageBuffer;  //! -- The memory where a process buffer writes messages
    BSKLogger bskLogger;                      //!< -- BSK Logging
private:
    uint64_t BufferStorageSize;  //! -- size of StorageBuffer in bytes
};

#endif /* _BlankStorage_H_ */
