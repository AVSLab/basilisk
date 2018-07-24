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

#ifndef _STATIC_MESSAGING_H_
#define _STATIC_MESSAGING_H_

#define MAX_STAT_MSG_LENGTH 128

#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */
#ifdef __cplusplus
extern "C" {
#endif
    
    void InitializeStorage(uint32_t StorageBytes);
    int32_t CreateNewMessage(char* MessageName, uint32_t MaxSize, char * MessageStruct,
        uint64_t moduleID);
    int32_t WriteMessage(uint32_t MessageID, uint64_t ClockTimeNanos, uint32_t MsgSize,
                         void *MsgPayload, uint64_t moduleID);
    int32_t ReadMessage(uint32_t MessageID, uint64_t *WriteTime, uint32_t *WriteSize,
                        uint32_t MaxBytes, void *MsgPayload, int64_t moduleID);
    int32_t subscribeToMessage(char *MessageName, uint64_t messageSize,
        int64_t moduleID);
    const char * FindMessageName(uint32_t MessageID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
