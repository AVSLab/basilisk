
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
                        uint32_t MaxBytes, void *MsgPayload);
    int32_t subscribeToMessage(char *MessageName, uint64_t messageSize,
        int64_t moduleID);
    const char * FindMessageName(uint32_t MessageID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
