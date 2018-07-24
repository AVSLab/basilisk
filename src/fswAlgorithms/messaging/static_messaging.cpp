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

#include "messaging/static_messaging.h"
#include "simulation/architecture/messaging/system_messaging.h"
#include <stdio.h>
#include <iostream>
#include <cstring>
#include "utilities/bsk_Print.h"

/*! This algorithm initializes the messaging storage for the flight system
 @return void
 @param StorageBytes The minimum number of bytes that we need for storage
 */
void InitializeStorage(uint32_t StorageBytes)
{
    return;
}

/*! This method creates a new message for use in the system.  It needs to know
 what name to use for it and the message size
 @return Returns an integer indicating failure (0) or success (1)
 @param MessageName The name of the message that we are creating
 @param MaxSize The maximum size that a given message can be
 */
int32_t CreateNewMessage(char* MessageName, uint32_t MaxSize, char* MessageStruct,
    uint64_t moduleID)
{
    return((int32_t)SystemMessaging::GetInstance()->CreateNewMessage(
        MessageName, MaxSize, 2, MessageStruct, moduleID));
}

/*! This method writes a new copy of the given message into the system.
 @return Returns an integer indicating failure (0) or success (1)
 @param MessageID The ID associated with the message being written
 @param ClockTimeNanos The timestamp we want on the message in nanoseconds
 @param MsgSize The size of the message that we are writing
 @param MsgPayload A pointer to the message buffer that we are writing
 */
int32_t WriteMessage(uint32_t MessageID, uint64_t ClockTimeNanos, uint32_t MsgSize,
                     void *MsgPayload, uint64_t moduleID)
{
    return(SystemMessaging::GetInstance()->WriteMessage(MessageID, ClockTimeNanos,
        MsgSize,  reinterpret_cast<uint8_t*> (MsgPayload), moduleID));
}

/*! This method reads the most recent message buffer from the messaging system
 and writes that data into the payload supplied by the user.
 @return Returns an integer indicating failure (0) or success (1)
 @param MessageID The ID associated with the message being written
 @param MaxBytes The maximum size that a given message can be
 @param MsgPayload A pointer to the message buffer that we are writing to
 @param WriteTime The time associated with the write we are reading
 @param WriteSize The number of bytes that get read out
 */
int32_t ReadMessage(uint32_t MessageID, uint64_t *WriteTime, uint32_t *WriteSize,
                    uint32_t MaxBytes, void *MsgPayload, int64_t moduleID)
{
    SingleMessageHeader LocalHeader;
    memset(&LocalHeader, 0x0, sizeof(SingleMessageHeader));
    bool TestResult = SystemMessaging::GetInstance()->ReadMessage(MessageID,
                                                                  &LocalHeader, MaxBytes, reinterpret_cast<uint8_t*> (MsgPayload), moduleID);
    *WriteTime = LocalHeader.WriteClockNanos;
    *WriteSize = (uint32_t)LocalHeader.WriteSize;
    return(TestResult);
}

/*! This method finds the ID associated with a given message name.
 @return Returns an integer indicating failure (0) or success (1)
 @param MessageName The name of the message that we want to find the ID for
 */
int32_t subscribeToMessage(char *MessageName, uint64_t messageSize,
    int64_t moduleID)
{
    int32_t localMsgID = (int32_t)SystemMessaging::GetInstance()->subscribeToMessage(
        MessageName, messageSize, moduleID);
    if(localMsgID < 0)
    {
        BSK_PRINT_BRIEF(MSG_WARNING, "failed to find a message to link for: %s", MessageName);
        
    }
    return(localMsgID);
}

/*! This method find the message name associated with a given ID.
 @return Returns a pointer to the message name we find
 @param MessageID MEssage ID that we want to find the message name for
 */
const char * FindMessageName(uint32_t MessageID)
{
    return(SystemMessaging::GetInstance()->FindMessageName(MessageID).c_str());
}
