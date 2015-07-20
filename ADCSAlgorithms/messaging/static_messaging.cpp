
#include "messaging/static_messaging.h"
#include "SimCode/architecture/messaging/system_messaging.h"
#include <stdio.h>

void InitializeStorage(uint32_t StorageBytes)
{
   return;
}

int32_t CreateNewMessage(char* MessageName, uint32_t MaxSize)
{
   return(SystemMessaging::GetInstance()->CreateNewMessage(
            MessageName, MaxSize, 2));
}

int32_t WriteMessage(uint32_t MessageID, uint32_t ClockTimeNanos, uint32_t MsgSize,
      void *MsgPayload)
{
   return(SystemMessaging::GetInstance()->WriteMessage(MessageID, ClockTimeNanos,
      MsgSize,  reinterpret_cast<uint8_t*> (MsgPayload)));
}

int32_t ReadMessage(uint32_t MessageID, uint64_t *WriteTime, uint32_t *WriteSize,
      uint32_t MaxBytes, void *MsgPayload)
{
   SingleMessageHeader LocalHeader;
  
   bool TestResult = SystemMessaging::GetInstance()->ReadMessage(MessageID, 
      &LocalHeader, MaxBytes, reinterpret_cast<uint8_t*> (MsgPayload));
   *WriteTime = LocalHeader.WriteClockNanos;
   *WriteSize = LocalHeader.WriteSize;
   return(TestResult);
}

int32_t FindMessageID(char *MessageName)
{
   return(SystemMessaging::GetInstance()->FindMessageID(
          MessageName));
}

const char * FindMessageName(uint32_t MessageID)
{
   return(SystemMessaging::GetInstance()->FindMessageName(MessageID).c_str());
}
