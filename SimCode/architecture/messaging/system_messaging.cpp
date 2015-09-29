
#include "architecture/messaging/system_messaging.h"
#include <cstring>
#include <iostream>

SystemMessaging* SystemMessaging::TheInstance = NULL;

SystemMessaging :: SystemMessaging()
{
    MessageStorage = NULL;
    CreateFails = 0;
    WriteFails = 0;
    ReadFails = 0;
}

SystemMessaging::~SystemMessaging()
{
}

SystemMessaging* SystemMessaging::GetInstance()
{
    if(TheInstance == NULL)
    {
        TheInstance = new SystemMessaging();
    }
    return(TheInstance);
}

void SystemMessaging::AttachStorageBucket(BlankStorage *InputStorage)
{
    MessageStorage = InputStorage;
    MessageStorage->IncreaseStorage(sizeof(uint64_t));
    return;
}

void SystemMessaging::SetNumMessages(uint64_t MessageCount)
{
    if(MessageStorage == NULL)
    {
        std::cerr << "I received a request to set num messages for a NULL buffer";
        std::cerr << std::endl;
        return;
    }
    memcpy(&(MessageStorage->StorageBuffer[0]), &MessageCount, sizeof(uint64_t));
}

void SystemMessaging::ClearMessageBuffer()
{
    memset(&(MessageStorage->StorageBuffer[0]), 0x0,
           MessageStorage->GetCurrentSize());
    SetNumMessages(0);
}

uint64_t SystemMessaging::GetMessageCount()
{
    uint64_t *CurrentMessageCount = reinterpret_cast<uint64_t*>
    (&MessageStorage->StorageBuffer[0]);
    return(*CurrentMessageCount);
}

uint64_t SystemMessaging::GetCurrentSize()
{
    uint64_t TotalBufferSize = sizeof(uint64_t); // -- The num-messages count;
    MessageHeaderData *MessHeader = reinterpret_cast<MessageHeaderData *>
    (&MessageStorage->StorageBuffer[sizeof(uint64_t)]);
    uint64_t TotalMessageCount = GetMessageCount();
    uint64_t SingleHeaderSize = sizeof(SingleMessageHeader);
    for(uint64_t i=0; i<TotalMessageCount; i++)
    {
        TotalBufferSize += sizeof(MessageHeaderData);
        TotalBufferSize += MessHeader->MaxNumberBuffers *
        (MessHeader->MaxMessageSize + SingleHeaderSize);
        if(i < TotalMessageCount - 1)
        {
            MessHeader++;
        }
    }
    return(TotalBufferSize);
}

int64_t SystemMessaging::CreateNewMessage(std::string MessageName,
    uint64_t MaxSize, uint64_t NumMessageBuffers, std::string messageStruct)
{
	if (FindMessageID(MessageName) >= 0)
	{
		std::cerr << "The message " << MessageName << " was created more than once.";
		std::cerr << std::endl;
		return(FindMessageID(MessageName));
	}
    if(NumMessageBuffers <= 0)
    {
        std::cerr << "I can't create a message with zero buffers.  I refuse.";
        std::cerr << std::endl;
        CreateFails++;
        return(-1);
    }
    if(NumMessageBuffers == 1)
    {
        std::cerr << "You created a message with only one buffer.";
        std::cerr << "  This might compromise the message integrity.  Watch out.";
        std::cerr << std::endl;
    }
    uint64_t InitSize = GetCurrentSize();
    uint64_t StorageRequired = InitSize + sizeof(MessageHeaderData) +
    (MaxSize+sizeof(MessageHeaderData))*NumMessageBuffers;
    MessageStorage->IncreaseStorage(StorageRequired);
    uint8_t *MessagingStart = &(MessageStorage->StorageBuffer[GetMessageCount()*
                                                              sizeof(MessageHeaderData) + sizeof(uint64_t)]);
    if(GetMessageCount() > 0)
    {
        uint8_t *NewMessagingStart = MessagingStart + sizeof(MessageHeaderData);
        memmove(NewMessagingStart, MessagingStart, InitSize -
                GetMessageCount()*sizeof(MessageHeaderData));
        memset(MessagingStart, 0x0, sizeof(MessageHeaderData));
        for(uint32_t i=0; i<GetMessageCount(); i++)
        {
            MessageHeaderData *UpdateHeader = FindMsgHeader(i);
            UpdateHeader->StartingOffset += sizeof(MessageHeaderData);
        }
    }
    MessageHeaderData* NewHeader = reinterpret_cast<MessageHeaderData *>
    (MessagingStart);
    uint32_t NameLength = MessageName.size();
    if(NameLength > MAX_MESSAGE_SIZE)
    {
        std::cout << "Your name length is too long, truncating name" <<std::endl;
        NameLength = MAX_MESSAGE_SIZE;
    }
    strncpy(NewHeader->MessageName, MessageName.c_str(), NameLength);
    NameLength = messageStruct.size();
    if(NameLength > MAX_MESSAGE_SIZE)
    {
        std::cout << "Your struct name length is too long, truncating name" <<std::endl;
        NameLength = MAX_MESSAGE_SIZE;
    }
    strncpy(NewHeader->messageStruct, messageStruct.c_str(), NameLength);
    NewHeader->UpdateCounter = 0;
    NewHeader->CurrentReadBuffer = 0;
    NewHeader->MaxNumberBuffers = NumMessageBuffers;
    NewHeader->MaxMessageSize = MaxSize;
    NewHeader->CurrentReadSize = 0;
    NewHeader->CurrentReadTime = 0;
    NewHeader->StartingOffset = InitSize + sizeof(MessageHeaderData);
    memset(&(MessageStorage->StorageBuffer[NewHeader->StartingOffset]), 0x0,
           NumMessageBuffers*(MaxSize + sizeof(SingleMessageHeader)));
    SetNumMessages(GetMessageCount() + 1);
    return(GetMessageCount() - 1);
}

bool SystemMessaging::WriteMessage(uint64_t MessageID, uint64_t ClockTimeNanos,
                                   uint64_t MsgSize, uint8_t *MsgPayload)
{
    if(MessageID >= GetMessageCount())
    {
        std::cerr << "Received a write request for invalid message ID: ";
        std::cerr << MessageID<<std::endl;
        WriteFails++;
        return(false);
    }
    MessageHeaderData* MsgHdr = FindMsgHeader(MessageID);
    if(MsgSize > MsgHdr->MaxMessageSize)
    {
        std::cerr << "Received a write request that was too large for: " <<
        MsgHdr->MessageName<<std::endl;
        WriteFails++;
        return(false);
    }
    uint8_t *WriteDataBuffer = &(MessageStorage->StorageBuffer[MsgHdr->
                                                               StartingOffset]);
    uint64_t AccessIndex = (MsgHdr->UpdateCounter%MsgHdr->MaxNumberBuffers)*
    (sizeof(SingleMessageHeader) + MsgHdr->MaxMessageSize);
    WriteDataBuffer += AccessIndex;
    SingleMessageHeader WriteHeader;
    WriteHeader.WriteClockNanos = ClockTimeNanos;
    WriteHeader.WriteSize = MsgSize;
    memcpy(WriteDataBuffer, &WriteHeader, sizeof(SingleMessageHeader));
    WriteDataBuffer += sizeof(SingleMessageHeader);
    memcpy(WriteDataBuffer, MsgPayload, MsgSize);
    MsgHdr->CurrentReadSize = MsgSize;
    MsgHdr->CurrentReadTime = ClockTimeNanos;
    MsgHdr->CurrentReadBuffer = MsgHdr->UpdateCounter%MsgHdr->MaxNumberBuffers;
    MsgHdr->UpdateCounter++;
    return(true);
}

/*! This method is static and is added so that other classes (ex. messageLogger)
 that have the messaging buffer layout can easily access their own internal
 buffers without having to re-write the same code.  Kind of overkill, but
 there you go.
 @return void
 @param MsgBuffer The base address of the message buffer we are reading
 @param MsgBytes The maximum number of bytes for a given message type
 @param CurrentOffset The message count that we want to ready out
 @param DataHeader The message header that we are writing out to
 @param OutputBuffer The output message buffer we are writing out to
 */
void SystemMessaging::AccessMessageData(uint8_t *MsgBuffer, uint64_t maxMsgBytes,
                                        uint64_t CurrentOffset, SingleMessageHeader *DataHeader,
                                        uint64_t maxReadBytes, uint8_t *OutputBuffer)
{
    MsgBuffer += CurrentOffset * (sizeof(SingleMessageHeader) +
                                  maxMsgBytes);
    memcpy(DataHeader, MsgBuffer, sizeof(SingleMessageHeader));
    uint64_t ReadSize = maxReadBytes < DataHeader->WriteSize ? maxReadBytes :
    DataHeader->WriteSize;
    MsgBuffer += sizeof(SingleMessageHeader);
    memcpy(OutputBuffer, MsgBuffer, ReadSize);
}

bool SystemMessaging::ReadMessage(uint64_t MessageID, SingleMessageHeader
                                  *DataHeader, uint64_t MaxBytes, uint8_t *MsgPayload, uint64_t CurrentOffset)
{
    if(MessageID >= GetMessageCount())
    {
        std::cerr << "Received a read request for invalid message ID: ";
        std::cerr << MessageID<<std::endl;
        ReadFails++;
        return(false);
    }
    MessageHeaderData* MsgHdr = FindMsgHeader(MessageID);
    /// - If there is no data just alert caller that nothing came back
    if(MsgHdr->UpdateCounter == 0)
    {
        return(false);
    }
    int64_t CurrentIndex = MsgHdr->UpdateCounter % MsgHdr->MaxNumberBuffers;
    CurrentIndex -= (1 + CurrentOffset);
    while(CurrentIndex < 0)
    {
        CurrentIndex += MsgHdr->MaxNumberBuffers;
    }
    uint8_t *ReadBuffer = &(MessageStorage->
                            StorageBuffer[MsgHdr->StartingOffset]);
    uint64_t MaxOutputBytes = MaxBytes < MsgHdr->MaxMessageSize ? MaxBytes :
    MsgHdr->MaxMessageSize;
    AccessMessageData(ReadBuffer, MsgHdr->MaxMessageSize, CurrentIndex,
                      DataHeader, MaxOutputBytes, MsgPayload);
    return(true);
}

void SystemMessaging::PrintAllMessageData()
{
    uint64_t TotalMessageCount = GetMessageCount();
    std::cout << "Number of Messages: " <<TotalMessageCount<<std::endl;
    for(uint64_t i=0; i<TotalMessageCount; i++)
    {
        PrintMessageStats(i);
    }
}

MessageHeaderData* SystemMessaging::FindMsgHeader(uint64_t MessageID)
{
    MessageHeaderData* MsgHdr;
    if(MessageID >= GetMessageCount())
    {
        return NULL;
    }
    MsgHdr = reinterpret_cast<MessageHeaderData*> (&(MessageStorage->
                                                     StorageBuffer[sizeof(uint64_t)]));
    MsgHdr += MessageID;
    return(MsgHdr);
}

void SystemMessaging::PrintMessageStats(uint64_t MessageID)
{
    MessageHeaderData* MsgHdr = FindMsgHeader(MessageID);
    if(MsgHdr == NULL)
    {
        std::cerr << "Received a print request for ID: "<<MessageID<<std::endl;
        std::cerr << "That ID is not valid."<<std::endl;
        return;
    }
    std::cout << "Name: "<< MsgHdr->MessageName << std::endl;
    std::cout << "Writes: "<< MsgHdr->UpdateCounter << std::endl;
    std::cout << "MsgSize: "<<MsgHdr->MaxMessageSize << std::endl;
    std::cout << "NumberMsgs: "<<MsgHdr->MaxNumberBuffers<<std::endl;
}

std::string SystemMessaging::FindMessageName(uint64_t MessageID)
{
    if(MessageID >= GetMessageCount())
    {
        std::cerr << "Asked to find a message for invalid ID: "<<MessageID;
        std::cerr << std::endl;
    }
    MessageHeaderData* MsgHdr = FindMsgHeader(MessageID);
    return(MsgHdr->MessageName);
    
}

int64_t SystemMessaging::FindMessageID(std::string MessageName)
{
    MessageHeaderData* MsgHdr;
    for(uint64_t i=0; i<GetMessageCount(); i++)
    {
        MsgHdr = FindMsgHeader(i);
        if(MessageName == std::string(MsgHdr->MessageName))
        {
            return(i);
        }
    }
    return(-1);
}
