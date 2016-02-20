
#include "architecture/messaging/system_messaging.h"
#include <cstring>
#include <string>
#include <iostream>

SystemMessaging* SystemMessaging::TheInstance = NULL;

SystemMessaging :: SystemMessaging()
{
    messageStorage = NULL;
    CreateFails = 0;
    WriteFails = 0;
    ReadFails = 0;
    nextModuleID = 0;
}

SystemMessaging::~SystemMessaging()
{
    messageStorage = NULL;
    
}

SystemMessaging* SystemMessaging::GetInstance()
{
    if(TheInstance == NULL)
    {
        TheInstance = new SystemMessaging();
    }
    return(TheInstance);
}

uint64_t SystemMessaging::AttachStorageBucket(std::string bufferName)
{
    uint64_t bufferCount;
    MessageStorageContainer *newContainer = new MessageStorageContainer();
    newContainer->messageStorage.IncreaseStorage(sizeof(uint64_t)+20000);
    dataBuffers.push_back(newContainer);
    bufferCount = dataBuffers.size() - 1;
    newContainer->bufferName = bufferName;
    messageStorage = *(dataBuffers.end()-1);
    SetNumMessages(0);
    return(bufferCount);
}

void SystemMessaging::selectMessageBuffer(uint64_t bufferUse)
{

    std::cout << __FUNCTION__ << ": " << bufferUse << std::endl;

    std::vector<MessageStorageContainer*>::iterator it;
    it = dataBuffers.begin();

    if(bufferUse >= dataBuffers.size())
    {
        std::cerr << "You've attempted to access a message buffer that does not exist. Yikes.";
        std::cerr << std::endl;
        messageStorage = *dataBuffers.begin();
        return;
    }
    it += bufferUse;
    messageStorage = (*it);
}

void SystemMessaging::SetNumMessages(uint64_t MessageCount)
{
    if(messageStorage == NULL)
    {
        std::cerr << "I received a request to set num messages for a NULL buffer";
        std::cerr << std::endl;
        return;
    }
    memcpy(&(messageStorage->messageStorage.StorageBuffer[0]), &MessageCount, sizeof(uint64_t));
}

void SystemMessaging::ClearMessageBuffer()
{
    memset(&(messageStorage->messageStorage.StorageBuffer[0]), 0x0,
           messageStorage->messageStorage.GetCurrentSize());
    SetNumMessages(0);
}

void SystemMessaging::clearMessaging()
{
    std::vector<MessageStorageContainer *>::iterator it;
    for(it=dataBuffers.begin(); it != dataBuffers.end(); it++)
    {
        std::cout << "In: " << __FUNCTION__ << ": " <<  (*it)->bufferName << std::endl;

        delete (*it);
    }
    dataBuffers.clear();
    nextModuleID = 0;
    messageStorage = NULL;
}

uint64_t SystemMessaging::GetMessageCount()
{
    uint64_t *CurrentMessageCount = reinterpret_cast<uint64_t*>
    (&messageStorage->messageStorage.StorageBuffer[0]);
    return(*CurrentMessageCount);
}

uint64_t SystemMessaging::GetCurrentSize()
{
    uint64_t TotalBufferSize = sizeof(uint64_t); // -- The num-messages count;
    MessageHeaderData *MessHeader = reinterpret_cast<MessageHeaderData *>
    (&messageStorage->messageStorage.StorageBuffer[sizeof(uint64_t)]);
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
    uint64_t MaxSize, uint64_t NumMessageBuffers, std::string messageStruct,
    int64_t moduleID)
{
    if (FindMessageID(MessageName) >= 0)
    {
    	std::cerr << "The message " << MessageName << " was created more than once.";
    	std::cerr << std::endl;
        if(moduleID >= 0)
        {
            std::vector<AllowAccessData>::iterator it;
            it = messageStorage->pubData.begin();
            it += FindMessageID(MessageName);
            it->accessList.insert(moduleID);
        }
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
    messageStorage->messageStorage.IncreaseStorage(StorageRequired);
    uint8_t *MessagingStart = &(messageStorage->messageStorage.StorageBuffer[GetMessageCount()*
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
    NewHeader->previousPublisher = -1;
    NewHeader->StartingOffset = InitSize + sizeof(MessageHeaderData);
    memset(&(messageStorage->messageStorage.StorageBuffer[NewHeader->StartingOffset]), 0x0,
           NumMessageBuffers*(MaxSize + sizeof(SingleMessageHeader)));
    SetNumMessages(GetMessageCount() + 1);
    AllowAccessData dataList;
    messageStorage->subData.push_back(dataList); //!< No subscribers yet
    if(moduleID >= 0)
    {
        dataList.accessList.insert(moduleID);
    }
    messageStorage->pubData.push_back(dataList);
    return(GetMessageCount() - 1);
}

int64_t SystemMessaging::subscribeToMessage(std::string messageName,
    uint64_t messageSize, int64_t moduleID)
{
    int64_t messageID;
    std::vector<AllowAccessData>::iterator it;
    messageID = FindMessageID(messageName);
    if(messageID < 0)
    {
        messageID = CreateNewMessage(messageName, messageSize, 2);
    }
    if(moduleID >= 0)
    {
        it = messageStorage->subData.begin();
        it += messageID;
        it->accessList.insert(moduleID);
    }
    return(messageID);
}

messageIdentData SystemMessaging::messagePublishSearch(std::string messageName)
{
    int64_t messageID;
    
    messageIdentData dataFound;
    dataFound.itemFound = false;
    dataFound.itemID = -1;
    dataFound.processBuffer = ~0;
    std::vector<MessageStorageContainer *>::iterator it;
    for(it=dataBuffers.begin(); it != dataBuffers.end(); it++)
    {
        messageStorage = (*it);
        messageID = FindMessageID(messageName);
        if(messageID < 0)
        {
            continue;
        }
        dataFound.itemFound = true;
        dataFound.itemID = messageID;
        dataFound.processBuffer = it - dataBuffers.begin();
        dataFound.bufferName = messageStorage->bufferName;
        std::vector<AllowAccessData>::iterator pubIt;
        pubIt=messageStorage->pubData.begin() + messageID;
        if(pubIt->accessList.size() > 0)
        {
            return(dataFound);
        }
    }
    return(dataFound);
}

bool SystemMessaging::WriteMessage(uint64_t MessageID, uint64_t ClockTimeNanos,
                                   uint64_t MsgSize, uint8_t *MsgPayload, int64_t moduleID)
{
    if(MessageID >= GetMessageCount())
    {
        std::cerr << "Received a write request for invalid message ID: ";
        std::cerr << MessageID<<std::endl;
        WriteFails++;
        return(false);
    }
    MessageHeaderData* MsgHdr = FindMsgHeader(MessageID);
    if(MsgHdr->previousPublisher != moduleID)
    {
        std::vector<AllowAccessData>::iterator it;
        it = messageStorage->pubData.begin();
        it += MessageID;
        if(it->accessList.find(moduleID) != it->accessList.end())
        {
            MsgHdr->previousPublisher = moduleID;
        }
        else
        {
            std::cerr << "Received a write request from a module that doesn't publish";
            std::cerr << " for " << FindMessageName(MessageID)<<std::endl;
            std::cerr << "You get nothing."<<std::endl;
            return(false);
        }
    }
    if(MsgSize != MsgHdr->MaxMessageSize)
    {
        std::cerr << "Received a write request that was incorrect size for: " <<
        MsgHdr->MessageName<<std::endl;
        WriteFails++;
        return(false);
    }
    uint8_t *WriteDataBuffer = &(messageStorage->messageStorage.StorageBuffer[MsgHdr->
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
    uint8_t *ReadBuffer = &(messageStorage->messageStorage.
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
    MsgHdr = reinterpret_cast<MessageHeaderData*> (&(messageStorage->messageStorage.
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
    std::cout << "Pats: FindMessageID sought messageName: " << MessageName <<std::endl;
    MessageHeaderData* MsgHdr;
    for(uint64_t i=0; i<GetMessageCount(); i++)
    {
        MsgHdr = FindMsgHeader(i);
        if(MessageName == std::string(MsgHdr->MessageName))
        {
            std::cout << "Pats: FindMessageID found messageName: " << std::string(MsgHdr->MessageName) <<std::endl;
            return(i);
        }
    }
    return(-1);
}

uint64_t SystemMessaging::checkoutModuleID()
{
    return(nextModuleID++);
}

int64_t SystemMessaging::findMessageBuffer(std::string bufferName)
{
    std::vector<MessageStorageContainer *>::iterator it;
    for(it = dataBuffers.begin(); it!= dataBuffers.end(); it++)
    {
        MessageStorageContainer *localContainer = (*it);
        if(localContainer->bufferName == bufferName)
        {
            return(it - dataBuffers.begin());
        }
    }
    return(-1);
}

std::set<std::string> SystemMessaging::getUnpublishedMessages()
{
    std::set<std::string> unpublishedList;
    std::vector<AllowAccessData>::iterator it;
    for(it=messageStorage->pubData.begin(); it!=messageStorage->pubData.end();
        it++)
    {
        if(it->accessList.size() <= 0)
        {
            std::string unknownPub = SystemMessaging::GetInstance()->
            FindMessageName(it - messageStorage->pubData.begin());
            unpublishedList.insert(unknownPub);
        }
    }
    return(unpublishedList);
}

std::set<std::string> SystemMessaging::getUniqueMessageNames()
{
    std::set<std::string> outputNames;
    std::vector<MessageStorageContainer *>::iterator it;
    for(it = dataBuffers.begin(); it != dataBuffers.end(); it++)
    {
        std::cout << __FUNCTION__ << std::endl;

        selectMessageBuffer(it - dataBuffers.begin());
        for(uint64_t i=0; i<GetMessageCount(); i++)
        {
            outputNames.insert(FindMessageName(i));
            
        }
    }
    return(outputNames);
}