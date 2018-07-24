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

#include "architecture/messaging/system_messaging.h"
#include <cstring>
#include <string>
#include <iostream>
#include "utilities/bsk_Print.h"


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
    std::vector<MessageStorageContainer*>::iterator it;
    it = dataBuffers.begin();

    if(bufferUse >= dataBuffers.size())
    {
        BSK_PRINT_BRIEF(MSG_ERROR,"You've attempted to access a message buffer that does not exist. Yikes.\n");
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
        BSK_PRINT_BRIEF(MSG_ERROR,"Received a request to set num messages for a NULL buffer.\n");
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
        delete (*it);
    }
    dataBuffers.clear();
    nextModuleID = 0;
    CreateFails = 0;
    WriteFails = 0;
    ReadFails = 0;
    messageStorage = NULL;
}

uint64_t SystemMessaging::GetMessageCount(int32_t bufferSelect)
{
    uint64_t *CurrentMessageCount;
    if(bufferSelect < 0)
    {
       CurrentMessageCount = reinterpret_cast<uint64_t*>
           (&messageStorage->messageStorage.StorageBuffer[0]);
    }
    else
    {
        std::vector<MessageStorageContainer *>::iterator it;
        it = dataBuffers.begin();
        it += bufferSelect;
        CurrentMessageCount = reinterpret_cast<uint64_t*>
        (&((*it)->messageStorage.StorageBuffer[0]));
    }
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
        BSK_PRINT_BRIEF(MSG_INFORMATION,"The message %s was created more than once.\n", MessageName.c_str());
        if(moduleID >= 0)
        {
            std::vector<AllowAccessData>::iterator it;
            it = messageStorage->pubData.begin();
            it += FindMessageID(MessageName);
            it->accessList.insert(moduleID);
            it->publishedHere = true;
        }
    	return(FindMessageID(MessageName));
    }
    if(MessageName == "")
    {
        BSK_PRINT_BRIEF(MSG_ERROR,"Module ID: %lld tried to create a message without a name.  Please try again.\n", moduleID);
        CreateFails++;
        return(-1);
    }
    if(NumMessageBuffers <= 0)
    {
        BSK_PRINT_BRIEF(MSG_ERROR,"I can't create a message with zero buffers.  I refuse.\n");
        CreateFails++;
        return(-1);
    }
    if(NumMessageBuffers == 1)
    {
        BSK_PRINT_BRIEF(MSG_WARNING,"You created a message with only one buffer. This might compromise the message integrity. Watch out.\n");
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
    uint32_t NameLength = (uint32_t)MessageName.size();
    if(NameLength > MAX_MESSAGE_SIZE)
    {
        BSK_PRINT_BRIEF(MSG_ERROR,"Your name length for: %s. is too long, truncating name\n", MessageName.c_str());
        CreateFails++;
        NameLength = MAX_MESSAGE_SIZE;
    }
    strncpy(NewHeader->MessageName, MessageName.c_str(), NameLength);
    NameLength = (uint32_t)messageStruct.size();
    if(NameLength > MAX_MESSAGE_SIZE)
    {
        BSK_PRINT_BRIEF(MSG_ERROR,"Your struct name length for: %s. is too long, truncating name\n", messageStruct.c_str());
        CreateFails++;
        NameLength = MAX_MESSAGE_SIZE;
    }
    strncpy(NewHeader->messageStruct, messageStruct.c_str(), NameLength);
    NewHeader->UpdateCounter = 0;
    NewHeader->CurrentReadBuffer = 0;
    NewHeader->MaxNumberBuffers = (uint32_t)NumMessageBuffers;
    NewHeader->MaxMessageSize = MaxSize;
    NewHeader->CurrentReadSize = 0;
    NewHeader->CurrentReadTime = 0;
    NewHeader->previousPublisher = -1;
    NewHeader->StartingOffset = InitSize + sizeof(MessageHeaderData);
    memset(&(messageStorage->messageStorage.StorageBuffer[NewHeader->StartingOffset]), 0x0,
           NumMessageBuffers*(MaxSize + sizeof(SingleMessageHeader)));
    SetNumMessages(GetMessageCount() + 1);
    AllowAccessData dataList;
    MessageExchangeData exList;
    dataList.publishedHere = false;
    messageStorage->subData.push_back(dataList); //!< No subscribers yet
    if(moduleID >= 0)
    {
        dataList.accessList.insert(moduleID);
        dataList.publishedHere = true;
    }
    messageStorage->pubData.push_back(dataList);
    messageStorage->exchangeData.push_back(exList);
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
    if(moduleID >= 0 && messageID >= 0)
    {
        it = messageStorage->subData.begin();
        it += messageID;
        it->accessList.insert(moduleID);
        it->publishedHere = false;
    }
    return(messageID);
}

bool SystemMessaging::obtainWriteRights(uint64_t messageID, int64_t moduleID)
{
    bool rightsObtained = false;
    
    if(moduleID >= 0 && messageID < GetMessageCount())
    {
        std::vector<AllowAccessData>::iterator it;
        it = messageStorage->pubData.begin();
        it += messageID;
        it->accessList.insert(moduleID);
        rightsObtained = true;
    }
    
    return(rightsObtained);
}

bool SystemMessaging::obtainReadRights(uint64_t messageID, int64_t moduleID)
{
 
    bool rightsObtained = false;
    
    if(moduleID >= 0 && messageID < GetMessageCount())
    {
        std::vector<AllowAccessData>::iterator it;
        it = messageStorage->subData.begin();
        it += messageID;
        it->accessList.insert(moduleID);
        rightsObtained = true;
    }
    
    return(rightsObtained);
    
}

MessageIdentData SystemMessaging::messagePublishSearch(std::string messageName)
{
    int64_t messageID;
    
    MessageIdentData dataFound;
    dataFound.itemFound = false;
    dataFound.itemID = -1;
    dataFound.processBuffer = ~0;
    std::vector<MessageStorageContainer *>::iterator it;
    for(it=dataBuffers.begin(); it != dataBuffers.end(); it++)
    {
        messageID = FindMessageID(messageName, it-dataBuffers.begin());
        if(messageID < 0)
        {
            continue;
        }
        dataFound.itemFound = true;
        dataFound.itemID = messageID;
        dataFound.processBuffer = it - dataBuffers.begin();
        dataFound.bufferName = (*it)->bufferName;
        std::vector<AllowAccessData>::iterator pubIt;
        pubIt=(*it)->pubData.begin() + messageID;
        if(pubIt->accessList.size() > 0 && pubIt->publishedHere)
        {
            return(dataFound);
        }
    }
    return(dataFound);
}

bool SystemMessaging::WriteMessage(uint64_t MessageID, uint64_t ClockTimeNanos,
                                   uint64_t MsgSize, void *MsgPayload, int64_t moduleID)
{
    if(MessageID >= GetMessageCount())
    {
        BSK_PRINT_BRIEF(MSG_ERROR, "Received a write request for invalid message ID: %lld \n", MessageID);
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
            BSK_PRINT_BRIEF(MSG_ERROR, "Received a write request from a module that doesn't publish for %s . You get nothing.\n",
                      FindMessageName(MessageID).c_str());
            WriteFails++;
            return(false);
        }
    }
    if(MsgSize != MsgHdr->MaxMessageSize)
    {
        BSK_PRINT_BRIEF(MSG_ERROR, "Received a write request that was incorrect size for: %s . You get nothing.\n",
                  MsgHdr->MessageName);
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
                                  *DataHeader, uint64_t MaxBytes, void *MsgPayload, int64_t moduleID, uint64_t CurrentOffset)
{
    if(MessageID >= GetMessageCount())
    {
        BSK_PRINT_BRIEF(MSG_ERROR, "Received a read request for invalid message ID: %lld \n", MessageID);
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
    std::vector<MessageExchangeData>::iterator exIt;
    std::vector<AllowAccessData>::iterator accIt;
    accIt = messageStorage->subData.begin();
    exIt = messageStorage->exchangeData.begin();
    accIt += MessageID;
    exIt += MessageID;
    if(accIt->accessList.find(moduleID) == accIt->accessList.end()
        && moduleID != -1)
    {
        BSK_PRINT_BRIEF(MSG_WARNING, "Message %s was read by module ID %lld who is not on access list.\n", MsgHdr->MessageName, moduleID);
    }
    
    exIt->exchangeList.insert(std::pair<long int, long int>
        (MsgHdr->previousPublisher, moduleID));
    
    uint8_t *ReadBuffer = &(messageStorage->messageStorage.
                            StorageBuffer[MsgHdr->StartingOffset]);
    uint64_t MaxOutputBytes = MaxBytes < MsgHdr->MaxMessageSize ? MaxBytes :
    MsgHdr->MaxMessageSize;
    AccessMessageData(ReadBuffer, MsgHdr->MaxMessageSize, CurrentIndex,
                      DataHeader, MaxOutputBytes, reinterpret_cast<uint8_t*>(MsgPayload));
    return(true);
}

void SystemMessaging::PrintAllMessageData()
{
    uint64_t TotalMessageCount = GetMessageCount();
    BSK_PRINT_BRIEF(MSG_INFORMATION, "Number of Messages: %lld \n", TotalMessageCount);
    for(uint64_t i=0; i<TotalMessageCount; i++)
    {
        PrintMessageStats(i);
    }
}

MessageHeaderData* SystemMessaging::FindMsgHeader(uint64_t MessageID, int32_t bufferSelect)
{
    MessageHeaderData* MsgHdr;
    if(MessageID >= GetMessageCount(bufferSelect))
    {
        return NULL;
    }
    MessageStorageContainer *localStorage = messageStorage;
    if(bufferSelect >= 0)
    {
        std::vector<MessageStorageContainer *>::iterator it;
        it = dataBuffers.begin();
        it += bufferSelect;
        localStorage = *it;
    }
    MsgHdr = reinterpret_cast<MessageHeaderData*> (&(localStorage->messageStorage.
                                                     StorageBuffer[sizeof(uint64_t)]));
    MsgHdr += MessageID;
    return(MsgHdr);
}

void SystemMessaging::PrintMessageStats(uint64_t MessageID)
{
    MessageHeaderData* MsgHdr = FindMsgHeader(MessageID);
    if(MsgHdr == NULL)
    {
        BSK_PRINT_BRIEF(MSG_ERROR, "Received a print request for ID: %lld That ID is not valid.\n", MessageID);
        return;
    }
    BSK_PRINT_BRIEF(MSG_INFORMATION, "INFORMATION:\n Name: %s\n Writes: %lld\nMsgSize: %lld\nNumberMsgs: %u\n",
              MsgHdr->MessageName, MsgHdr->UpdateCounter, MsgHdr->MaxMessageSize, MsgHdr->MaxNumberBuffers);
}

std::string SystemMessaging::FindMessageName(uint64_t MessageID, int32_t bufferSelect)
{
    if(MessageID >= GetMessageCount(bufferSelect))
    {
        BSK_PRINT_BRIEF(MSG_WARNING, "WARING: Asked to find a message for invalid ID: %lld", MessageID);
    }
    MessageHeaderData* MsgHdr = FindMsgHeader(MessageID, bufferSelect);
    return(MsgHdr->MessageName);
    
}

int64_t SystemMessaging::FindMessageID(std::string MessageName, int32_t bufferSelect)
{
    MessageHeaderData* MsgHdr;
    for(uint64_t i=0; i<GetMessageCount(bufferSelect); i++)
    {
        MsgHdr = FindMsgHeader(i, bufferSelect);
        if(MessageName == std::string(MsgHdr->MessageName))
        {
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
        for(uint64_t i=0; i<GetMessageCount(it - dataBuffers.begin()); i++)
        {
            outputNames.insert(FindMessageName(i, it - dataBuffers.begin()));
            
        }
    }
    return(outputNames);
}

std::set<std::pair<long int, long int>>
    SystemMessaging::getMessageExchangeData(uint64_t messageID)
{
    std::vector<MessageExchangeData>::iterator it;
    it = messageStorage->exchangeData.begin();
    it += messageID;
    return(it->exchangeList);
}
