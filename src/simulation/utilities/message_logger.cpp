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

#include "utilities/message_logger.h"
#include "utilities/bsk_Print.h"
#include <cstring>
#include <iostream>
#include <map>
#include <fstream>


/*! This constructor is used to initialize the message logging data.  It clears
 out the message log list and resets the logger to a clean state.
 */
messageLogger::messageLogger()
{
    logData.clear();
    allLogsLinked = true;
    initBufferSize = 50000;
}

/*! Nothing to destroy really */
messageLogger::~messageLogger()
{
}

/*! This method inserts a new message onto the logging vector so that it gets
 logged out appropriately.  If the message has already been added once
 already it will just update the log period and return from there.
 @return void
 @param messageName -- The name of the message that we want to log
 @param messagePeriod ns The minimum time between messages that we want to allow
 */
void messageLogger::addMessageLog(std::string messageName, uint64_t messagePeriod)
{
    //! Begin  method steps
    //! - Check to see if we have already commanded a log for this message, if so just update the rate
    std::vector<messageLogContainer>::iterator it;
    for(it=logData.begin(); it != logData.end(); it++)
    {
        if(it->messageName ==  messageName)
        {
            it->writeDelta = messagePeriod;
            return;
        }
    }
    //! - If the message hasn't been added yet, allocate new container, init data, and add to log vector
    messageLogContainer newContainer;
    newContainer.messageName = messageName;
    newContainer.messageID = -1;
    newContainer.lastLogTime = 0;
    newContainer.logInstanceCount = 0;
    newContainer.lastWriteCheck = 0;
    newContainer.writeDelta = messagePeriod;
    newContainer.bufferOffset = 0;
    newContainer.storOff.clear();
    logData.push_back(newContainer);
    //! - Since we have a new message, note that we need to link it during run
    allLogsLinked = false;
}
/*! This method is used to find the appropriate messaged ID associated with
 each message name that has been added to logging.  It warns the user if any
 of the added messages have not been successfully linked.
 @return void
 */
void messageLogger::linkMessages()
{
    //! Begin  method steps
    //! - Since we are linking, go ahead and just relink all messages
    std::vector<messageLogContainer>::iterator it;
    //! - Set the linker indicator to good and only falsify it if we don't find a message
    allLogsLinked = true;
    MessageIdentData messageData;
    for(it=logData.begin(); it != logData.end(); it++)
    {
        messageData = SystemMessaging::GetInstance()->
        messagePublishSearch(it->messageName);
        if(messageData.itemFound)
        {
            it->processID = messageData.processBuffer;
            it->messageID = (int32_t)messageData.itemID;
            it->messageBuffer.IncreaseStorage(initBufferSize);
        }
        //! - Warn the user if linking failed and note that logging won't work for that message
        else
        {
            BSK_PRINT_BRIEF(MSG_WARNING, "failed to find message: %s Disabling logging for it.", it->messageName.c_str());
        }
    }
}
/*! This method executes the actual log generation activity.  It is focused on
 simplicity currently so there is room to speed things up if this
 functionality begins to consume significant parts of our runtime
 @return void
 */
void messageLogger::logAllMessages()
{
    //! Begin  method steps
    //! - Iterate through the message list and log any needed messages
    std::vector<messageLogContainer>::iterator it;
    for(it=logData.begin(); it != logData.end(); it++)
    {
        //! - Message ID is invalid if it is less than zero
        if(it->messageID <0)
        {
            continue;
        }
        //! - Get the current message header and check to see if it is new and if enough time has elapsed since the last log
        SystemMessaging::GetInstance()->selectMessageBuffer(it->processID);
        MessageHeaderData* localHeader = SystemMessaging::GetInstance()->
        FindMsgHeader(it->messageID);
        bool bufferNew = it->lastWriteCheck != localHeader->UpdateCounter;
        bufferNew = bufferNew ? (localHeader->CurrentReadTime - it->lastLogTime)
        >= it->writeDelta || it->lastWriteCheck == 0 : bufferNew;
        it->lastWriteCheck = localHeader->UpdateCounter;
        if(bufferNew)
        {
            //! - For valid message logging instance, increase storage buffer if necessary
            while((it->bufferOffset + sizeof(SingleMessageHeader) +
                   localHeader->CurrentReadSize) > it->messageBuffer.GetCurrentSize())
            {
                it->messageBuffer.IncreaseStorage(
                                                  it->messageBuffer.GetCurrentSize()*2+1);
            }
            //! - Read out current message and reset log parameter so that we grab the appropriate message next time
            uint8_t * localPtr =
            &(it->messageBuffer.StorageBuffer[it->bufferOffset]);
            SystemMessaging::GetInstance()->ReadMessage(it->messageID,
                                                        reinterpret_cast<SingleMessageHeader *> (localPtr),
                                                        localHeader->CurrentReadSize, &localPtr[sizeof(SingleMessageHeader)]);
            it->storOff.push_back(it->bufferOffset);
            it->bufferOffset += sizeof(SingleMessageHeader) +
            localHeader->CurrentReadSize;
            it->lastLogTime = localHeader->CurrentReadTime;
            it->logInstanceCount++;
        }
    }
}
bool messageLogger::readLog(MessageIdentData & messageID, SingleMessageHeader *dataHeader,
                            uint64_t maxBytes, uint8_t *msgPayload, uint64_t currentOffset)
{
    //! Begin  method steps
    //! - Iterate through the message list and find the requested ID
    std::vector<messageLogContainer>::iterator it;
    SingleMessageHeader *headPtr;
    for(it=logData.begin(); it != logData.end(); it++)
    {
        if(it->messageID != messageID.itemID ||
            it->processID != messageID.processBuffer)
        {
            continue;
        }
        int64_t currentIndex = it->logInstanceCount;
        currentIndex -= (1 + currentOffset);
        while(currentIndex < 0)
        {
            currentIndex += it->logInstanceCount;
        }
        std::vector<uint64_t>::iterator storIt;
        storIt = it->storOff.begin();
        storIt += currentIndex;
        uint8_t *dataPtr = &(it->messageBuffer.StorageBuffer[*storIt]);
        headPtr = reinterpret_cast<SingleMessageHeader*> (dataPtr);
        memcpy(dataHeader, headPtr, sizeof(SingleMessageHeader));
        dataPtr += sizeof(SingleMessageHeader);
        uint64_t bytesUse = maxBytes > headPtr->WriteSize ? headPtr->WriteSize : 
        maxBytes;
        memcpy(msgPayload, dataPtr, bytesUse);
        return(true);
    }
    
    return false;
}

uint64_t messageLogger::getLogCount(int64_t processID, int64_t messageID)
{
    std::vector<messageLogContainer>::iterator it;
    
    uint64_t messageCount = 0;

    for(it=logData.begin(); it != logData.end(); it++)
    {
        if(it->messageID != messageID || it->processID != processID)
        {
            continue;
        }
        messageCount = it->logInstanceCount;
        return(messageCount);
    }
    return(messageCount);
}

void messageLogger::clearLogs()
{
    std::vector<messageLogContainer>::iterator it;
    std::map<std::string, uint64_t> logMap;
    std::map<std::string, uint64_t>::iterator mapIt;
    for(it=logData.begin(); it != logData.end(); it++)
    {
        std::string messageName = it->messageName;
        uint64_t writeDelta = it->writeDelta;
        logMap.insert(std::pair<std::string, uint64_t>
            (messageName, writeDelta));
    }
    logData.clear();
    for(mapIt = logMap.begin(); mapIt != logMap.end(); mapIt++)
    {
        addMessageLog(mapIt->first, mapIt->second);
    }
    
}

void messageLogger::archiveLogsToDisk(std::string outFileName)
{
	uint32_t totalMessageCount, messageNameLength;
	char zero = 0x0;
	std::fstream ofile;
	std::vector<messageLogContainer>::iterator it;
	ofile.open(outFileName, std::ios::out | std::ios::trunc | std::ios::binary);
	totalMessageCount = logData.size();
	ofile.write(reinterpret_cast<const char*>(&totalMessageCount), sizeof(totalMessageCount));
	for (it = logData.begin(); it != logData.end(); it++)
	{
		messageNameLength = it->messageName.size()+1;
		ofile.write(reinterpret_cast<const char*>(&messageNameLength), sizeof(messageNameLength));
		ofile.write(it->messageName.c_str(), messageNameLength - 1);
		ofile.write(&zero, 1);
		ofile.write(reinterpret_cast<const char*>(&(it->messageID)), sizeof(it->messageID));
		ofile.write(reinterpret_cast<const char*>(&(it->logInstanceCount)), sizeof(it->logInstanceCount));
		ofile.write(reinterpret_cast<const char*>(&(it->bufferOffset)), sizeof(it->bufferOffset));
		ofile.write(reinterpret_cast<const char*>(it->messageBuffer.StorageBuffer), it->bufferOffset);
	}
	ofile.close();
}

void messageLogger::loadArchiveFromDisk(std::string inFileName)
{
	std::ifstream iFile;
	uint32_t totalMessageCount, messageNameLength;
	uint64_t dataBufferSize;

	logData.clear();
	iFile.open(inFileName, std::ios::in | std::ios::binary);
	iFile.read(reinterpret_cast<char*> (&totalMessageCount), sizeof(totalMessageCount));
	for (int i = 0; i < totalMessageCount; i++)
	{
		messageLogContainer newContainer;
		iFile.read(reinterpret_cast<char*> (&messageNameLength), sizeof(messageNameLength));
		char *msgName = new char[messageNameLength];
		iFile.read(msgName, messageNameLength);
		newContainer.messageName = msgName;
		iFile.read(reinterpret_cast<char*> (&newContainer.messageID), 
			sizeof(newContainer.messageID));
		iFile.read(reinterpret_cast<char*> (&newContainer.logInstanceCount), 
			sizeof(newContainer.logInstanceCount));
		iFile.read(reinterpret_cast<char*> (&dataBufferSize), sizeof(dataBufferSize));
		newContainer.messageBuffer.ClearStorage();
		newContainer.messageBuffer.IncreaseStorage(dataBufferSize);
		iFile.read(reinterpret_cast<char*> (newContainer.messageBuffer.StorageBuffer), dataBufferSize);
		newContainer.storOff.clear();
		SingleMessageHeader *headPtr;
		uint64_t bytesRead = 0;
		uint8_t *dataPtr; 
		while (bytesRead < dataBufferSize)
		{
			dataPtr = &(newContainer.messageBuffer.StorageBuffer[bytesRead]);
			headPtr = reinterpret_cast<SingleMessageHeader*> (dataPtr);
			newContainer.storOff.push_back(bytesRead);
			bytesRead += sizeof(SingleMessageHeader);
			bytesRead += headPtr->WriteSize;
		}
		logData.push_back(newContainer);
	}

	iFile.close();
}
