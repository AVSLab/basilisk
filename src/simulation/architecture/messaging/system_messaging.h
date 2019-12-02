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

#ifndef _SystemMessaging_HH_
#define _SystemMessaging_HH_

#include <vector>
#include <stdint.h>
#include <string>
#include <set>
#include <mutex>
#include "architecture/messaging/blank_storage.h"
#include "utilities/bskLogging.h"
#include "utilities/bskPrint.h"

/*! \addtogroup SimArchGroup
 * @{
 */

#define MAX_MESSAGE_SIZE 512

/*!
 * This struct holds all the key information for a message
 * it's kind of like an key to a message
 */
typedef struct {
    char MessageName[MAX_MESSAGE_SIZE];  //! -- Fix the max length of a message name
    char messageStruct[MAX_MESSAGE_SIZE]; //! -- Fix the max length of message struct names
    uint64_t UpdateCounter;  //! -- Number of times this message has been updated
    uint32_t CurrentReadBuffer;  //! -- current buffer to read the message from
    uint32_t MaxNumberBuffers;  //! -- Max buffers this message will have
    uint64_t MaxMessageSize;  //! -- Maximum allowable message size in bytes
    uint64_t CurrentReadSize;  //! -- Current size available for reading
    uint64_t CurrentReadTime;  //! [ns] Current time of last read
    uint64_t StartingOffset;  //! -- Starting offset in the storage buffer
    int64_t previousPublisher;  //! (-) The module who last published the message
}MessageHeaderData;

/*!
 * This header has more concise information and is stored
 * physically with the message data.
 */
typedef struct {
    uint64_t WriteClockNanos;  //! ns Time that message was written into buffer
    uint64_t WriteSize;  //! -- Number of bytes that were written to buffer
}SingleMessageHeader;

/*!
 * Used to store permissions info for messages.
 */
typedef struct {
    std::set<int64_t> accessList;  //! (-) List of modules who are allowed to read/write message
    bool publishedHere;            //! (-) Indicator about whether or not the message is published in this proc. buffer
}AllowAccessData;

/*!
 * Not sure yet how this is different than AllowAccessData
 */
typedef struct {
    std::set<std::pair<long int, long int>> exchangeList; //! (-) history of write/read pairs for message
}MessageExchangeData;

/*!
 * Basically the container for a single process buffer
 */
typedef struct {
    std::string bufferName;  //! (-) Name of this process buffer for application access
    BlankStorage messageStorage;  //! (-) The storage buffer associated with this process
    std::vector<AllowAccessData> pubData;  //! (-) Entry of publishers for each message ID
    std::vector<AllowAccessData> subData;  //! (-) Entry of subscribers for each message ID
    std::vector<MessageExchangeData> exchangeData;  //! [-] List of write/read pairs
}MessageStorageContainer;

/*!
 * another header/key to a message
 */
typedef struct {
    std::string bufferName;  //! (-) String associated with the message buffer
    int64_t processBuffer;  //! (-) Buffer ID for where this message originally lives
    int64_t itemID;  //! (-) ID associated with request
    bool itemFound;  //! (-) Indicator of whether the buffer was found
}MessageIdentData;

#ifdef _WIN32
class __declspec( dllexport) SystemMessaging
#else
class SystemMessaging
#endif

{

public:
    static SystemMessaging* GetInstance();  //! -- returns a pointer to the sim instance of SystemMessaging
    int64_t AttachStorageBucket(std::string bufferName = "");  //! -- adds a new buffer to the messaging system
    void SetNumMessages(int64_t MessageCount);  //! --updates message count in buffer header
    int64_t GetMessageCount(int32_t bufferSelect = -1);  //! --gets the number of messages in buffer bufferSelect
    void ClearMessageBuffer();  //! -- sets current buffer to zeros
    uint64_t GetCurrentSize();  //! -- returns size of current buffer
    int64_t CreateNewMessage(std::string MessageName, uint64_t MaxSize,
        uint64_t NumMessageBuffers = 2, std::string messageStruct = "", int64_t moduleID = -1);
    bool WriteMessage(int64_t MessageID, uint64_t ClockTimeNanos, uint64_t MsgSize,
                      void *MsgPayload, int64_t moduleID = -1);
    bool ReadMessage(int64_t MessageID, SingleMessageHeader *DataHeader,
                     uint64_t MaxBytes, void *MsgPayload, int64_t moduleID=-1, uint64_t CurrentOffset=0);
    static void AccessMessageData(uint8_t *MsgBuffer, uint64_t maxMsgBytes,
                                  uint64_t CurrentOffset, SingleMessageHeader *DataHeader,
                                  uint64_t maxReadBytes, uint8_t *OutputBuffer);
    MessageHeaderData* FindMsgHeader(int64_t MessageID, int32_t bufferSelect=-1);  //! -- returns a MessageHeaderData
    void PrintAllMessageData();  //! -- prints data for messages in current buffer
    void PrintMessageStats(int64_t MessageID);  //! -- prints data for a single message by ID
    std::string FindMessageName(int64_t MessageID, int32_t bufferSelect=-1);  //! -- searches only the selected buffer
    int64_t FindMessageID(std::string MessageName, int32_t bufferSelect=-1);  //! -- searches only the selected buffer
    int64_t subscribeToMessage(std::string messageName, uint64_t messageSize,
        int64_t moduleID);
    int64_t checkoutModuleID();  //! -- Assigns next integer module ID
    void selectMessageBuffer(int64_t bufferUse);  //! -- sets a default buffer for everything to use
    uint64_t getProcessCount() {return(this->dataBuffers.size());}
    MessageIdentData messagePublishSearch(std::string messageName);  //! -- returns MessageIdentData if found
    int64_t findMessageBuffer(std::string bufferName);
    std::set<std::string> getUnpublishedMessages();  //! -- returns msgs no one has access rights to
    std::set<std::string> getUniqueMessageNames();  //! -- searched across all buffers
    std::set<std::pair<long int, long int>>
        getMessageExchangeData(int64_t messageID);
    void clearMessaging();  //! -- wipes out all messages and buffers. total messaging system reset.
    bool obtainWriteRights(int64_t messageID, int64_t moduleID);  //! -- grants rights to the requesting module
    bool obtainReadRights(int64_t messageID, int64_t moduleID);  //! -- grants rights to the requesting module
    uint64_t getFailureCount() {return (this->CreateFails + this->ReadFails + this->WriteFails);}

private:
    SystemMessaging();
    ~SystemMessaging();
    SystemMessaging(SystemMessaging const &) {};
    SystemMessaging& operator =(SystemMessaging const &){return(*this);};

private:
    static SystemMessaging *TheInstance;
    std::vector<MessageStorageContainer *> dataBuffers;
    MessageStorageContainer *messageStorage; // this is a pointer to the currently selected message buffer above
    uint64_t WriteFails;  //! the number of times we tried to write invalidly
    uint64_t ReadFails;  //! the number of times we tried to read invalidly
    uint64_t CreateFails;  //! the number of times we tried to create invalidly
    int64_t nextModuleID;  //! the next module ID to give out when a module comes online
};

/* @} */

#endif /* _SystemMessaging_H_ */
