
#ifndef _SystemMessaging_HH_
#define _SystemMessaging_HH_

#include <vector>
#include <stdint.h>
#include <string>
#include <set>
#include "architecture/messaging/blank_storage.h"
#define MAX_MESSAGE_SIZE 512


typedef struct {
    char MessageName[MAX_MESSAGE_SIZE];// -- It pains me, but let's fix name
    char messageStruct[MAX_MESSAGE_SIZE]; // -- Again, pain, but it's better
    uint64_t UpdateCounter;      // -- Counter for number of updates in port
    uint32_t CurrentReadBuffer;  // -- Index for the current read buffer
    uint32_t MaxNumberBuffers;   // -- Length of message ring buffer
    uint64_t MaxMessageSize;     // -- Maximum allowable message size
    uint64_t CurrentReadSize;    // -- Current size available for reading
    uint64_t CurrentReadTime;    // ns Current time of last write
    uint64_t StartingOffset;     // -- Starting offset in the storage buffer
    int64_t previousPublisher;   // (-) The module who last published the message
}MessageHeaderData;

typedef struct {
    uint64_t WriteClockNanos;   // ns Time that message was written into buffer
    uint64_t WriteSize;         // -- Number of bytes that were written to buffer
}SingleMessageHeader;

typedef struct {
    std::set<uint64_t> accessList; // (-) List of modules who are allowed to access message
}AllowAccessData;

typedef struct {
    std::string bufferName;     // (-) Name of this message buffer for application access
    BlankStorage messageStorage; // (-) The storage buffer associated with this module
    std::vector<AllowAccessData> pubData; // (-) Entry of publishers for each message ID
    std::vector<AllowAccessData> subData; // (-) Entry of subscribers for each message ID
}MessageStorageContainer;

typedef struct {
    std::string bufferName;     // (-) String associated with the message buffer
    uint64_t processBuffer;     // (-) Buffer selection for this set of msg
    uint64_t itemID;            // (-) ID associated with request
    bool itemFound;             // (-) Indicator of whether the buffer was found
}messageIdentData;

#ifdef _WIN32
class __declspec( dllexport) SystemMessaging
#else
class SystemMessaging
#endif

{
    
public:
    static SystemMessaging* GetInstance();
    uint64_t AttachStorageBucket(std::string bufferName = "");
    void SetNumMessages(uint64_t MessageCount);
    uint64_t GetMessageCount();
    void ClearMessageBuffer();
    uint64_t GetCurrentSize();
    int64_t CreateNewMessage(std::string MessageName, uint64_t MaxSize,
        uint64_t NumMessageBuffers = 2, std::string messageStruct = "", int64_t moduleID = -1);
    bool WriteMessage(uint64_t MessageID, uint64_t ClockTimeNanos, uint64_t MsgSize,
                      uint8_t *MsgPayload, int64_t moduleID = -1);
    bool ReadMessage(uint64_t MessageID, SingleMessageHeader *DataHeader,
                     uint64_t MaxBytes, uint8_t *MsgPayload, uint64_t CurrentOffset=0);
    static void AccessMessageData(uint8_t *MsgBuffer, uint64_t maxMsgBytes,
                                  uint64_t CurrentOffset, SingleMessageHeader *DataHeader,
                                  uint64_t maxReadBytes, uint8_t *OutputBuffer);
    MessageHeaderData* FindMsgHeader(uint64_t MessageID);
    void PrintAllMessageData();
    void PrintMessageStats(uint64_t MessageID);
    std::string FindMessageName(uint64_t MessageID);
    int64_t FindMessageID(std::string MessageName);
    int64_t subscribeToMessage(std::string messageName, uint64_t messageSize,
        int64_t moduleID);
    uint64_t checkoutModuleID();
    void selectMessageBuffer(uint64_t bufferUse);
    uint64_t getProcessCount() {return(dataBuffers.size());}
    messageIdentData messagePublishSearch(std::string messageName);
    int64_t findMessageBuffer(std::string bufferName);
    std::set<std::string> getUnpublishedMessages();
    std::set<std::string> getUniqueMessageNames();
    void clearMessaging();
    
    
private:
    SystemMessaging();
    ~SystemMessaging();
    SystemMessaging(SystemMessaging const &) {};
    SystemMessaging& operator =(SystemMessaging const &){return(*this);};
    
private:
    static SystemMessaging *TheInstance;
    std::vector<MessageStorageContainer *> dataBuffers;
    MessageStorageContainer *messageStorage;
    uint64_t WriteFails;
    uint64_t ReadFails;
    uint64_t CreateFails;
    uint64_t nextModuleID;
};

#endif /* _SystemMessaging_H_ */
