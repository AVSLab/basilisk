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
#ifndef _MessageLogger_HH_
#define _MessageLogger_HH_

#include <vector>
#include <string>
#include <stdint.h>
#include "architecture/messaging/system_messaging.h"

/*! \addtogroup SimArchGroup
 * @{
 */

typedef struct {
    std::string messageName;    //!< -- The message name associated with the log
    int32_t messageID;          //!< -- The message ID associated with the log
    uint64_t processID;         //!< -- Process ID associated with the message
    uint64_t lastLogTime;       //!< ns The last valid log time observed
    uint64_t logInstanceCount;  //!< -- The number of message logs that we have made
    uint64_t lastWriteCheck;    //!< ns The last write count that we examined
    uint64_t writeDelta;        //!< ns The minimum time between log steps
    uint64_t bufferOffset;      //!< -- The current offset in the log to access
    BlankStorage messageBuffer; //!< The storage buffer associated with the log
    std::vector<uint64_t> storOff; //!< -- Vector of storage buffer offset offsets for access
}messageLogContainer;

//Note that when archiving, this is the file format:
/* uint32_t uniqueMessageCount
   for each message:
	   uint32_t messageNameLength
	   char * messageName of length messageNameLength
	   int32_t messageID
	   uint64_t messageLogCount
	   uint64_t dataBufferSize
	   uint8_t *dataBuffer of length dataBufferSize*/

//! The top-level container for an entire simulation
class messageLogger
{
public:
    messageLogger(); //!< The MessageLogger constructor
    ~messageLogger();//!< MessageLogger destructor
    void addMessageLog(std::string messageName, uint64_t messagePeriod=0);
    void linkMessages();
    bool messagesLinked() {return allLogsLinked;} //!< Getter for link success
    void logAllMessages();
    bool readLog(MessageIdentData & messageID, SingleMessageHeader *dataHeader,
                 uint64_t maxBytes, uint8_t *msgPayload, uint64_t currentOffset=0);
    uint64_t getLogCount(int64_t processID, int64_t messageID);
    void clearLogs();
	void archiveLogsToDisk(std::string outFileName);
	void loadArchiveFromDisk(std::string inFileName);
    
public:
    uint64_t initBufferSize; //!< Default buffer size fo message log storage
    std::vector<messageLogContainer> logData; //!< Vector of log elements
private:
    bool allLogsLinked; //!< Indicator of whether or not messages are all linked
};

/*! @} */
#endif /* _MessageLogger_H_ */
