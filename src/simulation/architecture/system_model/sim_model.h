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
#ifndef _SimModel_HH_
#define _SimModel_HH_

#include <vector>
#include <stdint.h>
#include <set>
#include "architecture/system_model/sys_process.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/message_logger.h"

/*! \addtogroup SimArchGroup
 *  This architecture group contains the source used to drive/schedule/interface
 *  with the simulation.
 * @{
 */

typedef enum varAccessType {
    messageBuffer = 0,
    logBuffer = 1
}VarAccessType;

//! The top-level container for an entire simulation
class SimModel
{
public:
    SimModel(); //!< The SimModel constructor
    ~SimModel();//!< SimModel destructor
    void selfInitSimulation();  //!< Method to initialize all added Tasks
    void crossInitSimulation();  //!< Method to initialize all added Tasks
    void resetInitSimulation();  //!< Method to reset all added tasks
    void StepUntilStop(uint64_t SimStopTime, int64_t stopPri); //!< Step simulation until stop time uint64_t reached
    void SingleStepProcesses(int64_t stopPri=-1); //!< Step only the next Task in the simulation
    void PrintSimulatedMessageData(); //!< Print out all messages that have been created
    void addNewProcess(SysProcess *newProc);
    uint64_t GetWriteData(std::string MessageName, uint64_t MaxSize,
                          void *MessageData, VarAccessType logType = messageBuffer,
                          uint64_t LatestOffset=0); //!< Grab a particular MessageName with MaxSize limited
    void ResetSimulation(); //!< Reset simulation back to zero
    void WriteMessageData(std::string MessageName, uint64_t MessageSize,
                          uint64_t ClockTime, void *MessageData); //!< Write in a single message
    void CreateNewMessage(std::string processName, std::string MessageName,
        uint64_t MessageSize, uint64_t NumBuffers=2,
        std::string messageStruct = ""); //!< Create a new message for use
    void logThisMessage(std::string messageName, uint64_t messagePeriod=0);
    uint64_t getNumMessages(); //!< Get number of messages in simulation
    std::string getMessageName(uint64_t messageID); //!< Get name for specified message ID
    MessageIdentData getMessageID(std::string messageName); //!< Get the ID associated with message name
    void populateMessageHeader(std::string messageName,
        MessageHeaderData* headerOut); //!< Get header data associated with msg
    std::set<std::string> getUniqueMessageNames();
    std::set<std::pair<long int, long int>> getMessageExchangeData(std::string messageName,
        std::set<unsigned long> procList  = std::set<unsigned long>());
    void terminateSimulation();
    
public:
    std::vector<SysProcess *> processList;          //!< -- List of processes we've created
    std::string SimulationName;                     //!< -- Identifier for Task
    uint64_t CurrentNanos;                          //!< ns Current clock time
    uint64_t NextTaskTime;                          //!< ns time for the next Task
    int64_t nextProcPriority;                       //!< [-] Priority level for the next process
    messageLogger messageLogs;                      //!< -- Message log data
};

/*! @} */
#endif /* _SimModel_H_ */
