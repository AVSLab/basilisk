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

#include "sim_model.h"
#include <cstring>
#include <iostream>

/*! This Constructor is used to initialize the top-level sim model.
 */
SimModel::SimModel()
{
    this->CurrentNanos = 0;
    this->NextTaskTime = 0;
    this->nextProcPriority = -1;
}

/*! Nothing to destroy really */
SimModel::~SimModel()
{
}

/*! This method exists to provide the python layer with a handle to call to
 print out the internal message stats.
 @return void*/
void SimModel::PrintSimulatedMessageData()
{
    SystemMessaging::GetInstance()->PrintAllMessageData();
}

/*! This method exists to provide a hook into the messaging system for obtaining
 message data that was written in the simulation.
 @return uint64_t Message Write time that we got
 @param MessageName String name for the message we are querying
 */
uint64_t SimModel::IsMsgCreated(std::string MessageName)
{
    MessageIdentData MessageID;

    //! Begin Method steps
    //! - Grab the message ID associated with name if it exists
    MessageID = SystemMessaging::GetInstance()->messagePublishSearch(MessageName);
    //! - If we got an invalid message ID back, alert the user and quit
    if(!MessageID.itemFound)
    {
        return(0);      /* no message found */
    } else {
        return(1);      /* message has been created */
    }

}

/*! This method exists to provide a hook into the messaging system for obtaining
 message data that was written in the simulation.
 @return uint64_t Message Write time that we got
 @param MessageName String name for the message we are querying
 @param MaxSize Maximum size of the message that we can pull
 @param MessageData A shapeshifting buffer that we can chunk data into
 @param logType log type variable
 @param LatestOffset An offset from the latest message to pull (default as zero)
 */
uint64_t SimModel::GetWriteData(std::string MessageName, uint64_t MaxSize,
                                void *MessageData, VarAccessType logType, uint64_t LatestOffset)
{
    MessageIdentData MessageID;
    SingleMessageHeader DataHeader;

    //! - Grab the message ID associated with name if it exists
    MessageID = SystemMessaging::GetInstance()->messagePublishSearch(MessageName);
    //! - If we got an invalid message ID back, alert the user and quit
    if(!MessageID.itemFound)
    {
        bskLogger.bskLog(BSK_ERROR, "You requested a message name: %s that message does not exist.", MessageName.c_str());
        return(0);
    }
    //! - For valid message names, get the data buffer associated with message
    switch(logType)
    {
        case messageBuffer:
            SystemMessaging::GetInstance()->
                selectMessageBuffer(MessageID.processBuffer);
            SystemMessaging::GetInstance()->ReadMessage(MessageID.itemID, &DataHeader, MaxSize, reinterpret_cast<uint8_t*> (MessageData), -1, LatestOffset);
            break;
        case logBuffer:
            this->messageLogs.readLog(MessageID, &DataHeader,
                                MaxSize, reinterpret_cast<uint8_t*> (MessageData), LatestOffset);
            break;
        default:
            bskLogger.bskLog(BSK_ERROR, "I don't know how to access the log type: %u", logType);
            break;
    }
    return(DataHeader.WriteClockNanos);
}

/*! This method allows the user to attach a process to the simulation for
    execution.  Note that the priority level of the process determines what
    order it gets called in: higher priorities are called before lower
    priorities. If priorities are the same, the proc added first goes first.
    @return void
    @param newProc the new process to be added
*/
void SimModel::addNewProcess(SysProcess *newProc)
{
    std::vector<SysProcess *>::iterator it;
    for(it = this->processList.begin(); it != this->processList.end(); it++)
    {
        if(newProc->processPriority > (*it)->processPriority)
        {
            this->processList.insert(it, newProc);
            return;
        }
    }
    this->processList.push_back(newProc);
}

/*! This method goes through all of the processes in the simulation,
 *  all of the tasks within each process, and all of the models within
 *  each task and self-inits them.
 @return void
 */
void SimModel::selfInitSimulation()
{
    std::vector<SysProcess *>::iterator it;
    for(it=this->processList.begin(); it!= this->processList.end(); it++)
    {
        (*it)->selfInitProcess();
    }
    if(SystemMessaging::GetInstance()->getFailureCount() > 0)
    {
        throw std::range_error("Message creation failed during self.  Please examine output.\n");
    }
    this->NextTaskTime = 0;
    this->CurrentNanos = 0;
    it=this->processList.begin();
    this->nextProcPriority = (*it)->processPriority;
}
/*! This method goes through all of the processes in the simulation,
 *  all of the tasks within each process, and all of the models within
 *  each task and cross-inits them.
 @return void
 */
void SimModel::crossInitSimulation()
{
    std::vector<SysProcess *>::iterator it;
    for(it=this->processList.begin(); it!= this->processList.end(); it++)
    {
        (*it)->crossInitProcess();
    }
    if(SystemMessaging::GetInstance()->getFailureCount() > 0)
    {
        throw std::range_error("Message creation failed during cross.  Please examine output.\n");
    }

}
/*! This method goes through all of the processes in the simulation,
 *  all of the tasks within each process, and all of the models within
 *  each task and resets them.
 @return void
 */
void SimModel::resetInitSimulation()
{
    std::vector<SysProcess *>::iterator it;
    for(it=this->processList.begin(); it!= this->processList.end(); it++)
    {
        (*it)->resetProcess(0);
    }
    if(SystemMessaging::GetInstance()->getFailureCount() > 0)
    {
        throw std::range_error("Message creation failed during reset.  Please examine output.\n");
    }
}

/*! This method steps all of the processes forward to the current time.  It also
    increments the internal simulation time appropriately as the simulation
    processes are triggered
    @param stopPri The priority level below which the sim won't go
    @return void
*/
void SimModel::SingleStepProcesses(int64_t stopPri)
{
    uint64_t nextCallTime = ~((uint64_t) 0);
    std::vector<SysProcess *>::iterator it = this->processList.begin();
    this->CurrentNanos = this->NextTaskTime;
    while(it!= this->processList.end())
    {
        SysProcess *localProc = (*it);
        if(localProc->processEnabled())
        {
            while(localProc->nextTaskTime < this->CurrentNanos ||
                (localProc->nextTaskTime == this->CurrentNanos &&
                  localProc->processPriority >= stopPri))
            {
                localProc->singleStepNextTask(this->CurrentNanos);
            }
            if(localProc->getNextTime() < nextCallTime)
            {
                nextCallTime = localProc->getNextTime();
                this->nextProcPriority = localProc->processPriority;
            }
            else if(localProc->getNextTime() == nextCallTime &&
                localProc->processPriority > this->nextProcPriority)
            {
                this->nextProcPriority = localProc->processPriority;
            }
        }
        it++;
    }
    if(SystemMessaging::GetInstance()->getFailureCount() > 0)
    {
        throw std::range_error("Message reads or writes failed.  Please examine output.\n");
    }
    this->NextTaskTime = nextCallTime != ~((uint64_t) 0) ? nextCallTime : this->CurrentNanos;
    //! - If a message has been added to logger, link the message IDs
    if(!this->messageLogs.messagesLinked())
    {
        this->messageLogs.linkMessages();
    }
    this->messageLogs.logAllMessages();
}

/*! This method steps the simulation until the specified stop time and
 stop priority have been reached.
 @return void
 @param SimStopTime Nanoseconds to step the simulation for
 @param stopPri The priority level below which the sim won't go
 */
void SimModel::StepUntilStop(uint64_t SimStopTime, int64_t stopPri)
{
    /*! - Note that we have to step until both the time is greater and the next
     Task's start time is in the future. If the NextTaskTime is less than
     SimStopTime, then the inPri shouldn't come into effect, so set it to -1
     (that's less than all process priorities, so it will run through the next
     process)*/
    int64_t inPri = SimStopTime == this->NextTaskTime ? stopPri : -1;
    while(this->NextTaskTime < SimStopTime || (this->NextTaskTime == SimStopTime &&
            this->nextProcPriority >= stopPri) )
    {
        this->SingleStepProcesses(inPri);
        inPri = SimStopTime == this->NextTaskTime ? stopPri : -1;
    }
}

/*! This method is used to reset a simulation to time 0. It sets all process and
 * tasks back to the initial call times. It clears all message logs. However,
 * it does not clear all message buffers and does not reset individual models.
 @return void
 */
void SimModel::ResetSimulation()
{
    std::vector<SysProcess *>::iterator it;
    //! - Iterate through model list and call the Task model initializer
    for(it = this->processList.begin(); it != this->processList.end(); it++)
    {
        (*it)->reInitProcess();
    }
    this->messageLogs.clearLogs();
    this->CurrentNanos = 0;
    this->NextTaskTime = 0;
}

/*! This method exists to provide a hook into the messaging system for creating
 messages for use by the simulation.
 @return void
 @param processName Name of process that we create the message in
 @param MessageName Name for the message we are creating
 @param MessageSize Size in bytes of the message we want to make
 @param NumBuffers The count of message buffers to create
 @param messageStruct A name for the message struct type */
void SimModel::CreateNewMessage(std::string processName, std::string MessageName,
    uint64_t MessageSize, uint64_t NumBuffers, std::string messageStruct)
{
    int64_t processID = SystemMessaging::GetInstance()->
        findMessageBuffer(processName);

    if(processID >= 0)
    {
        SystemMessaging::GetInstance()->selectMessageBuffer(processID);
        SystemMessaging::GetInstance()->CreateNewMessage(MessageName, MessageSize,
                                                     NumBuffers, messageStruct);
    }
    else
    {
        bskLogger.bskLog(BSK_ERROR, "You tried to create a message in a process that doesn't exist. No dice.");
        throw std::range_error("Message creation failed.  Please examine output.\n");
    }

}

/*! This method exists to provide a hook into the messaging system for writing
 message data into existing messages
 @return void
 @param MessageName Name for the message we are writing
 @param MessageSize Size in bytes of the message we're trying to write
 @param ClockTime The time that we are writing the message in nanoseconds
 @param MessageData A shapeshifting buffer that we can chunk data into*/
void SimModel::WriteMessageData(std::string MessageName, uint64_t MessageSize,
                                uint64_t ClockTime, void *MessageData)
{
    MessageIdentData MessageID; // A class with all of the message identifying information, including the ID

    //! - Grab the message ID associated with name if it exists
    MessageID = SystemMessaging::GetInstance()->
        messagePublishSearch(MessageName);
    //! - If we got an invalid message ID back, alert the user and quit
    if(!MessageID.itemFound)
    {
        bskLogger.bskLog(BSK_ERROR, "You tried to write to message name: %s that message does not exist.",
                MessageName.c_str());
        return;
    }
    SystemMessaging::GetInstance()->selectMessageBuffer(MessageID.processBuffer);
    SystemMessaging::GetInstance()->WriteMessage(MessageID.itemID, ClockTime,
                                                 MessageSize, reinterpret_cast<uint8_t*> (MessageData), -2);
}
/*! This method functions as a pass-through to the message logging structure
 when adding messages to log.  The main point is to serve as an API at the
 main simulation level without having to hook in the message logger
 somewhere else.
 @return void
 @param messageName The name of the message that we want to log
 @param messagePeriod The minimum time between messages to allow in ns
 */
void SimModel::logThisMessage(std::string messageName, uint64_t messagePeriod)
{
    this->messageLogs.addMessageLog(messageName, messagePeriod);
}

/*! This method gets the current number of messages that have been created in
    the simulation.
    @return uint64_t The number of messages that have been created
*/
int64_t SimModel::getNumMessages() {
    return(SystemMessaging::GetInstance()->GetMessageCount());
}
/*! This method finds the name associated with the message ID that is passed
    in.
    @return std::string messageName The message name for the ID
    @param messageID The message id that we wish to find the name for
*/
std::string SimModel::getMessageName(int64_t messageID)
{
    return(SystemMessaging::GetInstance()->FindMessageName(messageID));
}

/*! This method obtains the header information associated with a given message.
   Note the copy out to the incoming message.  The assumption is that this
   method is called from the python level where the storage for headerOut is
   created.  This way we don't connect a pointer to the internal message at the
   python level
   @return void
   @param messageName The name of the message we want to pull header information for
   @param headerOut The output header information we extract from the simulation
*/
void SimModel::populateMessageHeader(std::string messageName,
                           MessageHeaderData* headerOut)
{
    MessageIdentData messageID = SystemMessaging::GetInstance()->
        messagePublishSearch(messageName);
    SystemMessaging::GetInstance()->selectMessageBuffer(messageID.processBuffer);
    MessageHeaderData *locHeader = SystemMessaging::GetInstance()->
        FindMsgHeader(messageID.itemID);
    memcpy(headerOut, locHeader, sizeof(MessageHeaderData));
}

/*! This method finds the ID associated with the message name and returns it to
    the caller.  Mostly used to make sure a message is valid. If it's not, you'll
    just get -1, not a warning or failure.
    @return int64_t messageID ID of the message associated with messageName
    @param messageName The name of the message that you want the ID for
*/
MessageIdentData SimModel::getMessageID(std::string messageName)
{
    MessageIdentData messageID = SystemMessaging::GetInstance()->
    messagePublishSearch(messageName);
    return(messageID);
}

/*! This method gets the list of unique message names present in the simulation
    so that the user can see what messages have been created with no duplicates
    @return std::set<std::string> set of strings that constitute the unique names
*/
std::set<std::string> SimModel::getUniqueMessageNames()
{
    std::set<std::string> outputSet;
    outputSet = SystemMessaging::GetInstance()->getUniqueMessageNames();
    return(outputSet);
}

/*! This method clears all messages.  Note that once you do this, the simulation
    object itself is really dead.
    @return void
*/
void SimModel::terminateSimulation()
{
    SystemMessaging::GetInstance()->clearMessaging();
}

/*! This method returns all of the read/write pairs for the entire simulation
    for a given message.  That allows us to capture and analyze our data flow in
    a very clean manner.
    @return std::set<std::pair> returnPairs Write/Read pairs for the entire simulation run
    @param messageName The name of the message to find pairs for
    @param procList procs to check for read/write pairs
*/
std::set<std::pair<long int, long int>> SimModel::getMessageExchangeData(std::string messageName,
     std::set<unsigned long> procList)
{
    std::set<std::pair<long int, long int>> returnPairs;
    bool messageFound = false;
    for(uint64_t i=0; i<SystemMessaging::GetInstance()->getProcessCount(); i++)
    {
        if(procList.find((uint64_t)i) == procList.end() && procList.size() > 0)
        {
            continue;
        }
        SystemMessaging::GetInstance()->
            selectMessageBuffer(i);
        int64_t messageID = SystemMessaging::GetInstance()->
            FindMessageID(messageName);
        if(messageID >= 0)
        {
            std::set<std::pair<long int, long int>> localPairs;
            localPairs = SystemMessaging::GetInstance()->
                getMessageExchangeData(messageID);
            returnPairs.insert(localPairs.begin(), localPairs.end());
            messageFound = true;
        }

    }

    if(!messageFound)
    {
        bskLogger.bskLog(BSK_WARNING, "I couldn't find a message with the name:"
                                     " %s Can't give you exchange pairs for it.", messageName.c_str());
    }
    return(returnPairs);

}
