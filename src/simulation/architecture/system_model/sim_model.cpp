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

#include "architecture/system_model/sim_model.h"
#include <cstring>
#include <iostream>
#include "utilities/bsk_Print.h"

/*! This Constructor is used to initialize the top-level sim model.  It inits a
 couple of variables and then initializes the messaging system.  It only does
 that here because it needs to happen somewhere and the top-level sim model
 is a good place for it.
 */
SimModel::SimModel()
{
    CurrentNanos = 0;
    NextTaskTime = 0;
    nextProcPriority = -1;
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
 @param MaxSize Maximum size of the message that we can pull
 @param MessageData A shapeshifting buffer that we can chunk data into
 @param LatestOffset An offset from the latest message to pull (default as zero)*/
uint64_t SimModel::GetWriteData(std::string MessageName, uint64_t MaxSize,
                                void *MessageData, VarAccessType logType, uint64_t LatestOffset)
{
    MessageIdentData MessageID;
    SingleMessageHeader DataHeader;
    
    //! Begin Method steps
    //! - Grab the message ID associated with name if it exists
    MessageID = SystemMessaging::GetInstance()->messagePublishSearch(MessageName);
    //! - If we got an invalid message ID back, alert the user and quit
    if(!MessageID.itemFound)
    {
        BSK_PRINT_BRIEF(MSG_ERROR, "You requested a message name: %s that message does not exist.", MessageName.c_str());
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
            messageLogs.readLog(MessageID, &DataHeader,
                                MaxSize, reinterpret_cast<uint8_t*> (MessageData), LatestOffset);
            break;
        default:
            BSK_PRINT_BRIEF(MSG_ERROR, "I don't know how to access the log type: %u", logType);
            break;
    }
    return(DataHeader.WriteClockNanos);
}

/*! This method allows the user to attach a process to the simulation for 
    execution.  Note that the priority level of the process determines what 
    order it gets called in.  Otherwise it's first-in, first-out
    @return void
*/
void SimModel::addNewProcess(SysProcess *newProc)
{
    std::vector<SysProcess *>::iterator it;
    for(it = processList.begin(); it != processList.end(); it++)
    {
        if(newProc->processPriority > (*it)->processPriority)
        {
            processList.insert(it, newProc);
            return;
        }
    }
    processList.push_back(newProc);
}

/*! This method goes through all of the Task models that have been added and
 calls those Tasks to self-init their lower level models.
 @return void
 */
void SimModel::selfInitSimulation()
{
    std::vector<SysProcess *>::iterator it;
    for(it=processList.begin(); it!= processList.end(); it++)
    {
        (*it)->selfInitProcess();
    }
    if(SystemMessaging::GetInstance()->getFailureCount() > 0)
    {
        throw std::range_error("Message creation failed during self.  Please examine output.\n");
    }
    NextTaskTime = 0;
    CurrentNanos = 0;
    it=processList.begin();
    nextProcPriority = (*it)->processPriority;
}
/*! This method goes through all of the Task models that have been added and
 calls those Tasks to cross-init their lower level models.
 @return void
 */
void SimModel::crossInitSimulation()
{
    std::vector<SysProcess *>::iterator it;
    for(it=processList.begin(); it!= processList.end(); it++)
    {
        (*it)->crossInitProcess();
    }
    if(SystemMessaging::GetInstance()->getFailureCount() > 0)
    {
        throw std::range_error("Message creation failed during cross.  Please examine output.\n");
    }
    
}
/*! This method goes through all of the Task models that have been added and
 calls those Tasks to reset-init their lower level models.
 @return void
 */
void SimModel::resetInitSimulation()
{
    std::vector<SysProcess *>::iterator it;
    for(it=processList.begin(); it!= processList.end(); it++)
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
    uint64_t nextCallTime = ~0;
    std::vector<SysProcess *>::iterator it = processList.begin();
    CurrentNanos = NextTaskTime;
    while(it!= processList.end())
    {
        SysProcess *localProc = (*it);
        if(localProc->processEnabled())
        {
            while(localProc->nextTaskTime < CurrentNanos ||
                (localProc->nextTaskTime == CurrentNanos &&
                  localProc->processPriority >= stopPri))
            {
                localProc->singleStepNextTask(CurrentNanos);
            }
            if(localProc->getNextTime() < nextCallTime)
            {
                nextCallTime = localProc->getNextTime();
                nextProcPriority = localProc->processPriority;
            }
            else if(localProc->getNextTime() == nextCallTime &&
                localProc->processPriority > nextProcPriority)
            {
                nextProcPriority = localProc->processPriority;
            }
        }
        it++;
    }
    if(SystemMessaging::GetInstance()->getFailureCount() > 0)
    {
        throw std::range_error("Message reads or writes failed.  Please examine output.\n");
    }
    NextTaskTime = nextCallTime != ~0 ? nextCallTime : CurrentNanos;
    //! - If a message has been added to logger, link the message IDs
    if(!messageLogs.messagesLinked())
    {
        messageLogs.linkMessages();
    }
    messageLogs.logAllMessages();
}

/*! This method steps the simulation until the specified stop time and 
 stop priority have been reached.
 @return void
 @param SimStopTime Nanoseconds to step the simulation for
 @param stopPri The priority level below which the sim won't go
 */
void SimModel::StepUntilStop(uint64_t SimStopTime, int64_t stopPri)
{
    //! Begin Method steps
    /*! - Note that we have to step until both the time is greater and the next
     Task's start time is in the future */
    int64_t inPri = SimStopTime == NextTaskTime ? stopPri : -1;
    while(NextTaskTime < SimStopTime || (NextTaskTime == SimStopTime &&
        nextProcPriority >= stopPri) )
    {
        SingleStepProcesses(inPri);
        inPri = SimStopTime == NextTaskTime ? stopPri : -1;
    }
}
/*! This method is used to push the current simulation forward in time by the
 next Task in the schedule.  It calls the next Task, schedules it
 according to when it thinks it should be called next, and sets the current
 simulation time information.
 @return void
 */

void SimModel::ResetSimulation()
{
    std::vector<SysProcess *>::iterator it;
    //! - Iterate through model list and call the Task model initializer
    for(it = processList.begin(); it != processList.end(); it++)
    {
        (*it)->reInitProcess();
    }
    messageLogs.clearLogs();
    CurrentNanos = 0;
    NextTaskTime = 0;
}

/*! This method exists to provide a hook into the messaging system for creating
 messages for use by the simulation
 @return void
 @param MessageName String name for the message we are querying
 @param MessageSize Maximum size of the message that we can pull
 @param NumBuffers The count of message buffers to create*/
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
        BSK_PRINT_BRIEF(MSG_ERROR, "You tried to create a message in a process that doesn't exist. No dice.");
        throw std::range_error("Message creation failed.  Please examine output.\n");
    }
        
}

/*! This method exists to provide a hook into the messaging system for writing
 message data into existing messages
 @return void
 @param MessageName String name for the message we are querying
 @param MessageSize Maximum size of the message that we can pull
 @param MessageData A shapeshifting buffer that we can chunk data into
 @param ClockTime The time that the message was written to*/
void SimModel::WriteMessageData(std::string MessageName, uint64_t MessageSize,
                                uint64_t ClockTime, void *MessageData)
{
    MessageIdentData MessageID;
    
    //! Begin Method steps
    //! - Grab the message ID associated with name if it exists
    //! - Note that if you are trying to write  a shared message with no publisher things will get weird
    MessageID = SystemMessaging::GetInstance()->
        messagePublishSearch(MessageName);
    //! - If we got an invalid message ID back, alert the user and quit
    if(!MessageID.itemFound)
    {
        BSK_PRINT_BRIEF(MSG_ERROR, "You requested a message name: %s that message does not exist.", MessageName.c_str());
        return;
    }
    SystemMessaging::GetInstance()->selectMessageBuffer(MessageID.processBuffer);
    SystemMessaging::GetInstance()->WriteMessage(MessageID.itemID, ClockTime,
                                                 MessageSize, reinterpret_cast<uint8_t*> (MessageData));
}
/*! This method functions as a pass-through to the message logging structure
 when adding messages to log.  The main point is to serve as an API at the
 main simulation level without having to hook in the message logger
 somewhere else.
 @return void
 @param messageName -- The name of the message that we want to log
 @param messagePeriod ns The minimum time between messages that we want to allow
 */ 
void SimModel::logThisMessage(std::string messageName, uint64_t messagePeriod)
{
    messageLogs.addMessageLog(messageName, messagePeriod);
}

/*! This method gets the current number of messages that have been created in 
    the simulation.
    @return uint64_t The number of messages that have been created
*/
uint64_t SimModel::getNumMessages() {
    return(SystemMessaging::GetInstance()->GetMessageCount());
}
/*! This method finds the name associated with the message ID that is passed 
    in.
    @return std::string The message name for the ID
    @param messageID The message id that we wish to find the name for
*/
std::string SimModel::getMessageName(uint64_t messageID)
{
    return(SystemMessaging::GetInstance()->FindMessageName(messageID));
}

/*! This method obtains the header information associated with a given message.
   Note the copy out to the incoming message.  The assumption is that this 
   method is called from the python level where the storage for headerOut is 
   created.  This way we don't connect a pointer to the internal messager at the 
   python level
   @return void
   @param messageID The ID we want to pull header information for
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

/*! This method find the ID associated with the message name and returns it to 
    the caller.  Mostly used to make sure a message is valid.
    @return int64_t ID of the message associated with messageName
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
    from the shadow messages
    @return set of strings that constitute the unique names
*/
std::set<std::string> SimModel::getUniqueMessageNames()
{
    std::set<std::string> outputSet;
    outputSet = SystemMessaging::GetInstance()->getUniqueMessageNames();
    return(outputSet);
}

/*! This method clears out the messaging interface so that the process can be 
    used again for another run.  Note that once you do this, the simulation 
    object itself is really dead.
    @return void
*/
void SimModel::terminateSimulation()
{
    SystemMessaging::GetInstance()->clearMessaging();
}

/*! This method returns all of the read/write pairs for the entire simulation 
    for a given message.  That alloww us to capture and analyze our data flow in
    a very clean manner.
    @return Write/Read pairs for the entire simulation run
    @param messageName The name of the message to find pairs for
*/
std::set<std::pair<long int, long int>> SimModel::getMessageExchangeData(std::string messageName,
     std::set<unsigned long> procList)
{
    std::set<std::pair<long int, long int>> returnPairs;
    bool messageFound = false;
    for(uint64_t i=0; i<SystemMessaging::GetInstance()->getProcessCount(); i++)
    {
        if(procList.find(i) == procList.end() && procList.size() > 0)
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
        BSK_PRINT_BRIEF(MSG_WARNING, "I couldn't find a message with the name: %s Can't give you exchange pairs for it.", messageName.c_str());
    }
    return(returnPairs);
    
}
