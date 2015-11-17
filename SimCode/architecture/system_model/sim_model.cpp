
#include "architecture/system_model/sim_model.h"
#include <cstring>
#include <iostream>

/*! This Constructor is used to initialize the top-level sim model.  It inits a
 couple of variables and then initializes the messaging system.  It only does
 that here because it needs to happen somewhere and the top-level sim model
 is a good place for it.
 */
SimModel::SimModel()
{
    CurrentNanos = 0;
    NextTaskTime = 0;
    
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
    messageIdentData MessageID;
    SingleMessageHeader DataHeader;
    
    //! Begin Method steps
    //! - Grab the message ID associated with name if it exists
    MessageID = SystemMessaging::GetInstance()->messagePublishSearch(MessageName);
    //! - If we got an invalid message ID back, alert the user and quit
    if(!MessageID.itemFound)
    {
        std::cerr << "You requested a message name: " << MessageName<<std::endl;
        std::cerr << "That message does not exist."<<std::endl;
        return(0);
    }
    //! - For valid message names, get the data buffer associated with message
    switch(logType)
    {
        case messageBuffer:
            SystemMessaging::GetInstance()->
                selectMessageBuffer(MessageID.processBuffer);
            SystemMessaging::GetInstance()->ReadMessage(MessageID.itemID, &DataHeader,
                                                        MaxSize, reinterpret_cast<uint8_t*> (MessageData), LatestOffset);
            break;
        case logBuffer:
            messageLogs.readLog(MessageID, &DataHeader,
                                MaxSize, reinterpret_cast<uint8_t*> (MessageData), LatestOffset);
            break;
        default:
            std::cout << "I don't know how to access the log type: "<<logType;
            std::cout << std::endl;
            break;
    }
    
    return(DataHeader.WriteClockNanos);
    
}

/*! This method allows the user to attach a process to the simulation for 
    execution.  Note that for a single time, processes will be executed in 
    the order they were added in
*/
void SimModel::addNewProcess(SysProcess *newProc)
{
    processList.push_back(newProc);
}

/*! This method goes through all of the Task models that have been added and
 calls those Tasks to init their lower level models.
 @return void
 */
void SimModel::InitSimulation()
{

    std::vector<SysProcess *>::iterator it;
    for(it=processList.begin(); it!= processList.end(); it++)
    {
        (*it)->selfInitProcess();
    }
    for(it=processList.begin(); it!= processList.end(); it++)
    {
        (*it)->crossInitProcess();
    }
    //! - If a message has been added to logger, link the message IDs
    if(!messageLogs.messagesLinked())
    {
        messageLogs.linkMessages();
    }
}
/*! This method steps all of the processes forward to the current time.  It also 
    increments the internal simulation time appropriately as the simulation 
    processes are triggered
    @return void
*/
void SimModel::SingleStepProcesses()
{
    uint64_t nextCallTime = ~0;
    std::vector<SysProcess *>::iterator it = processList.begin();
    while(it!= processList.end())
    {
        SysProcess *localProc = (*it);
        if(localProc->processEnabled())
        {
            while(localProc->nextTaskTime <= CurrentNanos)
            {
                localProc->singleStepNextTask(CurrentNanos);
            }
            if(localProc->getNextTime() < nextCallTime)
            {
                nextCallTime = localProc->getNextTime();
            }
        }
        it++;
    }
    CurrentNanos = nextCallTime != ~0 ? nextCallTime : CurrentNanos;
    messageLogs.logAllMessages();
}

/*! This method steps the simulation until the specified stop time has been
 reached.
 @return void
 @param SimStopTime Nanoseconds to step the simulation for
 */
void SimModel::StepUntilTime(uint64_t SimStopTime)
{
    //! Begin Method steps
    /*! - Note that we have to step until both the time is greater and the next
     Task's start time is in the future */
    while(CurrentNanos <= SimStopTime)
    {
        SingleStepProcesses();
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
        (*it)->resetProcess();
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
        std::cerr << "You tried to create a message in a process that doesn't exist.";
        std::cerr << "  No dice."<<std::endl;
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
    messageIdentData MessageID;
    
    //! Begin Method steps
    //! - Grab the message ID associated with name if it exists
    //! - Note that if you are trying to write  a shared message with no publisher things will get weird
    MessageID = SystemMessaging::GetInstance()->
        messagePublishSearch(MessageName);
    //! - If we got an invalid message ID back, alert the user and quit
    if(!MessageID.itemFound)
    {
        std::cerr << "You requested a message name: " << MessageName<<std::endl;
        std::cerr << "That message does not exist."<<std::endl;
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
    messageIdentData messageID = SystemMessaging::GetInstance()->
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
messageIdentData SimModel::getMessageID(std::string messageName)
{
    messageIdentData messageID = SystemMessaging::GetInstance()->
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
