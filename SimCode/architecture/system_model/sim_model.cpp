
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
   NextThreadTime = 0;
   MessageBucket.IncreaseStorage(20000); //! arbitrary init size, can be deltaed
   SystemMessaging::GetInstance()->AttachStorageBucket(&MessageBucket);
   SystemMessaging::GetInstance()->ClearMessageBuffer();
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
   void *MessageData, uint64_t LatestOffset)
{
   int64_t MessageID;
   SingleMessageHeader DataHeader;

   //! Begin Method steps
   //! - Grab the message ID associated with name if it exists
   MessageID = SystemMessaging::GetInstance()->FindMessageID(MessageName);
   //! - If we got an invalid message ID back, alert the user and quit
   if(MessageID < 0)
   {
      std::cerr << "You requested a message name: " << MessageName<<std::endl;
      std::cerr << "That message does not exist."<<std::endl;
      return(0);
   }
   //! - For valid message names, get the data buffer associated with message
   SystemMessaging::GetInstance()->ReadMessage(MessageID, &DataHeader, 
      MaxSize, reinterpret_cast<uint8_t*> (MessageData), LatestOffset);

   return(DataHeader.WriteClockNanos);
   
}

/*! This method is used to place the thread from the caller into the correct 
    place in the simulation schedule.  The transaction for this model is that 
    the caller will set the correct parameters in the calling argument and that 
    the simulation will faithfully schedule it.
    @return void
    @param ThreadCall Pointer to a struct that contains start time and thread handle.
    */
void SimModel::ScheduleThread(ModelScheduleEntry *ThreadCall)
{

   //! Begin Method steps
   std::vector<ModelScheduleEntry>::iterator it;
   //! - Iteratre through all of the thread models to find correct place
   for(it = ThreadModels.begin(); it != ThreadModels.end(); it++)
   {
      /// - If the next thread starts after new thread, pop it on just prior
      if(it->NextThreadStart > ThreadCall->NextThreadStart)
      {
         ThreadModels.insert(it, *ThreadCall);
         return;
      }
   }
   //! - Default case is to put the thread at the end of the schedule
   ThreadModels.push_back(*ThreadCall);
}

/*! This method adds a new thread to the system.  The main behavior of the code 
    is to take a handle reference and schedule it for its first call.  All other 
    subsequent calls are handled by the runtime logic.
    @return void
    @param NewThread Handle to the thread that is being added.
*/
void SimModel::AddNewThread(SysModelThread *NewThread)
{
   ModelScheduleEntry NewEntry;
   //! Begin Method steps
   //! - Initialize the entry that is being added and call scheduler.
   NewEntry.NextThreadStart = NewThread->NextStartTime;
   NewEntry.ThreadUpdatePeriod = NewThread->ThreadPeriod;
   NewEntry.ThreadPtr = NewThread;
   ScheduleThread(&NewEntry);
}

/*! This method goes through all of the thread models that have been added and 
    calls those threads to init their lower level models.
    @return void
*/
void SimModel::InitThreads()
{
   //! Begin Method steps
   std::vector<ModelScheduleEntry>::iterator it;
   //! - Iterate through model list and call the thread model self-initializer
   for(it = ThreadModels.begin(); it != ThreadModels.end(); it++)
   {
      SysModelThread *LocalThread = it->ThreadPtr;
      LocalThread->SelfInitThreadList();
   }
   //! - Iterate through model list and call the thread model cross-initializer
   for(it = ThreadModels.begin(); it != ThreadModels.end(); it++)
   {
      SysModelThread *LocalThread = it->ThreadPtr;
      LocalThread->CrossInitThreadList();
   }
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
         thread's start time is in the future */
   while(CurrentNanos < SimStopTime || NextThreadTime <= SimStopTime)
   {
      SingleStepNextThread();
   }
}
/*! This method is used to push the current simulation forward in time by the 
    next thread in the schedule.  It calls the next thread, schedules it 
    according to when it thinks it should be called next, and sets the current 
    simulation time information.
    @return void
*/
void SimModel::SingleStepNextThread()
{
   std::vector<ModelScheduleEntry>::iterator it;
   //! Begin Method steps
   //! - Check to make sure that there are models to be called.
   it = ThreadModels.begin();
   if(it == ThreadModels.end())
   {
      std::cerr << "Received a step command on sim that has no active threads.";
      std::cerr << std::endl;
      return;
   }
   //! - Call the next scheduled model, and set the time to its start
   SysModelThread *LocalThread = it->ThreadPtr;
   CurrentNanos = it->NextThreadStart;
   LocalThread->ExecuteThreadList(CurrentNanos);

   //! - Erase the current call from the stack and schedule the next call
   ThreadModels.erase(it);
   AddNewThread(LocalThread);

   //! - Figure out when we are going to be called next for scheduling purposes
   it = ThreadModels.begin();
   NextThreadTime = it->NextThreadStart;
   
}

void SimModel::ResetSimulation()
{
   std::vector<ModelScheduleEntry>::iterator it;
   //! - Iterate through model list and call the thread model initializer
   for(it = ThreadModels.begin(); it != ThreadModels.end(); it++)
   {
      it->ThreadPtr->ResetThread();
      it->NextThreadStart = it->ThreadPtr->FirstThreadTime;
   }
   CurrentNanos = 0;
   NextThreadTime = 0;
}

/*! This method exists to provide a hook into the messaging system for creating
    messages for use by the simulation
    @return void
    @param MessageName String name for the message we are querying
    @param MessageSize Maximum size of the message that we can pull
    @param NumBuffers The count of message buffers to create*/
void SimModel::CreateNewMessage(std::string MessageName, uint64_t MessageSize, 
   uint64_t NumBuffers)
{
   SystemMessaging::GetInstance()->CreateNewMessage(MessageName, MessageSize, 
      NumBuffers);
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
   int64_t MessageID;

   //! Begin Method steps
   //! - Grab the message ID associated with name if it exists
   MessageID = SystemMessaging::GetInstance()->FindMessageID(MessageName);
   //! - If we got an invalid message ID back, alert the user and quit
   if(MessageID < 0)
   {
      std::cerr << "You requested a message name: " << MessageName<<std::endl;
      std::cerr << "That message does not exist."<<std::endl;
      return;
   }
   SystemMessaging::GetInstance()->WriteMessage(MessageID, ClockTime, 
      MessageSize, reinterpret_cast<uint8_t*> (MessageData)); 
}
