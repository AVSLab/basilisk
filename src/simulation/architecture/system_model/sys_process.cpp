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

#include "architecture/system_model/sys_process.h"
#include <cstring>
#include <iostream>
#include "utilities/bsk_Print.h"

/*! The task constructor.  */
SysProcess :: SysProcess()
{
    nextTaskTime = 0;
    processActive = true;
    processPriority = -1;
    disableProcess();
}
/*! A construction option that allows the user to set all Task parameters.
 Note that the only required argument is InputPeriod.
 @param InputPeriod The amount of nanoseconds between calls to this Task.
 @param InputDelay How long to delay the input by in nanoseconds
 @param FirstStartTime The offset in a given frame to start the Task with.
 */
SysProcess::SysProcess(std::string messageContainer)
{
    nextTaskTime = 0;
    processActive = true;
    processName = messageContainer;
    messageBuffer = SystemMessaging::GetInstance()->
        AttachStorageBucket(messageContainer);
    SystemMessaging::GetInstance()->ClearMessageBuffer();
    prevRouteTime = 0xFF;
    disableProcess();
}

//! The destructor.  Everything is handled by STL.
SysProcess::~SysProcess()
{
    
}

/*! This method self-initializes all of the models that have been added to the Task.
 @return void
 */
void SysProcess::selfInitProcess()
{
    //! Begin Method steps
    std::vector<ModelScheduleEntry>::iterator it;
    SystemMessaging::GetInstance()->selectMessageBuffer(messageBuffer);
    nextTaskTime = 0;
    //! - Iterate through model list and call the Task model self-initializer
    for(it = processTasks.begin(); it != processTasks.end(); it++)
    {
        SysModelTask *localTask = it->TaskPtr;
        localTask->SelfInitTaskList();
    }
}

/*! This method cross-initializes all of the models that have been added to the Task.
 @return void
 */
void SysProcess::crossInitProcess()
{
    //! Begin Method steps
    std::vector<ModelScheduleEntry>::iterator it;
    SystemMessaging::GetInstance()->selectMessageBuffer(messageBuffer);
    for(it = processTasks.begin(); it != processTasks.end(); it++)
    {
        SysModelTask *localTask = it->TaskPtr;
        localTask->CrossInitTaskList();
    }
    
    return;
}

/*! This method resets each task and associated model-set inside the process 
    ensuring that all parameters go back to their default state.
    @return void
    @param currentTime [ns] Current simulation time that reset is occurring at
*/
void SysProcess::resetProcess(uint64_t currentTime)
{
    //! Begin Method steps
    std::vector<ModelScheduleEntry>::iterator it;
    SystemMessaging::GetInstance()->selectMessageBuffer(messageBuffer);
    for(it = processTasks.begin(); it != processTasks.end(); it++)
    {
        SysModelTask *localTask = it->TaskPtr;
        localTask->ResetTaskList(currentTime); //! Time of reset is zero as we are starting over
    }
    
    return;
}

/*! This method re-initializes the process and the various tasks contained inside 
    of it to zero-time for cases when a simulation is re-run or another sim is 
    started using a previously-run set of simulation architecture.
    @return void
*/
void SysProcess::reInitProcess()
{
    //! Begin Method steps
    std::vector<ModelScheduleEntry>::iterator it;
    std::vector<ModelScheduleEntry> taskPtrs;
    std::vector<ModelScheduleEntry>::iterator taskIt;
    SystemMessaging::GetInstance()->selectMessageBuffer(messageBuffer);
    for(it = processTasks.begin(); it != processTasks.end(); it++)
    {
        SysModelTask *localTask = it->TaskPtr;
        localTask->ResetTask();
    }
    taskPtrs = processTasks;
    processTasks.clear();
    for(taskIt = taskPtrs.begin(); taskIt != taskPtrs.end(); taskIt++)
    {
        addNewTask(taskIt->TaskPtr, taskIt->taskPriority);
    }
    
    
    return;
}

void SysProcess::singleStepNextTask(uint64_t currentNanos)
{
    std::vector<ModelScheduleEntry>::iterator it;
    int32_t localPriority;
    //! Begin Method steps
    //! - Check to make sure that there are models to be called.
    it = processTasks.begin();
    if(it == processTasks.end())
    {
        BSK_PRINT_BRIEF(MSG_WARNING, "Received a step command on sim that has no active Tasks.");
        return;
    }
    //! - If the requested time does not meet our next start time, just return
    if(it->NextTaskStart > currentNanos)
    {
        nextTaskTime = it->NextTaskStart;
        return;
    }
    //! - Call the next scheduled model, and set the time to its start
    if(currentNanos != prevRouteTime)
    {
        routeInterfaces();
        prevRouteTime = currentNanos;
    }
    SystemMessaging::GetInstance()->selectMessageBuffer(messageBuffer);
    SysModelTask *localTask = it->TaskPtr;
    localTask->ExecuteTaskList(currentNanos);
    
    //! - Erase the current call from the stack and schedule the next call
    localPriority = it->taskPriority;
    processTasks.erase(it);
    addNewTask(localTask, localPriority);
    
    //! - Figure out when we are going to be called next for scheduling purposes
    it = processTasks.begin();
    nextTaskTime = it->NextTaskStart;
}

/*! This method adds a new model into the Task list.  Note that the Priority
 parameter is option as it defaults to -1 (lowest)
 @return void
 @param NewModel The new model that we are adding to the Task
 @param Priority The selected priority of the model being added
 */
void SysProcess::addNewTask(SysModelTask *newTask, int32_t taskPriority)
{
    ModelScheduleEntry localEntry;
    localEntry.TaskPtr = newTask;
    localEntry.TaskUpdatePeriod = newTask->TaskPeriod;
    localEntry.NextTaskStart = newTask->NextStartTime;
    localEntry.taskPriority = taskPriority;
    scheduleTask(localEntry);
    enableProcess();
}

/*! This method is used to place the task from the caller into the correct
 place in the simulation schedule.  The transaction for this model is that
 the caller will set the correct parameters in the calling argument and that
 the simulation will faithfully schedule it.
 @return void
 @param TaskCall Pointer to a struct that contains start time and task handle.
 */
void SysProcess::scheduleTask(ModelScheduleEntry & taskCall)
{
    
    //! Begin Method steps
    std::vector<ModelScheduleEntry>::iterator it;
    //! - Iteratre through all of the task models to find correct place
    for(it = processTasks.begin(); it != processTasks.end(); it++)
    {
        //! - If the next Task starts after new Task, pop it on just prior
        if(it->NextTaskStart > taskCall.NextTaskStart ||
           (it->NextTaskStart == taskCall.NextTaskStart &&
            taskCall.taskPriority > it->taskPriority))
        {
            processTasks.insert(it, taskCall);
            return;
        }
    }
    //! - Default case is to put the Task at the end of the schedule
    processTasks.push_back(taskCall);
}

/*! This method is used to ensure that all necessary input messages are routed 
    from their source buffer to the process' message buffer.  It needs to be 
    executed prior to dispatching the process' models
    @return void
*/
void SysProcess::routeInterfaces()
{
    std::vector<SysInterface *>::iterator it;
    for(it=intRefs.begin(); it!= intRefs.end(); it++)
    {
        (*it)->routeInputs(messageBuffer);
    }
}

/*! The name kind of says it all right?  It is a shotgun used to disable all of 
    a process' tasks.  It is handy for a FSW scheme where you have tons of tasks
    and you are really only turning one on at a time.
    @return void
*/
void SysProcess::disableAllTasks()
{
    //! Begin Method steps
    std::vector<ModelScheduleEntry>::iterator it;
    //! - Iteratre through all of the task models to disable them
    for(it = processTasks.begin(); it != processTasks.end(); it++)
    {
        it->TaskPtr->disableTask();
    }
}
/*! The name kind of says it all right?  It is a shotgun used to enable all of
 a processes tasks.  It is handy for a process that starts out almost entirely 
 inhibited but you want to turn it all on at once.
 @return void
 */
void SysProcess::enableAllTasks()
{
    //! Begin Method steps
    std::vector<ModelScheduleEntry>::iterator it;
    //! - Iteratre through all of the task models to disable them
    for(it = processTasks.begin(); it != processTasks.end(); it++)
    {
        it->TaskPtr->enableTask();
    }
}

/*! This method updates a specified task's period once it locates that task 
    in the list.  It will warn the user if a task is not found and will then 
	seed the task into the task-list appropriately.
	@param taskName The name of the task you want to change period of
	@param newPeriod the new number of nanoseconds you want between calls
    @return void
*/
void SysProcess::changeTaskPeriod(std::string taskName, uint64_t newPeriod)
{
	//! Begin Method steps
	std::vector<ModelScheduleEntry>::iterator it;
	//! - Iteratre through all of the task models to disable them
	for (it = processTasks.begin(); it != processTasks.end(); it++)
	{
		if (it->TaskPtr->TaskName == taskName)
		{
			it->TaskPtr->updatePeriod(newPeriod);
			ModelScheduleEntry localEntry;
			localEntry.TaskPtr = it->TaskPtr;
			localEntry.TaskUpdatePeriod = it->TaskPtr->TaskPeriod;
			localEntry.NextTaskStart = it->TaskPtr->NextStartTime;
			localEntry.taskPriority = it->taskPriority;
			processTasks.erase(it);
			scheduleTask(localEntry);
			return;
		}
	}
    BSK_PRINT_BRIEF(MSG_WARNING, "You attempted to change the period of task: %s I couldn't find that in process: %s", taskName.c_str(), this->processName.c_str());
}

//void SysProcess::getAllMessageDefinitions()
//{
//    SystemMessaging *messageSys = SystemMessaging::GetInstance();
//    std::set<std::string>::iterator it;
//    messageSys->selectMessageBuffer(this->messageBuffer);
//    for(it=unknownPublisher.begin(); it!=unknownPublisher.end(); it++)
//    {
//        int64_t messageID = messageSys->FindMessageID(*it);
//        if(messageID >= 0)
//        {
//            MessageInterfaceMatch newMessage;
//            newMessage.source = -1;
//            newMessage.destination = -1;
//            newMessage.messageSource = *it;
//            newMessage.messageDest = "";
//            newMessage.updateCounter = 0;
//            messageTraffic.push_back(newMessage);
//        }
//    }
//}

