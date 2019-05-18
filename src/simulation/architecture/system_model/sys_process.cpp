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
    this->nextTaskTime = 0;
    this->processActive = true;
    this->processPriority = -1;
    this->disableProcess();
}
/*! Make a process AND attach a storage bucket with the provided name. Give
 * the process the same name.
 @return void
 @param std::string messagecontainer The amount of nanoseconds between calls to this Task.
 */
SysProcess::SysProcess(std::string messageContainer)
{
    this->nextTaskTime = 0;
    this->processActive = true;
    this->processName = messageContainer;
    this->messageBuffer = SystemMessaging::GetInstance()->
        AttachStorageBucket(messageContainer);
    SystemMessaging::GetInstance()->ClearMessageBuffer();
    this->prevRouteTime = 0xFF;
    this->disableProcess();
}

/*! The destructor does nothing.
 @return void
 */
SysProcess::~SysProcess()
{
}

/*! This method sets the nextTaskTime = 0 and calls SelfInitTaskList() for
 * all process tasks.
 @return void
 */
void SysProcess::selfInitProcess()
{
    std::vector<ModelScheduleEntry>::iterator it;
    SystemMessaging::GetInstance()->selectMessageBuffer(this->messageBuffer);
    this->nextTaskTime = 0;
    //! - Iterate through model list and call the Task model self-initializer
    for(it = this->processTasks.begin(); it != this->processTasks.end(); it++)
    {
        SysModelTask *localTask = it->TaskPtr;
        localTask->SelfInitTaskList();
    }
}

/*! This method asks all process tasks to CrossInitTaskLists.
 @return void
 */
void SysProcess::crossInitProcess()
{
    std::vector<ModelScheduleEntry>::iterator it;
    SystemMessaging::GetInstance()->selectMessageBuffer(this->messageBuffer);
    for(it = this->processTasks.begin(); it != this->processTasks.end(); it++)
    {
        SysModelTask *localTask = it->TaskPtr;
        localTask->CrossInitTaskList();
    }
    return;
}

/*! This method resets each task and associated model-set inside the process 
    ensuring that all parameters go back to their default state.
    @return void
    @param uint64_t currentTime Current simulation time in ns that reset is occurring at
*/
void SysProcess::resetProcess(uint64_t currentTime)
{
    std::vector<ModelScheduleEntry>::iterator it;
    SystemMessaging::GetInstance()->selectMessageBuffer(this->messageBuffer);
    for(it = this->processTasks.begin(); it != this->processTasks.end(); it++)
    {
        SysModelTask *localTask = it->TaskPtr;
        localTask->ResetTaskList(currentTime); //! Time of reset. Models that utilize currentTime will start at this.
    }
    return;
}

/*! This method does two things: 1) resets the next task time for
 *  all process tasks to the first task time. 2) clears the process list
 *  and then adds everything back into the process with the correct priority.
    @return void
*/
void SysProcess::reInitProcess()
{
    std::vector<ModelScheduleEntry>::iterator it;
    std::vector<ModelScheduleEntry> taskPtrs;
    std::vector<ModelScheduleEntry>::iterator taskIt;
    SystemMessaging::GetInstance()->selectMessageBuffer(this->messageBuffer);
    for(it = this->processTasks.begin(); it != this->processTasks.end(); it++)
    {
        SysModelTask *localTask = it->TaskPtr;
        localTask->ResetTask();
    }
    taskPtrs = this->processTasks;
    this->processTasks.clear();
    for(taskIt = taskPtrs.begin(); taskIt != taskPtrs.end(); taskIt++)
    {
        this->addNewTask(taskIt->TaskPtr, taskIt->taskPriority);
    }
    return;
}

/*! This method steps the next task up to currentNanos
 * unless it isn't supposed to run yet.
 @return void
 */
void SysProcess::singleStepNextTask(uint64_t currentNanos)
{
    std::vector<ModelScheduleEntry>::iterator it;
    int32_t localPriority;

    //! - Check to make sure that there are models to be called.
    it = this->processTasks.begin();
    if(it == this->processTasks.end())
    {
        BSK_PRINT_BRIEF(MSG_WARNING, "Received a step command on sim that has no active Tasks.");
        return;
    }
    //! - If the requested time does not meet our next start time, just return
    if(it->NextTaskStart > currentNanos)
    {
        this->nextTaskTime = it->NextTaskStart;
        return;
    }
    //! - Call the next scheduled model, and set the time to its start
    if(currentNanos != this->prevRouteTime)
    {
        routeInterfaces();
        this->prevRouteTime = currentNanos;
    }
    SystemMessaging::GetInstance()->selectMessageBuffer(this->messageBuffer);
    SysModelTask *localTask = it->TaskPtr;
    localTask->ExecuteTaskList(currentNanos);
    
    //! - Erase the current call from the stack and schedule the next call
    localPriority = it->taskPriority;
    this->processTasks.erase(it);
    this->addNewTask(localTask, localPriority);
    
    //! - Figure out when we are going to be called next for scheduling purposes
    it = this->processTasks.begin();
    this->nextTaskTime = it->NextTaskStart;
}

/*! This method adds a new task into the T\task list.  Note that
 * taskPriority parameter is option as it defaults to -1 (lowest)
 @return void
 @param SysModelTask* newTask The new task that we are adding to the list
 @param int32_t taskPriority The selected priority of the task being added
 */
void SysProcess::addNewTask(SysModelTask *newTask, int32_t taskPriority)
{
    ModelScheduleEntry localEntry;
    localEntry.TaskPtr = newTask;
    localEntry.TaskUpdatePeriod = newTask->TaskPeriod;
    localEntry.NextTaskStart = newTask->NextStartTime;
    localEntry.taskPriority = taskPriority;
    this->scheduleTask(localEntry);
    this->enableProcess();
}

/*! This method is used to place the task from the caller into the correct
 place in the simulation schedule.  The transaction for this model is that
 the caller will set the correct parameters in the calling argument and that
 the simulation will faithfully schedule it.
 @return void
 @param ModelScheduleEntry& taskCall Pointer to a struct that contains start time and task handle.
 */
void SysProcess::scheduleTask(ModelScheduleEntry & taskCall)
{
    std::vector<ModelScheduleEntry>::iterator it;
    //! - Iteratre through all of the task models to find correct place
    for(it = this->processTasks.begin(); it != this->processTasks.end(); it++)
    {
        //! - If the next Task starts after new Task, pop it on just prior
        if(it->NextTaskStart > taskCall.NextTaskStart ||
           (it->NextTaskStart == taskCall.NextTaskStart &&
            taskCall.taskPriority > it->taskPriority))
        {
            this->processTasks.insert(it, taskCall);
            return;
        }
    }
    //! - Default case is to put the Task at the end of the schedule
    this->processTasks.push_back(taskCall);
}

/*! This method is used to ensure that all necessary input messages are routed 
    from their source buffer to this process' message buffer.
    It needs to be executed prior to dispatching the process' models
    @return void
*/
void SysProcess::routeInterfaces()
{
    std::vector<SysInterface *>::iterator it;
    for(it=this->intRefs.begin(); it!= this->intRefs.end(); it++)
    {
        (*it)->routeInputs(this->messageBuffer);
    }
}

/*! The name kind of says it all right?  It is a shotgun used to disable all of 
    a process' tasks.  It is handy for a FSW scheme where you have tons of tasks
    and you are really only turning one on at a time.
    @return void
*/
void SysProcess::disableAllTasks()
{
    std::vector<ModelScheduleEntry>::iterator it;
    //! - Iterate through all of the tasks to disable them
    for(it = this->processTasks.begin(); it != this->processTasks.end(); it++)
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
    std::vector<ModelScheduleEntry>::iterator it;
    //! - Iterate through all of the task models to disable them
    for(it = this->processTasks.begin(); it != this->processTasks.end(); it++)
    {
        it->TaskPtr->enableTask();
    }
}

/*! This method updates a specified task's period once it locates that task 
    in the list.  It will warn the user if a task is not found.
    @return void
	@param std::string taskName The name of the task you want to change period of
	@param uint64_t newPeriod the new number of nanoseconds you want between calls
*/
void SysProcess::changeTaskPeriod(std::string taskName, uint64_t newPeriod)
{
	std::vector<ModelScheduleEntry>::iterator it;
	//! - Iteratre through all of the task models to disable them
	for (it = this->processTasks.begin(); it != this->processTasks.end(); it++)
	{
		if (it->TaskPtr->TaskName == taskName)
		{
			it->TaskPtr->updatePeriod(newPeriod);
			ModelScheduleEntry localEntry;
			localEntry.TaskPtr = it->TaskPtr;
			localEntry.TaskUpdatePeriod = it->TaskPtr->TaskPeriod;
			localEntry.NextTaskStart = it->TaskPtr->NextStartTime;
			localEntry.taskPriority = it->taskPriority;
            this->processTasks.erase(it);
            this->scheduleTask(localEntry);
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

