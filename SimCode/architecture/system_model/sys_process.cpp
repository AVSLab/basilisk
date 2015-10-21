
#include "architecture/system_model/sys_process.h"
#include "architecture/messaging/system_messaging.h"
#include <cstring>
#include <iostream>

/*! The task constructor.  */
SysProcess :: SysProcess()
{
    nextTaskTime = 0;
    processActive = true;
}
/*! A construction option that allows the user to set all Task parameters.
 Note that the only required argument is InputPeriod.
 @param InputPeriod The amount of nanoseconds between calls to this Task.
 @param InputDelay How long to delay the input by in nanoseconds
 @param FirstStartTime The offset in a given frame to start the Task with.
 */
SysProcess :: SysProcess(std::string messageContainer)
{
    nextTaskTime = 0;
    processActive = true;
    processName = messageContainer;
    messageBuffer = SystemMessaging::GetInstance()->
        AttachStorageBucket(messageContainer);
    SystemMessaging::GetInstance()->ClearMessageBuffer();
}

//! The destructor.  Everything is handled by STL.
SysProcess :: ~SysProcess()
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
    //! - Iterate through model list and call the Task model self-initializer
    for(it = taskModels.begin(); it != taskModels.end(); it++)
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
    for(it = taskModels.begin(); it != taskModels.end(); it++)
    {
        SysModelTask *localTask = it->TaskPtr;
        localTask->CrossInitTaskList();
    }
    
    return;
}

void SysProcess::resetProcess()
{
    //! Begin Method steps
    std::vector<ModelScheduleEntry>::iterator it;
    std::vector<SysModelTask *> taskPtrs;
    std::vector<SysModelTask *>::iterator taskIt;
    SystemMessaging::GetInstance()->selectMessageBuffer(messageBuffer);
    for(it = taskModels.begin(); it != taskModels.end(); it++)
    {
        SysModelTask *localTask = it->TaskPtr;
        localTask->ResetTask();
        taskPtrs.push_back(localTask);
    }
    taskModels.clear();
    for(taskIt = taskPtrs.begin(); taskIt != taskPtrs.end(); taskIt++)
    {
        addNewTask(*(taskIt));
    }
    
    
    return;
}

void SysProcess::singleStepNextTask(uint64_t currentNanos)
{
    std::vector<ModelScheduleEntry>::iterator it;
    //! Begin Method steps
    //! - Check to make sure that there are models to be called.
    SystemMessaging::GetInstance()->selectMessageBuffer(messageBuffer);
    it = taskModels.begin();
    if(it == taskModels.end())
    {
        std::cerr << "Received a step command on sim that has no active Tasks.";
        std::cerr << std::endl;
        return;
    }
    //! - If the requested time does not meet our next start time, just return
    if(it->NextTaskStart > currentNanos)
    {
        nextTaskTime = it->NextTaskStart;
        return;
    }
    //! - Call the next scheduled model, and set the time to its start
    SysModelTask *localTask = it->TaskPtr;
    localTask->ExecuteTaskList(currentNanos);
    
    //! - Erase the current call from the stack and schedule the next call
    taskModels.erase(it);
    addNewTask(localTask);
    
    //! - Figure out when we are going to be called next for scheduling purposes
    it = taskModels.begin();
    nextTaskTime = it->NextTaskStart;
}

/*! This method adds a new model into the Task list.  Note that the Priority
 parameter is option as it defaults to -1 (lowest)
 @return void
 @param NewModel The new model that we are adding to the Task
 @param Priority The selected priority of the model being added
 */
void SysProcess::addNewTask(SysModelTask *newTask)
{
    ModelScheduleEntry localEntry;
    localEntry.TaskPtr = newTask;
    localEntry.TaskUpdatePeriod = newTask->TaskPeriod;
    localEntry.NextTaskStart = newTask->NextStartTime;
    scheduleTask(localEntry);
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
    for(it = taskModels.begin(); it != taskModels.end(); it++)
    {
        /// - If the next Task starts after new Task, pop it on just prior
        if(it->NextTaskStart > taskCall.NextTaskStart)
        {
            taskModels.insert(it, taskCall);
            return;
        }
    }
    //! - Default case is to put the Task at the end of the schedule
    taskModels.push_back(taskCall);
}

