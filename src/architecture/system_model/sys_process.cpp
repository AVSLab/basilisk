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

#include "sys_process.h"
#include <iostream>
#include <utility>

/*! Make a process AND attach a storage bucket with the provided name. Give
    the process the same name.
 */
SysProcess::SysProcess(std::string name) : SysProcess()
{
    this->processName = std::move(name);
}

/*! This method sets the nextTaskTime = 0 and calls SelfInitTaskList() for
 * all process tasks.
 */
void SysProcess::selfInitProcess()
{

    this->nextTaskTime = 0;
    //! - Iterate through model list and call the Task model self-initializer
    for(auto const& process : this->processTasks)
    {
        SysModelTask *localTask = process.TaskPtr;
        localTask->SelfInitTaskList();
    }
}



/*! This method resets each task and associated model-set inside the process
    ensuring that all parameters go back to their default state.
    @param currentTime Current simulation time in ns that reset is occurring at
*/
void SysProcess::resetProcess(uint64_t currentTime)
{

    for(auto const& process : this->processTasks)
    {
        SysModelTask *localTask = process.TaskPtr;
        localTask->ResetTaskList(currentTime); //! Time of reset. Models that utilize currentTime will start at this.
    }
    this->nextTaskTime = currentTime;
}

/*! This method does two things: 1) resets the next task time for
 *  all process tasks to the first task time. 2) clears the process list
 *  and then adds everything back into the process with the correct priority.
*/
void SysProcess::reInitProcess()
{

    for(auto const& task : this->processTasks)
    {
        SysModelTask *localTask = task.TaskPtr;
        localTask->ResetTask();
    }
    std::vector<ModelScheduleEntry> taskPtrs = this->processTasks;
    this->processTasks.clear();
    for(auto const& task : taskPtrs)
    {
        this->addNewTask(task.TaskPtr, task.taskPriority);
    }
}

/*! This method steps the next task up to currentNanos
 * unless it isn't supposed to run yet.
 */
void SysProcess::singleStepNextTask(uint64_t currentNanos)
{

    //! - Check to make sure that there are models to be called.
    if(this->processTasks.begin() == this->processTasks.end())
    {
        bskLogger.bskLog(BSK_WARNING, "Received a step command on sim that has no active Tasks.");
        return;
    }
    auto fireIt = this->processTasks.begin();
    //! - If the requested time does not meet our next start time, just return
    for(auto it=this->processTasks.begin(); it!=this->processTasks.end(); it++)
    {
        if(it->NextTaskStart < fireIt->NextTaskStart ||
                (it->NextTaskStart==fireIt->NextTaskStart && it->taskPriority > fireIt->taskPriority))
        {
            fireIt = it;
        }
    }
    if(fireIt->NextTaskStart > currentNanos)
    {
        this->nextTaskTime = fireIt->NextTaskStart;
        return;
    }
    //! - Call the next scheduled model, and set the time to its start
    SysModelTask *localTask = fireIt->TaskPtr;
    localTask->ExecuteTaskList(currentNanos);
    fireIt->NextTaskStart = localTask->NextStartTime;

    //! - Figure out when we are going to be called next for scheduling purposes
    fireIt=this->processTasks.begin();
    //! - If the requested time does not meet our next start time, just return
    for(auto it=this->processTasks.begin(); it!=this->processTasks.end(); it++)
    {
        if(it->NextTaskStart < fireIt->NextTaskStart ||
           (it->NextTaskStart==fireIt->NextTaskStart && it->taskPriority > fireIt->taskPriority))
        {
            fireIt = it;
        }
    }
    this->nextTaskTime = fireIt->NextTaskStart;
}

/*! This method adds a new task into the Task list.  Note that
 * taskPriority parameter is option as it defaults to -1 (lowest)
 @param newTask The new task that we are adding to the list
 @param taskPriority The selected priority of the task being added
 */
void SysProcess::addNewTask(SysModelTask *newTask, int32_t taskPriority)
{
    ModelScheduleEntry localEntry;
    localEntry.TaskPtr = newTask;
    localEntry.TaskUpdatePeriod = newTask->TaskPeriod;
    localEntry.NextTaskStart = newTask->NextStartTime;
    localEntry.taskPriority = taskPriority;
    this->scheduleTask(localEntry);
    newTask->updateParentProc(processName);
    this->enableProcess();
}

/*! This method is used to place the task from the caller into the correct
 place in the simulation schedule.  The transaction for this model is that
 the caller will set the correct parameters in the calling argument and that
 the simulation will faithfully schedule it.
 @param taskCall Pointer to a struct that contains start time and task handle.
 */
void SysProcess::scheduleTask(const ModelScheduleEntry& taskCall)
{
    //! - Iteratre through all of the task models to find correct place
    for(auto it = this->processTasks.begin(); it != this->processTasks.end(); it++)
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

/*! The name kind of says it all right?  It is a shotgun used to disable all of
    a process' tasks.  It is handy for a FSW scheme where you have tons of tasks
    and you are really only turning one on at a time.
*/
void SysProcess::disableAllTasks() const
{
    //! - Iterate through all of the tasks to disable them
    for(auto const& scheduleEntry : this->processTasks)
    {
        scheduleEntry.TaskPtr->disableTask();
    }
}
/*! The name kind of says it all right?  It is a shotgun used to enable all of
 a processes tasks.  It is handy for a process that starts out almost entirely
 inhibited but you want to turn it all on at once.
 */
void SysProcess::enableAllTasks() const
{
    //! - Iterate through all of the task models to disable them
    for(auto const& scheduleEntry : this->processTasks)
    {
        scheduleEntry.TaskPtr->enableTask();
    }
}

/*! This method updates a specified task's period once it locates that task
    in the list.  It will warn the user if a task is not found.
	@param taskName The name of the task you want to change period of
	@param newPeriod the new number of nanoseconds you want between calls
*/
void SysProcess::changeTaskPeriod(std::string const& taskName, uint64_t newPeriod)
{
	//! - Iteratre through all of the task models to disable them
    for (ModelScheduleEntry &scheduleEntry : this->processTasks)
	{
        if (scheduleEntry.TaskPtr->TaskName == taskName)
		{
            scheduleEntry.TaskPtr->updatePeriod(newPeriod);
            scheduleEntry.NextTaskStart = scheduleEntry.TaskPtr->NextStartTime;
            scheduleEntry.TaskUpdatePeriod = scheduleEntry.TaskPtr->TaskPeriod;
			return;
		}
	}
    bskLogger.bskLog(BSK_WARNING, "You attempted to change the period of task: %s I couldn't find that in process: %s", taskName.c_str(), this->processName.c_str());
}
