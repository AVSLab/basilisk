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

#include "sys_model_task.h"

/*! A construction option that allows the user to set some task parameters.
 Note that the only required argument is InputPeriod.
 @param InputPeriod The amount of nanoseconds between calls to this Task.
 @param FirstStartTime The amount of time in nanoseconds to hold a task dormant before starting.
        After this time the task is executed at integer amounts of InputPeriod again
 */
SysModelTask::SysModelTask(uint64_t InputPeriod, uint64_t FirstStartTime) :
    NextStartTime(FirstStartTime), TaskPeriod(InputPeriod), FirstTaskTime(FirstStartTime)
{
    this->NextPickupTime = this->NextStartTime + this->TaskPeriod;
}

/*! This method self-initializes all of the models that have been added to the Task.
 */
void SysModelTask::SelfInitTaskList() const
{
    //! - Loop over all models and do the self init for each
    for(auto const& modelPair : this->TaskModels)
    {
        SysModel* NonIt = modelPair.ModelPtr;
        NonIt->SelfInit();
    }
}


/*! This method resets all of the models that have been added to the Task at the CurrentSimTime.
 See sys_model_task.h for related method ResetTask()
 @param CurrentSimTime The time to start at after reset
*/
void SysModelTask::ResetTaskList(uint64_t CurrentSimTime)
{
	for (auto const& modelPair : this->TaskModels)
	{
		modelPair.ModelPtr->Reset(CurrentSimTime);
	}
	this->NextStartTime = CurrentSimTime;
    this->NextPickupTime = this->NextStartTime + this->TaskPeriod;
}

/*! This method executes all of the models on the Task during runtime.
 Then, it sets its NextStartTime appropriately.
 @param CurrentSimNanos The current simulation time in [ns]
 */
void SysModelTask::ExecuteTaskList(uint64_t CurrentSimNanos)
{
    //! - Loop over all of the models in the simulation and call their UpdateState
    for(auto ModelPair = this->TaskModels.begin(); (ModelPair != this->TaskModels.end() && this->taskActive);
             ModelPair++)
    {
        SysModel* NonIt = (ModelPair->ModelPtr);
        NonIt->UpdateState(CurrentSimNanos);
        NonIt->CallCounts += 1;
    }
    //! - NextStartTime is set to allow the scheduler to fit the next call in
    this->NextStartTime += this->TaskPeriod;
}

/*! This method adds a new model into the Task list.  Note that the Priority
 parameter is option as it defaults to -1 (lowest, latest)
 @param NewModel The new model that we are adding to the Task
 @param Priority The selected priority of the model being added (highest goes first)
 */
void SysModelTask::AddNewObject(SysModel *NewModel, int32_t Priority)
{
    ModelPriorityPair LocalPair;

    //! - Set the local pair with the requested priority and mode
    LocalPair.CurrentModelPriority = Priority;
    LocalPair.ModelPtr = NewModel;
//    SystemMessaging::GetInstance()->addModuleToProcess(NewModel->moduleID,
//            parentProc);
    //! - Loop through the ModelPair vector and if Priority is higher than next, insert
    for(auto ModelPair = this->TaskModels.begin(); ModelPair != this->TaskModels.end();
        ModelPair++)
    {
        if(Priority > ModelPair->CurrentModelPriority)
        {
            this->TaskModels.insert(ModelPair, LocalPair);
            return;
        }
    }
    //! - If we make it to the end of the loop, this is lowest priority, put it at end
    this->TaskModels.push_back(LocalPair);
}

/*! This method changes the period of a given task over to the requested period.
   It attempts to keep the same offset relative to the original offset that
   was specified at task creation.
 @param newPeriod The period that the task should run at going forward
 */
void SysModelTask::updatePeriod(uint64_t newPeriod)
{
    //! - If the requested time is above the min time, set the next time based on the previous time plus the new period
    if(this->NextStartTime > this->TaskPeriod)
    {
        uint64_t newStartTime = (this->NextStartTime/newPeriod)*newPeriod;
        if(newStartTime <= (this->NextStartTime - this->TaskPeriod))
        {
            newStartTime += newPeriod;
        }
        this->NextStartTime = newStartTime;
    }
    //! - Otherwise, we just should keep the original requested first call time for the task
    else
    {
        this->NextStartTime = this->FirstTaskTime;
    }
    //! - Change the period of the task so that future calls will be based on the new period
    this->TaskPeriod = newPeriod;

}
