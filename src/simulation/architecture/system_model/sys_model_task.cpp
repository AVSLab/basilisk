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

#include "architecture/system_model/sys_model_task.h"
#include <cstring>
#include <iostream>

/*! The task constructor.  */
SysModelTask :: SysModelTask()
{
    TaskModels.clear();
    TaskName.clear();
    TaskPeriod = 1000;
    NextStartTime = 0;
    NextPickupTime = 0;
    PickupDelay = 0;
    FirstTaskTime = 0;
	taskActive = true;
    
}
/*! A construction option that allows the user to set all Task parameters.
 Note that the only required argument is InputPeriod.
 @param InputPeriod The amount of nanoseconds between calls to this Task.
 @param InputDelay How long to delay the input by in nanoseconds
 @param FirstStartTime The offset in a given frame to start the Task with.
 */
SysModelTask :: SysModelTask(uint64_t InputPeriod, uint64_t InputDelay,
                                 uint64_t FirstStartTime)
{
    TaskPeriod = InputPeriod;
    PickupDelay = InputDelay;
    NextStartTime = FirstStartTime;
    NextPickupTime = NextStartTime + TaskPeriod;
    FirstTaskTime = FirstStartTime;
	taskActive = true;
}

//! The destructor.  Everything is handled by STL.
SysModelTask :: ~SysModelTask()
{
    
}

/*! This method self-initializes all of the models that have been added to the Task.
 @return void
 */
void SysModelTask::SelfInitTaskList()
{
    std::vector<ModelPriorityPair>::iterator ModelPair;
    SysModel* NonIt;
    
    //! BEgin method steps
    //! - Loop over all models and do the self init for each
    for(ModelPair = TaskModels.begin(); ModelPair != TaskModels.end();
        ModelPair++)
    {
        NonIt = (ModelPair->ModelPtr);
        NonIt->SelfInit();
    }
    return;
}

/*! This method cross-initializes all of the models that have been added to the Task.
 @return void
 */
void SysModelTask::CrossInitTaskList()
{
    std::vector<ModelPriorityPair>::iterator ModelPair;
    SysModel* NonIt;
    
    //! BEgin method steps
    //! - Loop over all of the models and do the CrossInit
    for(ModelPair = TaskModels.begin(); ModelPair != TaskModels.end();
        ModelPair++)
    {
        NonIt = (ModelPair->ModelPtr);
        NonIt->CrossInit();
    }
    return;
}

/*! This method resets all of the models that have been added to the Task.
@return void
*/
void SysModelTask::ResetTaskList(uint64_t CurrentSimTime)
{

	std::vector<ModelPriorityPair>::iterator ModelPair;
	for (ModelPair = TaskModels.begin(); ModelPair != TaskModels.end();
	ModelPair++)
	{
		(*ModelPair).ModelPtr->Reset(CurrentSimTime);
	}

}

/*! This method executes all of the models on the Task during runtime.
 Once they have all been iterated through it sets itself up for scheduling
 @return void
 @param CurrentSimNanos The current simulation time
 */
void SysModelTask::ExecuteTaskList(uint64_t CurrentSimNanos)
{
    
    std::vector<ModelPriorityPair>::iterator ModelPair;
    SysModel* NonIt;
    
    //! Begin method steps
    //! - Loop over all of the models in the simulation and call their UpdateState
    for(ModelPair = TaskModels.begin(); (ModelPair != TaskModels.end() && taskActive);
        ModelPair++)
    {
        NonIt = (ModelPair->ModelPtr);
        NonIt->UpdateState(CurrentSimNanos);
        NonIt->CallCounts += 1;
    }
    //! - NextStartTime is set to allow the scheduler to fit the next call in
    NextStartTime += TaskPeriod;
}

/*! This method adds a new model into the Task list.  Note that the Priority
 parameter is option as it defaults to -1 (lowest)
 @return void
 @param NewModel The new model that we are adding to the Task
 @param Priority The selected priority of the model being added
 */
void SysModelTask::AddNewObject(SysModel *NewModel, int32_t Priority)
{
    std::vector<ModelPriorityPair>::iterator ModelPair;
    ModelPriorityPair LocalPair;
    
    //! Begin method steps
    //! - Set the local pair with the requested priority and mode
    LocalPair.CurrentModelPriority = Priority;
    LocalPair.ModelPtr = NewModel;
    //! - Loop through the ModelPair vector and if Priority is higher than next, insert
    for(ModelPair = TaskModels.begin(); ModelPair != TaskModels.end();
        ModelPair++)
    {
        if(Priority > ModelPair->CurrentModelPriority)
        {
            TaskModels.insert(ModelPair, LocalPair);
            return;
        }
    }
    //! - If we make it to the end of the loop, this is lowest priority, put it at end
    TaskModels.push_back(LocalPair);
}
