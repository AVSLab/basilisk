
#include "architecture/system_model/sys_model_thread.h"
#include <cstring>
#include <iostream>

/*! The thread constructor.  */
SysModelThread :: SysModelThread()
{
   ThreadModels.clear();
   ThreadName.clear();
   ThreadPeriod = 1000;
   NextStartTime = 0;
   NextPickupTime = 0;
   PickupDelay = 0;
   FirstThreadTime = 0;

}
/*! A construction option that allows the user to set all thread parameters.
    Note that the only required argument is InputPeriod.
    @param InputPeriod The amount of nanoseconds between calls to this thread.
    @param InputDelay How long to delay the input by in nanoseconds
    @param FirstStartTime The offset in a given frame to start the thread with.
*/
SysModelThread :: SysModelThread(uint64_t InputPeriod, uint64_t InputDelay, 
   uint64_t FirstStartTime)
{
   ThreadPeriod = InputPeriod;
   PickupDelay = InputDelay;
   NextStartTime = FirstStartTime;
   NextPickupTime = NextStartTime + ThreadPeriod;
   FirstThreadTime = FirstStartTime;
}

//! The destructor.  Everything is handled by STL.
SysModelThread :: ~SysModelThread()
{

}

/*! This method self-initializes all of the models that have been added to the thread.
    @return void
*/
void SysModelThread::SelfInitThreadList()
{
   std::vector<ModelPriorityPair>::iterator ModelPair;
   SysModel* NonIt;

   //! BEgin method steps
   //! - Loop over all models and do the self init for each
   for(ModelPair = ThreadModels.begin(); ModelPair != ThreadModels.end();
      ModelPair++)
   {
       NonIt = (ModelPair->ModelPtr);
       NonIt->SelfInit();
   }
   return;
}

/*! This method cross-initializes all of the models that have been added to the thread.
    @return void
*/
void SysModelThread::CrossInitThreadList()
{
   std::vector<ModelPriorityPair>::iterator ModelPair;
   SysModel* NonIt;

   //! BEgin method steps
   //! - Loop over all of the models and do the CrossInit
   for(ModelPair = ThreadModels.begin(); ModelPair != ThreadModels.end();
      ModelPair++)
   {
      NonIt = (ModelPair->ModelPtr);
      NonIt->CrossInit();
   }
   return;
}

/*! This method executes all of the models on the thread during runtime.
    Once they have all been iterated through it sets itself up for scheduling
    @return void
    @param CurrentSimNanos The current simulation time 
*/
void SysModelThread::ExecuteThreadList(uint64_t CurrentSimNanos)
{

   std::vector<ModelPriorityPair>::iterator ModelPair;
   SysModel* NonIt;

    //! Begin method steps
    //! - Loop over all of the models in the simulation and call their UpdateState
   for(ModelPair = ThreadModels.begin(); ModelPair != ThreadModels.end(); 
      ModelPair++)
   {
       NonIt = (ModelPair->ModelPtr);
       NonIt->UpdateState(CurrentSimNanos);
       NonIt->CallCounts += 1;
   }
   //! - NextStartTime is set to allow the scheduler to fit the next call in
   NextStartTime += ThreadPeriod;   
}

/*! This method adds a new model into the thread list.  Note that the Priority 
    parameter is option as it defaults to -1 (lowest)
    @return void
    @param NewModel The new model that we are adding to the thread
    @param Priority The selected priority of the model being added
*/
void SysModelThread::AddNewObject(SysModel *NewModel, int32_t Priority)
{
   std::vector<ModelPriorityPair>::iterator ModelPair;
   ModelPriorityPair LocalPair;

   //! Begin method steps   
   //! - Set the local pair with the requested priority and mode
   LocalPair.CurrentModelPriority = Priority;
   LocalPair.ModelPtr = NewModel;
   //! - Loop through the ModelPair vector and if Priority is higher than next, insert
   for(ModelPair = ThreadModels.begin(); ModelPair != ThreadModels.end();
      ModelPair++)
   {
      if(Priority > ModelPair->CurrentModelPriority)
      {
         ThreadModels.insert(ModelPair, LocalPair);
         return;
      }
   }
   //! - If we make it to the end of the loop, this is lowest priority, put it at end 
   ThreadModels.push_back(LocalPair);
}
