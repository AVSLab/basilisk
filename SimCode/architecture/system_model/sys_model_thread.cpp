
#include "architecture/system_model/sys_model_thread.h"
#include <cstring>
#include <iostream>

SysModelThread :: SysModelThread()
{
   ThreadModels.clear();
   ThreadName.clear();
   ThreadPeriod = 1000;
   NextStartTime = 0;
   NextPickupTime = 0;
   PickupDelay = 0;

}
SysModelThread :: SysModelThread(uint64_t InputPeriod, uint64_t InputDelay, 
   uint64_t FirstStartTime)
{
   ThreadPeriod = InputPeriod;
   PickupDelay = InputDelay;
   NextStartTime = FirstStartTime;
   NextPickupTime = NextStartTime + ThreadPeriod;
}

SysModelThread :: ~SysModelThread()
{

}

void SysModelThread::InitThreadList()
{
   std::vector<ModelPriorityPair>::iterator ModelPair;
   SysModel* NonIt;

   for(ModelPair = ThreadModels.begin(); ModelPair != ThreadModels.end();
      ModelPair++)
   {
       NonIt = (ModelPair->ModelPtr);
       NonIt->SelfInit();
   }
   for(ModelPair = ThreadModels.begin(); ModelPair != ThreadModels.end();
      ModelPair++)
   {
      NonIt = (ModelPair->ModelPtr);
      NonIt->CrossInit();
   }
   return;
}

void SysModelThread::ExecuteThreadList(uint64_t CurrentSimNanos)
{

   std::vector<ModelPriorityPair>::iterator ModelPair;
   SysModel* NonIt;

   for(ModelPair = ThreadModels.begin(); ModelPair != ThreadModels.end(); 
      ModelPair++)
   {
       NonIt = (ModelPair->ModelPtr);
       NonIt->UpdateState(CurrentSimNanos);
       NonIt->CallCounts += 1;
   }
   NextStartTime += ThreadPeriod;   
}

void SysModelThread::AddNewObject(SysModel *NewModel, int32_t Priority)
{
   std::vector<ModelPriorityPair>::iterator ModelPair;
   ModelPriorityPair LocalPair;
         
   LocalPair.CurrentModelPriority = Priority;
   LocalPair.ModelPtr = NewModel;
   for(ModelPair = ThreadModels.begin(); ModelPair != ThreadModels.end();
      ModelPair++)
   {
      if(Priority > ModelPair->CurrentModelPriority)
      {
         ThreadModels.insert(ModelPair, LocalPair);
         return;
      }
   } 
   ThreadModels.push_back(LocalPair);
}
