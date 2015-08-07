
#ifndef _SysModelThread_HH_
#define _SysModelThread_HH_

#include <vector>
#include <stdint.h>
#include "utilities/sys_model.h"
/*! \addtogroup SimArchGroup
 * @{
 */

//! Structure used to pair a model and its requested priority
typedef struct {
   int32_t CurrentModelPriority;   //!< The current model priority (lower is lower priority)
   SysModel *ModelPtr;             //!< The model associated with this priority
}ModelPriorityPair;

//! Class used to group a set of models into one "thread" of execution
class SysModelThread
{

   public:
      SysModelThread();
      SysModelThread(uint64_t InputPeriod, uint64_t InputDelay=0, 
         uint64_t FirstStartTime=0);
      ~SysModelThread();
      void AddNewObject(SysModel *NewModel, int32_t Priority = -1);
      void InitThreadList();
      void ExecuteThreadList(uint64_t CurrentSimTime);
      void ResetThread() {NextStartTime = FirstThreadTime;}

   public:
      std::vector<ModelPriorityPair> ThreadModels; //!< -- Array that has pointers to all GNC laws
      std::string ThreadName;                      //!< -- Identified for thread
      uint64_t NextStartTime;                      //!< ns Next time to start model
      uint64_t NextPickupTime;                     //!< ns Next time read thread outputs
      uint64_t ThreadPeriod;                       //!< ns Cycle rate for thread
      uint64_t PickupDelay;                        //!< ns Time between dispatches
      uint64_t FirstThreadTime;                     //!< ns Time to start thread for first time
};

/*! @} */
#endif /* _SysModelThread_H_ */
