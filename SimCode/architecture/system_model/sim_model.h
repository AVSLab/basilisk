
#ifndef _SimModel_HH_
#define _SimModel_HH_

#include <vector>
#include <stdint.h>
#include "architecture/system_model/sys_model_thread.h"
#include "architecture/messaging/system_messaging.h"

///@brief Structure that contains the information needed to call a thread
typedef struct {
   uint64_t NextThreadStart;  //! <Time to call thread next
   uint64_t ThreadUpdatePeriod;   //! <Period of update for thread
   SysModelThread *ThreadPtr;  //! <Handle to the thread that needs to be called
}ModelScheduleEntry;

class SimModel
{

 public:
   SimModel();
   ~SimModel();
   void AddNewThread(SysModelThread *NewThread); ///@brief Add new set of models
   void InitThreads(); ///@brief Initialize all of the active model threads
   void StepUntilTime(uint64_t SimStopTime); ///@brief Step until specified stop time
   void SingleStepNextThread(); ///@brief Step only the next thread on the stack
   void ScheduleThread(ModelScheduleEntry *ThreadCall); ///@brief Place thread in schedule
   void PrintSimulatedMessageData(); ///@brief Print out stats on active messages
    

 public:
   std::vector<ModelScheduleEntry> ThreadModels; //! <-- Array that has pointers to all GNC laws
   std::string SimulationName;                      //! <-- Identified for thread
   uint64_t CurrentNanos;                        //! <ns Current clock time
   uint64_t NextThreadTime;                      //! <ns time for the next thread
 private:
   BlankStorage MessageBucket;                      //! <-- Messaging data
};

#endif /* _SimModel_H_ */
