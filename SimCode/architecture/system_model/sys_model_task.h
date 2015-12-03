
#ifndef _SysModelTask_HH_
#define _SysModelTask_HH_

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

//! Class used to group a set of models into one "Task" of execution
class SysModelTask
{
    
public:
    SysModelTask();
    SysModelTask(uint64_t InputPeriod, uint64_t InputDelay=0,
                   uint64_t FirstStartTime=0);
    ~SysModelTask();
    void AddNewObject(SysModel *NewModel, int32_t Priority = -1);
    void SelfInitTaskList();
    void CrossInitTaskList();
    void ExecuteTaskList(uint64_t CurrentSimTime);
    void ResetTask() {NextStartTime = FirstTaskTime;}
	void enableTask() { taskActive = true; }
	void disableTask() { taskActive = false; }
	void updatePeriod(uint64_t newPeriod) { TaskPeriod = newPeriod; }
    
public:
    std::vector<ModelPriorityPair> TaskModels; //!< -- Array that has pointers to all GNC laws
    std::string TaskName;                      //!< -- Identified for Task
    uint64_t NextStartTime;                      //!< ns Next time to start model
    uint64_t NextPickupTime;                     //!< ns Next time read Task outputs
    uint64_t TaskPeriod;                       //!< ns Cycle rate for Task
    uint64_t PickupDelay;                        //!< ns Time between dispatches
    uint64_t FirstTaskTime;                    //!< ns Time to start Task for first time
	bool taskActive;                           //!< -- Flag indicating whether the Task has been disabled 
};

/*! @} */
#endif /* _SysModelTask_H_ */
