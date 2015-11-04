
#ifndef _SysProcess_HH_
#define _SysProcess_HH_

#include <vector>
#include <stdint.h>
#include "architecture/system_model/sys_model_task.h"
#include "architecture/system_model/sys_interface.h"
/*! \addtogroup SimArchGroup
 * @{
 */

//! Structure that contains the information needed to call a Task
typedef struct {
    uint64_t NextTaskStart;  /*!< Time to call Task next*/
    uint64_t TaskUpdatePeriod;   /*!< Period of update for Task*/
    SysModelTask *TaskPtr;  /*!< Handle to the Task that needs to be called*/
}ModelScheduleEntry;

//! Class used to group a set of models into one "Task" of execution
class SysProcess
{
    
public:
    SysProcess();
    SysProcess(std::string messageContainer);
    ~SysProcess();
    void addNewTask(SysModelTask *newTask);
    void selfInitProcess();
    void crossInitProcess();
    void resetProcess();
    void enableProcess() { processActive = true; }
    void disableProcess() { processActive = false; }
    void scheduleTask(ModelScheduleEntry & taskCall);
    void setProcessName(std::string newName){processName = newName;}
    std::string getProcessName() { return(processName);}
    uint64_t getNextTime() { return(nextTaskTime);}
    void singleStepNextTask(uint64_t currentNanos);
    bool processEnabled() {return processActive;}
    void addInterfaceRef(SysInterface *newInt) {intRefs.push_back(newInt);}
    void routeInterfaces();
    
public:
    std::vector<SysInterface*> intRefs;         //!< -- Interface references to move data to process
    std::vector<ModelScheduleEntry> taskModels; //!< -- Array that has pointers to all GNC laws
    uint64_t messageBuffer;                     //!< -- Message buffer for data
    uint64_t nextTaskTime;                      //!< ns time for the next Task
    std::string processName;                      //!< -- Identified for Task
	bool processActive;                           //!< -- Flag indicating whether the Task has been disabled 
};

/*! @} */
#endif /* _SysProcess_H_ */
