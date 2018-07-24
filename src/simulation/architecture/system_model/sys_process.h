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

#ifndef _SysProcess_HH_
#define _SysProcess_HH_

#include <vector>
#include <stdint.h>
#include "architecture/system_model/sys_model_task.h"
#include "_GeneralModuleFiles/sys_interface.h"
#include "architecture/messaging/system_messaging.h"
/*! \addtogroup SimArchGroup
 * @{
 */

//! Structure that contains the information needed to call a Task
typedef struct {
    uint64_t NextTaskStart;  /*!< Time to call Task next*/
    uint64_t TaskUpdatePeriod;   /*!< Period of update for Task*/
    int32_t taskPriority;   /*!< [-] Priority level for the task*/
    SysModelTask *TaskPtr;  /*!< Handle to the Task that needs to be called*/
}ModelScheduleEntry;

//! Class used to group a set of models into one "Task" of execution
class SysProcess
{
    
public:
    SysProcess();
    SysProcess(std::string messageContainer);
    ~SysProcess();
    void addNewTask(SysModelTask *newTask, int32_t taskPriority = -1);
    void selfInitProcess();
    void crossInitProcess();
    void resetProcess(uint64_t currentTime);
    void reInitProcess();
    void enableProcess() { processActive = true; }
    void disableProcess() { processActive = false; }
    void scheduleTask(ModelScheduleEntry & taskCall);
    void selectProcess()
    {SystemMessaging::GetInstance()->selectMessageBuffer(messageBuffer);}
    void setProcessName(std::string newName){processName = newName;}
    std::string getProcessName() { return(processName);}
    uint64_t getNextTime() { return(nextTaskTime);}
    void singleStepNextTask(uint64_t currentNanos);
    bool processEnabled() {return processActive;}
    void addInterfaceRef(SysInterface *newInt) {intRefs.push_back(newInt);}
	void changeTaskPeriod(std::string taskName, uint64_t newPeriod);
    void setPriority(int64_t newPriority) {processPriority = newPriority;}
    void routeInterfaces();
    void disableAllTasks();
    void enableAllTasks();
    
public:
    std::vector<SysInterface*> intRefs;         //!< -- Interface references to move data to process
    std::vector<ModelScheduleEntry> processTasks; //!< -- Array that has pointers to all GNC laws
    uint64_t messageBuffer;                     //!< -- Message buffer for data
    uint64_t nextTaskTime;                      //!< ns time for the next Task
    uint64_t prevRouteTime;                     //!< ns Time that interfaces were previously routed
    std::string processName;                      //!< -- Identified for Task
	bool processActive;                           //!< -- Flag indicating whether the Task has been disabled
    int64_t processPriority;                    //!< [-] Priority level for process (higher first)
};

/*! @} */
#endif /* _SysProcess_H_ */
