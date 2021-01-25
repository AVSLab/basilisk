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

#include "sim_model.h"
#include <cstring>
#include <iostream>

/*! This Constructor is used to initialize the top-level sim model.
 */
SimModel::SimModel()
{
    this->CurrentNanos = 0;
    this->NextTaskTime = 0;
    this->nextProcPriority = -1;
}

/*! Nothing to destroy really */
SimModel::~SimModel()
{
}

/*! This method allows the user to attach a process to the simulation for
    execution.  Note that the priority level of the process determines what
    order it gets called in: higher priorities are called before lower
    priorities. If priorities are the same, the proc added first goes first.
    @return void
    @param newProc the new process to be added
*/
void SimModel::addNewProcess(SysProcess *newProc)
{
    std::vector<SysProcess *>::iterator it;
    for(it = this->processList.begin(); it != this->processList.end(); it++)
    {
        if(newProc->processPriority > (*it)->processPriority)
        {
            this->processList.insert(it, newProc);
            return;
        }
    }
    this->processList.push_back(newProc);
}

/*! This method goes through all of the processes in the simulation,
 *  all of the tasks within each process, and all of the models within
 *  each task and self-inits them.
 @return void
 */
void SimModel::selfInitSimulation()
{
    std::vector<SysProcess *>::iterator it;
    for(it=this->processList.begin(); it!= this->processList.end(); it++)
    {
        (*it)->selfInitProcess();
    }

    this->NextTaskTime = 0;
    this->CurrentNanos = 0;
    it=this->processList.begin();
    this->nextProcPriority = (*it)->processPriority;
}

/*! This method goes through all of the processes in the simulation,
 *  all of the tasks within each process, and all of the models within
 *  each task and resets them.
 @return void
 */
void SimModel::resetInitSimulation()
{
    std::vector<SysProcess *>::iterator it;
    for(it=this->processList.begin(); it!= this->processList.end(); it++)
    {
        (*it)->resetProcess(0);
    }
}

/*! This method steps all of the processes forward to the current time.  It also
    increments the internal simulation time appropriately as the simulation
    processes are triggered
    @param stopPri The priority level below which the sim won't go
    @return void
*/
void SimModel::SingleStepProcesses(int64_t stopPri)
{
    uint64_t nextCallTime = ~((uint64_t) 0);
    std::vector<SysProcess *>::iterator it = this->processList.begin();
    this->CurrentNanos = this->NextTaskTime;
    while(it!= this->processList.end())
    {
        SysProcess *localProc = (*it);
        if(localProc->processEnabled())
        {
            while(localProc->nextTaskTime < this->CurrentNanos ||
                (localProc->nextTaskTime == this->CurrentNanos &&
                  localProc->processPriority >= stopPri))
            {
                localProc->singleStepNextTask(this->CurrentNanos);
            }
            if(localProc->getNextTime() < nextCallTime)
            {
                nextCallTime = localProc->getNextTime();
                this->nextProcPriority = localProc->processPriority;
            }
            else if(localProc->getNextTime() == nextCallTime &&
                localProc->processPriority > this->nextProcPriority)
            {
                this->nextProcPriority = localProc->processPriority;
            }
        }
        it++;
    }

    this->NextTaskTime = nextCallTime != ~((uint64_t) 0) ? nextCallTime : this->CurrentNanos;
    //! - If a message has been added to logger, link the message IDs
}

/*! This method steps the simulation until the specified stop time and
 stop priority have been reached.
 @return void
 @param SimStopTime Nanoseconds to step the simulation for
 @param stopPri The priority level below which the sim won't go
 */
void SimModel::StepUntilStop(uint64_t SimStopTime, int64_t stopPri)
{
    /*! - Note that we have to step until both the time is greater and the next
     Task's start time is in the future. If the NextTaskTime is less than
     SimStopTime, then the inPri shouldn't come into effect, so set it to -1
     (that's less than all process priorities, so it will run through the next
     process)*/
    int64_t inPri = SimStopTime == this->NextTaskTime ? stopPri : -1;
    while(this->NextTaskTime < SimStopTime || (this->NextTaskTime == SimStopTime &&
            this->nextProcPriority >= stopPri) )
    {
        this->SingleStepProcesses(inPri);
        inPri = SimStopTime == this->NextTaskTime ? stopPri : -1;
    }
}

/*! This method is used to reset a simulation to time 0. It sets all process and
 * tasks back to the initial call times. It clears all message logs. However,
 * it does not clear all message buffers and does not reset individual models.
 @return void
 */
void SimModel::ResetSimulation()
{
    std::vector<SysProcess *>::iterator it;
    //! - Iterate through model list and call the Task model initializer
    for(it = this->processList.begin(); it != this->processList.end(); it++)
    {
        (*it)->reInitProcess();
    }
    this->CurrentNanos = 0;
    this->NextTaskTime = 0;
}


/*! This method clears all messages.  Note that once you do this, the simulation
    object itself is really dead.
    @return void
*/
void SimModel::terminateSimulation()
{
}

