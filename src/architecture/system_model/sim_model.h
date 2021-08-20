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
#ifndef _SimModel_HH_
#define _SimModel_HH_

#include <vector>
#include <stdint.h>
#include <set>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include "architecture/system_model/sys_process.h"
#include "architecture/utilities/bskLogging.h"

/*! \addtogroup SimArchGroup Simulation Architecture Classes
 *  This architecture group contains the source used to drive/schedule/interface
 *  with the simulation.
 * @{
 */

class SimThreadExecution
{
public:
    SimThreadExecution();
    SimThreadExecution(uint64_t threadIdent, uint64_t currentSimNanos=0);    //!< Constructor for a given sim thread
    ~SimThreadExecution();   //!< Destructor for given sim thread
    void updateNewStopTime(uint64_t newStopNanos) {stopThreadNanos = newStopNanos;}
    void clearProcessList() {processList.clear();}
    void selfInitProcesses();
    void crossInitProcesses();
    void resetProcesses();
    void addNewProcess(SysProcess* newProc) {processList.push_back(newProc);}
    bool threadActive() {return this->threadRunning;};
    void threadReady() {this->threadRunning=true;}
    void waitOnInit();
    void postInit();
    bool threadValid() {return (!this->terminateThread);}
    void killThread() {this->terminateThread=true;}
    void lockThread();
    void unlockThread();
    void lockMaster();
    void unlockMaster();
    void StepUntilStop();  //!< Step simulation until stop time uint64_t reached
    void SingleStepProcesses(int64_t stopPri=-1); //!< Step only the next Task in the simulation
    void moveProcessMessages();
public:
    uint64_t currentThreadNanos;  //!< Current simulation time available at thread
    uint64_t stopThreadNanos;   //!< Current stop conditions for the thread
    int64_t stopThreadPriority; //!< Current stop priority for thread
    uint64_t threadID;          //!< Identifier for thread
    std::thread *threadContext;
    uint64_t CurrentNanos;  //!< [ns] Current sim time
    uint64_t NextTaskTime;  //!< [ns] time for the next Task
    int64_t nextProcPriority;  //!< [-] Priority level for the next process
    bool selfInitNow;              //!< Flag requesting self init
    bool crossInitNow;             //!< Flag requesting cross-init
    bool resetNow;
private:
    bool threadRunning;            //!< Flag that will allow for easy concurrent locking
    bool terminateThread;          //!< Flag that indicates that it is time to take thread down
    std::mutex masterThreadLock;   //!< Lock that ensures master thread won't proceed
    std::mutex selfThreadLock;     //!< Lock that ensures this thread only reaches allowed time
    std::vector<SysProcess*> processList;  //!< List of processes associated with thread
    std::mutex initReadyLock;      //!< Lock function to ensure runtime locks are configured
    std::condition_variable initHoldVar; //!< Conditional variable used to prevent race conditions
};

//! The top-level container for an entire simulation
class SimModel
{
public:
    SimModel();  //!< The SimModel constructor
    ~SimModel();  //!< SimModel destructorS
    
    void selfInitSimulation();  //!< Method to initialize all added Tasks
    void resetInitSimulation();  //!< Method to reset all added tasks
    void StepUntilStop(uint64_t SimStopTime, int64_t stopPri);  //!< Step simulation until stop time uint64_t reached
    void SingleStepProcesses(int64_t stopPri=-1); //!< Step only the next Task in the simulation
    void addNewProcess(SysProcess *newProc);
    void ResetSimulation();  //!< Reset simulation back to zero
    void clearProcsFromThreads();
    void resetThreads(uint64_t threadCount);
    void deleteThreads();
    void assignRemainingProcs();
    uint64_t getThreadCount() {return threadList.size();}

    BSKLogger bskLogger;                      //!< -- BSK Logging

public:
    std::vector<SysProcess *> processList;  //!< -- List of processes we've created
    std::vector<SimThreadExecution*> threadList;  //!< -- Array of threads that we're running on
    std::string SimulationName;  //!< -- Identifier for Sim
    uint64_t CurrentNanos;  //!< [ns] Current sim time
    uint64_t NextTaskTime;  //!< [ns] time for the next Task
    int64_t nextProcPriority;  //!< [-] Priority level for the next process
};

#endif /* _SimModel_H_ */
