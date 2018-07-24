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

#ifndef VIS_CLOCK_SYNCH_H
#define VIS_CLOCK_SYNCH_H

#include <string>
#include <vector>
#include <chrono>
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/syncClockSimMsg.h"
#include "simMessages/realTimeFactorSimMsg.h"

/*! \addtogroup SimArchGroup
 * @{
 */

//!@brief The clock synchronization module is used to slave the simulation to realtime.
/*!  The module is controlled by specifying an acceleration factor which can be adjusted 
     dynamically if the timeInitialized factor is also reset dynamically.*/
class VisClockSynch: public SysModel {
public:
    VisClockSynch();
    ~VisClockSynch();
   
    void SelfInit();
    void CrossInit(); 
    void UpdateState(uint64_t CurrentSimNanos);
    void readInputMessages();
    
public:
    bool timeInitialized;        //!< (-) Number of output state buffers in msg
	double accelFactor;          //!< (-) Factor used to accelerate sim-time relative to clock
    SynchClockSimMsg clockOutMsgData; //!< (-) Output data for the synch module
    std::string clockOutMsgName; //!< (-) Name of the output message that we are using
    uint64_t outputBufferCount;  //!< (-) Count on the number of output buffers that we have
    int64_t accuracyNanos;       //!< ns Level of accuracy that we want out of the timer
private:
	std::chrono::high_resolution_clock::time_point startTime; //! (-) first time pass through data
    int64_t realTimeFactorInMsgId;          //!< (-) Input Msg ID for real time factor
    std::string realTimeFactorInMsgName;    //!< (-) Input Msg name for real time factor
    RealTimeFactorSimMsg realTimeFactor;    //!< (-) Factor by which to multiple the sim time rate.
    std::chrono::high_resolution_clock::time_point prevFrameStartTime;
    uint64_t prevFrameSimTime; //!< ns Previous simulation time frame
};

/*! @} */

#endif
