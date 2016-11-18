/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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

#ifndef CLOCK_SYNCH_H
#define CLOCK_SYNCH_H

#include <string>
#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include <chrono>
/*! \addtogroup SimArchGroup
 * @{
 */

//! @brief Output diagnostic structure used for analyzing how the synch is performing.
typedef struct {
    double initTimeDelta;       //!< s Time remaining in synch frame on arrival
    double finalTimeDelta;      //!< s Time remaining in synch frame on departure
    uint64_t overrunCounter;    //!< (-) Indicator of how many times we've missed the synch frame
}SynchClockOutput;

//!@brief The clock synchronization module is used to slave the simulation to realtime.
/*!  The module is controlled by specifying an acceleration factor which can be adjusted 
     dynamically if the timeInitialized factor is also reset dynamically.*/
class ClockSynch: public SysModel {
public:
    ClockSynch();
    ~ClockSynch();
   
    void SelfInit();
    void CrossInit(); 
    void UpdateState(uint64_t CurrentSimNanos);
    
public:
    bool timeInitialized;        //!< (-) Number of output state buffers in msg
	double accelFactor;          //!< (-) Factor used to accelerate sim-time relative to clock
    SynchClockOutput outputData; //!< (-) Output data for the synch module
    std::string clockOutputName; //!< (-) Name of the output message that we are using
    uint64_t outputBufferCount;  //!< (-) Count on the number of output buffers that we have
    int64_t accuracyNanos;       //!< ns Level of accuracy that we want out of the timer
	bool displayTime;            //!< [-] Flag indicating that we want to display the time elapsed in cmd line
private:
	std::chrono::high_resolution_clock::time_point startTime; //! (-) first time pass through data
    uint64_t startSimTime;                 //!< ns Previous simulation time observed
    int64_t clockOutputID;                //!< (-) Output ID for clock module
    
};

/*! @} */

#endif
