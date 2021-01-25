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

#ifndef CLOCK_SYNCH_H
#define CLOCK_SYNCH_H

#include <string>
#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include <chrono>

#include "architecture/msgPayloadDefC/SynchClockMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief clock sync model class */
class ClockSynch: public SysModel {
public:
    ClockSynch();
    ~ClockSynch();
   
    void Reset(uint64_t currentSimNanos);
    void UpdateState(uint64_t currentSimNanos);
    
public:
	double accelFactor;          //!< [-] Factor used to accelerate sim-time relative to clock
    SynchClockMsgPayload outputData; //!< [-] Output data for the synch module
    Message<SynchClockMsgPayload> clockOutMsg; //!< [-] output message

    int64_t accuracyNanos;       //!< ns Level of accuracy that we want out of the timer, default is 10ms
	bool displayTime;            //!< [-] Flag indicating that we want to display the time elapsed in cmd line, default is off
  BSKLogger bskLogger;                      //!< -- BSK Logging
private:
    bool timeInitialized;        //!< [-] Flag that the module has been reset
	std::chrono::high_resolution_clock::time_point startTime; //! [-] first time stamp of pass through data
    uint64_t startSimTimeNano;   //!< [ns] Previous simulation time observed    
};


#endif
