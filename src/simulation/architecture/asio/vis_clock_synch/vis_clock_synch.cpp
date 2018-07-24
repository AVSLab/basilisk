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
#include <iostream>
#include <cstring>
#include <thread>
#include "architecture/asio/vis_clock_synch/vis_clock_synch.h"
#include "architecture/messaging/system_messaging.h"
#include "simMessages/realTimeFactorSimMsg.h"

/*! This is the constructor for the clock synch model.  It sets default variable 
    values and initializes the various parts of the model */
VisClockSynch::VisClockSynch()
{
	this->timeInitialized = false;
	this->accuracyNanos = 10000;
	this->accelFactor = 1.0;
    this->outputBufferCount = 2;
    this->realTimeFactorInMsgId = -1;
    this->realTimeFactorInMsgName = "real_time_factor";
    this->realTimeFactor.speedFactor = 1.0;
    
    return;
}

/*! Destructor.  Nothing here. */
VisClockSynch::~VisClockSynch()
{
    return;
}

/*! This is the self-init routine for the clock synch model.  It initializes the 
    output message for the model which is used for diagnostic information on the 
	model's behavior.
    @return void
*/
void VisClockSynch::SelfInit()
{

}

/*! This method is blank but included in case we need to add functionality to the 
     model.
    @return void
*/
void VisClockSynch::CrossInit()
{
    this->realTimeFactorInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->realTimeFactorInMsgName, sizeof(RealTimeFactorSimMsg), this->moduleID);
}

/*! This method performs all of the runtime behavior for the clock synch model.
    It initializes the timers present in the model on the first pass and then 
	ensures that the model stays stuck on the synchronization pulse.  Note that we 
	do the init on the first pass instead of the init routines so that we don't 
	have a big lag between initialization and runtime which messes up our clocking.
    @return void
    @param CurrentSimNanos The clock time associated with the model call
*/
void VisClockSynch::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    
    if (this->realTimeFactor.speedFactor >= 1.0) {
        this->accelFactor = this->realTimeFactor.speedFactor;
    }
    
	std::chrono::high_resolution_clock::time_point currentTime;
	int64_t sleepAmount;

    //! Begin method steps
    
    //! - If we haven't initialized the timers yet, initialize the start times and flag it as initialized
	if (!this->timeInitialized)
	{
        this->prevFrameStartTime = std::chrono::high_resolution_clock::now();
        this->prevFrameSimTime = CurrentSimNanos;
        this->timeInitialized = true;
	}
    
    //! - Compute the number of nanoseconds that have elapsed in this simulation
    // frame
	int64_t elapsedSimFrameTime = CurrentSimNanos - this->prevFrameSimTime;
    
    //! - Compute the current time and get the wall elapsed nanoseconds since
    // the last sime frame
	currentTime = std::chrono::high_resolution_clock::now();
	std::chrono::nanoseconds elapsedWallFrameTime = std::chrono::duration_cast<std::chrono::nanoseconds>
		(currentTime - this->prevFrameStartTime);
    
    /*! - Loop behavior is fairly straightforward.  While we haven't reached the specified accuracy:
            -# Compute the current time
            -# Determine how many nanoseconds are left in our synch frame
            -# Sleep for half of those nanoseconds
            -# Lather/rinse/repeat
    */
    sleepAmount = 0;
	while (((int64_t) elapsedWallFrameTime.count() - (int64_t) (elapsedSimFrameTime/this->accelFactor))
        < this->accuracyNanos)
	{
//        int64_t tmp = ((int64_t) elapsedWallFrameTime.count() - (int64_t) (elapsedSimFrameTime/this->accelFactor))/1E3;
		currentTime = std::chrono::high_resolution_clock::now();
		elapsedWallFrameTime = std::chrono::duration_cast<std::chrono::nanoseconds>
			(currentTime - this->prevFrameStartTime);
		sleepAmount = (elapsedSimFrameTime/this->accelFactor - elapsedWallFrameTime.count()) / (2);
		std::this_thread::sleep_for(std::chrono::nanoseconds(sleepAmount));
	}

    //! - Update prev frame times for next time around
    this->prevFrameSimTime = CurrentSimNanos;
    this->prevFrameStartTime = std::chrono::high_resolution_clock::now();
}

void VisClockSynch::readInputMessages()
{
    SingleMessageHeader tmpHeader;
    SystemMessaging::GetInstance()->ReadMessage(this->realTimeFactorInMsgId, &tmpHeader, sizeof(RealTimeFactorSimMsg), reinterpret_cast<uint8_t*>(&this->realTimeFactor), this->moduleID);
}
