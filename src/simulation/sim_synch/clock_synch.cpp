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
#include "sim_synch/clock_synch.h"
#include "architecture/messaging/system_messaging.h"
#include <iostream>
#include <cstring>
#include <thread>
#include "simFswInterfaceMessages/macroDefinitions.h"

/*! This is the constructor for the clock synch model.  It sets default variable
    values and initializes the various parts of the model */
ClockSynch::ClockSynch()
{
	this->timeInitialized = false;
	this->startSimTimeNano = 0;
	this->accuracyNanos = (int64_t) (0.01*NANO2SEC);
	this->accelFactor = 1.0;
    this->clockOutputID = -1;
    this->outputBufferCount = 2;
    this->clockOutputName = "clock_synch_data";
	this->displayTime = false;
    return;
}

/*! Destructor.  Nothing here. */
ClockSynch::~ClockSynch()
{
    return;
}

/*! This is the self-init routine for the clock synch model.  It initializes the
    output message for the model which is used for diagnostic information on the
	model's behavior.
    @return void
*/
void ClockSynch::SelfInit()
{
    //! - Initialize the output message
    clockOutputID = SystemMessaging::GetInstance()->
    CreateNewMessage(this->clockOutputName, sizeof(SynchClockSimMsg), this->outputBufferCount,
                     "SynchClockSimMsg", this->moduleID);

}

/*! This method is blank but included in case we need to add functionality to the
     model.
    @return void
*/
void ClockSynch::CrossInit()
{

}

/*! Reset the module variables.
    @return void
*/
void ClockSynch::Reset(uint64_t currentSimNanos)
{
    this->timeInitialized = false;
    //! - Set the overrun counter to zero
    this->outputData.overrunCounter = 0;
}


/*! This method performs all of the runtime behavior for the clock synch model.
    It initializes the timers present in the model on the first pass and then
	ensures that the model stays stuck on the synchronization pulse.  Note that we
	do the init on the first pass instead of the init routines so that we don't
	have a big lag between initialization and runtime which messes up our clocking.
    @return void
    @param CurrentSimNanos The clock time associated with the model call
*/
void ClockSynch::UpdateState(uint64_t currentSimNanos)
{
	std::chrono::high_resolution_clock::time_point currentTime;     // current time variable
	int64_t sleepAmountNano;                    // [ns] time for module to sleep
    int64_t simTimeNano;                        // [ns] simTimeNano simulation time since reset
    int64_t realTimeNano;                       // [ns] real time elapsed since reset

    //! - If we haven't initialized the timers yet, initialize the start times and flag it as initialized
	if (!timeInitialized)
	{
		this->startTime = std::chrono::high_resolution_clock::now();
		this->startSimTimeNano = currentSimNanos;
		this->timeInitialized = true;
	}

    //! - Compute the number of nanoseconds that have elapsed in the simulation
    simTimeNano = (int64_t) ((currentSimNanos - this->startSimTimeNano)/this->accelFactor);

    //! - Compute the current time and get the actually elapsed nanoseconds since init time
	currentTime = std::chrono::high_resolution_clock::now();
	realTimeNano = (int64_t) (std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime - this->startTime)).count();

    //! - Save off the observed time-delta for analysis and flag any unexpected overruns
    this->outputData.initTimeDelta = simTimeNano - realTimeNano;

    if(this->outputData.initTimeDelta < -this->accuracyNanos)
    {
        this->outputData.overrunCounter++;
    }
	this->outputData.initTimeDelta *= NANO2SEC;

    /*! - Loop behavior is fairly straightforward.  While we haven't reached the specified accuracy:
            -# Compute the current time
            -# Determine how many nanoseconds are left in our synch frame
            -# Sleep for half of those nanoseconds
            -# Lather/rinse/repeat
    */
    sleepAmountNano = 0;
	while (realTimeNano - simTimeNano < -this->accuracyNanos)
	{
		currentTime = std::chrono::high_resolution_clock::now();
		realTimeNano = (int64_t) (std::chrono::duration_cast<std::chrono::nanoseconds>
			(currentTime - this->startTime)).count();
		sleepAmountNano = (simTimeNano - realTimeNano) / (2);
		std::this_thread::sleep_for(std::chrono::nanoseconds(sleepAmountNano));
	}

    //! - Save off the output message information for analysis
    this->outputData.finalTimeDelta = simTimeNano - realTimeNano - sleepAmountNano;
    this->outputData.finalTimeDelta *= NANO2SEC;

    //! - Write the composite information into the output synch message.
    SystemMessaging::GetInstance()->
    WriteMessage(this->clockOutputID, currentSimNanos, sizeof(SynchClockSimMsg),
                 reinterpret_cast<uint8_t*> (&this->outputData), this->moduleID);

	if (this->displayTime)
	{
        bskPrint.printMessage(MSG_INFORMATION, "Seconds Elapsed: %f", currentSimNanos*NANO2SEC);
	}
}
