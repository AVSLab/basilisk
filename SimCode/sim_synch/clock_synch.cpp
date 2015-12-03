#include "sim_synch/clock_synch.h"
#include "architecture/messaging/system_messaging.h"
#include <iostream>
#include <cstring>
#include <thread>

/*! This is the constructor for the clock synch model.  It sets default variable 
    values and initializes the various parts of the model */
ClockSynch::ClockSynch()
{
	timeInitialized = false;
	startSimTime = 0;
	accuracyNanos = -1000000;
	accelFactor = 1.0;
    clockOutputID = -1;
    outputBufferCount = 2;
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
    //! Begin method steps
    //! - Initialize the output message
    clockOutputID = SystemMessaging::GetInstance()->
    CreateNewMessage(clockOutputName, sizeof(SynchClockOutput), outputBufferCount,
                     "SynchClockOutput", moduleID);
    //! - Set the overrun counter to zero
    outputData.overrunCounter = 0;

}

/*! This method is blank but included in case we need to add functionality to the 
     model.
    @return void
*/
void ClockSynch::CrossInit()
{

}

/*! This method performs all of the runtime behavior for the clock synch model.
    It initializes the timers present in the model on the first pass and then 
	ensures that the model stays stuck on the synchronization pulse.  Note that we 
	do the init on the first pass instead of the init routines so that we don't 
	have a big lag between initialization and runtime which messes up our clocking.
    @return void
    @param CurrentSimNanos The clock time associated with the model call
*/
void ClockSynch::UpdateState(uint64_t CurrentSimNanos)
{
	std::chrono::high_resolution_clock::time_point currentTime;
	int64_t sleepAmount;

    //! Begin method steps
    
    //! - If we haven't initialized the timers yet, initialize the start times and flag it as initialized
	if (!timeInitialized)
	{
		startTime = std::chrono::high_resolution_clock::now(); 
		startSimTime = CurrentSimNanos;
		timeInitialized = true;
	}
	
    //! - Compute the number of nanoseconds that have elapsed in the simulation
	int64_t nanosDelta = CurrentSimNanos - startSimTime;
    
    //! - Compute the current time and get the actually elapsed nanoseconds since init time
	currentTime = std::chrono::high_resolution_clock::now();
	std::chrono::nanoseconds diffNanos = std::chrono::duration_cast<std::chrono::nanoseconds>
		(currentTime - startTime);
    
    //! - Save off the observed time-delta for analysis and flag any unexpected overruns
    outputData.initTimeDelta = (int64_t) (nanosDelta/accelFactor) -
        (int64_t) diffNanos.count();
    outputData.initTimeDelta *= 1.0E-9;
    if(outputData.initTimeDelta < 0)
    {
        outputData.overrunCounter++;
    }
    
    /*! - Loop behavior is fairly straightforward.  While we haven't reached the specified accuracy:
            -# Compute the current time
            -# Determine how many nanoseconds are left in our synch frame
            -# Sleep for half of those nanoseconds
            -# Lather/rinse/repeat
    */
    sleepAmount = 0;
	while (((int64_t) diffNanos.count() - (int64_t) (nanosDelta/accelFactor))
        < accuracyNanos)
	{
		currentTime = std::chrono::high_resolution_clock::now();
		diffNanos = std::chrono::duration_cast<std::chrono::nanoseconds>
			(currentTime - startTime);
		sleepAmount = (nanosDelta/accelFactor - diffNanos.count()) / (2);
		std::this_thread::sleep_for(std::chrono::nanoseconds(sleepAmount));
	}
    
    //! - Save off the output message information for analysis
    outputData.finalTimeDelta = (int64_t) (nanosDelta/accelFactor) -
    (int64_t) diffNanos.count() - sleepAmount;
    outputData.finalTimeDelta *= 1.0E-9;
    
    //! - Write the composite information into the output synch message.
    SystemMessaging::GetInstance()->
    WriteMessage(clockOutputID, CurrentSimNanos, sizeof(SynchClockOutput),
                 reinterpret_cast<uint8_t*> (&outputData), moduleID);
}
