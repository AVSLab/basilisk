#include "sim_synch/clock_synch.h"
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

	if (!timeInitialized)
	{
		startTime = std::chrono::high_resolution_clock::now(); 
		startSimTime = CurrentSimNanos;
		timeInitialized = true;
	}
	
	int64_t nanosDelta = CurrentSimNanos - startSimTime;
	currentTime = std::chrono::high_resolution_clock::now();
	std::chrono::nanoseconds diffNanos = std::chrono::duration_cast<std::chrono::nanoseconds>
		(currentTime - startTime);
	while (((int64_t) diffNanos.count() - (int64_t) (nanosDelta/accelFactor)) < accuracyNanos)
	{
		currentTime = std::chrono::high_resolution_clock::now();
		diffNanos = std::chrono::duration_cast<std::chrono::nanoseconds>
			(currentTime - startTime);
		sleepAmount = (nanosDelta/accelFactor - diffNanos.count()) / (2);
		std::this_thread::sleep_for(std::chrono::nanoseconds(sleepAmount));
	}
}
