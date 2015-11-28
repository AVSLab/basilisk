
#ifndef CLOCK_SYNCH_H
#define CLOCK_SYNCH_H

#include <vector>
#include "utilities/sys_model.h"
#include <chrono>
/*! \addtogroup SimModelGroup
 * @{
 */

//!@brief Simple navigation model used to provide error-ed truth (or truth)
/*! This class is used to perturb the truth state away using a gauss-markov 
 error model.  It is designed to look like a random walk process put on top of 
 the nominal position, velocity, attitude, and attitude rate.  This is meant to 
 be used in place of the nominal navigation system output*/
class ClockSynch: public SysModel {
public:
    ClockSynch();
    ~ClockSynch();
   
    void SelfInit();
    void CrossInit(); 
    void UpdateState(uint64_t CurrentSimNanos);
    
public:
    bool timeInitialized;        //!< -- Number of output state buffers in msg
	double accelFactor;          //!< -- Factor used to accelerate sim-time relative to clock
private:
	std::chrono::high_resolution_clock::time_point startTime; //! -- first time pass through data
    uint64_t startSimTime;                 //!< ns Previous simulation time observed
	int64_t accuracyNanos;                //!< ns Level of accuracy that we want out of the timer
};

/*! @} */

#endif
