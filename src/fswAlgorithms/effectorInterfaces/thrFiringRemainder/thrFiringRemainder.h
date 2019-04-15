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

#ifndef _THR_FIRING_REMAINDER_
#define _THR_FIRING_REMAINDER_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "fswUtilities/fswDefinitions.h"
#include "fswMessages/thrArrayConfigFswMsg.h"
#include "fswMessages/thrArrayCmdForceFswMsg.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "simFswInterfaceMessages/thrArrayOnTimeCmdIntMsg.h"


/*! \defgroup thrFiringRemainder
 * @brief A thruster force message is read in and converted to a thruster on-time output message.
 *
 * The module ensures the requested on-time is at least as large as the thruster's minimum on time.  If not
 * then the on-time is zeroed, but the unimplemented thrust time is kept as a remainder calculation.  If these
 * add up to reach the minimum on time, then a thruster pulse is requested.  If the thruster on time is larger
 * than the control period, then an on-time that is 1.1 times the control period is requested.
 * More information can be found in the [PDF Description](Basilisk-thrFiringRemainder-2019-03-28.pdf).
 * The paper [Steady-State Attitude and Control Effort Sensitivity Analysis of Discretized Thruster Implementations](https://doi.org/10.2514/1.A33709) includes a detailed discussion on the Remainder Trigger algorithm and compares it to other thruster firing methods.
 * @{
 */


/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
	double              pulseRemainder[MAX_EFF_CNT];            //!< [-] Unimplemented thrust pulses (number of minimum pulses)
	double              thrMinFireTime;              			//!< [s] Minimum fire time
	uint32_t 			numThrusters;							//!< [-] The number of thrusters available on vehicle
	double				maxThrust[MAX_EFF_CNT];					//!< [N] Max thrust
	int					baseThrustState;						//!< [-] Indicates on-pulsing (0) or off-pusling (1)

	uint64_t			prevCallTime;							//!< callTime from previous function call
	

	/* declare module IO interfaces */
	char 				thrForceInMsgName[MAX_STAT_MSG_LENGTH];        	//!< The name of the Input message
	int32_t 			thrForceInMsgId;                             	//!< ID for the incoming message
	char 				onTimeOutMsgName[MAX_STAT_MSG_LENGTH];       	//!< The name of the output message, onTimeOutMsgName
	int32_t 			onTimeOutMsgId;                            		//!< ID for the outgoing message
	char 				thrConfInMsgName[MAX_STAT_MSG_LENGTH];			//!< The name of the thruster cluster Input message
	int32_t  			thrConfInMsgId;                   				//!< ID for the incoming Thruster configuration data
    
}thrFiringRemainderConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_thrFiringRemainder(thrFiringRemainderConfig *ConfigData, uint64_t moduleID);
    void CrossInit_thrFiringRemainder(thrFiringRemainderConfig *ConfigData, uint64_t moduleID);
    void Update_thrFiringRemainder(thrFiringRemainderConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_thrFiringRemainder(thrFiringRemainderConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
