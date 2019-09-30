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

#ifndef _THR_FIRING_SCHMITT_H
#define _THR_FIRING_SCHMITT_H

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "fswUtilities/fswDefinitions.h"
#include "fswMessages/thrArrayConfigFswMsg.h"
#include "fswMessages/thrArrayCmdForceFswMsg.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "simFswInterfaceMessages/thrArrayOnTimeCmdIntMsg.h"


/*! \defgroup thrFiringSchmitt
 * @brief A Schmitt trigger logic is implemented to map a desired thruster force value into a thruster on command time.
 *
 * The module reads in the attitude control thruster force values for both on- and off-pulsing scenarios, and then maps this into a time which specifies how long a thruster should be on.  The thruster configuration data is read in through a separate input message in the reset method.  The Schmitt trigger allows for an upper and lower bound where the thruster is either turned on or off.
 * More information can be found in the [PDF Description](Basilisk-thrFiringSchmitt-2019-03-29.pdf).
 * The paper [Steady-State Attitude and Control Effort Sensitivity Analysis of Discretized Thruster Implementations](https://doi.org/10.2514/1.A33709) includes a detailed discussion on the Schmitt Trigger and compares it to other thruster firing methods.  
 * @{
 */


/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module public variables */
    double              level_on;                               //!< [-] ON duty cycle fraction
    double              level_off;                              //!< [-] OFF duty cycle fraction 
    double              thrMinFireTime;                         //!< [s] Minimum ON time for thrusters
    int                 baseThrustState;                        //!< [-] Indicates on-pulsing (0) or off-pusling (1)

    /* declare module private variables */
	uint32_t 			numThrusters;							//!< [-] The number of thrusters available on vehicle
	double				maxThrust[MAX_EFF_CNT];					//!< [N] Max thrust
	boolean_t			lastThrustState[MAX_EFF_CNT];			//!< [-] ON/OFF state of thrusters from previous call

	uint64_t			prevCallTime;							//!< callTime from previous function call

    /* declare module IO interfaces */
    char 				thrForceInMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the Input message
    int32_t 			thrForceInMsgId;                        //!< ID for the incoming message
	char 				onTimeOutMsgName[MAX_STAT_MSG_LENGTH];  //!< The name of the output message*, onTimeOutMsgName
	int32_t 			onTimeOutMsgId;                         //!< ID for the outgoing message
	char 				thrConfInMsgName[MAX_STAT_MSG_LENGTH];	//!< The name of the thruster cluster Input message
	int32_t  			thrConfInMsgId;                   		//!< ID for the incoming Thruster configuration data

}thrFiringSchmittConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_thrFiringSchmitt(thrFiringSchmittConfig *configData, int64_t moduleID);
    void CrossInit_thrFiringSchmitt(thrFiringSchmittConfig *configData, int64_t moduleID);
    void Update_thrFiringSchmitt(thrFiringSchmittConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_thrFiringSchmitt(thrFiringSchmittConfig *configData, uint64_t callTime, int64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
