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

#ifndef _FSW_MODULE_TEMPLATE_H_
#define _FSW_MODULE_TEMPLATE_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "effectorInterfaces/errorConversion/vehEffectorOut.h"
#include "ADCSUtilities/ADCSDefinitions.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */


/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
	double				level_on[MAX_EFF_CNT];
	double				level_off[MAX_EFF_CNT];
	double              level[MAX_EFF_CNT];                     /*!< thruster duty cycle percentage */
	double				controlPeriod;
	uint32_t 			numThrusters;							/*!< The number of thrusters available on vehicle */
	double				maxThrust[MAX_EFF_CNT];					/*!< Max thrust */
	double				pulseTimeMin[MAX_EFF_CNT];
	boolean_t			baseThrustState[MAX_EFF_CNT];
	boolean_t			thrustState[MAX_EFF_CNT];
	double 				onTime[MAX_EFF_CNT];

	uint64_t			prevCallTime;							/*!< callTime from previous function call */

    /* declare module IO interfaces */
    char 				outputDataName[MAX_STAT_MSG_LENGTH];       	/*!< The name of the output message*/
    int32_t 			outputMsgID;                            	/*!< ID for the outgoing message */
    char 				inputDataName[MAX_STAT_MSG_LENGTH];        	/*!< The name of the Input message*/
    int32_t 			inputMsgID;                             	/*!< ID for the incoming message */
	char 				inputThrusterConfName[MAX_STAT_MSG_LENGTH];	/*!< The name of the thruster cluster Input message*/
	int32_t  			inputThrusterConfID;                   		/*!< [-] ID for the incoming Thruster configuration data*/

	vehEffectorOut thrFiringSchmittIn;								/*!< -- copy of the input message */
	vehEffectorOut thrFiringSchmittOut;								/*!< -- copy of the output message */

}thrFiringSchmittConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t moduleID);
    void CrossInit_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t moduleID);
    void Update_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
