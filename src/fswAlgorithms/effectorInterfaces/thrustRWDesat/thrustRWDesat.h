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

#ifndef _THRUST_RW_DESAT_H_
#define _THRUST_RW_DESAT_H_

#include "messaging/static_messaging.h"
#include "effectorInterfaces/errorConversion/dvAttEffect.h"
#include "fswMessages/vehicleConfigFswMsg.h"
#include "fswMessages/thrArrayConfigFswMsg.h"
#include "fswMessages/rwConstellationFswMsg.h"
#include "simFswInterfaceMessages/thrArrayOnTimeCmdIntMsg.h"
#include "fswMessages/vehicleConfigFswMsg.h"
#include <stdint.h>
#include <stdlib.h>


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the DV attitude effector management algorithm.  
   This algorithm is used to control both the RCS and DV thrusters when 
   executing a trajectory adjustment.*/
typedef struct {
    char inputSpeedName[MAX_STAT_MSG_LENGTH]; /*!< (-) The name of the input RW speeds message*/
    char inputRWConfigData[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the RWA configuration message*/
    char inputThrConfigName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the thruster configuration message*/
    char inputMassPropsName[MAX_STAT_MSG_LENGTH]; /*!< [-] Tha name of the input mass properties message*/
	char outputThrName[MAX_STAT_MSG_LENGTH];  /*!< (-) The name of the output thrust command block*/
	double rwAlignMap[3 * MAX_EFF_CNT]; /*!< (-) Alignment of the reaction wheel spin axes*/
	double thrAlignMap[3 * MAX_EFF_CNT]; /*!< (-) Alignment of the vehicle thrusters*/
	double thrTorqueMap[3 * MAX_EFF_CNT]; /*!< (-) Alignment of the vehicle thruster torques*/
	double maxFiring;          /*!< (s) Maximum time to fire a jet for*/
	double thrFiringPeriod;    /*!< (s) The amount of time to rest between thruster firings*/
	uint32_t numRWAs;          /*!< (-) Number of reaction wheels being desaturated*/
	uint32_t numThrusters;     /*!< (-) Number of thrusters available in the align map*/
	double accumulatedImp[3];  /*!< (s) The accumulated firing in the body frame*/
	double currDMDir[3];       /*!< (-) The current direction of momentum reduction*/
	double totalAccumFiring;   /*!< (s) The total thruster duration we've commanded*/
	double DMThresh;           /*!< (r/s) The point at which to stop decrementing momentum*/
	uint64_t previousFiring;   /*!< (ns) Time that the last firing command was given*/
    int32_t inputRWConfID;      /*!< [-] ID for the incoming RWA configuration data*/
    int32_t inputSpeedID;      /*!< (-) ID for the incoming RW speeds*/
    int32_t inputThrConID;     /*!< [-] ID for the thruster configuration data*/
    int32_t inputMassPropID;   /*!< [-] ID for the incoming mass property information*/
	int32_t outputThrID;       /*!< (-) ID for the outgoing thruster commands*/
}thrustRWDesatConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_thrustRWDesat(thrustRWDesatConfig *ConfigData, uint64_t moduleID);
    void CrossInit_thrustRWDesat(thrustRWDesatConfig *ConfigData, uint64_t moduleID);
    void Update_thrustRWDesat(thrustRWDesatConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
	void Reset_thrustRWDesat(thrustRWDesatConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
