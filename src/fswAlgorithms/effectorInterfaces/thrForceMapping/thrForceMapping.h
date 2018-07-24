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

#ifndef _THRUSTER_FORCE_MAPPING_H_
#define _THRUSTER_FORCE_MAPPING_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "fswMessages/vehicleConfigFswMsg.h"
#include "fswMessages/thrArrayConfigFswMsg.h"
#include "fswMessages/thrArrayCmdForceFswMsg.h"
#include "fswMessages/vehicleConfigFswMsg.h"
#include "simFswInterfaceMessages/cmdTorqueBodyIntMsg.h"

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*!@brief Data structure for module to map a command torque onto thruster forces.

 The module
 [PDF Description](Basilisk-ThrusterForces-20160627.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 */

typedef struct {
    /* declare module private variables */
    double   controlAxes_B[3*3];                    /*!< []      array of the control unit axes */
    double   rThruster_B[MAX_EFF_CNT][3];           /*!< [m]     local copy of the thruster locations */
    double   gtThruster_B[MAX_EFF_CNT][3];          /*!< []      local copy of the thruster force unit direction vectors */
    uint32_t numOfAxesToBeControlled;               /*!< []      counter indicating how many orthogonal axes are controlled */
    uint32_t numThrusters;                          /*!< []      The number of thrusters available on vehicle */
    int32_t  thrForceSign;                          /*!< []      Flag indicating if pos (+1) or negative (-1) thruster
                                                                 solutions are found */
    double thrForcMag[MAX_EFF_CNT];

    /* declare module IO interfaces */
    char     outputDataName[MAX_STAT_MSG_LENGTH];   /*!< The name of the output message*/
    int32_t  outputMsgID;                           /*!< ID for the outgoing message */
    char inputVehControlName[MAX_STAT_MSG_LENGTH];  /*!< The name of the vehicle control (Lr) Input message*/
    int32_t  inputVehControlID;                     /*!< ID for the incoming Lr control message */
    char inputThrusterConfName[MAX_STAT_MSG_LENGTH];/*!< The name of the thruster cluster Input message*/
    int32_t  inputThrusterConfID;                   /*!< [-] ID for the incoming Thruster configuration data*/
    char inputVehicleConfigDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
    int32_t inputVehicleConfigDataID;               /*!< [] ID for the incoming static vehicle data */
    VehicleConfigFswMsg   sc;                      /*!< spacecraft configuration message */
    double   epsilon;

    THRArrayCmdForceFswMsg thrusterForceOut;       /*!< -- copy of the output message */

}thrForceMappingConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_thrForceMapping(thrForceMappingConfig *ConfigData, uint64_t moduleID);
    void CrossInit_thrForceMapping(thrForceMappingConfig *ConfigData, uint64_t moduleID);
    void Update_thrForceMapping(thrForceMappingConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_thrForceMapping(thrForceMappingConfig *ConfigData, uint64_t callTime, uint64_t moduleID);

    void substractMin(double *F, uint32_t size);
    void findMinimumNormForce(thrForceMappingConfig *ConfigData,
                              double D[MAX_EFF_CNT][3], double Lr_B[3], uint32_t numForces, double F[MAX_EFF_CNT]);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
