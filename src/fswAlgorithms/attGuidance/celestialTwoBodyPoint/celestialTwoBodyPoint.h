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

#ifndef _CELESTIAL_BODY_POINT_H_
#define _CELESTIAL_BODY_POINT_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "simFswInterfaceMessages/ephemerisIntMsg.h"
#include "simFswInterfaceMessages/navTransIntMsg.h"
#include "fswMessages/attRefFswMsg.h"

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*!@brief Data structure for module to compute the two-body celestial pointing navigation solution.

 The module
 [PDF Description](celestialTwoBodyPoint.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 */

typedef struct {
    /* Declare module private variables */
    double singularityThresh;                       /*!< (r) Threshold for when to fix constraint axis*/
    double R_P1[3];
    double R_P2[3];
    double v_P1[3];
    double v_P2[3];
    double a_P1[3];
    double a_P2[3];
    
    
    /* Declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!< The name of the output message*/
    char inputNavDataName[MAX_STAT_MSG_LENGTH];     /*<! The name of the incoming attitude command*/
    char inputCelMessName[MAX_STAT_MSG_LENGTH];     /*<! The name of the celestial body message*/
    char inputSecMessName[MAX_STAT_MSG_LENGTH];     /*<! The name of the secondary body to constrain point*/
    int32_t outputMsgID;                            /*!< (-) ID for the outgoing body estimate message*/
    int32_t inputNavID;                             /*!< (-) ID for the incoming IMU data message*/
    int32_t inputCelID;                             /*!< (-) ID for the incoming mass properties message*/
    int32_t inputSecID;                             /*!< (-) ID for the secondary constraint message*/
    
    /* Output attitude reference data to send */
    AttRefFswMsg attRefOut;
}celestialTwoBodyPointConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_celestialTwoBodyPoint(celestialTwoBodyPointConfig *ConfigData, uint64_t moduleID);
    void CrossInit_celestialTwoBodyPoint(celestialTwoBodyPointConfig *ConfigData, uint64_t moduleID);
    void Update_celestialTwoBodyPoint(celestialTwoBodyPointConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_celestialTwoBodyPoint(celestialTwoBodyPointConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void parseInputMessages(celestialTwoBodyPointConfig *ConfigData, uint64_t moduleID);
    void computecelestialTwoBodyPoint(celestialTwoBodyPointConfig *ConfigData, uint64_t callTime);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
