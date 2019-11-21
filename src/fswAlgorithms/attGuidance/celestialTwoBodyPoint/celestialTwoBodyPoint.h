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
#include "simulation/utilities/bskPrint.h"



/*!@brief Data structure for module to compute the two-body celestial pointing navigation solution.
 */
typedef struct {
    /* Declare module private variables */
    double singularityThresh;       //!< (r) Threshold for when to fix constraint axis*/
    double R_P1B_N[3];              //!< [m] planet 1 position vector relative to inertial frame, in N-frame components
    double R_P2B_N[3];              //!< [m] planet 2 position vector relative to inertial frame, in N-frame components
    double v_P1B_N[3];              //!< [m/s] planet 1 velocity vector relative to inertial frame, in N-frame components
    double v_P2B_N[3];              //!< [m/s] planet 2 velocity vector relative to inertial frame, in N-frame components
    double a_P1B_N[3];              //!< [m/s^2] planet 1 acceleration vector relative to inertial frame, in N-frame components
    double a_P2B_N[3];              //!< [m/s^2] planet 2 acceleration vector relative to inertial frame, in N-frame components


    /* Declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       //!< The name of the output message*/
    char inputNavDataName[MAX_STAT_MSG_LENGTH];     //!< The name of the incoming attitude command*/
    char inputCelMessName[MAX_STAT_MSG_LENGTH];     //!< The name of the celestial body message*/
    char inputSecMessName[MAX_STAT_MSG_LENGTH];     //!< The name of the secondary body to constrain point*/
    int32_t outputMsgID;                            //!< (-) ID for the outgoing body estimate message*/
    int32_t inputNavID;                             //!< (-) ID for the incoming IMU data message*/
    int32_t inputCelID;                             //!< (-) ID for the incoming mass properties message*/
    int32_t inputSecID;                             //!< (-) ID for the secondary constraint message*/

    /* Output attitude reference data to send */
    AttRefFswMsg attRefOut;

    BSKPrint *bskPrint;                             //!< BSK Logging
}celestialTwoBodyPointConfig;

#ifdef __cplusplus
extern "C" {
#endif

    void SelfInit_celestialTwoBodyPoint(celestialTwoBodyPointConfig *configData, int64_t moduleID);
    void CrossInit_celestialTwoBodyPoint(celestialTwoBodyPointConfig *configData, int64_t moduleID);
    void Update_celestialTwoBodyPoint(celestialTwoBodyPointConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_celestialTwoBodyPoint(celestialTwoBodyPointConfig *configData, uint64_t callTime, int64_t moduleID);
    void parseInputMessages(celestialTwoBodyPointConfig *configData, int64_t moduleID);
    void computeCelestialTwoBodyPoint(celestialTwoBodyPointConfig *configData, uint64_t callTime);

#ifdef __cplusplus
}
#endif


#endif
