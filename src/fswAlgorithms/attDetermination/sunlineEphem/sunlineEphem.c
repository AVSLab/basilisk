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

#include "sunlineEphem.h"
#include <string.h>
#include <math.h>
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"

/*! This method sets up the module output message of type [NavAttIntMsg](\ref NavAttIntMsg)
 @return void
 @param configData The configuration data associated with this module
 */
void SelfInit_sunlineEphem(sunlineEphemConfig *configData, int64_t moduleID)
{
    configData->bskPrint = _BSKPrint();
    /*! - Create output message for module */
    configData->navStateOutMsgId = CreateNewMessage(configData->navStateOutMsgName,
                                                    sizeof(NavAttIntMsg), "NavAttIntMsg", moduleID);

}


/*! This method performs the second stage of initialization for this module.
 Its primary function is to link the input messages that were created elsewhere.  The required
 input messages are the sun ephemeris message of type [EphemerisIntMsg](\ref EphemerisIntMsg),
 the spacecraft translational navigation message of type [NavTransIntMsg](\ref NavTransIntMsg)
 and the spacecraft attitude navigation message of type [NavAttIntMsg](\ref NavAttIntMsg).
 @return void
 @param configData The configuration data associated with this module
 */
void CrossInit_sunlineEphem(sunlineEphemConfig *configData, int64_t moduleID)
{
    /*! - Find the message ID for the sun direction */
    configData->sunPositionInMsgId = subscribeToMessage(configData->sunPositionInMsgName,
                                                        sizeof(EphemerisIntMsg), moduleID);
    
    /*! - Find the messgae ID for the spacecraft direction */
    configData->scPositionInMsgId = subscribeToMessage(configData->scPositionInMsgName,
                                                       sizeof(NavTransIntMsg), moduleID);
    
    /*! - Find the messgae ID for the spacecraft attitude */
    configData->scAttitudeInMsgId = subscribeToMessage(configData->scAttitudeInMsgName,
                                                       sizeof(NavAttIntMsg), moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 */
void Reset_sunlineEphem(sunlineEphemConfig *configData, uint64_t callTime, int64_t moduleID)
{
    
}

/*! Updates the sun heading based on ephemeris data. Returns the heading as a unit vector in the body frame.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_sunlineEphem(sunlineEphemConfig *configData, uint64_t callTime, int64_t moduleID)
{
    uint64_t timeOfMsgWritten;      /* [ns] Read time when message was written*/
    uint32_t sizeOfMsgWritten;      /* [-] Non-zero size indicates we received ST msg*/
    double r_SB_N[3];               /* [m] difference between the sun and spacecrat in the inertial frame (unit length) */
    double r_SB_N_hat[3];           /* [m] difference between the sun and spacecrat in the inertial frame (unit length) */
    double r_SB_B_hat[3];           /* [m] difference between the sun and spacecrat in the body frame (of unit length) */
    double BN[3][3];                /* [-] direction cosine matrix used to rotate the inertial frame to body frame */
    NavAttIntMsg outputSunline;     /* [-] Output sunline estimate data */
    EphemerisIntMsg sunEphemBuffer; /* [-] Input sun ephemeris data */
    NavTransIntMsg scTransBuffer;   /* [-] Input spacecraft position data */
    NavAttIntMsg scAttBuffer;       /* [-] Input spacecraft attitude data */
    
    /*! - Read the input messages */
    memset(&outputSunline, 0x0, sizeof(NavAttIntMsg));
    memset(&sunEphemBuffer, 0x0, sizeof(EphemerisIntMsg));
    memset(&scTransBuffer, 0x0, sizeof(NavTransIntMsg));
    memset(&scAttBuffer, 0x0, sizeof(NavAttIntMsg));
    ReadMessage(configData->sunPositionInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(EphemerisIntMsg), (void*) &sunEphemBuffer, moduleID);
    ReadMessage(configData->scPositionInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(NavTransIntMsg), (void*) &scTransBuffer, moduleID);
    ReadMessage(configData->scAttitudeInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(NavAttIntMsg), (void*) &scAttBuffer, moduleID);

    /*! - Calculate Sunline Heading from Ephemeris Data*/
    v3Subtract(sunEphemBuffer.r_BdyZero_N, scTransBuffer.r_BN_N, r_SB_N);
    v3Normalize(r_SB_N, r_SB_N_hat);
    MRP2C(scAttBuffer.sigma_BN, BN);
    m33MultV3(BN, r_SB_N_hat, r_SB_B_hat);
    v3Normalize(r_SB_B_hat, r_SB_B_hat);
    
    /*! - store the output message*/
    v3Copy(r_SB_B_hat, outputSunline.vehSunPntBdy);
    WriteMessage(configData->navStateOutMsgId, callTime, sizeof(NavAttIntMsg),
                 &(outputSunline), moduleID);
    return;
}
