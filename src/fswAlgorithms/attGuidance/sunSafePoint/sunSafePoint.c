/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "attGuidance/sunSafePoint/sunSafePoint.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the sun safe attitude guidance.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the sun safe guidance
 */
void SelfInit_sunSafePoint(sunSafePointConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
        sizeof(AttGuidFswMsg), "AttGuidFswMsg", moduleID);
    memset(ConfigData->attOut.omega_RN_B, 0x0, 3*sizeof(double));
    memset(ConfigData->attOut.domega_RN_B, 0x0, 3*sizeof(double));
    
}

/*! This method performs the second stage of initialization for the sun safe attitude
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the sun safe attitude guidance
 */
void CrossInit_sunSafePoint(sunSafePointConfig *ConfigData, uint64_t moduleID)
{
    /*! - Loop over the number of sensors and find IDs for each one */
    ConfigData->inputMsgID = subscribeToMessage(ConfigData->inputSunVecName,
        sizeof(NavAttIntMsg), moduleID);
    ConfigData->imuMsgID = subscribeToMessage(ConfigData->inputIMUDataName,
        sizeof(IMUSensorBodyFswMsg), moduleID);
    
}

/*! This method takes the estimated body-observed sun vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param ConfigData The configuration data associated with the sun safe attitude guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_sunSafePoint(sunSafePointConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    NavAttIntMsg navMsg;
    uint64_t clockTime;
    uint32_t readSize;
    double ctSNormalized;
    double e_hat[3];
    double sigma_BR[3];
    IMUSensorBodyFswMsg LocalIMUData;
    /*! Begin method steps*/
    /*! - Read the current sun body vector estimate*/
    ReadMessage(ConfigData->inputMsgID, &clockTime, &readSize,
                sizeof(NavAttIntMsg), (void*) &(navMsg), moduleID);
    ReadMessage(ConfigData->imuMsgID, &clockTime, &readSize,
                sizeof(IMUSensorBodyFswMsg), (void*) &(LocalIMUData), moduleID);
    
    /*! - Compute the current error vector if it is valid*/
    if(v3Norm(navMsg.vehSunPntBdy) > ConfigData->minUnitMag)
    {
        /* a good sun direction vector is available */
        ctSNormalized = v3Dot(ConfigData->sHatBdyCmd, navMsg.vehSunPntBdy);
        ctSNormalized = fabs(ctSNormalized) > 1.0 ?
        ctSNormalized/fabs(ctSNormalized) : ctSNormalized;
        ConfigData->sunAngleErr = acos(ctSNormalized);
        v3Cross(navMsg.vehSunPntBdy, ConfigData->sHatBdyCmd, e_hat);
        v3Normalize(e_hat, ConfigData->sunMnvrVec);
        v3Scale(tan(ConfigData->sunAngleErr*0.25), ConfigData->sunMnvrVec,
                sigma_BR);
        v3Copy(sigma_BR, ConfigData->attOut.sigma_BR);
        MRPswitch(ConfigData->attOut.sigma_BR, 1.0, ConfigData->attOut.sigma_BR);
    } else {
        /* no proper sun direction vector is available */
        v3SetZero(ConfigData->attOut.sigma_BR);
    }
    v3Copy(LocalIMUData.AngVelBody, ConfigData->attOut.omega_BR_B);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(AttGuidFswMsg),
                 (void*) &(ConfigData->attOut), moduleID);
    
    return;
}
