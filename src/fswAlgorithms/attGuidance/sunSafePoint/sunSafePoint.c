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
#include "simulation/utilities/bsk_Print.h"
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
    ConfigData->attGuidanceOutMsgID = CreateNewMessage(ConfigData->attGuidanceOutMsgName,
        sizeof(AttGuidFswMsg), "AttGuidFswMsg", moduleID);
    memset(ConfigData->attGuidanceOutBuffer.omega_RN_B, 0x0, 3*sizeof(double));
    memset(ConfigData->attGuidanceOutBuffer.domega_RN_B, 0x0, 3*sizeof(double));
    
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
    ConfigData->sunDirectionInMsgID = subscribeToMessage(ConfigData->sunDirectionInMsgName,
        sizeof(NavAttIntMsg), moduleID);
    ConfigData->imuInMsgID = subscribeToMessage(ConfigData->imuInMsgName,
        sizeof(IMUSensorBodyFswMsg), moduleID);
    
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the guidance module
 */
void Reset_sunSafePoint(sunSafePointConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    double v1[3];

    /* compute an Eigen axis orthogonal to sHatBdyCmd */
    if (v3Norm(ConfigData->sHatBdyCmd)  < 0.1) {
        BSK_PRINT(MSG_ERROR,"The module vector sHatBdyCmd is not setup as a unit vector [%f, %f %f]",
                  ConfigData->sHatBdyCmd[0], ConfigData->sHatBdyCmd[1], ConfigData->sHatBdyCmd[2]);
    } else {
        v3Set(1., 0., 0., v1);
        v3Normalize(ConfigData->sHatBdyCmd, ConfigData->sHatBdyCmd);    /* ensure that this vector is a unit vector */
        v3Cross(ConfigData->sHatBdyCmd, v1, ConfigData->eHat180_B);
        if (v3Norm(ConfigData->eHat180_B) < 0.1) {
            v3Set(0., 1., 0., v1);
            v3Cross(ConfigData->sHatBdyCmd, v1, ConfigData->eHat180_B);
        }
        v3Normalize(ConfigData->eHat180_B, ConfigData->eHat180_B);
    }

    return;
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
    double sNorm;                   /*!< --- Norm of measured direction vector */
    double e_hat[3];                /*!< --- Eigen Axis */
    double omega_BN_B[3];           /*!< r/s inertial body angular velocity vector in B frame components */
    IMUSensorBodyFswMsg localImuDataInBuffer;
    /*! Begin method steps*/
    /*! - Read the current sun body vector estimate*/
    ReadMessage(ConfigData->sunDirectionInMsgID, &clockTime, &readSize,
                sizeof(NavAttIntMsg), (void*) &(navMsg), moduleID);
    ReadMessage(ConfigData->imuInMsgID, &clockTime, &readSize,
                sizeof(IMUSensorBodyFswMsg), (void*) &(localImuDataInBuffer), moduleID);
    v3Copy(localImuDataInBuffer.AngVelBody, omega_BN_B);

    /*! - Compute the current error vector if it is valid*/
    sNorm = v3Norm(navMsg.vehSunPntBdy);
    if(sNorm > ConfigData->minUnitMag)
    {
        /* a good sun direction vector is available */
        ctSNormalized = v3Dot(ConfigData->sHatBdyCmd, navMsg.vehSunPntBdy)/sNorm;
        ctSNormalized = fabs(ctSNormalized) > 1.0 ?
        ctSNormalized/fabs(ctSNormalized) : ctSNormalized;
        ConfigData->sunAngleErr = acos(ctSNormalized);

        /*
            Compute the heading error relative to the sun direction vector 
         */
        if (ConfigData->sunAngleErr < ConfigData->smallAngle) {
            /* sun heading and desired body axis are essentially aligned.  Set attitude error to zero. */
             v3SetZero(ConfigData->attGuidanceOutBuffer.sigma_BR);
        } else {
            if (M_PI - ConfigData->sunAngleErr < ConfigData->smallAngle) {
                /* the commanded body vector nearly is opposite the sun heading */
                v3Copy(ConfigData->eHat180_B, e_hat);
            } else {
                /* normal case where sun and commanded body vectors are not aligned */
                v3Cross(navMsg.vehSunPntBdy, ConfigData->sHatBdyCmd, e_hat);
            }
            v3Normalize(e_hat, ConfigData->sunMnvrVec);
            v3Scale(tan(ConfigData->sunAngleErr*0.25), ConfigData->sunMnvrVec,
                    ConfigData->attGuidanceOutBuffer.sigma_BR);
            MRPswitch(ConfigData->attGuidanceOutBuffer.sigma_BR, 1.0, ConfigData->attGuidanceOutBuffer.sigma_BR);
        }

        /* rate tracking error are the body rates to bring spacecraft to rest */
        v3Copy(omega_BN_B, ConfigData->attGuidanceOutBuffer.omega_BR_B);
        v3SetZero(ConfigData->attGuidanceOutBuffer.omega_RN_B);
    } else {
        /* no proper sun direction vector is available */
        v3SetZero(ConfigData->attGuidanceOutBuffer.sigma_BR);

        /* specify a body-fixed constant search rotation rate */
        v3Subtract(omega_BN_B, ConfigData->omega_RN_B, ConfigData->attGuidanceOutBuffer.omega_BR_B);
        v3Copy(ConfigData->omega_RN_B, ConfigData->attGuidanceOutBuffer.omega_RN_B);
    }

    /* write the Guidance output message */
    WriteMessage(ConfigData->attGuidanceOutMsgID, callTime, sizeof(AttGuidFswMsg),
                 (void*) &(ConfigData->attGuidanceOutBuffer), moduleID);
    
    return;
}
