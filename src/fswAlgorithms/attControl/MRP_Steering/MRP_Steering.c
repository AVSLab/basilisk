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
/*
    MRP_STEERING Module
 
 */

#include "attControl/MRP_Steering/MRP_Steering.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "fswMessages/rateCmdFswMsg.h"
#include "simulation/utilities/astroConstants.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_MRP_Steering(MRP_SteeringConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
        sizeof(RateCmdFswMsg), "RateCmdFswMsg", moduleID);

}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_MRP_Steering(MRP_SteeringConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message IDs*/
    ConfigData->inputGuidID = subscribeToMessage(ConfigData->inputGuidName,
                                                 sizeof(AttGuidFswMsg), moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_MRP_Steering(MRP_SteeringConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    return;
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param ConfigData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_MRP_Steering(MRP_SteeringConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    AttGuidFswMsg       guidCmd;            /*!< Guidance Message */
    uint64_t            clockTime;
    uint32_t            readSize;

    /*! Begin method steps*/
    
    memset(&guidCmd, 0x0, sizeof(AttGuidFswMsg));
    /*! - Read the dynamic input messages */
    ReadMessage(ConfigData->inputGuidID, &clockTime, &readSize,
                sizeof(AttGuidFswMsg), (void*) &(guidCmd), moduleID);

    /* evalute MRP kinematic steering law */
    MRPSteeringLaw(ConfigData, guidCmd.sigma_BR, ConfigData->outMsg.omega_BastR_B, ConfigData->outMsg.omegap_BastR_B);

    /* Store the output message and pass it to the message bus */
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(RateCmdFswMsg),
                 (void*) &(ConfigData->outMsg), moduleID);
    
    return;
}


/*! This method computes the MRP Steering law.  A commanded body rate is returned given the MRP
 attitude error measure of the body relative to a reference frame.  The function returns the commanded
 body rate, as well as the body frame derivative of this rate command.
 @return void
 @param ConfigData  The configuration data associated with this module
 @param sigma_BR    MRP attitude error of B relative to R
 @param omega_ast   Commanded body rates
 @param omega_ast_p Body frame derivative of the commanded body rates
 */
void MRPSteeringLaw(MRP_SteeringConfig *ConfigData, double sigma_BR[3], double omega_ast[3], double omega_ast_p[3])
{
    double  sigma_i;        /*!< ith component of sigma_B/R */
    double  B[3][3];        /*!< B-matrix of MRP differential kinematic equations */
    double  sigma_p[3];     /*!< the MRP rate equivalent of the steering rates */
    double  value;
    int     i;

    /* Determine the desired steering rates  */
    for (i=0;i<3;i++) {
        sigma_i      = sigma_BR[i];
        value        = atan(M_PI_2/ConfigData->omega_max*(ConfigData->K1*sigma_i
                       + ConfigData->K3*sigma_i*sigma_i*sigma_i))/M_PI_2*ConfigData->omega_max;
        omega_ast[i] = -value;

    }

    if (ConfigData->ignoreOuterLoopFeedforward) {
        v3SetZero(omega_ast_p);
    } else {
        /* Determine the body frame derivative of the steering rates */
        BmatMRP(sigma_BR, B);
        m33MultV3(B, omega_ast, sigma_p);
        v3Scale(0.25, sigma_p, sigma_p);
        for (i=0;i<3;i++) {
            sigma_i        = sigma_BR[i];
            value          = (3*ConfigData->K3*sigma_i*sigma_i + ConfigData->K1)/(pow(M_PI_2/ConfigData->omega_max*(ConfigData->K1*sigma_i + ConfigData->K3*sigma_i*sigma_i*sigma_i),2) + 1);
            omega_ast_p[i] = - value*sigma_p[i];
        }
    }
    return;
}
