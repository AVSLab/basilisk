/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "fswAlgorithms/attGuidance/locationPointing/locationPointing.h"
#include "string.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simulation/utilities/astroConstants.h"
#include <math.h>

/*!
    This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_locationPointing(locationPointingConfig  *configData, int64_t moduleID)
{
    AttGuidMsg_C_init(&configData->AttGuidOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_locationPointing(locationPointingConfig *configData, uint64_t callTime, int64_t moduleID)
{

    // check if the required message has not been connected
    if (!SCStatesMsg_C_isLinked(&configData->SCInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: locationPointing.SCInMsg was not connected.");
    }
    if (!GroundStateMsg_C_isLinked(&configData->LocationInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: locationPointing.LocationInMsg was not connected.");
    }

    configData->counter = 1;

    //configData->pHat = configData.pHat;

    configData->attGuidanceOutBuffer = AttGuidMsg_C_zeroMsgPayload();

    configData->sigma_RB_new = 0;

    configData->time_new = 0;

    return;
}


/*! This method takes the estimated body states and position relative to the ground to compute the current attitude/attitude rate errors and pass them to control.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_locationPointing(locationPointingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /* Local copies*/
    SCStatesMsgPayload SCInMsgBuffer;  //!< local copy of message buffer
    GroundStateMsgPayload LocationInMsgBuffer;  //!< local copy of message buffer
    AttGuidMsgPayload AttGuidOutMsgBuffer;  //!< local copy of message buffer

    // add variables
    double r_LC_N[3];               /*!< Position vector of location w.r.t spacecraft CoM in inertial frame*/
    //double pHat[3];                 /*!< Pointing vector of spacecraft (r_BN_N)*/
    double eHat[3];                /*!< --- Eigen Axis */
    double phi;                     /*!< angle phi for MRP calculations */
    double dsigma_RB[3];                /*< time derivative of sigma_BR*/
    double sigma_RB[3];
    double difference[3];
    double time_diff;
    double Bmat[3][3];                  /*!< BinvMRP for dsigma_RB_R calculations*/

    // zero output buffer
    AttGuidOutMsgBuffer = AttGuidMsg_C_zeroMsgPayload();

    // read in the input messages
    SCInMsgBuffer = SCStatesMsg_C_read(&configData->SCInMsg);
    LocationInMsgBuffer = GroundStateMsg_C_read(&configData->LocationInMsg);

    // PROCESS

    /* calculate r_LC_N*/
    v3Subtract(LocationInMsgBuffer.r_LN_N, SCInMsgBuffer.r_CN_N, r_LC_N);
    // convert from R_CL_N to R_LC_N
    //v3Scale(-1, r_LC_N, r_LC_N);

    /* populate pHat*/
    pHat = configData->pHat

    /* calculate eHat*/
    v3Cross(pHat, r_LC_N, eHat);
    v3Normalize(eHat, eHat);

    /* Calculate phi*/
    phi = acos(v3Dot(pHat, r_LC_N)/v3Norm(r_LC_N);

    /* can calculate sigma now*/
    v3Scale(tan(phi / 4), eHat, sigma_RB);
    v3Scale(-1, sigma_RB, configData->attGuidanceOutBuffer.sigma_BR);

    /* use sigma_BR to compute dsigma_BR if at least two data points*/
        /* counter keeps track of how many update cycles have run, need 2 for dsigma_BR and 4 for domega_RN_B*/
    if (configData->counter >= 2) {
        // update values for accurate finite diff
        configData->sigma_RB_old = configData->sigma_RB_new;
        configData->sigma_RB_new = sigma_RB;

        // get time data
        configData->time_old = configData->time_new;
        configData->time_new = callTime;
            // converted to seconds
        time_diff = (configData->time_new - configData->time_old)*1.0e-9;

        // assume difference is known - calculate dsigma_BR
        v3Subtract(sigma_RB_new, sigma_RB_old, difference);
        v3Scale(1/(time_diff), difference, dsigma_RB);

        // calculate BinvMRP
        BinvMRP(sigma_RB, B);
        
        // compute omega_BR_R
        configData->attGuidanceOutBuffer.omega_BR_B = 4*B*dsigma_RB;
        // IMPORTANT: figure out how to transform frame of w_RB_R

        /* compute omega_RN_B (subtract as need omega_RB not omega_BR)*/
        v3Subtract(SCInMsgBuffer.omega_BN_B, configData->attGuidanceOutBuffer.omega_BR_B, configData->attGuidanceOutBuffer.omega_RN_B)
           
        // if performed finite diff twice, then have enough for domega
        if (configData->counter >= 4) {
            // update time dependent data
            configData->omega_RN_B_old = configData->omega_RN_B_new;
            configData->omega_RN_B_new = configData->attGuidanceOutBuffer.omega_RN_B;

            // perform difference and compute finite diff
            v3Subtract(omega_RN_B_new, omega_RN_B_old, difference);
            v3Scale(1 / (time_diff), difference, configData->attGuidanceOutBuffer.domega_RN_B);

        }
        // else fill output with 0s
        else {
            v3SetZero(configData->attGuidanceOutBuffer.domega_RN_B);
        }


    }
    // else fill output with 0s
    else {
        v3SetZero(configData->attGuidanceOutBuffer.omega_BR_B);
    }

    // update counter and data buffers
    configData->counter++;
    configData->sigma_BR_new = configData->attGuidanceOutBuffer.sigma_BR;
    configData->omega_RN_B_new = configData->attGuidanceOutBuffer.omega_RN_B;
    configData->time_new = callTime;

    // write to the output messages
    AttGuidMsg_C_write(&AttGuidOutMsgBuffer, &configData->AttGuidOutMsg, moduleID, callTime);
}

