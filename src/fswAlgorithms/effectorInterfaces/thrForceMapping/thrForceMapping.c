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
    Simple Thruster Force Evaluation from

 */

/* modify the path to reflect the new module names */
#include "effectorInterfaces/thrForceMapping/thrForceMapping.h"

/* update this include to reflect the required module input messages */
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>
#include <math.h>


/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/bsk_Print.h"


/*! This method initializes the configData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the ConfigData
 */
void SelfInit_thrForceMapping(thrForceMappingConfig *configData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Create output message for module */
    configData->outputMsgID = CreateNewMessage(configData->outputDataName,
                                               sizeof(THRArrayCmdForceFswMsg),
                                               "THRArrayCmdForceFswMsg",
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the ConfigData
 */
void CrossInit_thrForceMapping(thrForceMappingConfig *configData, uint64_t moduleID)
{
    /*! - Get the input message ID's */
    configData->controlTorqueInMsgID = subscribeToMessage(configData->inputVehControlName,
                                                sizeof(CmdTorqueBodyIntMsg),
                                                moduleID);

    configData->inputThrusterConfID = subscribeToMessage(configData->inputThrusterConfName,
                                                       sizeof(THRArrayConfigFswMsg),
                                                       moduleID);

    configData->inputVehicleConfigDataID = subscribeToMessage(configData->inputVehicleConfigDataName,
                                                              sizeof(VehicleConfigFswMsg), moduleID);

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param moduleID The ID associated with the ConfigData
 */
void Reset_thrForceMapping(thrForceMappingConfig *configData, uint64_t callTime, uint64_t moduleID)
{
    double             *pAxis;                 /*!< pointer to the current control axis */
    int                 i;
    THRArrayConfigFswMsg   localThrusterData;     /*!< local copy of the thruster data message */
    uint64_t            timeOfMsgWritten;
    uint32_t            sizeOfMsgWritten;

    /* configure the number of axes that are controlled */
    configData->numControlAxes = 0;
    for (i=0;i<3;i++)
    {
        pAxis = configData->controlAxes_B + 3*configData->numControlAxes;
        if (v3Norm(pAxis) > 0.1) {
            v3Normalize(pAxis,pAxis);
            configData->numControlAxes += 1;
        } else {
            break;
        }
    }
    if (configData->numControlAxes==0) {
        BSK_PRINT(MSG_WARNING,"thrForceMapping() is not setup to control any axes!\n");
    }
    if (configData->thrForceSign==0) {
        BSK_PRINT(MSG_WARNING,"thrForceMapping() must have posThrustFlag set to either +1 or -1\n");
    }


    /* read in the support messages */
    memset(&localThrusterData, 0x0, sizeof(THRArrayConfigFswMsg));
    ReadMessage(configData->inputThrusterConfID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(THRArrayConfigFswMsg), &localThrusterData, moduleID);
    memset(&configData->sc, 0x0, sizeof(VehicleConfigFswMsg));
    ReadMessage(configData->inputVehicleConfigDataID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(VehicleConfigFswMsg), (void*) &(configData->sc), moduleID);

    /* read in the thruster position and thruster force heading information */
    /* Note: we will still need to correct for the S to B transformation */
    configData->numThrusters = localThrusterData.numThrusters;
    for(i=0; i<configData->numThrusters; i=i+1)
    {
        v3Copy(localThrusterData.thrusters[i].rThrust_B, configData->rThruster_B[i]);
        v3Copy(localThrusterData.thrusters[i].tHatThrust_B, configData->gtThruster_B[i]);
        configData->thrForcMag[i] = localThrusterData.thrusters[i].maxThrust;
    }
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the ConfigData
 */
void Update_thrForceMapping(thrForceMappingConfig *configData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t    timeOfMsgWritten;
    uint32_t    sizeOfMsgWritten;
    double      F[MAX_EFF_CNT];               /*!< [N]     vector of commanded thruster forces */
    double      Fbar[MAX_EFF_CNT];            /*!< [N]     vector of intermediate thruster forces */
    int         i,c;
    int         counterPosForces;             /*!< []      counter for number of positive thruster forces */
    double      D[MAX_EFF_CNT][3];            /*!< [m]     mapping matrix from thruster forces to body torque */
    double      Dbar[MAX_EFF_CNT][3];         /*!< [m]     reduced mapping matrix
                                                           Note, the vectors are stored as row vectors, not column vectors */
    double      Lr_B[3];                      /*!< [Nm]    commanded ADCS control torque */
    double      Lr_offset[3];
    double      LrLocal[3];
    int         thrusterUsed[MAX_EFF_CNT];    /*!< []      Array of flags indicating if this thruster is used for the Lr_j */
    double      rThrusterRelCOM_B[MAX_EFF_CNT][3];/*!< [m]     local copy of the thruster locations relative to COM */
    uint32_t    numAvailThrusters;      /*!< []      number of available thrusters */
    double      BLr_B[3];                     /*!< [Nm]    Control torque that we actually control*/
    double      maxFractUse;                  /*!< []      ratio of maximum requested thruster force relative to maximum thruster limit */

    THRArrayCmdForceFswMsg thrusterForceOut;
    
    /*! Begin method steps*/
    /*! - Read the input messages */
    CmdTorqueBodyIntMsg LrInputMsg;
    
    memset(&LrInputMsg, 0x0, sizeof(CmdTorqueBodyIntMsg));
    ReadMessage(configData->controlTorqueInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CmdTorqueBodyIntMsg), (void*) &(LrInputMsg), moduleID);
    memset(&configData->sc, 0x0, sizeof(VehicleConfigFswMsg));
    ReadMessage(configData->inputVehicleConfigDataID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(VehicleConfigFswMsg), (void*) &(configData->sc), moduleID);

    v3Copy(LrInputMsg.torqueRequestBody, Lr_B);

    /* compute thruster locations relative to COM */
    for (i=0;i<configData->numThrusters;i++) {
        v3Subtract(configData->rThruster_B[i], configData->sc.CoM_B, rThrusterRelCOM_B[i]); /* Part 1 of Eq. 4 */
    }


    /* clear the net thruster force output array */
    vSetZero(F, MAX_EFF_CNT);

    /* temporary available thruster assignment until the thruster availability is read in through a message */
    numAvailThrusters = configData->numThrusters;


    /* compute general mapping matrix */
    v3SetZero(Lr_offset);
    for(i=0; i<numAvailThrusters; i=i+1)
    {
        v3Cross(rThrusterRelCOM_B[i], configData->gtThruster_B[i], D[i]); /* Eq. 6 */
        if(configData->thrForceSign < 0)  /* Exclude thrusters generate a positive thrust */
        {
            v3Scale(configData->thrForcMag[i], D[i], LrLocal); /* Computing local torques from each thruster -- Individual terms in Eq. 7*/
            v3Subtract(Lr_offset, LrLocal, Lr_offset); /* Summing of individual torques -- Eq. 5 & Eq. 7 */
        }
    }
    v3Add(Lr_offset, Lr_B, Lr_B);
    findMinimumNormForce(configData, D, Lr_B, numAvailThrusters, F, BLr_B);
    if (configData->thrForceSign>0)
    {
        substractMin(F, numAvailThrusters);
    }

    /* if force solution includes negative forces, or if not all thrusters are available, recompute D excluding those thrusters */
    if (numAvailThrusters != configData->numThrusters || configData->thrForceSign<0) {
        counterPosForces = 0;
        memset(thrusterUsed,0x0,MAX_EFF_CNT*sizeof(int));
        for (i=0;i<numAvailThrusters;i++) {
            if (F[i]*configData->thrForceSign > configData->epsilon) {
                thrusterUsed[i] = 1;
                v3Copy(D[i], Dbar[counterPosForces]);
                counterPosForces += 1;
            }
        }
        findMinimumNormForce(configData, Dbar, Lr_B, counterPosForces, Fbar, BLr_B);

        c = 0;
        for (i=0;i<numAvailThrusters;i++) {
            if (thrusterUsed[i]) {
                F[i] = Fbar[c];
                c += 1;
            } else {
                F[i] = 0.0;
            }
        }
        if (configData->thrForceSign>0)
        {
            substractMin(F, numAvailThrusters);
        }
    }

    configData->outTorqAngErr = computeTorqueAngErr(D, BLr_B, numAvailThrusters, F,
        configData->thrForcMag);
    maxFractUse = 0.0;
    /*
        check if the angle between the request and actual torque exceeds a limit.  If then, then uniformly scale
        all thruster forces values to not exceed saturation.
        If the angle threshold is negative, then this scaling is bypassed.
     */
    if(configData->outTorqAngErr > configData->angErrThresh)
    {
        for(i=0; i<numAvailThrusters; i++)
        {
            if(configData->thrForcMag[i] > 0.0 && fabs(F[i])/configData->thrForcMag[i] > maxFractUse)
            {
                maxFractUse = fabs(F[i])/configData->thrForcMag[i];
            }
        }
        /* only scale the requested thruster force if one or more thrusters are saturated */
        if(maxFractUse > 1.0)
        {
            vScale(1.0/maxFractUse, F, numAvailThrusters, F);
            configData->outTorqAngErr = computeTorqueAngErr(D, BLr_B, numAvailThrusters, F,
                                                            configData->thrForcMag);
        }
    }

    /*
     store the output message 
     */
    mCopy(F, configData->numThrusters, 1, thrusterForceOut.thrForce);
    WriteMessage(configData->outputMsgID, callTime, sizeof(THRArrayCmdForceFswMsg),   /* update module name */
                 (void*) &(thrusterForceOut), moduleID);

    return;
}


void substractMin(double *F, uint32_t size)
{
    double minValue;                        /*!< [N]    min or max value of the force set */
    int    i;

    minValue = F[0];
    for (i=1;i<size;i++) {
        if (F[i] < minValue) {
            minValue = F[i];
        }
    }
    for (i=0;i<size;i++){
        F[i] -= minValue;
    }

    return;
}



void findMinimumNormForce(thrForceMappingConfig *configData,
                          double D[MAX_EFF_CNT][3], double Lr_B[3], uint32_t numForces, double F[MAX_EFF_CNT], double Lr_B_Bar[3])
{
    
    int i,j,k;                                  /*!< []     counters */
    double      DT[3][MAX_EFF_CNT];
    double      DDT[3][3];                      /*!< [m^2]  [D].[D]^T matrix */
    double      DDTInv[3][3];                   /*!< [m^2]  ([D].[D]^T)^-1 matrix */
    double      C[3][3];                        /*!< [m^2]  (C) matrix */
    double      CT[3][3];                       /*!< [m^2]  ([C]^T) matrix */
    double      CTC[3][3];                      /*!< [m^2]  ([C]^T.[C]) matrix */
    double      DDTInvLr[3];

    m33SetZero(C);
    
    for (i=0;i<configData->numControlAxes;i++) {
        v3Copy(&configData->controlAxes_B[3*i], C[i]);
    }
    
    m33Transpose(C, CT);
    m33MultM33(CT, C, CTC);
    m33MultV3(CTC, Lr_B, Lr_B_Bar);
    
    /* zero the output force vector */
    vSetZero(F, MAX_EFF_CNT);
    
    /* find [D].[D]^T */
    m33SetIdentity(DDT);
    
    for(i=0; i<3; i++) { /* Needed for DV thrusters in which there is only one control axis*/
        for(j=0; j<3; j++) {
            DDT[i][j] = 0.0;
            for (k=0;k<numForces;k++) {
                DDT[i][j] += D[k][i] * D[k][j]; /* Part of Eq. 9 */
            }
        }
    }
    m33PrintScreen("DDT_2", DDT);
    
    if (m33Determinant(DDT) < configData->epsilon)
    {
        for (i=0;i<3;i++)
        {
            if(DDT[i][i] == 0)
            {
                DDT[i][i] = 1.0; /* Accounts for DV thrusters */
            }
        }
    }
    
    m33Inverse(DDT, DDTInv);
    
    if(m33Determinant(DDT) > configData->epsilon)
    {
        v3SetZero(DDTInvLr);
        m33MultV3(DDTInv, Lr_B_Bar, DDTInvLr);
        v3PrintScreen("DDTInvLr", DDTInvLr);
        for (i=0;i<numForces;i++) {
            F[i] = 0.0;
            for (k=0;k<3;k++) {
                F[i] += D[i][k]*DDTInvLr[k]; /* I think this is supposed to be D[k][i] */
            }
        }
    }
    return;

}

double computeTorqueAngErr(double D[MAX_EFF_CNT][3], double BLr_B[3], uint32_t numForces,
                           double F[MAX_EFF_CNT], double FMag[MAX_EFF_CNT])

{
    double returnAngle = 0.0;       /*!< [rad]  angle between requested and actual torque vector */
    /* make sure a control torque is requested, otherwise just return a zero angle error */
    if (v3Norm(BLr_B) > 0.0000000001) {
        
        double tauActual_B[3];          /*!< [Nm]   control torque with current thruster solution */
        double BLr_hat_B[3];            /*!< []     normalized BLr_B vector */
        double LrEffector_B[3];         /*!< [Nm]   torque of an individual thruster effector */
        double thrusterForce;           /*!< [N]    saturation constrained thruster force */
        int i;
        v3Normalize(BLr_B, BLr_hat_B);
        v3SetZero(tauActual_B);

        /* loop over all thrusters and compute the actual torque to be applied */
        for(i=0; i<numForces; i++)
        {
            thrusterForce = fabs(F[i]) < FMag[i] ? F[i] : FMag[i]*fabs(F[i])/F[i];
            v3Scale(thrusterForce, D[i], LrEffector_B);
            v3Add(tauActual_B, LrEffector_B, tauActual_B);
        }

        /* evaluate the angle between the requested and thruster implemented torque vector */
        v3Normalize(tauActual_B, tauActual_B);
        if(v3Dot(BLr_hat_B, tauActual_B) < 1.0)
        {
            returnAngle = acos(v3Dot(BLr_hat_B, tauActual_B));
        }
    }

    return(returnAngle);
    
}
