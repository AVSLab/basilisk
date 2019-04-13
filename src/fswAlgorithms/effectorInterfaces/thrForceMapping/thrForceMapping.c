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

#include "effectorInterfaces/thrForceMapping/thrForceMapping.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>
#include <math.h>
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/bsk_Print.h"


/*! This method creates the module output message of type [THRArrayCmdForceFswMsg](\ref THRArrayCmdForceFswMsg).
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the ConfigData
 */
void SelfInit_thrForceMapping(thrForceMappingConfig *configData, uint64_t moduleID)
{
    /*! - Create output message for module */
    configData->thrusterForceOutMsgId = CreateNewMessage(configData->outputDataName,
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
    /*! - subscribe to the attitude control torque input message */
    configData->controlTorqueInMsgId = subscribeToMessage(configData->inputVehControlName,
                                                sizeof(CmdTorqueBodyIntMsg),
                                                moduleID);

    /*! - subscribe to the thruster configuration input message */
    configData->thrusterConfigInMsgId = subscribeToMessage(configData->inputThrusterConfName,
                                                       sizeof(THRArrayConfigFswMsg),
                                                       moduleID);

    /*! - subscribe to the vehicle configuration input message */
    configData->vehicleConfigDataInMsgId = subscribeToMessage(configData->inputVehicleConfigDataName,
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
    double             *pAxis;                  /* pointer to the current control axis */
    int                 i;
    THRArrayConfigFswMsg   localThrusterData;   /* local copy of the thruster data message */
    uint64_t            timeOfMsgWritten;
    uint32_t            sizeOfMsgWritten;

    /*! - configure the number of axes that are controlled */
    configData->numControlAxes = 0;
    for (i=0;i<3;i++)
    {
        pAxis = configData->controlAxes_B + 3*configData->numControlAxes;
        if (v3Norm(pAxis) > configData->epsilon) {
            v3Normalize(pAxis,pAxis);
            configData->numControlAxes += 1;
        } else {
            break;
        }
    }
    if (configData->numControlAxes==0) {
        BSK_PRINT(MSG_ERROR,"thrForceMapping() is not setup to control any axes!\n");
    }
    if (configData->thrForceSign==0) {
        BSK_PRINT(MSG_ERROR,"thrForceMapping() must have posThrustFlag set to either +1 or -1\n");
    }


    /*! - read in the support thruster and vehicle configuration messages */
    memset(&localThrusterData, 0x0, sizeof(THRArrayConfigFswMsg));
    ReadMessage(configData->thrusterConfigInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(THRArrayConfigFswMsg), &localThrusterData, moduleID);
    memset(&configData->sc, 0x0, sizeof(VehicleConfigFswMsg));
    ReadMessage(configData->vehicleConfigDataInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(VehicleConfigFswMsg), (void*) &(configData->sc), moduleID);

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    configData->numThrusters = localThrusterData.numThrusters;
    for(i=0; i<configData->numThrusters; i=i+1)
    {
        v3Copy(localThrusterData.thrusters[i].rThrust_B, configData->rThruster_B[i]);
        v3Copy(localThrusterData.thrusters[i].tHatThrust_B, configData->gtThruster_B[i]);
        if(localThrusterData.thrusters[i].maxThrust <= 0.0){
            BSK_PRINT(MSG_ERROR, "A configured thruster has a non-sensible saturation limit of <= 0 N!\n");
        } else {
            configData->thrForcMag[i] = localThrusterData.thrusters[i].maxThrust;
        }
    }
}

/*! The module takes a body frame torque vector and projects it onto available RCS or DV thrusters.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Update_thrForceMapping(thrForceMappingConfig *configData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t    timeOfMsgWritten;
    uint32_t    sizeOfMsgWritten;
    int         i,j,c;
    int         counterPosForces;             /* []      counter for number of positive thruster forces */
    double      F[MAX_EFF_CNT];               /* [N]     vector of commanded thruster forces */
    double      Fbar[MAX_EFF_CNT];            /* [N]     vector of intermediate thruster forces */
    double      D[3][MAX_EFF_CNT];            /* [m]     mapping matrix from thruster forces to body torque */
    double      Dbar[3][MAX_EFF_CNT];         /* [m]     reduced mapping matrix*/
    double      Lr_B[3];                      /* [Nm]    commanded ADCS control torque */
    double      Lr_offset[3];
    double      LrLocal[3];                   /* [Nm]    Torque provided by indiviual thruster */
    int         thrusterUsed[MAX_EFF_CNT];    /* []      Array of flags indicating if this thruster is used for the Lr_j */
    double      rThrusterRelCOM_B[MAX_EFF_CNT][3];/* [m]     local copy of the thruster locations relative to COM */
    uint32_t    numAvailThrusters;            /* []      number of available thrusters */
    double      BLr_B[3];                     /* [Nm]    Control torque that we actually control*/
    double      maxFractUse;                  /* []      ratio of maximum requested thruster force relative to maximum thruster limit */
    double      rCrossGt[3];
    CmdTorqueBodyIntMsg LrInputMsg;
    THRArrayCmdForceFswMsg thrusterForceOut;
    /*! - zero all message copies */
    memset(&LrInputMsg, 0x0, sizeof(CmdTorqueBodyIntMsg));
    memset(&configData->sc, 0x0, sizeof(VehicleConfigFswMsg));
    memset(&thrusterForceOut, 0x0, sizeof(THRArrayCmdForceFswMsg));
    
    /*! - clear arrays of the thruster mapping algorithm */
    vSetZero(F, MAX_EFF_CNT);
    mSetZero(D, 3, MAX_EFF_CNT);
    mSetZero(Dbar, 3, MAX_EFF_CNT);
    
    /*! - Read the input messages */
    ReadMessage(configData->controlTorqueInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CmdTorqueBodyIntMsg), (void*) &(LrInputMsg), moduleID);
    ReadMessage(configData->vehicleConfigDataInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(VehicleConfigFswMsg), (void*) &(configData->sc), moduleID);

    /*! - copy the request 3D attitude control torque vector */
    v3Copy(LrInputMsg.torqueRequestBody, Lr_B);

    /*! - compute thruster locations relative to COM */
    for (i=0;i<configData->numThrusters;i++) {
        v3Subtract(configData->rThruster_B[i], configData->sc.CoM_B, rThrusterRelCOM_B[i]); /* Part 1 of Eq. 4 */
    }

    /* temporary available thruster assignment until the thruster availability is read in through a message */
    numAvailThrusters = configData->numThrusters;
   
    /*! - compute general thruster force mapping matrix */
    v3SetZero(Lr_offset);
    for(i=0; i<numAvailThrusters; i=i+1)
    {
        if (configData->thrForcMag[i] <= 0)
        {
            continue; /* In the case where a hruster's maxThrust is purposefully (or accidentally) set to 0.0 or less; we exclude it from mapping solution. */
        }
        v3Cross(rThrusterRelCOM_B[i], configData->gtThruster_B[i], rCrossGt); /* Eq. 6 */
        for(j=0; j<3; j++)
        {
            D[j][i] = rCrossGt[j];
        }
        if(configData->thrForceSign < 0)  /* Need clarification -- Something to do with remove the torques produced by the fact that the torque vector in the body frame needs to be projected onto the COM */
        {
            v3Scale(configData->thrForcMag[i], rCrossGt, LrLocal); /* Computing local torques from each thruster -- Individual terms in Eq. 7*/
            v3Subtract(Lr_offset, LrLocal, Lr_offset); /* Summing of individual torques -- Eq. 5 & Eq. 7 */
        }
    }
    
    v3Add(Lr_offset, Lr_B, Lr_B);

    /*! - 1st iteration of finding a set of force vectors to implement the control torque */
    findMinimumNormForce(configData, D, Lr_B, numAvailThrusters, F, BLr_B);
    
    /*! - Remove forces components that are contributing to the RCS Null space (this is due to the geometry of the thrusters) */
    if (configData->thrForceSign>0)
    {
        substractMin(F, numAvailThrusters);
    }
    
    /* if the RCS Force solution includes negative forces, or if not all (DV or RCS) thrusters are available, recompute D excluding those thrusters */
    if (numAvailThrusters != configData->numThrusters || configData->thrForceSign<0) {
        counterPosForces = 0;
        memset(thrusterUsed,0x0,MAX_EFF_CNT*sizeof(int));
        for (i=0;i<numAvailThrusters;i++) {
            if (F[i]*configData->thrForceSign > 0) {
                thrusterUsed[i] = 1; /* Eq. 11 */
                for(j=0; j<3; j++)
                {
                    Dbar[j][counterPosForces] = D[j][i]; /* Eq. 12 */
                }
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
    
    configData->outTorqAngErr = computeTorqueAngErr(D, BLr_B, numAvailThrusters, configData->epsilon, F,
        configData->thrForcMag); /* Eq. 16*/
    maxFractUse = 0.0;
    /*  check if the angle between the request and actual torque exceeds a limit.  If so, then uniformly scale
        all thruster forces values to not exceed saturation.
        If the angle threshold is negative, then this scaling is bypassed.*/
    if(configData->outTorqAngErr > configData->angErrThresh)
    {
        for(i=0; i<numAvailThrusters; i++)
        {
            if(configData->thrForcMag[i] > 0.0 && fabs(F[i])/configData->thrForcMag[i] > maxFractUse) /* confirming that maxThrust > 0 */
            {
                maxFractUse = fabs(F[i])/configData->thrForcMag[i];
            }
        }
        /* only scale the requested thruster force if one or more thrusters are saturated */
        if(maxFractUse > 1.0)
        {
            vScale(1.0/maxFractUse, F, numAvailThrusters, F);
            configData->outTorqAngErr = computeTorqueAngErr(D, BLr_B, numAvailThrusters, configData->epsilon, F,
                                                            configData->thrForcMag);
        }
    }

    /* store the output message */
    mCopy(F, configData->numThrusters, 1, thrusterForceOut.thrForce);
    WriteMessage(configData->thrusterForceOutMsgId, callTime, sizeof(THRArrayCmdForceFswMsg),   /* update module name */
                 (void*) &(thrusterForceOut), moduleID);

    return;
}

/*!
 Take a stack of force values find the smallest value, and subtract if from all force values.  Here the smallest values
 will become zero, while other forces increase.  This assumes that the thrusters are aligned such that if all
 thrusters are firing, then no torque or force is applied.  This ensures only positive force values are computed.
 */
void substractMin(double *F, uint32_t size)
{
    double minValue;                        /* [N]    min or max value of the force set */
    int    i;

    /*! - initilize minimum force search by setting minimum to first force element */
    minValue = F[0];
    /*! - loop over all force element and search for smallest force */
    for (i=1;i<size;i++) {
        if (F[i] < minValue) {
            minValue = F[i];
        }
    }

    /*! - loop over forces and subtract the smallest force value */
    for (i=0;i<size;i++){
        F[i] -= minValue;
    }

    return;
}


/*!
 Use a least square inverse to determine the smallest set of thruster forces that yield the desired torque vector.  Note
 that this routine does not constrain yet the forces to be either positive or negative
 */
void findMinimumNormForce(thrForceMappingConfig *configData,
                          double D[3][MAX_EFF_CNT], double Lr_B[3], uint32_t numForces, double F[MAX_EFF_CNT], double Lr_B_Bar[3])
{
    
    int         i,j,k;                          /* []     counters */
    double      DDT[3][3];                      /* [m^2]  [D].[D]^T matrix */
    double      DDTInv[3][3];                   /* [m^2]  ([D].[D]^T)^-1 matrix */
    double      C[3][3];                        /* [m^2]  (C) matrix */
    double      CTC[3][3];                      /* [m^2]  ([C]^T.[C]) matrix */
    double      DDTInvLr[3];
    double      CD[3][MAX_EFF_CNT];

    /*! - copy the control axes */
    m33SetZero(C);
    for (i=0;i<configData->numControlAxes;i++) {
        v3Copy(&configData->controlAxes_B[3*i], C[i]);
    }
    
    /*! - map the control torque onto the control axes*/
    m33tMultM33(C, C, CTC);
    m33MultV3(CTC, Lr_B, Lr_B_Bar); /* Note: Lr_B_Bar is projected only onto the available control axes. i.e. if using DV thrusters with only 1 control axis, Lr_B_Bar = [#, 0, 0] */
    
    /*! - zero the output force vector */
    vSetZero(F, MAX_EFF_CNT);
    
    /* find [D].[D]^T */
    mMultM(C, 3, 3, D, 3, MAX_EFF_CNT, CD);
    m33SetIdentity(DDT);
    for(i=0; i<configData->numControlAxes; i++) {
        for(j=0; j<configData->numControlAxes; j++) {
            DDT[i][j] = 0.0;
            for (k=0;k<numForces;k++) {
                DDT[i][j] += CD[i][k] * CD[j][k]; // Part of Eq. 9 For DV thrusters, this actually forces the [3][3] entry to zero and therefore does not work.
            }
        }
    }

    /*
    m33SetZero(DDT);
    mMultMt(D, 3, MAX_EFF_CNT, D, 3, MAX_EFF_CNT, DDT);
    if (m33Determinant(DDT) < configData->epsilon) {
        for (i=0;i<3;i++){
            if(DDT[i][i] == 0){
                DDT[i][i] = 1.0; // This is required to invert the matrix in cases when there are fewer than 3 control axes, or when all thrusters are pointed in the same direction./
            }
        }
    }
    */
    
    m33Inverse(DDT, DDTInv);
    m33MultV3(DDTInv, Lr_B_Bar, DDTInvLr); /* If fewer than 3 control axes, then the 1's along the diagonal of DDTInv will not conflict with the mapping, as Lr_B_Bar contains the nessessary 0s to inhibit projection */
    for (i=0;i<numForces;i++) {
        F[i] = 0.0;
        for (k=0;k<3;k++) {
            F[i] += CD[k][i]*DDTInvLr[k]; /* Eq. 15*/
        }
    }
    return;

}

/*!
 Determine the angle between the desired torque vector and the actual torque vector.
 */
double computeTorqueAngErr(double D[3][MAX_EFF_CNT], double BLr_B[3], uint32_t numForces, double epsilon,
                           double F[MAX_EFF_CNT], double FMag[MAX_EFF_CNT])

{
    double returnAngle = 0.0;       /* [rad]  angle between requested and actual torque vector */
    /*! - make sure a control torque is requested, otherwise just return a zero angle error */
    if (v3Norm(BLr_B) > epsilon) {
        
        double tauActual_B[3];          /* [Nm]   control torque with current thruster solution */
        double BLr_hat_B[3];            /* []     normalized BLr_B vector */
        double LrEffector_B[3];         /* [Nm]   torque of an individual thruster effector */
        double thrusterForce;           /* [N]    saturation constrained thruster force */
        int i;
        double DT[MAX_EFF_CNT][3];
        mTranspose(D, 3, MAX_EFF_CNT, DT);
        v3Normalize(BLr_B, BLr_hat_B);
        v3SetZero(tauActual_B);

        /*! - loop over all thrusters and compute the actual torque to be applied */
        for(i=0; i<numForces; i++)
        {
            if (FMag[i] <= 0.0)
            {
                thrusterForce = 0.0; /* ensures that we don't divide by zero*/
            } else {
                 thrusterForce = fabs(F[i]) < FMag[i] ? F[i] : FMag[i]*fabs(F[i])/F[i]; /* This could produce inf's as F[i] approaches 0 if FMag[i] is 0, as such we check if FMag[i] is equal to zero in reset() */
            }
            v3Scale(thrusterForce, DT[i], LrEffector_B);
            v3Add(tauActual_B, LrEffector_B, tauActual_B);
        }

        /*! - evaluate the angle between the requested and thruster implemented torque vector */
        v3Normalize(tauActual_B, tauActual_B);
        if(v3Dot(BLr_hat_B, tauActual_B) < 1.0)
        {
            returnAngle = acos(v3Dot(BLr_hat_B, tauActual_B)); /* Eq 16 */
        }
    }
    return(returnAngle);
    
}
