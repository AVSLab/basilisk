/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
#include "attControl/thrForceMapping/thrForceMapping.h"

/* update this include to reflect the required module input messages */
#include "attControl/_GeneralModuleFiles/vehControlOut.h"
#include "vehicleConfigData/vehicleConfigData.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include <string.h>


/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */
#include "SimCode/utilities/linearAlgebra.h"


/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_thrForceMapping(thrForceMappingConfig *ConfigData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(vehEffectorOut),
                                               "vehEffectorOut",          /* add the output structure name */
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_thrForceMapping(thrForceMappingConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the input message ID's */
    ConfigData->inputVehControlID = subscribeToMessage(ConfigData->inputVehControlName,
                                                sizeof(vehControlOut),
                                                moduleID);

    ConfigData->inputThrusterConfID = subscribeToMessage(ConfigData->inputThrusterConfName,
                                                       sizeof(vehControlOut),
                                                       moduleID);

    ConfigData->inputVehicleConfigDataID = subscribeToMessage(ConfigData->inputVehicleConfigDataName,
                                                              sizeof(vehicleConfigData), moduleID);

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_thrForceMapping(thrForceMappingConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    double             *pAxis;                 /*!< pointer to the current control axis */
    int                 i;
    ThrusterCluster     localThrusterData;     /*!< local copy of the thruster data message */
    uint64_t            clockTime;
    uint32_t            readSize;

    /* configure the number of axes that are controlled */
    ConfigData->numOfAxesToBeControlled = 0;
    for (i=0;i<3;i++)
    {
        pAxis = ConfigData->controlAxes_B + 3*ConfigData->numOfAxesToBeControlled;
        if (v3Norm(pAxis) > 0.1) {
            v3Normalize(pAxis,pAxis);
            ConfigData->numOfAxesToBeControlled += 1;
        } else {
            break;
        }
    }
    if (ConfigData->numOfAxesToBeControlled==0) {
        printf("WARNING: thrForceMapping() is not setup to control any axes!\n");
    }
    if (ConfigData->thrForceSign ==0) {
        printf("WARNING: thrForceMapping() must have posThrustFlag set to either +1 or -1\n");
    }


    /* read in the support messages */
    ReadMessage(ConfigData->inputThrusterConfID, &clockTime, &readSize,
                sizeof(ThrusterCluster), &localThrusterData, moduleID);
    ReadMessage(ConfigData->inputVehicleConfigDataID, &clockTime, &readSize,
                sizeof(vehicleConfigData), (void*) &(ConfigData->sc), moduleID);

    /* read in the thruster position and thruster force heading information */
    /* Note: we will still need to correct for the S to B transformation */
    for(i=0; i<ConfigData->numThrusters; i=i+1)
    {
        m33MultV3(RECAST3X3 ConfigData->sc.BS,
                  localThrusterData.thrusters[i].rThrust_S,
                  ConfigData->rThruster_B[i]);
        m33MultV3(RECAST3X3 ConfigData->sc.BS,
                  localThrusterData.thrusters[i].tHatThrust_S,
                  ConfigData->gtThruster_B[i]);
    }

}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_thrForceMapping(thrForceMappingConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t    clockTime;
    uint32_t    readSize;
    double      F[MAX_EFF_CNT];               /*!< [N]     vector of commanded thruster forces */
    double      forcePerAxis[MAX_EFF_CNT];    /*!< [N]     vector of commanded thruster forces to produce a torque about a body axis */
    int         i,j,c,k;
    int         counterPosForces;             /*!< []      counter for number of positive thruster forces */
    double      tempVec[3];
    double      Lrj[3];                       /*!< [Nm]    control axis specific desired control torque vector */
    double      D2;
    double      D[MAX_EFF_CNT];               /*!< [m]     mapping matrix from thruster forces to body torque */
    double      Dbar[MAX_EFF_CNT][3];         /*!< [m]     mapping matrix from positive thruster forces to body torque 
                                                           Note, the vectors are stored as row vectors, not column vectors */
    double      DbarDbarT33[3][3];
    double      DbarTDbar22[2][2];
    double      DbarTDbar11;
    double      DTDDTinv[MAX_EFF_CNT];        /*!< [1/m]   mapping matrix from command torque Lr to thruster force sets */
    double      matInv33[3][3];
    double      matInv22[2][2];
    double      Lr_B[3];                      /*!< [Nm]    commanded ADCS control torque */
    int         thrusterUsed[MAX_EFF_CNT];    /*!< []      Array of flags indicating if this thruster is used for the Lr_j */
    double      rThrusterRelCOM_B[MAX_EFF_CNT][3];/*!< [m]     local copy of the thruster locations relative to COM */

    /*! Begin method steps*/
    /*! - Read the input messages */
    ReadMessage(ConfigData->inputVehControlID, &clockTime, &readSize,
                sizeof(vehControlOut), (void*) &(Lr_B), moduleID);
    ReadMessage(ConfigData->inputVehicleConfigDataID, &clockTime, &readSize,
                sizeof(vehicleConfigData), (void*) &(ConfigData->sc), moduleID);

    /* Lr is assumed to be a negative torque onto the body */
    v3Scale(-1.0, Lr_B, Lr_B);

    /* compute thruster locations relative to COM */
    for (i=0;i<ConfigData->numThrusters;i++) {
        v3Subtract(ConfigData->rThruster_B[i], ConfigData->sc.CoM_B, rThrusterRelCOM_B[i]);
    }

    /* clear the net thruster force output array */
    memset(F,0x0,MAX_EFF_CNT*sizeof(double));


    for (k=0;k<ConfigData->numOfAxesToBeControlled;k++)
    {
        D2 = 0.;
        for(i=0; i<ConfigData->numThrusters; i=i+1)
        {
            v3Cross(rThrusterRelCOM_B[i], ConfigData->gtThruster_B[i], tempVec);
            D[i] = v3Dot(tempVec, ConfigData->controlAxes_B+3*k);
            D2 += D[i]*D[i];
        }
        /* do a pseudo inverse (i.e. SVD inverse) if the determinant is near zero */
        for(i=0; i<ConfigData->numThrusters; i=i+1)
        {
            if (D2 > ConfigData->epsilon) {
                DTDDTinv[i] = D[i]/ D2;
            } else {
                DTDDTinv[i] = 0.0;
            }
        }

        /* check which thrusters require a positive thrust force */
        counterPosForces = 0;
        memset(thrusterUsed,0x0,MAX_EFF_CNT*sizeof(int));
        for (i=0;i<ConfigData->numThrusters;i++) {
            forcePerAxis[i] = DTDDTinv[i] * v3Dot(Lr_B,ConfigData->controlAxes_B+3*k);

            if (forcePerAxis[i]*ConfigData->thrForceSign > 0.0) {
                thrusterUsed[i] = 1;
                v3Cross(rThrusterRelCOM_B[i], ConfigData->gtThruster_B[i], Dbar[counterPosForces]);
                counterPosForces += 1;
            }
        }

        /* compute Dbar.Dbar^T to check if this is of rank 3 */
        for (i=0;i<3;i++){
            for (j=0;j<3;j++) {
                DbarDbarT33[i][j] = 0.0;
                for (c=0;c<counterPosForces;c++) {
                    DbarDbarT33[i][j] += Dbar[c][i]*Dbar[c][j];
                }
            }
        }

        memset(forcePerAxis,0x0,MAX_EFF_CNT*sizeof(double));
        v3Scale(v3Dot(Lr_B,ConfigData->controlAxes_B+3*k), ConfigData->controlAxes_B+3*k, Lrj);
        if (m33Determinant(DbarDbarT33) > ConfigData->epsilon) {
            /* 
             do a minimum norm inverse 
             */
            m33Inverse(DbarDbarT33, matInv33);
            m33MultV3(matInv33, Lrj, tempVec);
            for (i=0;i<counterPosForces;i++) {
                forcePerAxis[i] = v3Dot(Dbar[i], tempVec);
            }

        } else {
            /* 
             do a least squares inverse 
             */

            switch (counterPosForces) {
                case 2:
                    for (i=0;i<2;i++){
                        for (j=0;j<2;j++) {
                            DbarTDbar22[i][j] = 0.0;
                            for (c=0;c<3;c++) {
                                DbarTDbar22[i][j] += Dbar[i][c]*Dbar[j][c];
                            }
                        }
                    }

                    if (m22Determinant(DbarTDbar22) > ConfigData->epsilon) {
                        /* matrix is full rank, do regular least squares inverse */
                        m22Inverse(DbarTDbar22, matInv22);
                        for (i=0;i<2;i++) {
                            tempVec[i] = v3Dot(Dbar[i], Lrj);
                        }
                        m22MultV2(matInv22, tempVec, forcePerAxis);
                    } else {
                        /* matrix is not full rank, only one axis can be controlled. */
                        forcePerAxis[0] = v3Dot(Dbar[0], Lrj)/v3Norm(Dbar[0])/(1+v3Norm(Dbar[1])/v3Norm(Dbar[0]));
                        forcePerAxis[1] = forcePerAxis[0];
                    }
                    break;

                case 1:
                    DbarTDbar11 = v3Dot(Dbar[0], Dbar[0]);
                    if (DbarTDbar11 > ConfigData->epsilon) {
                        forcePerAxis[0] = v3Dot(Dbar[0], Lrj)/DbarTDbar11;
                    } else {
                        forcePerAxis[0] = 0.0;
                    }

                default:
                    /* the current control axis cannot be achieved with the currently available thrusters */
                    break;
            }

        }

        /* add the per axis thrust solutions to the overall thruster force solution */
        c = 0;
        for (i=0;i<ConfigData->numThrusters;i++) {
            if (thrusterUsed[i]) {
                F[i] += forcePerAxis[c];
                c += 1;
            }
        }

    }


    /*
     store the output message 
     */
    mCopy(F, ConfigData->numThrusters, 1, ConfigData->thrusterForceOut.effectorRequest);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehEffectorOut),   /* update module name */
                 (void*) &(ConfigData->thrusterForceOut), moduleID);

    return;
}
