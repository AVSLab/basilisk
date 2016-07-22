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
#include "attControl/thrusterForceSimple/thrusterForceSimple.h"

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
void SelfInit_thrusterForceSimple(thrusterForceSimpleConfig *ConfigData, uint64_t moduleID)
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
void CrossInit_thrusterForceSimple(thrusterForceSimpleConfig *ConfigData, uint64_t moduleID)
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
void Reset_thrusterForceSimple(thrusterForceSimpleConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
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
        printf("WARNING: thrusterForceSimple() is not setup to control any axes!\n");
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
        v3Subtract(ConfigData->rThruster_B[i], ConfigData->sc.CoM_B, ConfigData->rThruster_B[i]);
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
void Update_thrusterForceSimple(thrusterForceSimpleConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t    clockTime;
    uint32_t    readSize;
    double      F[MAX_EFF_CNT];               /*!< [N] vector of commanded thruster forces */
    double      forcePerAxis[MAX_EFF_CNT];    /*!< [N] vector of commanded thruster forces to produce a torque about a body axis */
    int         i,j;
    double      dumVec[3];
    double      D2;
    double      D[3][MAX_EFF_CNT];            /*!< [m]     mapping matrix from thruster forces to body torque */
    double      Gt[3][MAX_EFF_CNT];           /*!< []      matrix containing the thrust direction unit vectors gHat_t_i */
    double      DTDDTinv[MAX_EFF_CNT][3];     /*!< [1/m]   mapping matrix from command torque Lr to thruster force sets */
    double      Lr_B[3];                      /*!< [Nm]    commanded ADCS control torque */

    /*! Begin method steps*/
    /*! - Read the input messages */
    ReadMessage(ConfigData->inputVehControlID, &clockTime, &readSize,
                sizeof(vehControlOut), (void*) &(Lr_B), moduleID);
    ReadMessage(ConfigData->inputVehicleConfigDataID, &clockTime, &readSize,
                sizeof(vehicleConfigData), (void*) &(ConfigData->sc), moduleID);

    /* Lr is assumed to be a negative torque onto the body */
    v3Scale(-1.0, Lr_B, Lr_B);


    /* clear the net thruster force vector */
    memset(F,0x0,MAX_EFF_CNT*sizeof(double));

    /* 
     setup the required projection matrices 
     */
    for(i=0; i<ConfigData->numThrusters; i=i+1)
    {
        v3Cross(ConfigData->rThruster_B[i], ConfigData->gtThruster_B[i], dumVec);
        for (j=0;j<ConfigData->numOfAxesToBeControlled; j++)
        {
            D[j][i] = v3Dot(dumVec, ConfigData->controlAxes_B+3*j);
        }
        for (j=0;j<3;j++)
        {
            Gt[j][i] = ConfigData->rThruster_B[i][j];
        }
    }



    for (j=0;j<ConfigData->numOfAxesToBeControlled;j++)
    {
        D2 = 0.;
        for(i=0; i<ConfigData->numThrusters; i=i+1)
        {
            D2 += D[j][i]*D[j][i];
        }
        /* do a pseudo inverse (i.e. SVD inverse) if the determinant is near zero */
        for(i=0; i<ConfigData->numThrusters; i=i+1)
        {
            if (D2 > ConfigData->epsilon) {
                DTDDTinv[i][j] = D[j][i]/ D2;
            } else {
                DTDDTinv[i][j] = 0.0;
            }
        }
    }


    /*
        Loop over each body axis and compute the set of positive thruster to yield Lr(j)
     */

    for (j=0;j<ConfigData->numOfAxesToBeControlled;j++) {

        memset(forcePerAxis,0x0,MAX_EFF_CNT*sizeof(double));      /* clear the per-axis force array */

        for (i=0;i<ConfigData->numThrusters;i++) {
            /* compute the minimum norm solution for each thruster */
            forcePerAxis[i] += DTDDTinv[i][j] * v3Dot(Lr_B,ConfigData->controlAxes_B+3*j);
            /* enforce that the thrust must be positive */
            if (forcePerAxis[i] > 0.0) {
                forcePerAxis[i] *= 2.0;
            } else {
                forcePerAxis[i] = 0.0;
            }

            /* add the per-axis thrust solution to the net thruster sum */
            F[i] += forcePerAxis[i];
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
