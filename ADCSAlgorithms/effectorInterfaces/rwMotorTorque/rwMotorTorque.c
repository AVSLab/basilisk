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
#include "effectorInterfaces/rwMotorTorque/rwMotorTorque.h"

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
void SelfInit_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t moduleID)
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
void CrossInit_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the input message ID's */
    ConfigData->inputVehControlID = subscribeToMessage(ConfigData->inputVehControlName,
                                                sizeof(vehControlOut),
                                                moduleID);

    ConfigData->inputRWConfID = subscribeToMessage(ConfigData->inputRWConfigDataName,
                                                       sizeof(RWConstellation),
                                                       moduleID);

    ConfigData->inputVehicleConfigDataID = subscribeToMessage(ConfigData->inputVehicleConfigDataName,
                                                              sizeof(vehicleConfigData), moduleID);
    
    ConfigData->inputRWsAvailID = subscribeToMessage(ConfigData->inputRWsAvailDataName,
                                                   sizeof(RWAvailabilityData),
                                                   moduleID);
    
    
    double             *pAxis;                 /*!< pointer to the current control axis */
    int                 i;
    RWConstellation     localRWData;           /*!< local copy of the RWA information */
    RWAvailabilityData  localRWsAvailData;
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
        printf("WARNING: rwMotorTorque() is not setup to control any axes!\n");
    }
    
    
    /* read in the support messages */
    ReadMessage(ConfigData->inputVehicleConfigDataID, &clockTime, &readSize,
                sizeof(vehicleConfigData), (void*) &(ConfigData->sc), moduleID);
    
    ReadMessage(ConfigData->inputRWConfID, &clockTime, &readSize,
                sizeof(RWConstellation), &localRWData, moduleID);
    /* read in the RW spin axis gsHat information */
    /* Note: we will still need to correct for the S to B transformation */
    ConfigData->numRW = localRWData.numRW;
    for(i=0; i<ConfigData->numRW; i=i+1)
    {
        m33MultV3(RECAST3X3 ConfigData->sc.BS,
                  localRWData.reactionWheels[i].gsHat_S,
                  ConfigData->gsHat_B[i]);
    }

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{

}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t    clockTime;
    uint32_t    readSize;
    double      us[MAX_EFF_CNT];              /*!< [Nm]     vector of RW motor torque commands */
    int         i,j,k;
    double      GsTCT[MAX_EFF_CNT][3];        /*!< []       [Gs]^T C^T */
    double      mat3x3[3][3];
    double      mat2x2[2][2];
    double      mat1x1;
    double      vec[3];
    double      CLr[3];
    double      Lr_B[3];                      /*!< [Nm]    commanded ADCS control torque */

    /*! Begin method steps*/
    /*! - Read the input messages */
    ReadMessage(ConfigData->inputVehControlID, &clockTime, &readSize,
                sizeof(vehControlOut), (void*) &(Lr_B), moduleID);
    
    /* #TODO: Do something with availability information */
    RWAvailabilityData wheelsAvailability;
    ReadMessage(ConfigData->inputRWsAvailID, &clockTime, &readSize,
                sizeof(RWAvailabilityData), &wheelsAvailability, moduleID);


    /* Lr is assumed to be a negative torque onto the body */
    v3Scale(+1.0, Lr_B, Lr_B);

    /* clear the RW motoro torque output array */
    memset(us,0x0,MAX_EFF_CNT*sizeof(double));

    /* compute [C].Lr */
    for (k=0;k<ConfigData->numOfAxesToBeControlled;k++) {
        CLr[k] = v3Dot(ConfigData->controlAxes_B+3*k, Lr_B);
    }

    /* compute [Gs]^T [C]^T */
    for (i=0;i<ConfigData->numRW;i++) {
        for (j=0;j<ConfigData->numOfAxesToBeControlled; j++) {
            GsTCT[i][j] = v3Dot(ConfigData->gsHat_B[i], ConfigData->controlAxes_B+3*j);
        }
    }


    if (ConfigData->numOfAxesToBeControlled == 3) {
        /* compute [C].[Gs].[Gs]^T.[C]^T */
        for (i=0;i<3;i++) {
            for (j=0;j<3;j++) {
                mat3x3[i][j] = 0.;
                for (k=0;k<ConfigData->numRW;k++) {
                    mat3x3[i][j] += GsTCT[k][i]*GsTCT[k][j];
                }
            }
        }

        m33Inverse(mat3x3, mat3x3);
        m33MultV3(mat3x3, CLr, vec);


    } else if (ConfigData->numOfAxesToBeControlled == 2) {
        /* compute [B].[Gs].[Gs]^T.[B]^T */
        for (i=0;i<2;i++) {
            for (j=0;j<2;j++) {
                mat2x2[i][j] = 0.;
                for (k=0;k<ConfigData->numRW;k++) {
                    mat2x2[i][j] += GsTCT[k][i]*GsTCT[k][j];
                }
            }
        }

        m22Inverse(mat2x2, mat2x2);
        m22MultV2(mat2x2, CLr, vec);

    } else {
        /* compute [B].[Gs].[Gs]^T.[B]^T */
        mat1x1 = 0.;
        for (k=0;k<ConfigData->numRW;k++) {
            mat1x1 += GsTCT[k][0]*GsTCT[k][0];
        }

        mat1x1 = 1./mat1x1;
        vec[0] = mat1x1*CLr[0];
    }

    /* compute the RW motor torques */
    for (i=0;i<ConfigData->numRW;i++) {
        us[i] = 0.0;
        for (j=0;j<ConfigData->numOfAxesToBeControlled;j++) {
            us[i] += GsTCT[i][j]*vec[j];
        }
    }




    /*
     store the output message 
     */
    mCopy(us, ConfigData->numRW, 1, ConfigData->rwMotorTorques.effectorRequest);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehEffectorOut),   /* update module name */
                 (void*) &(ConfigData->rwMotorTorques), moduleID);

    return;
}
