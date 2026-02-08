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


#include "fswAlgorithms/effectorInterfaces/forceTorqueThrForceMapping/forceTorqueThrForceMapping.h"
#include "string.h"
#include "architecture/utilities/linearAlgebra.h"

/*!
    This method initializes the output messages for this module.

 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_forceTorqueThrForceMapping(forceTorqueThrForceMappingConfig  *configData, int64_t moduleID)
{
    THRArrayCmdForceMsg_C_init(&configData->thrForceCmdOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.

 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_forceTorqueThrForceMapping(forceTorqueThrForceMappingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    if (!THRArrayConfigMsg_C_isLinked(&configData->thrConfigInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping.thrConfigInMsg was not connected.");
    }
    if (!VehicleConfigMsg_C_isLinked(&configData->vehConfigInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping.vehConfigInMsg was not connected.");
    }

    VehicleConfigMsgPayload vehConfigInMsgBuffer;  //!< local copy of message buffer
    THRArrayConfigMsgPayload thrConfigInMsgBuffer;  //!< local copy of message buffer
    THRArrayCmdForceMsgPayload thrForceCmdOutMsgBuffer;  //!< local copy of message buffer

    //!< read the rest of the input messages
    thrConfigInMsgBuffer = THRArrayConfigMsg_C_read(&configData->thrConfigInMsg);
    vehConfigInMsgBuffer = VehicleConfigMsg_C_read(&configData->vehConfigInMsg);

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    configData->numThrusters = (uint32_t) thrConfigInMsgBuffer.numThrusters;
    v3Copy(vehConfigInMsgBuffer.CoM_B, configData->CoM_B);
    if (configData->numThrusters > MAX_EFF_CNT) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping thruster configuration input message has a number of thrusters that is larger than MAX_EFF_CNT");
    }

    /*! - copy the thruster position and thruster force heading information into the module configuration data */
    for(uint32_t i = 0; i < configData->numThrusters; i++)
    {
        v3Copy(thrConfigInMsgBuffer.thrusters[i].rThrust_B, configData->rThruster_B[i]);
        v3Copy(thrConfigInMsgBuffer.thrusters[i].tHatThrust_B, configData->gtThruster_B[i]);
        if(thrConfigInMsgBuffer.thrusters[i].maxThrust <= 0.0){
            _bskLog(configData->bskLogger, BSK_ERROR, "Error: forceTorqueThrForceMapping: A configured thruster has a non-sensible saturation limit of <= 0 N!");
        }
    }

    /* zero the thruster force command output message */
    thrForceCmdOutMsgBuffer = THRArrayCmdForceMsg_C_zeroMsgPayload();
    THRArrayCmdForceMsg_C_write(&thrForceCmdOutMsgBuffer, &configData->thrForceCmdOutMsg, moduleID, callTime);
}


/*! This method reallocates the thruster inputs to ensure non-negative values
    while satisfying the desired force/torque outputs.
 @return void
 @param DG The configuration matrix relating thruster inputs to force/torque outputs
 @param rows The number of rows in the DG matrix
 @param cols The number of columns in the DG matrix
 @param force_B The original thruster input vector (can contain negative values)
 @param forceTorque_B The desired force/torque output vector
 @param force_B_mod The modified thruster input vector (non-negative values)
*/
void reallocate_thrusters(double DG[][MAX_EFF_CNT], size_t rows, size_t cols, double *force_B, double *forceTorque_B, double *force_B_mod) {
    double adjustment = 0.0;

    // Initialize force_B_mod to be the same as force_B
    for (uint32_t i = 0; i < cols; i++) {
        force_B_mod[i] = force_B[i];
    }

    // Identify and accumulate the negative values
    for (uint32_t i = 0; i < cols; i++) {
        if (force_B_mod[i] < 1e-6) {
            adjustment += -force_B_mod[i];
            force_B_mod[i] = 0;  // Set negative values to zero
        }
    }

    // Redistribute the accumulated adjustment to non-negative values proportionally
    uint32_t num_positive = 0;
    for (uint32_t i = 0; i < cols; i++) {
        if (force_B_mod[i] > 0) {
            num_positive++;
        }
    }

    if (num_positive > 0) {
        for (uint32_t i = 0; i < cols; i++) {
            if (force_B_mod[i] > 0) {
                force_B_mod[i] += adjustment / num_positive;
            }
        }
    }

    // Verify the adjusted values satisfy the system equation
    double result[MAX_EFF_CNT] = {0};
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < cols; j++) {
            result[i] += DG[i][j] * force_B_mod[j];
        }
    }

    // Array to store the indices sorted by descending order of force_B_mod
    uint32_t idx[MAX_EFF_CNT];
    for (uint32_t i = 0; i < cols; i++) {
        idx[i] = i;
    }
    for (uint32_t i = 0; i < cols - 1; i++) {
        for (uint32_t j = 0; j < cols - 1 - i; j++) {
            if (force_B_mod[idx[j]] < force_B_mod[idx[j + 1]]) {
                // Swap indices
                uint32_t temp = idx[j];
                idx[j] = idx[j + 1];
                idx[j + 1] = temp;
            }
        }
    }

    // Adjust manually to match the exact desired output
    uint32_t len_idx = cols;
    for (uint32_t i = 0; i < rows; i++) {
        double discrepancy = forceTorque_B[i] - result[i];
        if (discrepancy != 0) {
            // Try to find thrusters that can adjust to match the discrepancy
            for (uint32_t j = 0; j < len_idx; j++) {
                uint32_t index = idx[j];
                if (DG[i][index] != 0 && force_B_mod[index] + discrepancy / DG[i][index] >= 0) {
                    force_B_mod[index] += discrepancy / DG[i][index];
                    // Recalculate the result
                    for (uint32_t k = 0; k < rows; k++) {
                        result[k] = 0;
                        for (uint32_t l = 0; l < cols; l++) {
                            result[k] += DG[k][l] * force_B_mod[l];
                        }
                    }

                    // Remove element j from idx
                    for (size_t k = j; k < len_idx; k++) {
                        idx[k] = idx[k + 1];
                    }
                    len_idx--;
                    break;
                }
            }
        }
    }
}


/*! Add a description of what this main Update() routine does for this module

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_forceTorqueThrForceMapping(forceTorqueThrForceMappingConfig *configData, uint64_t callTime, int64_t moduleID)
{
    CmdTorqueBodyMsgPayload cmdTorqueInMsgBuffer;  //!< local copy of message buffer
    CmdForceBodyMsgPayload cmdForceInMsgBuffer;  //!< local copy of message buffer
    THRArrayCmdForceMsgPayload thrForceCmdOutMsgBuffer;  //!< local copy of message buffer

    // always zero the output message buffers before assigning values
    thrForceCmdOutMsgBuffer = THRArrayCmdForceMsg_C_zeroMsgPayload();

    /* Check if torque message is linked and read, zero out if not*/
    if (CmdTorqueBodyMsg_C_isLinked(&configData->cmdTorqueInMsg)) {
        cmdTorqueInMsgBuffer = CmdTorqueBodyMsg_C_read(&configData->cmdTorqueInMsg);
    } else{
        cmdTorqueInMsgBuffer = CmdTorqueBodyMsg_C_zeroMsgPayload();
    }

    /* Check if force message is linked and read, zero out if not*/
    if (CmdForceBodyMsg_C_isLinked(&configData->cmdForceInMsg)) {
        cmdForceInMsgBuffer = CmdForceBodyMsg_C_read(&configData->cmdForceInMsg);
    } else{
        cmdForceInMsgBuffer = CmdForceBodyMsg_C_zeroMsgPayload();
    }

    /*
    // REMOVE WHEN DEBUGGING FINISHED
    printf("cmdForceInMsgBuffer: \n");
    int numElements = sizeof(cmdForceInMsgBuffer.forceRequestBody) / sizeof(cmdForceInMsgBuffer.forceRequestBody[0]);
    for (int i = 0; i < numElements; i++) {
        // Print each float followed by a space
        printf("%.3f ", cmdForceInMsgBuffer.forceRequestBody[i]);
    }
    printf("\n");
    // REMOVE WHEN DEBUGGING FINISHED 
    */
    
    /* Initialize variables */
    double DG[6][MAX_EFF_CNT];
    double rThrusterRelCOM_B[MAX_EFF_CNT][3];
    double rCrossGt[3];
    double zeroVector[MAX_EFF_CNT];
    uint32_t zeroRows[6];
    uint32_t numZeroes;
    double force_B[MAX_EFF_CNT];
    double forceTorque_B[6];
    double forceSubtracted_B[MAX_EFF_CNT];
    vSetZero(force_B, (size_t) MAX_EFF_CNT);
    vSetZero(forceSubtracted_B, (size_t) MAX_EFF_CNT);

    for (uint32_t i = 0; i < 6; i++) {
        for (uint32_t j = 0; j < MAX_EFF_CNT; j++) {
            DG[i][j] = 0.0;
        }
    }

    /* Create the torque and force vector */
    for (uint32_t i = 0; i < 3; i++) {
        forceTorque_B[i] = cmdTorqueInMsgBuffer.torqueRequestBody[i];
        forceTorque_B[i+3] = cmdForceInMsgBuffer.forceRequestBody[i];
    }

    /* - compute thruster locations relative to COM */
    for (uint32_t i = 0; i<configData->numThrusters; i++) {
        v3Subtract(configData->rThruster_B[i], configData->CoM_B, rThrusterRelCOM_B[i]);
    }

    /* Fill DG with thruster directions and moment arms */
    for (uint32_t i = 0; i < configData->numThrusters; i++) {
        /* Compute moment arm and fill in */
        v3Cross(rThrusterRelCOM_B[i], configData->gtThruster_B[i], rCrossGt);
        for(uint32_t j = 0; j < 3; j++) {
            DG[j][i] = rCrossGt[j];
        }

        /* Fill in control axes */
        for(uint32_t j = 0; j < 3; j++) {
            DG[j+3][i] = configData->gtThruster_B[i][j];
        }
    }

    /* Check DG for zero rows */
    vSetZero(zeroVector, configData->numThrusters);
    numZeroes = 0;
    for(uint32_t j = 0; j < 6; j++) {
        if (vIsEqual(zeroVector, configData->numThrusters, DG[j], 0.0000001)) {
            zeroRows[j] = 1;
            numZeroes += 1;
        } else {
            zeroRows[j] = 0;
        }
    }

    /* Create the DG w/ zero rows removed */
    double DG_full[6*MAX_EFF_CNT];
    vSetZero(DG_full, (size_t) 6*MAX_EFF_CNT);
    uint32_t row_idx = 0;
    for(uint32_t i = 0; i < 6; i++) {
        if (!zeroRows[i]) {
            for(uint32_t j = 0; j < MAX_EFF_CNT; j++) {
                DG_full[MXINDEX(MAX_EFF_CNT, row_idx, j)] = DG[i][j];
            }
            row_idx++;
        }
    }

    /* Compute the minimum norm inverse of DG*/
    double DGT_DGDGT_inv[6*MAX_EFF_CNT];
    mMinimumNormInverse(DG_full, (size_t) 6-numZeroes, (size_t) MAX_EFF_CNT, DGT_DGDGT_inv);

    /* Add the computed pseudoinverse values back into the correct positions*/
    double DG_inv_full[6 * MAX_EFF_CNT];
    vSetZero(DG_inv_full, (size_t) 6*MAX_EFF_CNT);
    uint32_t colIndex = 0;
    for (uint32_t i = 0; i < 6; ++i) {
        if (!zeroRows[i]) {
            for (uint32_t j = 0; j < 6; ++j) {
                DG_inv_full[j * configData->numThrusters + i] = DGT_DGDGT_inv[j * (configData->numThrusters - numZeroes) + colIndex];
            }
            colIndex++;
        }
    }

    /* Compute the force for each thruster */
    // mMultV(DGT_DGDGT_inv, (size_t) configData->numThrusters, (size_t) 6-numZeroes, forceTorque_B, force_B);
    mMultV(DG_inv_full, (size_t) configData->numThrusters, (size_t) 6, forceTorque_B, force_B);

    // /* Subtract the minimum force */
    // for(uint32_t i = 0; i < configData->numThrusters; i++) {
    //     forceSubtracted_B[i] = force_B[i] - min_force;
    // }

    // Elias: Reallocate thrusters to ensure non-negative values
    // Reallocate thrusters to ensure non-negative values
    reallocate_thrusters(DG, (size_t) 6, (size_t) configData->numThrusters, force_B, forceTorque_B, forceSubtracted_B);

    /*
    // REMOVE WHEN DEBUGGING FINISHED
    printf("Force allocated before applying saturation: \n");
    int numUnsat = sizeof(forceSubtracted_B) / sizeof(forceSubtracted_B[0]);
    for (int i = 0; i < numUnsat; i++) {
        // Print each float followed by a space
        printf("%.3f ", forceSubtracted_B[i]);
    }
    printf("\n");
    // REMOVE WHEN DEBUGGING FINISHED
    */
    
    // Thomas as of 20240805: Apply thruster saturation after mapping the Cmd forces to thrusters by reading the `THRArrayConfigMsg`:
    THRArrayConfigMsgPayload thrConfigInMsgBuffer;  //!< local copy of `THRArrayConfigMsg` buffer from `configData`
    thrConfigInMsgBuffer = THRArrayConfigMsg_C_read(&configData->thrConfigInMsg);
    for(uint32_t i = 0; i < configData->numThrusters; i++)
    {
        // Apply saturation based on maxThrust of each thrusters:
        if(forceSubtracted_B[i] > thrConfigInMsgBuffer.thrusters[i].maxThrust){
            forceSubtracted_B[i] = thrConfigInMsgBuffer.thrusters[i].maxThrust;
        }
    }

    /* Write to the output messages */
    vCopy(forceSubtracted_B, configData->numThrusters, thrForceCmdOutMsgBuffer.thrForce);
    THRArrayCmdForceMsg_C_write(&thrForceCmdOutMsgBuffer, &configData->thrForceCmdOutMsg, moduleID, callTime);

    /*
    // REMOVE WHEN DEBUGGING FINISHED
    printf("thrForceCmdOutMsg: \n");
    int num = sizeof(thrForceCmdOutMsgBuffer.thrForce) / sizeof(thrForceCmdOutMsgBuffer.thrForce[0]);
    for (int i = 0; i < num; i++) {
        // Print each float followed by a space
        printf("%.3f ", thrForceCmdOutMsgBuffer.thrForce[i]);
    }
    printf("\n");
    // REMOVE WHEN DEBUGGING FINISHED
    */
}
