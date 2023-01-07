/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

/* Import the module header file */
#include "prescribedRot1DOF.h"

/* Other required files to import */
#include <stdbool.h>

/*! This method initializes the output messages and other parameters for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, int64_t moduleID)
{
    // Initialize the output messages
    PrescribedMotionMsg_C_init(&configData->prescribedMotionOutMsg);
    SpinningBodyMsg_C_init(&configData->spinningBodyOutMsg);

    // Set the initial time
    configData->tInit = 0.0;

    // Set the initial convergence to true to enter the required loop in Update_prescribedRot1DOF() on the first pass
    configData->convergence = true;

}


/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Reset_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // Check if the required input message is linked
    if (!SpinningBodyMsg_C_isLinked(&configData->spinningBodyInMsg))
    {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribedRot1DOF.spinningBodyInMsg wasn't connected.");
    }
}


/*! This method profiles the prescribed trajectory and updates the prescribed states as a function of time.
The prescribed states are then written to the output message.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Update_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, uint64_t callTime, int64_t moduleID)
{
}
