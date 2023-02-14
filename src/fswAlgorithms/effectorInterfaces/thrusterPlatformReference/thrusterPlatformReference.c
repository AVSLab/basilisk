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

#include "thrusterPlatformReference.h"
#include "string.h"
#include <math.h>

#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/astroConstants.h"

const double epsilon = 1e-12;                           // module tolerance for zero

/*! This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_thrusterPlatformReference(ThrusterPlatformReferenceConfig *configData, int64_t moduleID)
{
    HingedRigidBodyMsg_C_init(&configData->hingedRigidBodyRef1OutMsg);
    HingedRigidBodyMsg_C_init(&configData->hingedRigidBodyRef2OutMsg);
    BodyHeadingMsg_C_init(&configData->bodyHeadingOutMsg);
    CmdTorqueBodyMsg_C_init(&configData->thrusterTorqueOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_thrusterPlatformReference(ThrusterPlatformReferenceConfig *configData, uint64_t callTime, int64_t moduleID)
{
    if (!VehicleConfigMsg_C_isLinked(&configData->vehConfigInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, " thrusterPlatformReference.vehConfigInMsg wasn't connected.");
    }
    if (RWArrayConfigMsg_C_isLinked(&configData->rwConfigDataInMsg) && RWSpeedMsg_C_isLinked(&configData->rwSpeedsInMsg)) {
        configData->momentumDumping = Yes;

        /*! - read in the RW configuration message */
        configData->rwConfigParams = RWArrayConfigMsg_C_read(&configData->rwConfigDataInMsg);
    }
    else {
        configData->momentumDumping = No;
    }
}


/*! This method updates the platformAngles message based on the updated information about the system center of mass
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_thrusterPlatformReference(ThrusterPlatformReferenceConfig *configData, uint64_t callTime, int64_t moduleID)
{

}
