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
    FSW MODULE Template

 */

/* modify the path to reflect the new module names */
#include "cModuleTemplate.h"
#include <stdio.h>

/* Pull in support files using paths relative to the Basilisk src directory. */
#include "architecture/utilities/linearAlgebra.h"


/*!
    This method initializes the output messages for this module.

 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_cModuleTemplate(cModuleTemplateConfig *configData, int64_t moduleID)
{
    (void) moduleID;
    CModuleTemplateMsg_C_init(&configData->dataOutMsg);
}


/*! @brief Reset the runtime counter and publish a zero output payload.
 @note The sampleConfigVector configuration is preserved. The inputVector scratch
 storage is refreshed on the next update.

 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_cModuleTemplate(cModuleTemplateConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! Reset the runtime counter; preserve the user-configured sampleConfigVector. */
    configData->updateCounter = 0.0;
    char info[MAX_LOGGING_LENGTH];
    snprintf(info, sizeof(info), "Variable updateCounter set to %f in reset.", configData->updateCounter);
    _bskLog(configData->bskLogger, BSK_INFORMATION, info);

    /* initialize the output message to zero on reset */
    CModuleTemplateMsgPayload outMsgBuffer;       /*!< local output message copy */
    outMsgBuffer = CModuleTemplateMsg_C_zeroMsgPayload();
    CModuleTemplateMsg_C_write(&outMsgBuffer, &configData->dataOutMsg, moduleID, callTime);
}

/*! @brief Add the update counter to the first component of the optional input vector.

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_cModuleTemplate(cModuleTemplateConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // Zero the output buffer each update to avoid publishing uninitialized fields.
    CModuleTemplateMsgPayload outMsgBuffer = CModuleTemplateMsg_C_zeroMsgPayload();
    CModuleTemplateMsgPayload inMsgBuffer;        /*!< local copy of input message */

    // Use a zero vector when the optional input is not connected.
    v3SetZero(configData->inputVector);

    /*! - Read the optional input messages */
    if (CModuleTemplateMsg_C_isLinked(&configData->dataInMsg)) {
        inMsgBuffer = CModuleTemplateMsg_C_read(&configData->dataInMsg);
        v3Copy(inMsgBuffer.dataVector, configData->inputVector);
    }

    // Sample math: copy the input vector and add the counter to its first component.
    v3Copy(configData->inputVector, outMsgBuffer.dataVector);
    configData->updateCounter += 1.0;  // [-]
    outMsgBuffer.dataVector[0] += configData->updateCounter;

    /*! - Write the module output message */
    CModuleTemplateMsg_C_write(&outMsgBuffer, &configData->dataOutMsg, moduleID, callTime);

    /* this logging statement is not typically required.  It is done here to see in the
     quick-start guide which module is being executed */
    char info[MAX_LOGGING_LENGTH];
    snprintf(info, sizeof(info), "C Module ID %lld ran Update at %fs",
             (long long int) moduleID, (double) callTime/(1e9));
    _bskLog(configData->bskLogger, BSK_INFORMATION, info);

}
