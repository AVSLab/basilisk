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

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "transDetermination/ephemNavConverter/ephemNavConverter.h"
#include "simFswInterfaceMessages/ephemerisIntMsg.h"
#include "simFswInterfaceMessages/navTransIntMsg.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/linearAlgebra.h"

/*! This method creates the output navigation message (translation only) for
    the ephemeris model
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void SelfInit_ephemNavConverter(EphemNavConverterData *configData, int64_t moduleID)
{
    configData->bskLogger = _BSKLogger();
    configData->stateOutMsgID = CreateNewMessage(configData->stateOutMsgName,
                                                 sizeof(NavTransIntMsg),
                                                 "NavTransIntMsg",
                                                 moduleID);
}

/*! This method subscribes to the ephemeris interface message
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void CrossInit_ephemNavConverter(EphemNavConverterData *configData, int64_t moduleID)
{
    configData->ephInMsgID = subscribeToMessage(configData->ephInMsgName,
                                                sizeof(EphemerisIntMsg),
                                                moduleID);
}

/*! This resets the module to original states.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Reset_ephemNavConverter(EphemNavConverterData *configData, uint64_t callTime, int64_t moduleID)
{

}

/*! This method reads in the ephemeris messages and copies the translation
    ephemeris to the navigation translation interface message.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Update_ephemNavConverter(EphemNavConverterData *configData, uint64_t callTime, int64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    EphemerisIntMsg tmpEphemeris;
    NavTransIntMsg tmpOutputState;
    memset(&tmpEphemeris, 0x0, sizeof(EphemerisIntMsg));
    memset(&tmpOutputState, 0x0, sizeof(NavTransIntMsg));

    /*! - read input ephemeris message */
    ReadMessage(configData->ephInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(EphemerisIntMsg), &tmpEphemeris, moduleID);

    /*! - map timeTag, position and velocity vector to output message */
	tmpOutputState.timeTag = tmpEphemeris.timeTag;
	v3Copy(tmpEphemeris.r_BdyZero_N, tmpOutputState.r_BN_N);
	v3Copy(tmpEphemeris.v_BdyZero_N, tmpOutputState.v_BN_N);

    /*! - write output message */
    WriteMessage(configData->stateOutMsgID, callTime, sizeof(NavTransIntMsg),
                 &tmpOutputState, moduleID);

    return;
}
