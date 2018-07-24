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

#include "transDetermination/ephemNavConverter/ephemNavConverter.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/linearAlgebra.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/*! This method creates the output navigation message (translation only) for 
    the ephemeris model
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 */
void SelfInit_ephemNavConverter(EphemNavConverterData *ConfigData, uint64_t moduleID)
{
    ConfigData->stateOutMsgID = CreateNewMessage(ConfigData->stateOutMsgName,
        sizeof(NavTransIntMsg), "NavTransIntMsg", moduleID);
}

/*! This method initializes the input time correlation factor structure
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 */
void CrossInit_ephemNavConverter(EphemNavConverterData *ConfigData, uint64_t moduleID)
{

    ConfigData->ephInMsgID = subscribeToMessage(
        ConfigData->ephInMsgName, sizeof(EphemerisIntMsg), moduleID);

}

/*! This method takes the chebyshev coefficients loaded for the position 
    estimator and computes the coefficients needed to estimate the time 
    derivative of that position vector (velocity).
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Reset_ephemNavConverter(EphemNavConverterData *ConfigData, uint64_t callTime,
                         uint64_t moduleID)
{

}

/*! This method takes the current time and computes the state of the object
    using that time and the stored Chebyshev coefficients.  If the time provided 
    is outside the specified range, the position vectors rail high/low appropriately.
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_ephemNavConverter(EphemNavConverterData *ConfigData, uint64_t callTime, uint64_t moduleID)
{

    
    uint64_t writeTime;
    uint32_t writeSize;
    EphemerisIntMsg localEph;
    
    ReadMessage(ConfigData->ephInMsgID, &writeTime, &writeSize,
                sizeof(EphemerisIntMsg), &localEph, moduleID);
    
    memset(&ConfigData->outputState, 0x0, sizeof(NavTransIntMsg));

	ConfigData->outputState.timeTag = localEph.timeTag;
	v3Copy(localEph.r_BdyZero_N, ConfigData->outputState.r_BN_N);
	v3Copy(localEph.v_BdyZero_N, ConfigData->outputState.v_BN_N);
  
    WriteMessage(ConfigData->stateOutMsgID, callTime,
                 sizeof(NavTransIntMsg), &ConfigData->outputState, moduleID);

    return;

}
