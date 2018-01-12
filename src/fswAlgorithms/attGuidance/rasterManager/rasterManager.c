/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
 Inertial 3D Spin Module
 
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved
 
 */

#include "attGuidance/rasterManager/rasterManager.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"




void SelfInit_rasterManager(rasterManagerConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->outputEulerSetID = CreateNewMessage(ConfigData->outputEulerSetName,
                                                 sizeof(EulerAngleFswMsg),
                                                 "EulerAngleFswMsg",
                                                 moduleID);
    ConfigData->outputEulerRatesID = CreateNewMessage(ConfigData->outputEulerRatesName,
                                                    sizeof(EulerAngleFswMsg),
                                                    "EulerAngleFswMsg",
                                                    moduleID);
    ConfigData->mnvrActive = 0;
    ConfigData->scanSelector = 0;
    
}

void CrossInit_rasterManager(rasterManagerConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
}

void Reset_rasterManager(rasterManagerConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    ConfigData->mnvrActive = 0;
    ConfigData->scanSelector = 0;
}


void Update_rasterManager(rasterManagerConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t currentMnvrTime;
    ConfigData->scanSelector = ConfigData->scanSelector % ConfigData->numRasters;
    if (ConfigData->mnvrActive == 0)
    {
        ConfigData->mnvrStartTime = callTime;
        ConfigData->mnvrActive = 1;
    }
    currentMnvrTime = (callTime - ConfigData->mnvrStartTime) * 1E-9;
    if (currentMnvrTime < ConfigData->rasterTimes[ConfigData->scanSelector])
    {
        v3Copy(&ConfigData->scanningAngles[3 * ConfigData->scanSelector], ConfigData->outputAngleSet.set);
        v3Copy(&ConfigData->scanningRates[3 * ConfigData->scanSelector], ConfigData->outputAngleRates.set);
    } else {
        ConfigData->mnvrActive = 0.0;
        ConfigData->scanSelector += 1;
        printf("Raster: %i. AngleSet = [%f, %f, %f], RateSet = [%f, %f, %f] \n", ConfigData->scanSelector,
               ConfigData->outputAngleSet.set[0],
               ConfigData->outputAngleSet.set[1],
               ConfigData->outputAngleSet.set[2],
               ConfigData->outputAngleRates.set[0],
               ConfigData->outputAngleRates.set[1],
               ConfigData->outputAngleRates.set[2]);
    }
    
    
    WriteMessage(ConfigData->outputEulerSetID, callTime, sizeof(EulerAngleFswMsg),
                 (void*) &(ConfigData->outputAngleSet), moduleID);
    WriteMessage(ConfigData->outputEulerRatesID, callTime, sizeof(EulerAngleFswMsg),
                 (void*) &(ConfigData->outputAngleRates), moduleID);
    return;
}




