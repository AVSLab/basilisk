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

#include "transDetermination/dvAccumulation/dvAccumulation.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include <string.h>

/*! This method initializes the ConfigData for the nav aggregation algorithm.  
    It initializes the output message in the messaging system.
 @return void
 @param ConfigData The configuration data associated with the Nav aggregation interface
 */
void SelfInit_dvAccumulation(DVAccumulationData *ConfigData, uint64_t moduleID)
{
}

/*! This method performs the second stage of initialization for the nav aggregration 
    interface.  For each configured message, it subscribes to the target message 
    and saves the ID.
 @return void
 @param ConfigData The configuration data associated with the aggregate nav interface
 */
void CrossInit_dvAccumulation(DVAccumulationData *ConfigData, uint64_t moduleID)
{
}

/*! This method takes the navigation message snippets created by the various 
    navigation components in the FSW and aggregates them into a single complete 
    navigation message.
 @return void
 @param ConfigData The configuration data associated with the aggregate nav module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_dvAccumulation(DVAccumulationData *ConfigData, uint64_t callTime, uint64_t moduleID)
{ 
    return;
}
