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

#ifndef _VEHICLE_CONFIG_DATA_H_
#define _VEHICLE_CONFIG_DATA_H_

#include <stdint.h>
#include "messaging/static_messaging.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "fswMessages/vehicleConfigFswMsg.h"

/*! \addtogroup ADCSAlgGroup
 * @{
 */








/*! @brief Structure used to define a common structure for top level vehicle information*/
typedef struct {
    double ISCPntB_B[9];          /*!< [kg m^2] Spacecraft Inertia */
    double CoM_B[3];              /*!< [m] Center of mass of spacecraft in body*/
    char outputPropsName[MAX_STAT_MSG_LENGTH]; /*!< [-] Name of the output properties message*/
    int32_t outputPropsID;       /*!< [-] Message ID associated with the output properties message*/
}VehConfigInputData;


void Update_vehicleConfigData(VehConfigInputData *ConfigData, uint64_t callTime, uint64_t moduleID);
void SelfInit_vehicleConfigData(VehConfigInputData *ConfigData, uint64_t moduleID);
void CrossInit_vehicleConfigData(VehConfigInputData *ConfigData, uint64_t moduleID);
/*! @} */

#endif
