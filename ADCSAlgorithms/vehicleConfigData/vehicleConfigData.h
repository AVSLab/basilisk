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

#ifndef _VEHICLE_CONFIG_DATA_H_
#define _VEHICLE_CONFIG_DATA_H_

#include <stdint.h>
#include "../ADCSAlgorithms/messaging/static_messaging.h"

/*! \addtogroup ADCSAlgGroup
 * @{
 */

#define MAX_EFF_CNT 36

/*! @brief Structure used to define a common structure for top level vehicle information*/
typedef struct {
    double BS[9];               /*!< [-] DCM from structure frame S to ADCS body frame B (row major)*/
    double ISCPntB_B[9];                /*!< [kg m^2] Spacecraft Inertia */
    double CoM_B[3];              /*!< [m] Center of mass of spacecraft in body*/
    uint32_t CurrentADCSState;  /*!< [-] Current ADCS state for subsystem */
}vehicleConfigData;


typedef struct {
    double gsHat_S[3];          /*!< [-] Spin axis unit vector of the wheel in structure */
    double Js;                  /*!< [kgm2] Spin axis inertia of the wheel */
}RWConfigurationElement;

typedef struct {
    int numRW;
    RWConfigurationElement reactionWheels[MAX_EFF_CNT];  /*!< [-] array of the reaction wheels */
}RWConstellation;

typedef struct {
    double rThrust_S[3];      /*!< [m] Location of the thruster in the spacecraft*/
    double tHatThrust_S[3];     /*!< [-] Unit vector of the thrust direction*/
	double maxThrust;			/*!< [N] Max thrust*/
}ThrusterPointData;

typedef struct {
    int numThrusters;
    ThrusterPointData thrusters[MAX_EFF_CNT];  /*! [-] array of thruster configuration information*/
}ThrusterCluster;

/*! @brief Structure used to define a common structure for top level vehicle information*/
typedef struct {
    double BS[9];               /*!< [-] DCM from structure frame S to ADCS body frame B (row major)*/
    double ISCPntB_S[9];                /*!< [kg m^2] Spacecraft Inertia */
    double CoM_S[3];              /*!< [m] Center of mass of spacecraft in body*/
    char outputPropsName[MAX_STAT_MSG_LENGTH]; /*!< [-] Name of the output properties message*/
    int32_t outputPropsID;       /*!< [-] Message ID associated with the output properties message*/
}VehConfigInputData;

void Update_vehicleConfigData(VehConfigInputData *ConfigData, uint64_t callTime, uint64_t moduleID);
void SelfInit_vehicleConfigData(VehConfigInputData *ConfigData, uint64_t moduleID);
void CrossInit_vehicleConfigData(VehConfigInputData *ConfigData, uint64_t moduleID);
/*! @} */

#endif
