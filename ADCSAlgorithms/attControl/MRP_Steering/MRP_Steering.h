
#ifndef _MRP_STEERING_CONTROL_H_
#define _MRP_STEERING_CONTROL_H_

#include "messaging/static_messaging.h"
#include "../_GeneralModuleFiles/vehControlOut.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the MRP Steering attitude control routine. */
typedef struct {
    /* declare module private variables */
    double K1;                  /*!< [rad/sec] Proportional gain applied to MRP errors */
    double K3;                  /*!< [rad/sec] Cubic gain applied to MRP error in steering saturation function */
    double omega_max;           /*!< [rad/sec] Maximum rate command of steering control */
    double P;                   /*!< [N*m*s]   Rate error feedback gain applied  */
    double Ki;                  /*!< [N*m]     Integration feedback error on rate error  */
    double integralLimit;       /*!< [N*m]     Integration limit to avoid wind-up issue */
    uint64_t priorTime;         /*!< [ns]      Last time the attitude control is called */
    double z[3];                /*!< [rad]     integral state of delta_omega */

    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];   /*!< The name of the output message*/
    int32_t outputMsgID;                        /*!< [] ID for the outgoing body accel requests*/
    char inputGuidName[MAX_STAT_MSG_LENGTH];    /*!< The name of the Input message*/
    int32_t inputGuidID;                        /*!< [] ID for the incoming guidance errors*/
    char inputNavName[MAX_STAT_MSG_LENGTH];     /*!< The name of the Navigation 6Input message*/
    int32_t inputNavID;                         /*!< [] ID for the incoming navigation message */
    char inputVehicleConfigDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
    int32_t inputVehicleConfigDataID;           /*!< -- ID for the incoming static vehicle data */
    vehControlOut controlOut;                   /*!< -- Control output requests */
}MRP_SteeringConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_MRP_Steering(MRP_SteeringConfig *ConfigData, uint64_t moduleID);
    void CrossInit_MRP_Steering(MRP_SteeringConfig *ConfigData, uint64_t moduleID);
    void Update_MRP_Steering(MRP_SteeringConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_MRP_Steering(MRP_SteeringConfig *ConfigData);

    void MRPSteeringLaw(MRP_SteeringConfig *configData, double sigma_BR[3], double omega_ast[3], double omega_ast_p[3]);

    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
