
#ifndef _LOW_PASS_FILTER_TORQUE_COMMAND_
#define _LOW_PASS_FILTER_TORQUE_COMMAND_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/vehControlOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

#define NUM_LPF       2                             /*            number of states to track, including current state */


/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double   h;                                     /*!< [s]      filter time step (assumed to be fixed */
    double   wc;                                    /*!< [rad/s]  continuous filter cut-off frequency */
    double   hw;                                    /*!< [rad]    h times the prewarped discrete time cut-off frequency */
    double   a[NUM_LPF];                            /*!<          filter coefficients for output */
    double   b[NUM_LPF];                            /*!<          filter coefficients for input */
    double   Lr[NUM_LPF][3];                        /*!< [Nm]     prior torque command */
    double   LrF[NUM_LPF][3];                       /*!< [Nm]     prior filtered torque command */
    uint64_t priorTime;                             /*!< [ns]     Last time the filter is called */
    int32_t  firstRun;                              /*!<          flag indicating the filter being started up */

    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!< The name of the output message*/
    char inputDataName[MAX_STAT_MSG_LENGTH];        /*!< The name of the Input message*/
    int32_t outputMsgID;                            /*!< [] ID for the outgoing filtered torque message */
    int32_t inputMsgID;                             /*!< [] ID for the commanded torque message */

    vehControlOut controlOut;                       /*!< -- Control output message */

}lowPassFilterTorqueCommandConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_LowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData, uint64_t moduleID);
    void CrossInit_LowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData, uint64_t moduleID);
    void Update_LowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_LowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
