
#ifndef _HILL_POINT_
#define _HILL_POINT_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/attGuidOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */


typedef struct {
    unsigned int        i_r;                        /* body axis index that lines up with i_r */
    unsigned int        i_theta;                    /* body axis index that lines up with i_theta */
    unsigned int        i_h;                        /* body axis index that lines up with i_h */
    int                 i_rSign;                    /* sign of the i_r axis alignment */
    int                 i_thetaSign;                /* sign of the i_theta axis alignment */
    int                 i_hSign;                    /* sign of the i_h axis alignment */
    int                 o_spin;                     /* orbit frame axis about which to spin */
    unsigned int        b_spin;                     /* principal body frame axis about which to spin */
    double              omega_spin;                 /* desired spin rate */
} OrbitFrameStates;

typedef struct {
    /* declare module private variables */
    OrbitFrameStates OrbitFrameStates;              /*!<        Nadir pointing states */
    
    double sigma_R0N[3];                            /*!<        MRP from inertial frame N to original Reference frame R0 at initial time t0 */
    double sigma_R0R[3];                            /*!<        MRP from corrected reference frame to original reference frame R0
                                                                This is the same as [BcB] going from primary body frame B to the corrected body frame Bc */
    double *sigma_BcB;                              /*!<        MRP from primary body frame B to corrected body frame Bc */
    
    double omega_RN_N[3];                           /*!< [r/s]  angular velocity vector of R relative to inertial N in N-frame components */
    double domega_RN_N[3];                          /*!< [r/s]  angular acceleration vector of R relative to inertial N in N-frame components */
    double sigma_RN[3];                             /*!<        MRP from inertial frame N to corrected reference frame R */
    
    
    uint64_t priorTime;                             /*!< [ns]   Last time the guidance module is called */

    
    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!< The name of the output message*/
    char inputNavName[MAX_STAT_MSG_LENGTH];         /*!< The name of the Navigation Input message */
    int32_t outputMsgID;                            /*!< ID for the outgoing message */
    int32_t inputNavID;                             /*!< ID for the incoming navigation message */

    attGuidOut attGuidOut;                          /*!< -- copy of the output message */

}hillPointConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_hillPoint(hillPointConfig *ConfigData, uint64_t moduleID);
    void CrossInit_hillPoint(hillPointConfig *ConfigData, uint64_t moduleID);
    void Update_hillPoint(hillPointConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_hillPoint(hillPointConfig *ConfigData);

    void computeHillPointAttitudeError(double sigma_BN[3], double omega_BN_B[3], double r_BN_N[3], double v_BN_N[3],
                                          hillPointConfig *ConfigData,
                                          double sigma_BR[3],
                                          double omega_BR_B[3],
                                          double omega_RN_B[3],
                                          double domega_RN_B[3]);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
