
#ifndef _NAV_STATE_OUT_H_
#define _NAV_STATE_OUT_H_

#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {
    double r_N[3];           /*!< [m]   Current inertial spacecraft position vector in
                              inertial frame N components */
    double v_N[3];           /*!< [m/s] Current inertial velocity of the spacecraft in
                              inertial frame N components */
    double vehAccumDV[3];    /*!< [m/s] Total accumulated delta-velocity for s/c*/
    double sigma_BN[3];      /*!<       Current spacecraft attitude (MRPs) of body relative to inertial */
    double omega_BN_B[3];    /*!< [r/s] Current spacecraft angular velocity vector of body
                              frame B relative to inertial frame N, in B frame components */
    double vehSunPntBdy[3];  /*!<       Current sun pointing vector in body frame*/
}NavStateOut;

/*! @} */

#endif
