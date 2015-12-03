
#ifndef _ATT_GUID_OUT_H_
#define _ATT_GUID_OUT_H_

#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Structure used to define the format of an attitude maneuver command */
typedef struct {
   double sigma_BR[3];      /*!< -- Commanded body wrt reference that we are tracking*/
   double omega_BR[3];      /*!< r/s Body rate of that commanded attitude vector*/
}attCmdOut;

/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {
    double intsigma_BR[3];   /*!<        Current integral error of the MRPs*/
    double sigma_BR[3];      /*!<        Current attitude error estimate (MRPs) of B relative to R*/
    double omega_BR_B[3];    /*!< [r/s]  Current body error estimate of B relateive to R
                              in B frame compoonents */
    double omega_RN_B[3];    /*!< [r/s]  Commanded body rate of the vehicle of R relative to N 
                              in B frame components */
    double domega_RN_B[3];     /*!< [r/s2] Commanded inertial body accel of the vehicle of R relative
                                to N in B frame components */
}attGuidOut;

/*! @} */

#endif
