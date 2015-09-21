
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
    double intsigma_BR[3];   /*!< --  Current integral error of the MRPs*/
    double sigma_BR[3];      /*!< --  Current attitude error estimate (MRPs)*/
    double omega_BR[3];      /*!< r/s Current body error estimate */
    double omega_rB[3];      /*!< r/s Commanded body rate of the vehicle*/
    double domega_rB[3];     /*!< r/s2 Commanded body accel of the vehicle*/
}attGuidOut;

/*! @} */

#endif
