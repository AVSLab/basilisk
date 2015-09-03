
#ifndef _NAV_STATE_OUT_H_
#define _NAV_STATE_OUT_H_

#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {
    double vehPosition[3];   /*!< m  Current position of the spacecraft*/
    double vehVelocity[3];   /*!< m/s Current velocity of the spacecraft*/
    double vehAccumDV[3];    /*!< m/s Total accumulated delta-velocity for s/c*/
    double vehSigma[3];      /*!< -- Current spacecraft attitude (MRPs)*/
    double vehBodyRate[3];   /*!< r/s Current spacecraft body rate wrt reference*/
    double vehSunPntBdy[3];   /*!< -- Current sun pointing vector in body frame*/
}NavStateOut;

/*! @} */

#endif
