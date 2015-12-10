
#ifndef _VEH_CONTROL_OUT_H_
#define _VEH_CONTROL_OUT_H_

#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Structure used to define the output definition for vehicle control*/
typedef struct {
    double torqueRequestBody[3];     /*!< r/s2 Control output accel request*/
}vehControlOut;

/*! @} */

#endif
