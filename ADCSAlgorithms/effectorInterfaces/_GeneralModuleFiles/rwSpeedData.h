
#ifndef _RW_SPEED_DATA_H_
#define _RW_SPEED_DATA_H_

#include <stdint.h>
#include "../ADCSAlgorithms/effectorInterfaces/errorConversion/vehEffectorOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Structure used to define the output definition for reaction wheel speeds*/
typedef struct {
	double wheelSpeeds[MAX_EFF_CNT];                //!< r/s The current angular velocity of the wheel
}RWSpeedData;

/*! @} */

#endif
