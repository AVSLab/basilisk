
#ifndef _VEH_EFFECTOR_OUT_H_
#define _VEH_EFFECTOR_OUT_H_

#include <stdint.h>

#define MAX_NUM_EFFECTORS 36

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Structure used to define the output definition for vehicle effectors*/
typedef struct {
    double effectorRequest[MAX_NUM_EFFECTORS];     /*!< -- Control request fraction array*/
}vehEffectorOut;

/*! @} */

#endif
