
#ifndef _VEHICLE_CONFIG_DATA_H_
#define _VEHICLE_CONFIG_DATA_H_

#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Structure used to define a common structure for top level vehicle information*/
typedef struct {
    double BS[9];       /*!< -- DCM from vehicle structure frame S to ADCS body frame B (row major)*/
    uint32_t CurrentADCSState;  /*!< -- Current ADCS state for subsystem */
    double I[9];                /*!< kg m^2 Spacecraft Inertia */
}vehicleConfigData;

/*! @} */

#endif
