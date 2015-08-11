
#ifndef _VEHICLE_CONFIG_DATA_H_
#define _VEHICLE_CONFIG_DATA_H_

#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Structure used to define a common structure for top level vehicle information*/
typedef struct {
   double T_str2body[9];     /*!< -- DCM from vehicle structure to ADCS body (row major)*/
   uint32_t CurrentMode;     /*!< -- Current ADCS mode for subsystem */
}vehicleConfigData;

/*! @} */

#endif
