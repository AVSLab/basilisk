#ifndef _GENERAL_RW_OUT_MESSAGE_
#define _GENERAL_RW_OUT_MESSAGE_

#include <stdint.h>
#include "SimCode/utilities/simDefinitions.h"

/*! \addtogroup SimGroup
 * @{
 */

/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {

    DeviceConditionState_t speedState;      /*!<            if spin rate is > 6000 rpm */
    DeviceConditionState_t tempState;       /*!<            if temperature is above a threshold */
    double Omega                            /*!< [rad/s]    RW spin speed */
    double uRWc[3];                         /*!< [Nm]       Echo of the commanded RW torque command */
    int cmdCounter;                         /*!<            command ID counter */
    int current;                            /*!< [A]        rw motor current */
    int motorTemp1;                         /*!< [C]        motor temperature I */
    int motorTemp2;                         /*!< [C]        motor temperature II */
    int maxTorque;                          /*!< [micro-Nm] use set maximum torque value */

}rwGenearlOut;

/*! @} */

#endif
