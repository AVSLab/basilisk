
#ifndef _SUB_MODULE_OUT_H_
#define _SUB_MODULE_OUT_H_

#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Structure used to define the output of the sub-module.  This is the same
    output message that is used by all sub-modules in the module folder. */
typedef struct {
    double outputVector[3];     /*!< [units] sample output vector*/
}subModuleOut;

/*! @} */

#endif
